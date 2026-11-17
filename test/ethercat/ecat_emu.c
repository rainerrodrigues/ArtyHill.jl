/*
 * EtherCAT slave emulator for ArtyHill's integration tests.
 *
 *   ecat_emu [--sii] [--dc-required] <iface> <nslaves> [pos0 pos1 ...]
 *
 * Answers EtherCAT frames (EtherType 0x88A4) arriving on <iface> as a line of
 * <nslaves> identical servo drives, so an unmodified SOEM master on the other
 * end of a veth pair can scan, configure and run them. Emulated per slave:
 *
 *   ESC     64 KiB address space; all datagram types (APxx/FPxx/BRx/LRx/xRMW)
 *           with working counters; AL control/status with error + ack;
 *           SII EEPROM interface; FMMU logical<->physical mapping
 *   CoE     standard mailbox on SM0/SM1 with expedited SDO upload/download and
 *           aborts; object dictionary with 0x1C00/0x1C12/0x1C13 assignment,
 *           0x1600/0x1601/0x1A00/0x1A01 mappings (writable in PRE-OP only),
 *           0x6060 mode, 0x6065 following-error window, 0x1018 identity and
 *           0x2000 sync statistics. Process data is laid out from the active
 *           PDO mapping, so remapping over SDO really changes the image.
 *   DC      64-bit system time (CLOCK_MONOTONIC based) with offset, receive
 *           time latching, Sync0 start/cycle/activation registers. With
 *           --dc-required the drives refuse SAFE-OP -> OP (AL status code
 *           0x0030) unless Sync0 is active, like most real servo drives.
 *   CiA 402 controlword/statusword state machine, cyclic synchronous position
 *           (8) and velocity (9) modes, following-error fault.
 *
 * --sii disables the mailbox: PDOs then come from the SII image only.
 *
 * Fault injection: SIGUSR1 puts every drive into FAULT; SIGUSR2 toggles a
 * "pulled cable" (frames are dropped). SIGTERM/SIGINT exit.
 * Prints "ready" on stdout once listening. Linux only; needs CAP_NET_RAW.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define ETH_P_ECAT 0x88A4
#define MAXSLAVES 16
#define MEMSIZE 0x10000
#define EEPWORDS 256
#define MAXOD 64

#define MBX_OUT 0x1000      /* master -> slave (SM0) */
#define MBX_IN 0x1080       /* slave -> master (SM1) */
#define MBX_LEN 0x80
#define SM2_ADDR 0x1100
#define SM3_ADDR 0x1180
#define MAX_STEP 20000      /* counts per cycle the drive can follow */

enum { ST_SOD, ST_READY, ST_SWITCHED_ON, ST_OP, ST_QUICK_STOP, ST_FAULT };
static const uint16_t SW[] = {0x0250, 0x0231, 0x0233, 0x0237, 0x0217, 0x0218};

enum { RO, RW, RW_PREOP };
typedef struct { uint16_t idx; uint8_t sub, size, access; uint32_t val; } ODEntry;

typedef struct {
    uint8_t mem[MEMSIZE];
    uint16_t eep[EEPWORDS];
    ODEntry od[MAXOD];
    int nod;
    int drive_state;
    int32_t pos, vel;
    int16_t torque;
    uint16_t last_cw;
    int64_t phase_min, phase_max, phase_last, phase_sum;
    uint32_t frames;
} Slave;

static Slave slaves[MAXSLAVES];
static int nslaves, use_coe = 1, dc_required = 0;
static int64_t t0;
static volatile sig_atomic_t want_fault = 0, link_down = 0, stop = 0;

static void on_usr1(int s) { (void)s; want_fault = 1; }
static void on_usr2(int s) { (void)s; link_down = !link_down; }
static void on_term(int s) { (void)s; stop = 1; }

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)rd16(p) | ((uint32_t)rd16(p + 2) << 16); }
static uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, v & 0xffff); wr16(p + 2, v >> 16); }
static void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32)); }

static int64_t mono_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static uint64_t local_time(void) { return (uint64_t)(mono_ns() - t0 + 1000000000LL); }
static uint64_t system_time(Slave *s) { return local_time() + rd64(&s->mem[0x0920]); }
static int al_state(Slave *s) { return rd16(&s->mem[0x0130]) & 0x0f; }

/* ------------------------------------------------------------ object dictionary */

static void od_add(Slave *s, uint16_t idx, uint8_t sub, uint8_t size, uint8_t access, uint32_t val) {
    s->od[s->nod++] = (ODEntry){idx, sub, size, access, val};
}

static ODEntry *od_find(Slave *s, uint16_t idx, uint8_t sub, uint32_t *abort) {
    int idx_seen = 0;
    for (int i = 0; i < s->nod; i++) {
        if (s->od[i].idx != idx) continue;
        idx_seen = 1;
        if (s->od[i].sub == sub) return &s->od[i];
    }
    *abort = idx_seen ? 0x06090011 : 0x06020000;
    return NULL;
}

static uint32_t od_get(Slave *s, uint16_t idx, uint8_t sub, uint32_t dflt) {
    uint32_t a;
    ODEntry *e = od_find(s, idx, sub, &a);
    return e ? e->val : dflt;
}

static void build_od(Slave *s) {
    s->nod = 0;
    od_add(s, 0x1018, 0, 1, RO, 4);
    od_add(s, 0x1018, 1, 4, RO, 0x00004152);
    od_add(s, 0x1018, 2, 4, RO, 0x00000402);
    od_add(s, 0x1018, 3, 4, RO, 1);
    od_add(s, 0x1018, 4, 4, RO, 0);
    od_add(s, 0x1C00, 0, 1, RO, 4);
    for (int i = 1; i <= 4; i++) od_add(s, 0x1C00, (uint8_t)i, 1, RO, (uint32_t)i);
    od_add(s, 0x1C12, 0, 1, RW_PREOP, 1);
    od_add(s, 0x1C12, 1, 2, RW_PREOP, 0x1600);
    od_add(s, 0x1C13, 0, 1, RW_PREOP, 1);
    od_add(s, 0x1C13, 1, 2, RW_PREOP, 0x1A00);
    /* default RxPDO 0x1600: controlword, target position, mode, padding */
    static const uint32_t rx0[] = {0x60400010, 0x607A0020, 0x60600008, 0x00000008};
    static const uint32_t tx0[] = {0x60410010, 0x60640020, 0x606C0020, 0x60770010};
    od_add(s, 0x1600, 0, 1, RW_PREOP, 4);
    for (int i = 0; i < 8; i++) od_add(s, 0x1600, (uint8_t)(i + 1), 4, RW_PREOP, i < 4 ? rx0[i] : 0);
    od_add(s, 0x1601, 0, 1, RW_PREOP, 0);
    for (int i = 0; i < 8; i++) od_add(s, 0x1601, (uint8_t)(i + 1), 4, RW_PREOP, 0);
    od_add(s, 0x1A00, 0, 1, RW_PREOP, 4);
    for (int i = 0; i < 8; i++) od_add(s, 0x1A00, (uint8_t)(i + 1), 4, RW_PREOP, i < 4 ? tx0[i] : 0);
    od_add(s, 0x1A01, 0, 1, RW_PREOP, 0);
    for (int i = 0; i < 8; i++) od_add(s, 0x1A01, (uint8_t)(i + 1), 4, RW_PREOP, 0);
    od_add(s, 0x2000, 0, 1, RO, 5);
    od_add(s, 0x2000, 1, 4, RO, 0);       /* last frame phase after Sync0, ns */
    od_add(s, 0x2000, 2, 4, RW, 0);       /* min phase since reset (write resets) */
    od_add(s, 0x2000, 3, 4, RO, 0);       /* max phase since reset */
    od_add(s, 0x2000, 4, 4, RO, 0);       /* frames counted since reset */
    od_add(s, 0x2000, 5, 4, RO, 0);       /* mean phase since reset */
    od_add(s, 0x6060, 0, 1, RW, 8);       /* modes of operation */
    od_add(s, 0x6061, 0, 1, RO, 8);       /* modes of operation display */
    od_add(s, 0x6065, 0, 4, RW, 200000);  /* following error window */
    od_add(s, 0x6502, 0, 4, RO, 0x00000180); /* supported: CSP, CSV */
}

static int mappable(uint32_t m, int rx) {
    uint16_t idx = m >> 16;
    if (idx == 0) return 1;  /* padding */
    if (rx) return idx == 0x6040 || idx == 0x607A || idx == 0x60FF || idx == 0x6071 || idx == 0x6060;
    return idx == 0x6041 || idx == 0x6064 || idx == 0x606C || idx == 0x6077 || idx == 0x6061 || idx == 0x2000;
}

/* SDO write with CANopen access rules. Returns 0 or an abort code. */
static uint32_t od_write(Slave *s, uint16_t idx, uint8_t sub, uint32_t val, int size) {
    uint32_t abort;
    ODEntry *e = od_find(s, idx, sub, &abort);
    if (!e) return abort;
    if (e->access == RO) return 0x06010002;
    if (e->access == RW_PREOP && al_state(s) != 0x02) return 0x08000022;
    if (size != e->size) return 0x06070010;
    int is_map = (idx >= 0x1600 && idx <= 0x1601) || (idx >= 0x1A00 && idx <= 0x1A01);
    if (is_map && sub == 0 && val > 8) return 0x06090030;
    if (is_map && sub > 0 && !mappable(val, idx < 0x1A00)) return 0x06040041;
    if ((idx == 0x1C12 || idx == 0x1C13) && sub == 0 && val > 1) return 0x06090030;
    if (idx == 0x1C12 && sub == 1 && val != 0x1600 && val != 0x1601) return 0x06090030;
    if (idx == 0x1C13 && sub == 1 && val != 0x1A00 && val != 0x1A01) return 0x06090030;
    if (idx == 0x6060 && val != 8 && val != 9) return 0x06090030;
    e->val = val;
    if (idx == 0x2000 && sub == 2) { s->phase_min = INT64_MAX; s->phase_max = INT64_MIN; s->phase_sum = 0; s->frames = 0; }
    return 0;
}

/* ------------------------------------------------------------- PDO layout */

typedef struct { int cw, target_pos, target_vel, target_tq, mode, size; } RxLayout;
typedef struct { int sw, pos, vel, tq, mode_disp, phase, size; } TxLayout;

static void layout(Slave *s, RxLayout *rx, TxLayout *tx) {
    *rx = (RxLayout){-1, -1, -1, -1, -1, 0};
    *tx = (TxLayout){-1, -1, -1, -1, -1, -1, 0};
    for (int dir = 0; dir < 2; dir++) {
        uint16_t assign = dir == 0 ? 0x1C12 : 0x1C13;
        int bit = 0;
        uint32_t n = od_get(s, assign, 0, 0);
        for (uint32_t a = 1; a <= n; a++) {
            uint16_t pdo = (uint16_t)od_get(s, assign, (uint8_t)a, 0);
            uint32_t ne = od_get(s, pdo, 0, 0);
            for (uint32_t k = 1; k <= ne; k++) {
                uint32_t m = od_get(s, pdo, (uint8_t)k, 0);
                uint16_t idx = m >> 16;
                int off = bit / 8;
                if (dir == 0) {
                    if (idx == 0x6040) rx->cw = off;
                    if (idx == 0x607A) rx->target_pos = off;
                    if (idx == 0x60FF) rx->target_vel = off;
                    if (idx == 0x6071) rx->target_tq = off;
                    if (idx == 0x6060) rx->mode = off;
                } else {
                    if (idx == 0x6041) tx->sw = off;
                    if (idx == 0x6064) tx->pos = off;
                    if (idx == 0x606C) tx->vel = off;
                    if (idx == 0x6077) tx->tq = off;
                    if (idx == 0x6061) tx->mode_disp = off;
                    if (idx == 0x2000) tx->phase = off;
                }
                bit += m & 0xff;
            }
        }
        if (dir == 0) rx->size = bit / 8; else tx->size = bit / 8;
    }
}

/* ---------------------------------------------------------------- SII image */

static void build_eeprom(Slave *s) {
    uint16_t *e = s->eep;
    memset(e, 0xff, sizeof(s->eep));
    for (int i = 0; i < 0x40; i++) e[i] = 0;
    e[0x08] = 0x4152; e[0x09] = 0x0000;   /* vendor  0x00004152 ("AR")  */
    e[0x0a] = 0x0402; e[0x0b] = 0x0000;   /* product 0x00000402         */
    e[0x0c] = 0x0001; e[0x0d] = 0x0000;   /* revision 1                 */
    if (use_coe) {
        e[0x18] = MBX_OUT; e[0x19] = MBX_LEN; /* standard rx mailbox */
        e[0x1a] = MBX_IN;  e[0x1b] = MBX_LEN; /* standard tx mailbox */
        e[0x1c] = 0x0004;                     /* protocols: CoE */
    }
    int a = 0x40;
    /* FMMU category (40): FMMU0 outputs, FMMU1 inputs, FMMU2/3 unused */
    e[a++] = 40; e[a++] = 2;
    e[a++] = 0x0201; e[a++] = 0xffff;
    /* SM category (41): 4 sync managers, 4 words each */
    e[a++] = 41; e[a++] = 16;
    e[a++] = MBX_OUT; e[a++] = MBX_LEN; e[a++] = 0x0026; e[a++] = (1 << 8) | (use_coe ? 1 : 0);
    e[a++] = MBX_IN;  e[a++] = MBX_LEN; e[a++] = 0x0022; e[a++] = (2 << 8) | (use_coe ? 1 : 0);
    e[a++] = SM2_ADDR; e[a++] = 0; e[a++] = 0x0064; e[a++] = 0x0301;    /* SM2 outputs */
    e[a++] = SM3_ADDR; e[a++] = 0; e[a++] = 0x0020; e[a++] = 0x0401;    /* SM3 inputs  */
    /* TxPDO category (50): 0x1A00 on SM3, 4 entries */
    e[a++] = 50; e[a++] = 4 + 4 * 4;
    e[a++] = 0x1A00; e[a++] = (3 << 8) | 4; e[a++] = 0; e[a++] = 0;
    e[a++] = 0x6041; e[a++] = 0x0000; e[a++] = (16 << 8) | 0x06; e[a++] = 0;
    e[a++] = 0x6064; e[a++] = 0x0000; e[a++] = (32 << 8) | 0x04; e[a++] = 0;
    e[a++] = 0x606C; e[a++] = 0x0000; e[a++] = (32 << 8) | 0x04; e[a++] = 0;
    e[a++] = 0x6077; e[a++] = 0x0000; e[a++] = (16 << 8) | 0x03; e[a++] = 0;
    /* RxPDO category (51): 0x1600 on SM2, 4 entries (last one is padding) */
    e[a++] = 51; e[a++] = 4 + 4 * 4;
    e[a++] = 0x1600; e[a++] = (2 << 8) | 4; e[a++] = 0; e[a++] = 0;
    e[a++] = 0x6040; e[a++] = 0x0000; e[a++] = (16 << 8) | 0x06; e[a++] = 0;
    e[a++] = 0x607A; e[a++] = 0x0000; e[a++] = (32 << 8) | 0x04; e[a++] = 0;
    e[a++] = 0x6060; e[a++] = 0x0000; e[a++] = (8 << 8) | 0x02; e[a++] = 0;
    e[a++] = 0x0000; e[a++] = 0x0000; e[a++] = (8 << 8) | 0x00; e[a++] = 0;
    e[a++] = 0xffff;                       /* end of categories */
}

static void reset_slave(Slave *s, int index, int32_t pos) {
    memset(s->mem, 0, sizeof(s->mem));
    s->mem[0x0000] = 0x11;                 /* ESC type */
    s->mem[0x0001] = 0x01;                 /* revision */
    s->mem[0x0007] = 0x0f;                 /* port descriptor */
    wr16(&s->mem[0x0008], 0x000c);         /* ESC features: DC, 64-bit DC */
    wr16(&s->mem[0x0110], index == nslaves - 1 ? 0x0200 : 0x0a00); /* DL status */
    wr16(&s->mem[0x0130], 0x0001);         /* AL status: INIT */
    wr16(&s->mem[0x0140], 0x0008);         /* PDI control */
    build_eeprom(s);
    build_od(s);
    s->drive_state = ST_SOD;
    s->pos = pos; s->vel = 0; s->torque = 0; s->last_cw = 0;
    s->phase_min = INT64_MAX; s->phase_max = INT64_MIN; s->phase_last = 0; s->phase_sum = 0; s->frames = 0;
}

/* ------------------------------------------------------------- CoE mailbox */

static void mbx_reply(Slave *s, uint8_t cnt, uint8_t cmd, uint16_t idx, uint8_t sub, uint32_t data) {
    uint8_t *r = &s->mem[MBX_IN];
    memset(r, 0, MBX_LEN);
    wr16(r, 10);                           /* length */
    wr16(r + 2, 0);                        /* address */
    r[4] = 0;                              /* channel / priority */
    r[5] = (uint8_t)(0x03 | (cnt << 4));   /* CoE */
    /* CoE service: SDO response, except aborts, which travel as SDO request (ETG.1000.6) */
    wr16(r + 6, (uint16_t)((cmd == 0x80 ? 2 : 3) << 12));
    r[8] = cmd;
    wr16(r + 9, idx);
    r[11] = sub;
    wr32(r + 12, data);
    s->mem[0x080D] |= 0x08;                /* SM1: mailbox full */
}

static void handle_mailbox(Slave *s) {
    uint8_t *m = &s->mem[MBX_OUT];
    uint8_t type = m[5] & 0x0f, cnt = (m[5] >> 4) & 0x07;
    if (type != 0x03) return;              /* only CoE */
    uint16_t service = rd16(m + 6) >> 12;
    if (service != 2) return;              /* only SDO requests */
    uint8_t cmd = m[8], sub = m[11];
    uint16_t idx = rd16(m + 9);
    uint32_t abort = 0;
    if (cmd & 0x10) { mbx_reply(s, cnt, 0x80, idx, sub, 0x06010000); return; } /* no complete access */
    switch (cmd & 0xe0) {
    case 0x20: {                           /* download initiate */
        int size;
        uint32_t val;
        if (cmd & 0x02) { size = (cmd & 0x01) ? 4 - ((cmd >> 2) & 0x03) : 4; val = rd32(m + 12); }
        else { size = (int)rd32(m + 12); val = size <= 4 ? rd32(m + 16) : 0; }
        if (size > 4) { mbx_reply(s, cnt, 0x80, idx, sub, 0x06070010); return; }
        if (size < 4) val &= (1u << (8 * size)) - 1;
        abort = od_write(s, idx, sub, val, size);
        if (abort) mbx_reply(s, cnt, 0x80, idx, sub, abort);
        else mbx_reply(s, cnt, 0x60, idx, sub, 0);
        break;
    }
    case 0x40: {                           /* upload initiate */
        ODEntry *e = od_find(s, idx, sub, &abort);
        if (!e) { mbx_reply(s, cnt, 0x80, idx, sub, abort); return; }
        uint32_t val = e->val;
        if (idx == 0x2000 && sub == 1) val = (uint32_t)s->phase_last;
        if (idx == 0x2000 && sub == 2) val = s->frames ? (uint32_t)s->phase_min : 0;
        if (idx == 0x2000 && sub == 3) val = s->frames ? (uint32_t)s->phase_max : 0;
        if (idx == 0x2000 && sub == 4) val = s->frames;
        if (idx == 0x2000 && sub == 5) val = s->frames ? (uint32_t)(s->phase_sum / s->frames) : 0;
        mbx_reply(s, cnt, (uint8_t)(0x43 | ((4 - e->size) << 2)), idx, sub, val);
        break;
    }
    default:
        mbx_reply(s, cnt, 0x80, idx, sub, 0x05040001);
    }
}

/* ----------------------------------------------------- register side effects */

static int overlaps(uint16_t ado, uint16_t len, uint16_t reg, uint16_t rlen) {
    return ado < reg + rlen && reg < (uint32_t)ado + len;
}

static void before_read(Slave *s, uint16_t ado, uint16_t len) {
    if (overlaps(ado, len, 0x0910, 8)) wr64(&s->mem[0x0910], system_time(s));
}

static void after_read(Slave *s, uint16_t ado, uint16_t len) {
    if (use_coe && overlaps(ado, len, MBX_IN, 1)) s->mem[0x080D] &= (uint8_t)~0x08;  /* mailbox emptied */
}

static void after_write(Slave *s, uint16_t ado, uint16_t len) {
    if (overlaps(ado, len, 0x0120, 1)) {   /* AL control */
        uint16_t ctl = s->mem[0x0120];
        uint16_t req = ctl & 0x0f, cur = (uint16_t)al_state(s);
        uint16_t err = rd16(&s->mem[0x0130]) & 0x10;
        if (err && !(ctl & 0x10) && req > cur) return;          /* error not acknowledged */
        if (req == 0x08 && dc_required &&
            !((s->mem[0x0981] & 0x03) == 0x03 && rd32(&s->mem[0x09A0]) > 0)) {
            wr16(&s->mem[0x0130], cur | 0x10);                   /* refuse: invalid DC sync */
            wr16(&s->mem[0x0134], 0x0030);
            return;
        }
        if (req) wr16(&s->mem[0x0130], req);
        wr16(&s->mem[0x0134], 0);
        if (req < 0x08 && s->drive_state != ST_FAULT) s->drive_state = ST_SOD;
    }
    if (overlaps(ado, len, 0x0502, 1)) {   /* EEPROM control: execute command */
        uint16_t cmd = rd16(&s->mem[0x0502]);
        uint16_t addr = rd16(&s->mem[0x0504]);
        if ((cmd & 0x0700) == 0x0100) {    /* read */
            for (int i = 0; i < 2; i++) {
                uint16_t w = (addr + i < EEPWORDS) ? s->eep[addr + i] : 0xffff;
                wr16(&s->mem[0x0508 + 2 * i], w);
            }
        }
        wr16(&s->mem[0x0502], 0x0000);     /* idle, no errors, 4-byte reads */
    }
    if (overlaps(ado, len, 0x0900, 1)) {   /* latch DC receive times */
        uint64_t t = local_time();
        wr32(&s->mem[0x0900], (uint32_t)t);
        wr32(&s->mem[0x0904], (uint32_t)(t + 500));
        wr64(&s->mem[0x0918], t);
    }
    if (use_coe && overlaps(ado, len, MBX_OUT, 1)) handle_mailbox(s);
}

/* ----------------------------------------------------------- CiA 402 drive */

static void record_phase(Slave *s) {
    if ((s->mem[0x0981] & 0x03) != 0x03) return;
    int64_t cycle = rd32(&s->mem[0x09A0]);
    if (cycle <= 0) return;
    int64_t start = (int64_t)rd64(&s->mem[0x0990]);
    int64_t ph = ((int64_t)system_time(s) - start) % cycle;
    if (ph < 0) ph += cycle;
    s->phase_last = ph;
    if (ph < s->phase_min) s->phase_min = ph;
    if (ph > s->phase_max) s->phase_max = ph;
    s->phase_sum += ph;
    s->frames++;
}

static void drive_step(Slave *s) {
    RxLayout rx; TxLayout tx;
    layout(s, &rx, &tx);
    uint8_t *out = &s->mem[SM2_ADDR];
    uint8_t *in = &s->mem[SM3_ADDR];
    uint16_t cw = rx.cw >= 0 ? rd16(out + rx.cw) : 0;
    int8_t mode = rx.mode >= 0 ? (int8_t)out[rx.mode] : (int8_t)od_get(s, 0x6060, 0, 8);
    int al_op = al_state(s) == 0x08;
    int64_t follow_limit = od_get(s, 0x6065, 0, 200000);

    record_phase(s);
    if (want_fault) s->drive_state = ST_FAULT;
    switch (s->drive_state) {
    case ST_SOD:
        if ((cw & 0x87) == 0x06) s->drive_state = ST_READY;
        break;
    case ST_READY:
        if ((cw & 0x8f) == 0x07) s->drive_state = ST_SWITCHED_ON;
        else if ((cw & 0x82) == 0x00) s->drive_state = ST_SOD;
        break;
    case ST_SWITCHED_ON:
        if ((cw & 0x8f) == 0x0f && al_op) s->drive_state = ST_OP;
        else if ((cw & 0x87) == 0x06) s->drive_state = ST_READY;
        else if ((cw & 0x82) == 0x00) s->drive_state = ST_SOD;
        break;
    case ST_OP:
        if (!al_op) s->drive_state = ST_FAULT;           /* lost OP -> fault */
        else if ((cw & 0x8f) == 0x07) s->drive_state = ST_SWITCHED_ON;
        else if ((cw & 0x87) == 0x06) s->drive_state = ST_READY;
        else if ((cw & 0x86) == 0x02) s->drive_state = ST_QUICK_STOP;
        else if ((cw & 0x02) == 0x00) s->drive_state = ST_SOD;
        break;
    case ST_QUICK_STOP:
        if ((cw & 0x02) == 0x00) s->drive_state = ST_SOD;
        break;
    case ST_FAULT:
        if ((cw & 0x80) && !(s->last_cw & 0x80)) s->drive_state = ST_SOD;  /* reset on rising edge */
        break;
    }
    s->last_cw = cw;

    int32_t prev = s->pos;
    if (s->drive_state == ST_OP && mode == 8 && rx.target_pos >= 0) {
        int64_t err = (int64_t)(int32_t)rd32(out + rx.target_pos) - s->pos;
        if (err > follow_limit || err < -follow_limit) {
            s->drive_state = ST_FAULT;                     /* following error */
        } else {
            if (err > MAX_STEP) err = MAX_STEP;
            if (err < -MAX_STEP) err = -MAX_STEP;
            s->pos += (int32_t)err;
        }
    } else if (s->drive_state == ST_OP && mode == 9 && rx.target_vel >= 0) {
        int32_t v = (int32_t)rd32(out + rx.target_vel);   /* counts per cycle */
        if (v > MAX_STEP) v = MAX_STEP;
        if (v < -MAX_STEP) v = -MAX_STEP;
        s->pos += v;
    }
    int32_t v = s->pos - prev;
    s->torque = (int16_t)(v - s->vel);                   /* ~ acceleration */
    s->vel = v;
    for (int i = 0; i < s->nod; i++)
        if (s->od[i].idx == 0x6061) s->od[i].val = (uint8_t)mode;

    if (tx.sw >= 0) wr16(in + tx.sw, SW[s->drive_state]);
    if (tx.pos >= 0) wr32(in + tx.pos, (uint32_t)s->pos);
    if (tx.vel >= 0) wr32(in + tx.vel, (uint32_t)s->vel);
    if (tx.tq >= 0) wr16(in + tx.tq, (uint16_t)s->torque);
    if (tx.mode_disp >= 0) in[tx.mode_disp] = (uint8_t)mode;
    if (tx.phase >= 0) wr32(in + tx.phase, (uint32_t)s->phase_last);
}

/* ---------------------------------------------------------- datagram engine */

/* Logical access through the FMMUs of one slave. Returns its wkc increment. */
static int logical(Slave *s, uint8_t cmd, uint32_t laddr, uint8_t *data, uint16_t len, int *wrote) {
    int did_read = 0, did_write = 0;
    for (int f = 0; f < 16; f++) {
        uint8_t *fm = &s->mem[0x0600 + 16 * f];
        if (!fm[12]) continue;                              /* inactive */
        uint32_t lstart = rd32(fm);
        uint16_t flen = rd16(fm + 4);
        uint16_t pstart = rd16(fm + 8);
        uint8_t type = fm[11];
        uint32_t lo = laddr > lstart ? laddr : lstart;
        uint32_t hi = (laddr + len) < (lstart + flen) ? (laddr + len) : (lstart + flen);
        if (lo >= hi) continue;
        for (uint32_t a = lo; a < hi; a++) {
            uint8_t *m = &s->mem[(uint16_t)(pstart + (a - lstart))];
            uint8_t *d = &data[a - laddr];
            if ((type & 1) && (cmd == 10 || cmd == 12)) { *d = *m; did_read = 1; }
            if ((type & 2) && (cmd == 11 || cmd == 12)) { *m = *d; did_write = 1; }
        }
    }
    *wrote |= did_write;
    if (cmd == 12) return did_read + 2 * did_write;
    return did_read | did_write;
}

static int physical(Slave *s, int rd, int wrt, uint16_t ado, uint8_t *data, uint16_t len, int bcast) {
    int inc = 0;
    if ((uint32_t)ado + len > MEMSIZE) return 0;
    if (rd) {
        before_read(s, ado, len);
        for (int i = 0; i < len; i++) data[i] = bcast ? (data[i] | s->mem[ado + i]) : s->mem[ado + i];
        after_read(s, ado, len);
        inc += 1;
    }
    if (wrt) {
        if (!rd || !bcast) memcpy(&s->mem[ado], data, len);
        after_write(s, ado, len);
        inc += rd ? 2 : 1;
    }
    return inc;
}

static void process_datagrams(uint8_t *p, int plen) {
    if (plen < 2) return;
    uint16_t hdr = rd16(p);
    int total = hdr & 0x07ff;
    int stepped = 0;
    uint8_t *d = p + 2, *end = p + 2 + (total < plen - 2 ? total : plen - 2);
    while (d + 12 <= end) {
        uint8_t cmd = d[0];
        uint16_t adp = rd16(d + 2), ado = rd16(d + 4);
        uint32_t laddr = rd32(d + 2);
        uint16_t lenf = rd16(d + 6);
        uint16_t len = lenf & 0x07ff;
        uint8_t *data = d + 10;
        if (data + len + 2 > end) break;
        uint16_t wkc = rd16(data + len);
        int any_pd_write[MAXSLAVES] = {0};

        for (int i = 0; i < nslaves; i++) {
            Slave *s = &slaves[i];
            uint16_t station = rd16(&s->mem[0x0010]);
            switch (cmd) {
            case 1: case 2: case 3:                              /* APRD/APWR/APRW */
                if (adp == 0) wkc += physical(s, cmd != 2, cmd != 1, ado, data, len, 0);
                adp++;
                break;
            case 4: case 5: case 6:                              /* FPRD/FPWR/FPRW */
                if (adp == station) wkc += physical(s, cmd != 5, cmd != 4, ado, data, len, 0);
                break;
            case 7: case 8: case 9:                              /* BRD/BWR/BRW */
                wkc += physical(s, cmd != 8, cmd != 7, ado, data, len, 1);
                adp++;
                break;
            case 10: case 11: case 12:                           /* LRD/LWR/LRW */
                wkc += logical(s, cmd, laddr, data, len, &any_pd_write[i]);
                break;
            case 13:                                             /* ARMW */
                wkc += physical(s, adp == 0, adp != 0, ado, data, len, 0);
                adp++;
                break;
            case 14:                                             /* FRMW */
                wkc += physical(s, adp == station, adp != station, ado, data, len, 0);
                break;
            default:
                break;
            }
        }
        if (cmd <= 3 || (cmd >= 7 && cmd <= 9) || cmd == 13) wr16(d + 2, adp);
        wr16(data + len, wkc);
        if (cmd >= 10 && cmd <= 12)
            for (int i = 0; i < nslaves; i++)
                if (any_pd_write[i]) { drive_step(&slaves[i]); stepped = 1; }
        d = data + len + 2;
        if (!(lenf & 0x8000)) break;                         /* no more datagrams */
    }
    if (stepped && want_fault) want_fault = 0;               /* delivered to the drives */
}

int main(int argc, char **argv) {
    int argi = 1;
    while (argi < argc && strncmp(argv[argi], "--", 2) == 0) {
        if (!strcmp(argv[argi], "--sii")) use_coe = 0;
        else if (!strcmp(argv[argi], "--dc-required")) dc_required = 1;
        else { fprintf(stderr, "unknown option %s\n", argv[argi]); return 2; }
        argi++;
    }
    if (argc - argi < 2) {
        fprintf(stderr, "usage: %s [--sii] [--dc-required] <iface> <nslaves> [pos0 pos1 ...]\n", argv[0]);
        return 2;
    }
    const char *iface = argv[argi];
    nslaves = atoi(argv[argi + 1]);
    if (nslaves < 1 || nslaves > MAXSLAVES) { fprintf(stderr, "nslaves must be 1..%d\n", MAXSLAVES); return 2; }
    t0 = mono_ns();
    for (int i = 0; i < nslaves; i++)
        reset_slave(&slaves[i], i, (argi + 2 + i < argc) ? (int32_t)strtol(argv[argi + 2 + i], NULL, 0) : 0);

    int fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));
    if (fd < 0) { perror("socket"); return 1; }
    struct sockaddr_ll sll = {0};
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ECAT);
    sll.sll_ifindex = (int)if_nametoindex(iface);
    if (!sll.sll_ifindex) { perror("if_nametoindex"); return 1; }
    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) < 0) { perror("bind"); return 1; }
    struct timeval tv = {0, 100000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sigaction sa = {0};
    sa.sa_handler = on_usr1; sigaction(SIGUSR1, &sa, NULL);
    sa.sa_handler = on_usr2; sigaction(SIGUSR2, &sa, NULL);
    sa.sa_handler = on_term; sigaction(SIGTERM, &sa, NULL); sigaction(SIGINT, &sa, NULL);

    printf("ready\n");
    fflush(stdout);

    uint8_t buf[2048];
    while (!stop) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < 0) { if (errno == EAGAIN || errno == EINTR) continue; perror("recv"); break; }
        if (n < 16 || buf[12] != 0x88 || buf[13] != 0xA4) continue;
        if (link_down) continue;
        process_datagrams(buf + 14, (int)n - 14);
        if (send(fd, buf, (size_t)n, 0) < 0 && errno != EINTR) perror("send");
    }
    close(fd);
    return 0;
}
