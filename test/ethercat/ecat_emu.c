/*
 * Minimal EtherCAT slave emulator for ArtyHill's integration tests.
 *
 *   ecat_emu <iface> <nslaves> [pos0 pos1 ...]
 *
 * Answers EtherCAT frames (EtherType 0x88A4) arriving on <iface> as a line of
 * <nslaves> identical slaves, so an unmodified SOEM master on the other end
 * of a veth pair can scan, configure and run them. Emulated per slave:
 *
 *   - 64 KiB ESC address space: registers + process RAM
 *   - all datagram types: APxx/FPxx/BRx/LRx (+ARMW/FRMW), working counters
 *   - AL control -> AL status (every requested state is accepted)
 *   - SII EEPROM interface (0x0502..0x050B) backed by an SII image with
 *     identity, SM, FMMU and Rx/TxPDO categories (no mailbox)
 *   - FMMU logical <-> physical mapping for process data
 *   - a CiA 402 drive in cyclic synchronous position mode:
 *       RxPDO (8 B):  0x6040 controlword u16 | 0x607A target i32 | 0x6060 mode i8 | pad
 *       TxPDO (12 B): 0x6041 statusword u16 | 0x6064 position i32 |
 *                     0x606C velocity i32   | 0x6077 torque i16
 *
 * Fault injection: SIGUSR1 puts every drive into FAULT; SIGUSR2 toggles a
 * "pulled cable" (frames are dropped). SIGTERM/SIGINT exit.
 *
 * Prints "ready" on stdout once listening. Linux only; needs CAP_NET_RAW
 * (e.g. run inside `unshare -rn`).
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
#include <unistd.h>

#define ETH_P_ECAT 0x88A4
#define MAXSLAVES 16
#define MEMSIZE 0x10000
#define EEPWORDS 256

#define OUT_BYTES 8
#define IN_BYTES 12
#define SM2_ADDR 0x1100
#define SM3_ADDR 0x1180
#define MAX_STEP 20000      /* counts per cycle the drive can follow */
#define FOLLOW_LIMIT 200000 /* following error that trips a fault   */

enum { ST_SOD, ST_READY, ST_SWITCHED_ON, ST_OP, ST_QUICK_STOP, ST_FAULT };
static const uint16_t SW[] = {0x0250, 0x0231, 0x0233, 0x0237, 0x0217, 0x0218};

typedef struct {
    uint8_t mem[MEMSIZE];
    uint16_t eep[EEPWORDS];
    int drive_state;
    int32_t pos, vel;
    int16_t torque;
    uint16_t last_cw;
} Slave;

static Slave slaves[MAXSLAVES];
static int nslaves;
static volatile sig_atomic_t want_fault = 0, link_down = 0, stop = 0;

static void on_usr1(int s) { (void)s; want_fault = 1; }
static void on_usr2(int s) { (void)s; link_down = !link_down; }
static void on_term(int s) { (void)s; stop = 1; }

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)rd16(p) | ((uint32_t)rd16(p + 2) << 16); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, v & 0xffff); wr16(p + 2, v >> 16); }

/* ---------------------------------------------------------------- SII image */

static void build_eeprom(Slave *s) {
    uint16_t *e = s->eep;
    memset(e, 0xff, sizeof(s->eep));
    for (int i = 0; i < 0x40; i++) e[i] = 0;
    e[0x08] = 0x4152; e[0x09] = 0x0000;   /* vendor  0x00004152 ("AR")  */
    e[0x0a] = 0x0402; e[0x0b] = 0x0000;   /* product 0x00000402         */
    e[0x0c] = 0x0001; e[0x0d] = 0x0000;   /* revision 1                 */
    e[0x18] = 0; e[0x19] = 0;             /* no standard mailbox        */
    int a = 0x40;
    /* FMMU category (40): FMMU0 outputs, FMMU1 inputs, FMMU2/3 unused */
    e[a++] = 40; e[a++] = 2;
    e[a++] = 0x0201; e[a++] = 0xffff;
    /* SM category (41): 4 sync managers, 4 words each */
    e[a++] = 41; e[a++] = 16;
    e[a++] = 0x1000; e[a++] = 0x0080; e[a++] = 0x0026; e[a++] = 0x0100; /* SM0 mbx out (unused) */
    e[a++] = 0x1080; e[a++] = 0x0080; e[a++] = 0x0022; e[a++] = 0x0200; /* SM1 mbx in  (unused) */
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
    wr16(&s->mem[0x0008], 0x0000);         /* ESC features: no DC */
    wr16(&s->mem[0x0110], index == nslaves - 1 ? 0x0200 : 0x0a00); /* DL status */
    wr16(&s->mem[0x0130], 0x0001);         /* AL status: INIT */
    wr16(&s->mem[0x0140], 0x0008);         /* PDI control */
    build_eeprom(s);
    s->drive_state = ST_SOD;
    s->pos = pos; s->vel = 0; s->torque = 0; s->last_cw = 0;
}

/* ----------------------------------------------------- register side effects */

static void after_write(Slave *s, uint16_t ado, uint16_t len) {
    uint32_t end = (uint32_t)ado + len;
    if (ado <= 0x0120 && end > 0x0120) {   /* AL control */
        uint16_t req = s->mem[0x0120] & 0x0f;
        if (req) wr16(&s->mem[0x0130], req);
        wr16(&s->mem[0x0134], 0);
        if (req < 0x08 && s->drive_state != ST_FAULT) s->drive_state = ST_SOD;
    }
    if (ado <= 0x0502 && end > 0x0502) {   /* EEPROM control: execute command */
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
}

/* ----------------------------------------------------------- CiA 402 drive */

static void drive_step(Slave *s) {
    uint8_t *out = &s->mem[SM2_ADDR];
    uint8_t *in = &s->mem[SM3_ADDR];
    uint16_t cw = rd16(out);
    int32_t target = (int32_t)rd32(out + 2);
    int8_t mode = (int8_t)out[6];
    int al_op = (rd16(&s->mem[0x0130]) & 0x0f) == 0x08;

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
        if (!al_op) s->drive_state = ST_FAULT;           /* lost OP → fault */
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
    if (s->drive_state == ST_OP && mode == 8) {
        int64_t err = (int64_t)target - s->pos;
        if (err > FOLLOW_LIMIT || err < -FOLLOW_LIMIT) {
            s->drive_state = ST_FAULT;                     /* following error */
        } else {
            if (err > MAX_STEP) err = MAX_STEP;
            if (err < -MAX_STEP) err = -MAX_STEP;
            s->pos += (int32_t)err;
        }
    }
    int32_t v = s->pos - prev;
    s->torque = (int16_t)(v - s->vel);                   /* ~ acceleration */
    s->vel = v;

    wr16(in, SW[s->drive_state]);
    wr32(in + 2, (uint32_t)s->pos);
    wr32(in + 6, (uint32_t)s->vel);
    wr16(in + 10, (uint16_t)s->torque);
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
        for (int i = 0; i < len; i++) data[i] = bcast ? (data[i] | s->mem[ado + i]) : s->mem[ado + i];
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
    if (stepped && want_fault) want_fault = 0;   /* delivered to the drives */
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <iface> <nslaves> [pos0 pos1 ...]\n", argv[0]);
        return 2;
    }
    nslaves = atoi(argv[2]);
    if (nslaves < 1 || nslaves > MAXSLAVES) { fprintf(stderr, "nslaves must be 1..%d\n", MAXSLAVES); return 2; }
    for (int i = 0; i < nslaves; i++)
        reset_slave(&slaves[i], i, (3 + i < argc) ? (int32_t)strtol(argv[3 + i], NULL, 0) : 0);

    int fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));
    if (fd < 0) { perror("socket"); return 1; }
    struct sockaddr_ll sll = {0};
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ECAT);
    sll.sll_ifindex = (int)if_nametoindex(argv[1]);
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
