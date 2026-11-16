/* Hand-written stand-in for a Verilated model, exposing the same C ABI that
 * VeriJul's generated wrapper does (top_create/top_eval/top_settle/top_destroy,
 * get_<port>/set_<port>). Behaves like this RTL:
 *
 *   module accumulator(input clk, input [15:0] u, output reg [15:0] y);
 *     always @(posedge clk) y <= y + u;   // 16-bit wrap-around
 *   endmodule
 */
#include <stdint.h>
#include <stdlib.h>
typedef struct { uint8_t clk, last_clk; uint16_t u, y; } Handle;
Handle *top_create(void) { return calloc(1, sizeof(Handle)); }
void top_settle(Handle *h) { (void)h; }
void top_eval(Handle *h, uint64_t t) {
    (void)t;
    if (h->clk && !h->last_clk) h->y = (uint16_t)(h->y + h->u);
    h->last_clk = h->clk;
}
void top_destroy(Handle *h) { free(h); }
void set_clk(Handle *h, uint8_t v) { h->clk = v & 1; }
uint8_t get_clk(Handle *h) { return h->clk; }
void set_u(Handle *h, uint16_t v) { h->u = v; }
uint16_t get_u(Handle *h) { return h->u; }
uint16_t get_y(Handle *h) { return h->y; }
