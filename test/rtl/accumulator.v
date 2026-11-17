// 16-bit accumulator: the RTL counterpart of test/mock/accumulator.c.
// On every rising clock edge the signed input u is added to the output y
// (two's-complement wrap-around, like real 16-bit hardware).
module accumulator (
    input  wire        clk,
    input  wire [15:0] u,
    output reg  [15:0] y
);
    initial y = 16'd0;
    always @(posedge clk) y <= y + u;
endmodule
