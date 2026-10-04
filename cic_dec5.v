`timescale 1ns / 1ps
// =============================================================================
// cic_dec5 -- fixed 4-stage CIC decimator, R = 5  (50 MHz in -> 10 MHz out)
//
//  * Runs entirely on the 50 MHz clock. The 10 MHz rate is a 1-clock "dvalid"
//    pulse every 5th cycle (clock enable), NOT a divided clock.
//  * Gain = R^N = 5^4 = 625.  Register width = 12 + ceil(4*log2(5)) = 22 bits.
//    Output is scaled by >>> 10 (divide by 1024), so net gain = 625/1024 = 0.61.
//    A full-scale input (+-2047) therefore cannot overflow the 12-bit output.
//  * Integrators use wrap-around arithmetic on purpose (Hogenauer): the final
//    result is correct as long as it fits in W bits.
//  * dout/dvalid are registered together: dout is valid in the same cycle as
//    the dvalid pulse and HOLDS its value until the next pulse (5 clocks).
//  * Passband droop: ~3.9 dB at 0.4 * fs_out (4 MHz).  Use the inner ~40 % of
//    the 0..5 MHz span for amplitude-accurate readings.
// =============================================================================
module cic_dec5 #(
    parameter IN_W  = 12,
    parameter W     = 22,
    parameter SHIFT = 10
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire signed [IN_W-1:0]   din,
    output reg  signed [IN_W-1:0]   dout,
    output reg                      dvalid
);
    localparam integer R = 5;

    // decimation strobe (clock enable)
    reg [2:0] cnt;
    wire strobe = (cnt == R-1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)       cnt <= 3'd0;
        else if (strobe)  cnt <= 3'd0;
        else              cnt <= cnt + 3'd1;
    end

    // integrators (every clock)
    wire signed [W-1:0] x = {{(W-IN_W){din[IN_W-1]}}, din};
    reg  signed [W-1:0] i0, i1, i2, i3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i0 <= 0; i1 <= 0; i2 <= 0; i3 <= 0;
        end else begin
            i0 <= i0 + x;
            i1 <= i1 + i0;
            i2 <= i2 + i1;
            i3 <= i3 + i2;
        end
    end

    // combs (only on strobe)
    reg  signed [W-1:0] d0, d1, d2, d3;
    wire signed [W-1:0] c0 = i3 - d0;
    wire signed [W-1:0] c1 = c0 - d1;
    wire signed [W-1:0] c2 = c1 - d2;
    wire signed [W-1:0] c3 = c2 - d3;
    wire signed [W-1:0] scaled = c3 >>> SHIFT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0;
            dout <= 0; dvalid <= 1'b0;
        end else begin
            dvalid <= 1'b0;
            if (strobe) begin
                d0 <= i3; d1 <= c0; d2 <= c1; d3 <= c2;
                dout   <= scaled[IN_W-1:0];
                dvalid <= 1'b1;
            end
        end
    end
endmodule
