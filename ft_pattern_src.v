`timescale 1ns / 1ps
// Link self-test source: FWFT-style "FIFO" that always has a word and returns
// 0,1,2,3,... (16 bit, wraps). Temporarily wire it in place of the real FIFO
// on the ft232h_streamer (see ft_check.py). Never empty -> max throughput.
module ft_pattern_src (
    input  wire        ft_clk,
    input  wire        rst_n,
    input  wire        rdreq,
    output wire [15:0] q,
    output wire        rdempty
);
    reg [15:0] cnt = 16'd0;
    always @(posedge ft_clk or negedge rst_n)
        if (!rst_n)      cnt <= 16'd0;
        else if (rdreq)  cnt <= cnt + 16'd1;
    assign q       = cnt;
    assign rdempty = 1'b0;
endmodule
