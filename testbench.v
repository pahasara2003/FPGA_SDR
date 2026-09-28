`timescale 1ns/100ps
module testbench;

reg clk;
reg rst_n;
wire [11:0] fsin_o, fcos_o;
wire out_valid;

wire [22:0] real_power_sig, imag_power_sig;
wire fft_source_sop_sig;
wire sink_sop_sig, sink_eop_sig, sink_valid_sig;

wire signed [22:0] r_sig = real_power_sig;
wire signed [22:0] i_sig = imag_power_sig;

initial begin
    clk   = 0;
    rst_n = 0;
    #100;
    rst_n = 1;
end

always begin
    #10 clk = ~clk;
end

NCO nco_inst (
    .clk       (clk),
    .reset_n   (rst_n),
    .clken     (1'b1),
    .phi_inc_i (32'd171798692), // 2.0 MHz @ 50 MHz clk
    .fsin_o    (fsin_o),
    .fcos_o    (fcos_o),
    .out_valid (out_valid)
);

fft_wrapper fft_wrapper_inst (
    .clk            (clk),
    .in_signal      (fsin_o),
    .real_power     (real_power_sig),
    .imag_power     (imag_power_sig),
    .fft_source_sop (fft_source_sop_sig),
    .sink_sop       (sink_sop_sig),
    .sink_eop       (sink_eop_sig),
    .sink_valid     (sink_valid_sig),
    .reset_n        (rst_n)
);

endmodule
