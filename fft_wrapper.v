`timescale 1ns / 1ps

module fft_wrapper (
    input  wire        clk,
    input  wire        sample_en,      // 1-clk pulse: new decimated sample on in_signal
    input  wire [11:0] in_signal,
    output wire [11:0] real_power,
    output wire [11:0] imag_power,
    output wire [5:0]  source_exp,
    output wire        fft_source_sop,
    output wire        fft_source_valid,
    output wire        sink_sop,
    output wire        sink_eop,
    output wire        sink_valid,
    output wire [9:0]  sample_idx,
    input  wire        reset_n
);

    wire        sink_ready;
    wire        fft_source_eop;
    wire [11:0] real_to_fft_p;
    wire [11:0] imag_to_fft_p;
    wire [10:0] fft_pts;
    wire        inverse;
    wire [1:0]  sink_error;

    //------------------------------------------------------------------------
    //------Produce the control signals and apply Hann window-----------------
    //------------------------------------------------------------------------
    control_for_fft control_for_fft_longer_inst (
        .clk        (clk),
        .reset_n    (reset_n),
        .sample_en  (sample_en),
        .insignal   (in_signal),
        .sink_valid (sink_valid),
        .sink_ready (sink_ready),
        .sink_error (),
        .sink_sop   (sink_sop),
        .sink_eop   (sink_eop),
        .inverse    (inverse),
        .outreal    (real_to_fft_p),
        .outimag    (imag_to_fft_p),
        .fft_pts    (fft_pts),
        .sample_idx (sample_idx)
    );

    //------------------------------------------------------------------------
    //------Instantiation of the FFT Megafunction-----------------------------
    //------------------------------------------------------------------------
    fft fft_inst (
        .clk          (clk),
        .reset_n      (reset_n),
        .sink_valid   (sink_valid),
        .sink_ready   (sink_ready),
        .sink_error   (2'b00),
        .sink_sop     (sink_sop),
        .sink_eop     (sink_eop),
        .sink_real    (real_to_fft_p),
        .sink_imag    (imag_to_fft_p),
        .inverse      (inverse),
        .source_valid (fft_source_valid),
        .source_ready (1'b1),
        .source_error (),
        .source_sop   (fft_source_sop),
        .source_eop   (fft_source_eop),
        .source_real  (real_power),
        .source_imag  (imag_power),
        .source_exp   (source_exp)
    );

endmodule
