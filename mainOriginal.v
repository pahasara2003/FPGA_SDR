`timescale 1ns / 1ps

// =============================================================================
// Top-Level Module: main
// Project: Cyclone IV EP4CE6 Real-Time FFT Spectrum Analyzer
//
// Hardware:
//   - FPGA  : Cyclone IV E EP4CE6E22C8
//   - Clocks: 50.0 MHz onboard crystal (PIN 24), 60.0 MHz FT232H clock (PIN 67)
//   - USB   : FT232H High-Speed USB in 245 Synchronous FIFO mode (~30 MB/s)
//
// Pipeline Overview:
//   1. Input Source       : 1.0 MHz Square Wave Generator (50 MSPS, 25H / 25L)
//   2. Windowing          : 1024-Point Hann Window (w[n] * in_sig >>> 12)
//   3. Spectral Analysis  : 1024-Point Streaming Block Floating Point FFT Core
//   4. Magnitude Squaring : Power = Re^2 + Im^2 (24-bit) -> 12-bit scaled
//   5. Frame Buffer (CDC) : Dual-Clock FIFO (2048x16) with source_exp in Bin 0 SOP
//   6. High-Speed Streamer: FT232H 245 Synchronous FIFO controller (2 bytes/sample)
// =============================================================================

module mainOriginal (
    input  wire        clk,        // 50 MHz board clock pin (PIN 24)
    output wire        led,        // Heartbeat LED (PIN 1)

    // FT232H Synchronous 245 FIFO Interface
    input  wire        ft_clk,     // 60 MHz clock output from FT232H (PIN 67)
    input  wire        ft_txe_n,   // Active-low TX FIFO ready (PIN 68)
    output wire [7:0]  ft_data,    // FT232H 8-bit FIFO data bus (PIN 54..66)
    output wire        ft_wr_n     // Active-low write strobe to FT232H (PIN 69)
);

    // =========================================================================
    // 1. Power-on Reset Generator (holds reset active for first 250 cycles)
    // =========================================================================
    reg [7:0] rst_cnt = 8'd0;
    reg       sys_rst_n = 1'b0;

    always @(posedge clk) begin
        if (rst_cnt < 8'd250) begin
            rst_cnt   <= rst_cnt + 1'b1;
            sys_rst_n <= 1'b0;
        end else begin
            sys_rst_n <= 1'b1;
        end
    end

    // =========================================================================
    // 2. Heartbeat LED (~1.5 Hz on 50 MHz clock)
    // =========================================================================
    reg [24:0] blink_cnt = 25'd0;
    always @(posedge clk) begin
        blink_cnt <= blink_cnt + 1'b1;
    end
    assign led = blink_cnt[24];

    // =========================================================================
    // 3. 1024-Point Streaming FFT Core with integrated Hann Window
    // =========================================================================
    wire [11:0] real_power_sig;
    wire [11:0] imag_power_sig;
    wire [5:0]  source_exp_sig;
    wire        fft_source_sop_sig;
    wire        fft_source_valid_sig;
    wire        sink_sop_sig, sink_eop_sig, sink_valid_sig;
    wire [9:0]  fft_sample_idx;

    // Test Signal Generator: 1.5625 MHz Pure Symmetric Square Wave (Bin 32)
    // 50 MHz / 32 samples = 1.5625 MHz (Bin 32 of 1024).
    // Exactly 32.0000 complete periods per 1024 frame.
    // Perfectly 50.0% duty cycle: 16 samples High (+1000), 16 samples Low (-1000).
    // Derived directly from fft_sample_idx[4] -> 100% deterministic, 0 phase slip.
    wire signed [11:0] test_signal = (fft_sample_idx[4] == 1'b0) ? 12'sd1000 : -12'sd1000;

    fft_wrapper fft_wrapper_inst (
        .clk              (clk),
        .in_signal        (test_signal),
        .real_power       (real_power_sig),
        .imag_power       (imag_power_sig),
        .source_exp       (source_exp_sig),
        .fft_source_sop   (fft_source_sop_sig),
        .fft_source_valid (fft_source_valid_sig),
        .sink_sop         (sink_sop_sig),
        .sink_eop         (sink_eop_sig),
        .sink_valid       (sink_valid_sig),
        .sample_idx       (fft_sample_idx),
        .reset_n          (sys_rst_n)
    );

    // =========================================================================
    // 4. Power Spectrum Calculation & Scaling
    //    P[k] = Re^2 + Im^2 (24-bit) -> 12-bit scaled with saturation
    // =========================================================================
    wire signed [11:0] r_sig = real_power_sig;
    wire signed [11:0] i_sig = imag_power_sig;
    wire [23:0] power_comb = (r_sig * r_sig) + (i_sig * i_sig);

    // Scale 24-bit power down to 12-bit:
    // Bits [19:8] map full-scale peak (~1,044,621) smoothly into 0..4095
    wire [11:0] power_12 = (power_comb[23:20] != 4'd0) ? 12'hFFF : power_comb[19:8];

    // =========================================================================
    // 6. Spectrum Frame Buffer (Dual-Clock FIFO + Frame Gating)
    //    Bridges 50 MHz FFT write domain to 60 MHz FT232H read domain
    //    Embeds source_exp into Bin 0 SOP word for normalization in Rust/Python
    // =========================================================================
    wire [15:0] fifo_q;
    wire        fifo_rdempty;
    wire        fifo_rdreq;
    wire        frame_capturing;

    spectrum_frame_buffer u_frame_buffer (
        .wrclk        (clk),
        .rst_n        (sys_rst_n),
        .fft_sop      (fft_source_sop_sig),
        .fft_valid    (fft_source_valid_sig),
        .power_12     (power_12),
        .source_exp   (source_exp_sig),
        .capturing    (frame_capturing),
        .rdclk        (ft_clk),
        .rdreq        (fifo_rdreq),
        .fifo_q       (fifo_q),
        .fifo_rdempty (fifo_rdempty)
    );

    // =========================================================================
    // 7. FT232H Synchronous FIFO Streamer (60.0 MHz ft_clk domain)
    //    Streams 16-bit tagged samples as 2 bytes to PC via FT232H
    // =========================================================================
    ft232h_streamer u_ft232h_streamer (
        .rst_n        (sys_rst_n),
        .ft_clk       (ft_clk),
        .ft_txe_n     (ft_txe_n),
        .ft_data      (ft_data),
        .ft_wr_n      (ft_wr_n),
        .fifo_q       (fifo_q),
        .fifo_rdempty (fifo_rdempty),
        .fifo_rdreq   (fifo_rdreq)
    );

endmodule
