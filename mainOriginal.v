`timescale 1ns / 1ps
// =============================================================================
// Top-Level Module: main
// Project: Cyclone IV EP4CE6 Real-Time FFT Spectrum Analyzer
//
// Hardware:
//   - FPGA : Cyclone IV E EP4CE6E22C8
//   - Clocks: 50.0 MHz onboard crystal (PIN 24), 60.0 MHz FT232H clock (PIN 67)
//   - USB  : FT232H High-Speed USB in 245 Synchronous FIFO mode
//
// Pipeline Overview (effective sample rate = 10 MSPS):
//   1. Input source      : 50 MSPS test square wave (160-clock period = 312.5 kHz)
//                          -> replace `adc_sample` with the real ADC later
//   2. Decimator         : 4-stage CIC, R = 5  ->  10 MSPS (1-clock enable pulses)
//   3. Windowing         : 1024-point Hann window (runs on the enable pulses)
//   4. Spectral analysis : 1024-point streaming BFP FFT core
//   5. Magnitude squaring: Power = Re^2 + Im^2 (24-bit) -> 12-bit scaled
//   6. Frame buffer (CDC): dual-clock FIFO (2048x16), source_exp in Bin 0
//   7. FT232H streamer   : 245 synchronous FIFO, 2 bytes per bin
//
// Bin width = 10 MHz / 1024 = 9.765625 kHz.  Test tone lands exactly on bin 32.
// =============================================================================
module mainOriginal (
    input  wire       clk,       // 50 MHz board clock pin (PIN 24)
    output wire       led,       // Heartbeat LED (PIN 1)

    // FT232H Synchronous 245 FIFO Interface
    input  wire       ft_clk,    // 60 MHz clock output from FT232H (PIN 67)
    input  wire       ft_txe_n,  // Active-low TX FIFO ready (PIN 68)
    output wire [7:0] ft_data,   // FT232H 8-bit FIFO data bus (PIN 54..66)
    output wire       ft_wr_n    // Active-low write strobe to FT232H (PIN 69)
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
    // 3. 50 MSPS source  (test tone now; real ADC later)
    //    Square wave, 160 clocks per period = 312.5 kHz, +-1000.
    //    After /5 decimation that is 32 samples per period -> exactly bin 32,
    //    and 1024 samples = 5120 clocks = 32 whole periods, so every frame is
    //    coherent no matter where the frame boundary falls.
    // =========================================================================
    reg [7:0] tone_cnt = 8'd0;
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n)            tone_cnt <= 8'd0;
        else if (tone_cnt == 8'd159) tone_cnt <= 8'd0;
        else                       tone_cnt <= tone_cnt + 1'b1;
    end

    wire signed [11:0] adc_sample = (tone_cnt < 8'd80) ? 12'sd1000 : -12'sd1000;

    // =========================================================================
    // 4. CIC decimator  50 MSPS -> 10 MSPS  (R = 5, fixed)
    // =========================================================================
    wire signed [11:0] dec_sample;
    wire               dec_valid;     // 1-clock pulse every 5th clock

    cic_dec5 u_cic (
        .clk    (clk),
        .rst_n  (sys_rst_n),
        .din    (adc_sample),
        .dout   (dec_sample),
        .dvalid (dec_valid)
    );

    // =========================================================================
    // 5. 1024-Point Streaming FFT Core with integrated Hann Window
    // =========================================================================
    wire [11:0] real_power_sig;
    wire [11:0] imag_power_sig;
    wire [5:0]  source_exp_sig;
    wire        fft_source_sop_sig;
    wire        fft_source_valid_sig;
    wire        sink_sop_sig, sink_eop_sig, sink_valid_sig;
    wire [9:0]  fft_sample_idx;

    fft_wrapper fft_wrapper_inst (
        .clk              (clk),
        .sample_en        (dec_valid),
        .in_signal        (dec_sample),
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
    // 6. Power Spectrum Calculation & Scaling
    //    P[k] = Re^2 + Im^2 (24-bit) -> 12-bit scaled with saturation
    // =========================================================================
    wire signed [11:0] r_sig = real_power_sig;
    wire signed [11:0] i_sig = imag_power_sig;
    wire [23:0] power_comb = (r_sig * r_sig) + (i_sig * i_sig);

    // Bits [19:8] map full-scale peak into 0..4095, saturate above
    wire [11:0] power_12 = (power_comb[23:20] != 4'd0) ? 12'hFFF : power_comb[19:8];

    // =========================================================================
    // 7. Spectrum Frame Buffer (Dual-Clock FIFO + Frame Gating)
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
    // 8. FT232H Synchronous FIFO Streamer (60.0 MHz ft_clk domain)
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
