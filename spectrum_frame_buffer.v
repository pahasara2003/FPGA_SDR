`timescale 1ns / 1ps

// =============================================================================
// Module: spectrum_frame_buffer
// Project: Cyclone IV FFT Spectrum Analyzer
//
// Description:
//   Captures 1024-point FFT frames in the 50 MHz write clock domain and
//   transfers them via a dual-clock FIFO to the 60 MHz FT232H read domain.
//
// Features:
//   - Frame gating protection: checks FIFO write space before starting a frame
//     (fifo_wrusedw < 500 in 2048-word FIFO) to guarantee that whole 1024-word
//     frames are never truncated or fragmented.
//   - Word Tagging:
//       Bin 0 (SOP): 16'hB000 | {source_exp[3:0], power_12[7:0]} (or 16'hB000 | power_12)
//       Bins 1..1023: 16'hA000 | power_12
//   - Bubble immunity: tracks exact word count (0..1023) so valid bubbles
//     between FFT output bursts do not corrupt the frame structure.
//   - Dual-Clock FIFO: 2048 words x 16-bit in FWFT (show-ahead) mode.
// =============================================================================

module spectrum_frame_buffer (
    // Write Domain (50 MHz clk)
    input  wire        wrclk,           // 50 MHz write clock
    input  wire        rst_n,           // Asynchronous active-low reset
    input  wire        fft_sop,         // FFT start-of-packet
    input  wire        fft_valid,       // FFT output valid
    input  wire [11:0] power_12,        // 12-bit power value
    input  wire [5:0]  source_exp,      // FFT Block Floating Point exponent
    output wire        capturing,       // Status: actively capturing a frame

    // Read Domain (60 MHz ft_clk)
    input  wire        rdclk,           // 60 MHz FT232H clock
    input  wire        rdreq,           // Read request from FT232H streamer
    output wire [15:0] fifo_q,          // 16-bit word output (show-ahead)
    output wire        fifo_rdempty     // FIFO empty flag
);

    // -------------------------------------------------------------------------
    // 1. Write Domain Startup Sequencer (holds off writes until CDC stabilizes)
    // -------------------------------------------------------------------------
    reg [7:0] wr_rst_cnt = 8'd0;
    reg       wr_ready   = 1'b0;

    always @(posedge wrclk or negedge rst_n) begin
        if (!rst_n) begin
            wr_rst_cnt <= 8'd0;
            wr_ready   <= 1'b0;
        end else begin
            if (wr_rst_cnt < 8'd255) begin
                wr_rst_cnt <= wr_rst_cnt + 1'b1;
                wr_ready   <= 1'b0;
            end else begin
                wr_ready   <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. Gated Frame Capture Logic (1024 words per frame)
    // -------------------------------------------------------------------------
    wire [10:0] fifo_wrusedw;
    wire        fifo_wrfull;
    reg         capturing_frame = 1'b0;
    reg  [9:0]  frame_word_cnt  = 10'd0;
    reg  [15:0] fifo_in_reg     = 16'd0;
    reg         fifo_wr_en      = 1'b0;

    assign capturing = capturing_frame;

    always @(posedge wrclk or negedge rst_n) begin
        if (!rst_n) begin
            capturing_frame <= 1'b0;
            frame_word_cnt  <= 10'd0;
            fifo_in_reg     <= 16'd0;
            fifo_wr_en      <= 1'b0;
        end else if (!wr_ready) begin
            capturing_frame <= 1'b0;
            frame_word_cnt  <= 10'd0;
            fifo_in_reg     <= 16'd0;
            fifo_wr_en      <= 1'b0;
        end else begin
            fifo_wr_en <= 1'b0;
            if (fft_valid) begin
                if (capturing_frame) begin
                    // Stream Bins 1..1023 (Tag 0xA)
                    fifo_in_reg <= {4'hA, power_12};
                    fifo_wr_en  <= 1'b1;
                    if (frame_word_cnt == 10'd1022) begin
                        capturing_frame <= 1'b0;
                    end
                    frame_word_cnt <= frame_word_cnt + 1'b1;
                end else if (fft_sop) begin
                    // SOP: Start of frame (Bin 0, Tag 0xB)
                    // Check if FIFO has ample room for entire 1024-word frame (depth = 2048)
                    if (fifo_wrusedw < 11'd800 && !fifo_wrfull) begin
                        capturing_frame <= 1'b1;
                        frame_word_cnt  <= 10'd0;
                        fifo_in_reg     <= {4'hB, 6'd0, source_exp};
                        fifo_wr_en      <= 1'b1;
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. Dual-Clock FIFO Instance (2048 words x 16-bit, FWFT show-ahead mode)
    // -------------------------------------------------------------------------
    wire [10:0] fifo_rdusedw;

    fifo u_fifo (
        .aclr    (~rst_n),
        .data    (fifo_in_reg),
        .wrclk   (wrclk),
        .wrreq   (fifo_wr_en),
        .rdclk   (rdclk),
        .rdreq   (rdreq),
        .q       (fifo_q),
        .rdempty (fifo_rdempty),
        .rdusedw (fifo_rdusedw),
        .wrfull  (fifo_wrfull),
        .wrusedw (fifo_wrusedw)
    );

endmodule
