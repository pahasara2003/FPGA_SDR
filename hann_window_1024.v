`timescale 1ns / 1ps

// =============================================================================
// Module: hann_window_1024
//
// Description:
//   Applies a 1024-point Hann window to a 12-bit signed input signal.
//   - Hann coefficients w[n] = 0.5 * (1 - cos(2*pi*n/1024)) mapped to 0..4095 (Q12).
//   - Coefficients stored in 1 M9K block RAM (1024 x 12-bit ROM).
//   - Multiplication: (signal * win_coeff) >>> 12 (1 DSP 18-bit multiplier).
//   - Output latency: 2 clock cycles (pipelined for 50 MHz).
// =============================================================================

module hann_window_1024 (
    input  wire               clk,
    input  wire               reset_n,
    input  wire        [9:0]  sample_idx,      // 0..1023
    input  wire signed [11:0] data_in,         // 12-bit signed input sample
    output reg  signed [11:0] data_out         // 12-bit signed windowed sample
);

    // 1. Hann Window ROM (1024 words x 12-bit, initialized from MIF)
    (* ram_init_file = "hann_window_1024.mif" *)
    reg [11:0] win_rom [0:1023];

    reg [11:0] win_coeff_reg;
    reg signed [11:0] data_in_reg;

    // Stage 1: ROM Read & Data delay (1 cycle)
    always @(posedge clk) begin
        win_coeff_reg <= win_rom[sample_idx];
        data_in_reg   <= data_in;
    end

    // Stage 2: Signed Multiplier (12-bit signed * 12-bit unsigned -> 25-bit signed)
    wire signed [12:0] win_coeff_signed = {1'b0, win_coeff_reg};
    wire signed [24:0] mult_result = data_in_reg * win_coeff_signed;

    // Stage 3: Normalize (shift right by 12 bits)
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            data_out <= 12'sd0;
        end else begin
            data_out <= mult_result[23:12];
        end
    end

endmodule
