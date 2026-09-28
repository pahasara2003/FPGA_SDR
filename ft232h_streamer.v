`timescale 1ns / 1ps

// =============================================================================
// Module: ft232h_streamer
// Project: Cyclone IV FFT Spectrum Analyzer
//
// Description:
//   Transmits 16-bit words from a FIFO to an external FT232H device operating
//   in FT245 Synchronous FIFO mode at 60.0 MHz.
//
// Protocol:
//   - Each 16-bit sample is streamed as 2 consecutive bytes (little-endian):
//       Phase 0: D[7:0]  (low byte)
//       Phase 1: D[15:8] (high byte: tag [15:12] + power [11:8])
//   - Uses FT232H flow-control signal ft_txe_n:
//       ft_txe_n == 0: FT232H is ready to accept a byte.
//       ft_txe_n == 1: FT232H FIFO is full, transmission is paused.
//   - Directly advances the dual-clock FWFT (show-ahead) FIFO via fifo_rdreq.
// =============================================================================

module ft232h_streamer (
    input  wire        rst_n,        // Asynchronous active-low reset
    
    // FT232H Hardware Interface (60 MHz domain)
    input  wire        ft_clk,       // 60 MHz clock output from FT232H (PIN 67)
    input  wire        ft_txe_n,     // Active-low TX FIFO ready (PIN 68)
    output wire [7:0]  ft_data,      // FT232H 8-bit FIFO data bus (PIN 54..66)
    output wire        ft_wr_n,      // Active-low write strobe to FT232H (PIN 69)

    // FIFO Read Interface (in ft_clk domain)
    input  wire [15:0] fifo_q,       // 16-bit data word from FIFO (show-ahead)
    input  wire        fifo_rdempty, // FIFO empty flag
    output wire        fifo_rdreq    // FIFO read request (single-cycle pop strobe)
);

    // -------------------------------------------------------------------------
    // 1. Reset Synchronizer (4-stage shift register in ft_clk domain)
    // -------------------------------------------------------------------------
    reg [3:0] rd_rst_sync = 4'b1111;
    always @(posedge ft_clk or negedge rst_n) begin
        if (!rst_n)
            rd_rst_sync <= 4'b1111;
        else
            rd_rst_sync <= {rd_rst_sync[2:0], 1'b0};
    end
    wire rd_ready = ~rd_rst_sync[3];

    // -------------------------------------------------------------------------
    // 2. Continuous 2-Byte Synchronous Streamer (Slip-Free)
    //    Atomic pairing: Low Byte then High Byte, zero possibility of byte slip.
    // -------------------------------------------------------------------------
    reg        byte_phase  = 1'b0;
    reg [15:0] curr_sample = 16'd0;

    // A write is executed on this rising clock edge if:
    // 1. Domain reset is clear (rd_ready)
    // 2. FT232H has buffer space (!ft_txe_n)
    // 3. FIFO has data if starting a new word (!byte_phase ? !fifo_rdempty : 1'b1)
    wire can_write = rd_ready && !ft_txe_n && (!byte_phase ? !fifo_rdempty : 1'b1);

    assign ft_wr_n = ~can_write;
    assign ft_data = byte_phase ? curr_sample[15:8] : fifo_q[7:0];
    assign fifo_rdreq = can_write && byte_phase; // Advance FWFT FIFO on High Byte write

    always @(posedge ft_clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_phase  <= 1'b0;
            curr_sample <= 16'd0;
        end else if (!rd_ready) begin
            byte_phase  <= 1'b0;
            curr_sample <= 16'd0;
        end else if (can_write) begin
            if (!byte_phase) begin
                // Low Byte written on this edge; save sample for High Byte phase
                curr_sample <= fifo_q;
                byte_phase  <= 1'b1;
            end else begin
                // High Byte written on this edge; 16-bit word complete!
                byte_phase  <= 1'b0;
            end
        end
    end

endmodule
