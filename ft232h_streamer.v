`timescale 1ns / 1ps
// =============================================================================
// ft232h_streamer  --  FT232H 245 Synchronous FIFO writer (FPGA -> PC)
//
// Rules from AN_130 / TN_167:
//   * A byte is transferred on every CLKOUT rising edge where TXE# and WR# are
//     BOTH low at that edge.
//   * TXE# arrives up to ~9 ns after the edge, data/WR# need >= 7.5 ns setup
//     before the next edge  ->  nothing may combinationally depend on TXE#
//     on its way to a pin.  (TN_167 example keeps WR# registered/constant and
//     uses TXE# only as a flop enable.)
//
// Scheme:
//   * data_r / wr_n_r are registers that drive the pins directly.
//   * At every edge the FPGA knows exactly whether the byte that was on the
//     bus was taken:   accepted = (WR# low) & (TXE# low)   [same two signals
//     the FT232H looks at]. Only then is the next byte loaded; otherwise the
//     same byte is simply held (WR# stays low, harmless while TXE# is high).
//   * FIFO pop depends on registered state only (not on TXE#).
//   * 1 byte per clock when TXE# stays low, order never changes, no byte is
//     ever dropped or duplicated.
// =============================================================================
module ft232h_streamer (
    input  wire        rst_n,
    input  wire        ft_clk,
    input  wire        ft_txe_n,
    output wire [7:0]  ft_data,
    output wire        ft_wr_n,
    input  wire [15:0] fifo_q,        // FWFT / show-ahead
    input  wire        fifo_rdempty,
    output wire        fifo_rdreq
);
    // reset release synchronised to ft_clk
    reg [3:0] rd_rst_sync = 4'b1111;
    always @(posedge ft_clk or negedge rst_n)
        if (!rst_n) rd_rst_sync <= 4'b1111;
        else        rd_rst_sync <= {rd_rst_sync[2:0], 1'b0};
    wire rd_ready = ~rd_rst_sync[3];

    // pad registers (pack into IOE output registers)
    reg [7:0] data_r = 8'd0;
    reg       wr_n_r = 1'b1;
    assign ft_data = data_r;
    assign ft_wr_n = wr_n_r;

    // word staging
    reg [15:0] pre     = 16'd0;   // prefetched FIFO word
    reg        pre_v   = 1'b0;
    reg [7:0]  hi_byte = 8'd0;    // high byte waiting behind the low byte
    reg        hi_v    = 1'b0;

    assign fifo_rdreq = rd_ready & ~fifo_rdempty & ~pre_v;      // no TXE# here

    wire accepted  = ~wr_n_r & ~ft_txe_n;   // byte currently on the bus is taken
    wire slot_free = wr_n_r | accepted;     // bus may carry a new byte next edge

    always @(posedge ft_clk or negedge rst_n) begin
        if (!rst_n) begin
            data_r  <= 8'd0;  wr_n_r <= 1'b1;
            pre     <= 16'd0; pre_v  <= 1'b0;
            hi_byte <= 8'd0;  hi_v   <= 1'b0;
        end else if (!rd_ready) begin
            data_r  <= 8'd0;  wr_n_r <= 1'b1;
            pre     <= 16'd0; pre_v  <= 1'b0;
            hi_byte <= 8'd0;  hi_v   <= 1'b0;
        end else begin
            if (fifo_rdreq) begin
                pre   <= fifo_q;
                pre_v <= 1'b1;
            end
            if (slot_free) begin
                if (hi_v) begin
                    data_r <= hi_byte;  wr_n_r <= 1'b0;  hi_v <= 1'b0;
                end else if (pre_v) begin
                    data_r  <= pre[7:0];  hi_byte <= pre[15:8];
                    wr_n_r  <= 1'b0;      hi_v    <= 1'b1;  pre_v <= 1'b0;
                end else begin
                    wr_n_r <= 1'b1;
                end
            end
        end
    end
endmodule
