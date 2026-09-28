module control_for_fft (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [11:0] insignal,
    output reg         sink_valid,
    input  wire        sink_ready,
    output reg  [1:0]  sink_error,
    output reg         sink_sop,
    output reg         sink_eop,
    output reg         inverse,
    output wire [11:0] outreal,
    output wire [11:0] outimag,
    output reg  [10:0] fft_pts,
    output wire [9:0]  sample_idx
);

    reg [9:0] count;
    assign sample_idx = count;
    wire [11:0] windowed_signal;

    // Hann window multiplier instance
    hann_window_1024 u_hann (
        .clk        (clk),
        .reset_n    (reset_n),
        .sample_idx (count),
        .data_in    (insignal),
        .data_out   (windowed_signal)
    );

    // Latency matching: Delay sink_sop, sink_eop, sink_valid by exactly 2 cycles
    // to align cycle-for-cycle with the 2-cycle latency of hann_window_1024
    reg raw_sop_d1;
    reg raw_eop_d1;
    reg raw_valid_d1;

    initial begin
        count         = 10'd0;
        inverse       = 1'b0;
        sink_valid    = 1'b0;
        sink_error    = 2'b00;
        fft_pts       = 11'd1024;
        raw_sop_d1    = 1'b0;
        raw_eop_d1    = 1'b0;
        raw_valid_d1  = 1'b0;
        sink_sop      = 1'b0;
        sink_eop      = 1'b0;
    end

    assign outreal = windowed_signal;
    assign outimag = 12'd0;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count        <= 10'd0;
            raw_sop_d1   <= 1'b0;
            sink_sop     <= 1'b0;
            raw_eop_d1   <= 1'b0;
            sink_eop     <= 1'b0;
            raw_valid_d1 <= 1'b0;
            sink_valid   <= 1'b0;
            sink_error   <= 2'b00;
            inverse      <= 1'b0;
            fft_pts      <= 11'd1024;
        end else begin
            count <= count + 1'b1;

            // Stage 1 (1 cycle after count change)
            raw_sop_d1   <= (count == 10'd0);
            raw_eop_d1   <= (count == 10'd1023);
            raw_valid_d1 <= 1'b1;

            // Stage 2 (2 cycles after count change, exactly matches windowed_signal)
            sink_sop     <= raw_sop_d1;
            sink_eop     <= raw_eop_d1;
            sink_valid   <= raw_valid_d1;
        end
    end

endmodule
