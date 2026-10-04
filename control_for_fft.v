// =============================================================================
// control_for_fft -- queue decimated samples and honor the FFT Avalon-ST
// ready/valid handshake. Incoming samples are buffered while the FFT is busy;
// each queued sample is windowed once and held with its packet markers until
// the FFT accepts it.
// =============================================================================
module control_for_fft (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        sample_en,
    input  wire [11:0] insignal,
    output reg         sink_valid,
    input  wire        sink_ready,
    output reg  [1:0]  sink_error,
    output reg         sink_sop,
    output reg         sink_eop,
    output reg         inverse,
    output reg  [11:0] outreal,
    output wire [11:0] outimag,
    output reg  [10:0] fft_pts,
    output wire [9:0]  sample_idx
);
    // The CIC supplies one sample every five clocks, while the FFT can accept
    // one per clock. This queue absorbs short bursts of FFT backpressure.
    localparam integer QUEUE_DEPTH = 64;
    reg signed [11:0] sample_mem [0:QUEUE_DEPTH-1];
    reg [5:0] wr_ptr, rd_ptr;
    reg [6:0] queued;
    reg [9:0] write_idx, read_idx;

    // Metadata follows the two registered stages in hann_window_1024.
    reg pipe_valid_1, pipe_valid_2;
    reg pipe_sop_1, pipe_sop_2;
    reg pipe_eop_1, pipe_eop_2;

    wire launch = (queued != 0) && !sink_valid &&
                  !pipe_valid_1 && !pipe_valid_2;
    wire enqueue = sample_en && ((queued < QUEUE_DEPTH) || launch);
    wire dequeue = launch;

    // The index follows queued samples rather than the 50 MHz clock.
    assign sample_idx = write_idx;
    assign outimag = 12'd0;

    wire signed [11:0] windowed_signal;
    hann_window_1024 u_hann (
        .clk        (clk),
        .reset_n    (reset_n),
        .sample_idx (read_idx),
        .data_in    (sample_mem[rd_ptr]),
        .data_out   (windowed_signal)
    );

    initial begin
        wr_ptr = 0;
        rd_ptr = 0;
        queued = 0;
        write_idx = 0;
        read_idx = 0;
        sink_valid = 0;
        sink_sop = 0;
        sink_eop = 0;
        sink_error = 0;
        inverse = 0;
        fft_pts = 11'd1024;
        outreal = 0;
        pipe_valid_1 = 0;
        pipe_valid_2 = 0;
        pipe_sop_1 = 0;
        pipe_sop_2 = 0;
        pipe_eop_1 = 0;
        pipe_eop_2 = 0;
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            wr_ptr       <= 0;
            rd_ptr       <= 0;
            queued       <= 0;
            write_idx    <= 0;
            read_idx     <= 0;
            sink_valid   <= 0;
            sink_sop     <= 0;
            sink_eop     <= 0;
            sink_error   <= 0;
            inverse      <= 0;
            fft_pts      <= 11'd1024;
            outreal      <= 0;
            pipe_valid_1 <= 0;
            pipe_valid_2 <= 0;
            pipe_sop_1   <= 0;
            pipe_sop_2   <= 0;
            pipe_eop_1   <= 0;
            pipe_eop_2   <= 0;
        end else begin
            sink_error <= 2'b00;
            inverse <= 1'b0;
            fft_pts <= 11'd1024;

            if (enqueue) begin
                sample_mem[wr_ptr] <= insignal;
                wr_ptr <= wr_ptr + 1'b1;
                write_idx <= write_idx + 1'b1;
            end

            if (dequeue) begin
                rd_ptr <= rd_ptr + 1'b1;
                read_idx <= read_idx + 1'b1;
            end

            case ({enqueue, dequeue})
                2'b10: queued <= queued + 1'b1;
                2'b01: queued <= queued - 1'b1;
                default: queued <= queued;
            endcase

            // launch is sampled by the Hann ROM and multiplier at this edge.
            pipe_valid_1 <= launch;
            pipe_valid_2 <= pipe_valid_1;
            pipe_sop_1 <= launch && (read_idx == 10'd0);
            pipe_sop_2 <= pipe_sop_1;
            pipe_eop_1 <= launch && (read_idx == 10'd1023);
            pipe_eop_2 <= pipe_eop_1;

            // Capture the completed window result. It remains stable while
            // sink_ready is low, along with SOP/EOP, until accepted.
            if (pipe_valid_2) begin
                outreal <= windowed_signal;
                sink_valid <= 1'b1;
                sink_sop <= pipe_sop_2;
                sink_eop <= pipe_eop_2;
            end else if (sink_valid && sink_ready) begin
                sink_valid <= 1'b0;
                sink_sop <= 1'b0;
                sink_eop <= 1'b0;
            end
        end
    end
endmodule
