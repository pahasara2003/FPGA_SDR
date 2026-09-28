module dds_sine (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        clken,
    output reg  [11:0] fsin_o,
    output wire [11:0] fcos_o,
    output wire        out_valid
);
    reg [31:0] phase_acc;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            phase_acc <= 32'd0;
        else if (clken)
            phase_acc <= phase_acc + 32'd171798692; // 2.0 MHz @ 50 MHz clock
        else
            phase_acc <= phase_acc;
    end

    // 256-word 12-bit sine ROM initialized from sine_lut.hex
    (* romstyle = "M9K" *) reg [11:0] sine_rom [0:255];
    initial begin
        $readmemh("sine_lut.hex", sine_rom);
    end

    always @(posedge clk) begin
        fsin_o <= sine_rom[phase_acc[31:24]];
    end

    assign fcos_o = 12'd0;
    assign out_valid = 1'b1;

endmodule
