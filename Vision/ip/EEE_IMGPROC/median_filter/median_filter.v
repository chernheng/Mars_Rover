/*
Author: LIM Tian Yi

This module instanstiates a Bilateral Filter (despite the filename).

Latency: 7 cycles (owing in part to a 17-bit divider)

Bilateral Filter with kernel size 3x3.

3x3 Gaussian Kernel:
[1/16 1/8 1/16]
[1/8  1/4 1/8 ]
[1/16 1/8 1/16]
*/

module bilat_filt(
    input clk,
    input rst_n,

    input valid_in,
    input [7:0] in_gray,
    input [10:0] x,
    input [10:0] y,

    output reg valid_out,
    output reg [10:0] x_out,     // reference for output down the line
    output reg [10:0] y_out,     // reference for output down the line
    output reg [7:0] out_gray,   // output gray pixel

    input wen,
    input [7:0] data
);

wire [7:0] din_1, din_2, din_3;
// wire [9:0] addr_1, addr_2, addr_3;
wire [7:0] dout_1, dout_2, dout_3;
wire wen_1, wen_2, wen_3;

// wire [7:0] gray_thresh = (in_gray > thresh) ? 8'd255 : 8'd0;
wire [7:0] gray_out_thresh = (filt_out[7:0] > thresh) ? 8'd255 : 8'd0;
// blacks out areas that are below a given threshold, so as to better perform line detection.

reg [7:0] thresh;
parameter BLACK_THRESH = 10;
always @(posedge clk) begin
    if (~rst_n) thresh <= BLACK_THRESH;
    if (wen) thresh <= data;
end

// Start Convoltion Implementation

// Instanstiate line buffers
line_buf buf1(
    .address(x), .clock(clk), .data(in_gray), .wren(wen_1), .q(dout_1)
);
line_buf buf2(
    .address(x), .clock(clk), .data(in_gray), .wren(wen_2), .q(dout_2)
);
line_buf buf3(
    .address(x), .clock(clk), .data(in_gray), .wren(wen_3), .q(dout_3)
);

// Logic for writing and reading. Tracks the active read buffer.
reg [2:0] currBuf;
always @ (posedge clk) begin
    if (~rst_n) begin
        currBuf <= 3'b001;
    end
    if (valid_in && (x==639)) begin
        currBuf <= {currBuf[1:0], currBuf[2]};  // shift register
    end
end

assign wen_1 = currBuf[0] & valid_in;
assign wen_2 = currBuf[1] & valid_in;
assign wen_3 = currBuf[2] & valid_in;  // Whether it is read or write.

// required for calculations. It's basically a shift reg
/*
[ g_11 <-- g_12 <-- g_13 ] <-- data
[ g_21 <-- g_22 <-- g_23 ] <-- data
[ g_31 <-- g_32 <-- g_33 ] <-- data
*/

reg [7:0] g_11, g_12, g_13, g_21, g_22, g_23, g_31, g_32, g_33;
always @ (posedge clk) begin
    if (valid_in) begin
        g_12 <= g_13;
        g_11 <= g_12;
        g_22 <= g_23;
        g_21 <= g_22;
        g_32 <= g_33;
        g_31 <= g_32;

        g_33 <= in_gray;
        case (currBuf)
            3'b001: begin
                g_13 <= dout_2;
                g_23 <= dout_3;
            end
            3'b010: begin
                g_13 <= dout_3;
                g_23 <= dout_1;
            end
            3'b100: begin
                g_13 <= dout_1;
                g_23 <= dout_2;
            end
        endcase
    end
end

// End Convoltion Implementation. Perform calculations here.

// Coefficients for each of the brightness calcs (lookup table wtih subtraction)
wire [4:0] coeff_gauss_11, coeff_gauss_12, coeff_gauss_13, coeff_gauss_21, coeff_gauss_22, coeff_gauss_23, coeff_gauss_31, coeff_gauss_32, coeff_gauss_33;
gauss_kernel gauss_11(.center_pixel(g_22), .out_pixel(g_11), .num(coeff_gauss_11));
gauss_kernel gauss_12(.center_pixel(g_22), .out_pixel(g_12), .num(coeff_gauss_12));
gauss_kernel gauss_13(.center_pixel(g_22), .out_pixel(g_13), .num(coeff_gauss_13));
gauss_kernel gauss_21(.center_pixel(g_22), .out_pixel(g_21), .num(coeff_gauss_21));
gauss_kernel gauss_22(.center_pixel(g_22), .out_pixel(g_22), .num(coeff_gauss_22));
gauss_kernel gauss_23(.center_pixel(g_22), .out_pixel(g_23), .num(coeff_gauss_23));
gauss_kernel gauss_31(.center_pixel(g_22), .out_pixel(g_31), .num(coeff_gauss_31));
gauss_kernel gauss_32(.center_pixel(g_22), .out_pixel(g_32), .num(coeff_gauss_32));
gauss_kernel gauss_33(.center_pixel(g_22), .out_pixel(g_33), .num(coeff_gauss_33));

// Coefficients with distance values considered
wire [6:0] coeff_11, coeff_12, coeff_13, coeff_21, coeff_22, coeff_23, coeff_31, coeff_32, coeff_33;
// Multiply by the 3x3 gaussian kernel (over 16), see top.
// Ensuring stuff is not sign extended (stuff is always unsigned here)
assign coeff_11 = {2'b0, coeff_gauss_11};       // x1
assign coeff_12 = {1'b0, coeff_gauss_12, 1'b0}; // x2
assign coeff_13 = {2'b0, coeff_gauss_13};       // x1
assign coeff_21 = {1'b0, coeff_gauss_21, 1'b0}; // x2
assign coeff_22 = {coeff_gauss_22, 2'b0};       // x4
assign coeff_23 = {1'b0, coeff_gauss_23, 1'b0}; // x2
assign coeff_31 = {2'b0, coeff_gauss_31};       // x1
assign coeff_32 = {1'b0, coeff_gauss_32, 1'b0}; // x2
assign coeff_33 = {2'b0, coeff_gauss_33};       // x1

wire [10:0] Wp;
add_mult_7 Wp_sum (
    .clock(clk),
	.data0x(coeff_11),
	.data1x(coeff_12),
	.data2x(coeff_13),
	.data3x(coeff_21),
	.data4x(coeff_22),
	.data5x(coeff_23),
	.data6x(coeff_31),
	.data7x(coeff_32),
	.data8x(coeff_33),
	.result(Wp)
);

// Get output sum
wire [14:0] mult_11, mult_12, mult_13, mult_21, mult_22, mult_23, mult_31, mult_32, mult_33;
mult_coeffs mult_11_hw(.dataa(g_11), .datab(coeff_11),	.result(mult_11));
mult_coeffs mult_12_hw(.dataa(g_12), .datab(coeff_12),	.result(mult_12));
mult_coeffs mult_13_hw(.dataa(g_13), .datab(coeff_13),	.result(mult_13));
mult_coeffs mult_21_hw(.dataa(g_21), .datab(coeff_21),	.result(mult_21));
mult_coeffs mult_22_hw(.dataa(g_22), .datab(coeff_22),	.result(mult_22));
mult_coeffs mult_23_hw(.dataa(g_23), .datab(coeff_23),	.result(mult_23));
mult_coeffs mult_31_hw(.dataa(g_31), .datab(coeff_31),	.result(mult_31));
mult_coeffs mult_32_hw(.dataa(g_21), .datab(coeff_32),	.result(mult_32));
mult_coeffs mult_33_hw(.dataa(g_31), .datab(coeff_33),	.result(mult_33));

wire [18:0] Term;
add_mult Term_Sum(
    .clock(clk),
    .data0x(mult_11),
	.data1x(mult_12),
	.data2x(mult_13),
	.data3x(mult_21),
	.data4x(mult_22),
	.data5x(mult_23),
	.data6x(mult_31),
	.data7x(mult_32),
	.data8x(mult_33),
	.result(Term)
);

// Current slowest link. Consider pipelining.
wire [10:0] filt_out;
div_terms div(
    .clock(clk),
	.denom(Wp),
	.numer(Term),
	.quotient(filt_out)
);

reg valid_1, valid_2, valid_3, valid_4, valid_5, valid_6;
reg [10:0] x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4, x_5, y_5, x_6, y_6;
always @ (posedge clk) begin
    out_gray = gray_out_thresh;
    // out_gray = filt_out[7:0];    // Can be changed to the non-brightness filtered vision
    
    valid_1 <= valid_in;
    valid_2 <= valid_1;
    valid_3 <= valid_2;
    valid_4 <= valid_3;
    valid_5 <= valid_4;
    valid_6 <= valid_5;
    valid_out <= valid_6;

    x_1 <= x;
    x_2 <= x_1;
    x_3 <= x_2;
    x_4 <= x_3;
    x_5 <= x_4;
    x_6 <= x_5;
    x_out <= x_6;

    y_1 <= y;
    y_2 <= y_1;
    y_3 <= y_2;
    y_4 <= y_3;
    y_5 <= y_4;
    y_6 <= y_5;
    y_out <= y_6;
end

endmodule