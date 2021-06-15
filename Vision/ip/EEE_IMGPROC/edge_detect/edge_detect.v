/*
Author: LIM Tian Yi

This module instanstiates a Sobel Operator.
As operations can be done in parallel it is not necessary to use the "more efficient" way of convolution in 1d->1d.

Latency: 6 Cycles

Sobel Operator:
Gx:             Gy:
[+1 +0 -1]      [+1 +2 +1]
[+2 +0 -2]      [+0 +0 +0]
[+1 +0 -1]      [-1 -2 -1]

Output G:
Sqrt(Gx^2 + Gy^2)
*/
module edge_detect(
    input clk,
    input rst_n,

    input valid_in,
    input [7:0] in_gray,
    input [10:0] x,
    input [10:0] y,

    output reg valid_out,
    output reg [10:0] x_out,
    output reg [10:0] y_out,
    output reg [7:0] out_gray,   // output gray pixel
    output reg [13:0] Gx_out,
    output reg [13:0] Gy_out
);

wire [7:0] dout_1, dout_2, dout_3;
wire wen_1, wen_2, wen_3;

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

// Max value Gx, Gy: 4*255 => 10 bits reqd + 1 for sgn -> 11 bits wide
// Gx^2 needs only 21 bits as squares are always positive.
// however as we are only taking a magnitude, we take the top 16 bits only
// as the sqrt term will result in an 8 bit output.

// We also don't need to worry about the bottom line being processed as EOP will have happened by then.
// First line that is output will be correct, with uninitialised data; but it is irrelevant for further processing

// Converting unsigned to signed to calculate the gradient
wire[8:0] g_13_neg = ~{1'b0, g_13} + 9'd1;
wire[8:0] g_23_neg = ~{1'b0, g_23} + 9'd1;
wire[8:0] g_31_neg = ~{1'b0, g_31} + 9'd1;
wire[8:0] g_32_neg = ~{1'b0, g_32} + 9'd1;
wire[8:0] g_33_neg = ~{1'b0, g_33} + 9'd1;

wire [10:0] gx_11 = {3'b000, g_11};
wire [10:0] gx_21 = {2'b00, g_21, 1'b0};     // +2*g_21
wire [10:0] gx_31 = {3'b000, g_31};
wire [10:0] gx_13 = g_13==0 ? 11'b0 : {2'b11, g_13_neg};
wire [10:0] gx_23 = g_23==0 ? 11'b0 : {1'b1, g_23_neg, 1'b0}; // -2*g_23
wire [10:0] gx_33 = g_33==0 ? 11'b0 : {2'b11, g_33_neg};

wire [10:0] gy_11 = {3'b000, g_11};
wire [10:0] gy_12 = {2'b00, g_12, 1'b0};     // +2*g_12
wire [10:0] gy_13 = {3'b000, g_13};
wire [10:0] gy_31 = g_31==0 ? 11'b0 : {2'b11, g_31_neg};
wire [10:0] gy_32 = g_32==0 ? 11'b0 : {1'b1, g_32_neg, 1'b0}; // -2*g_32
wire [10:0] gy_33 = g_33==0 ? 11'b0 : {2'b11, g_33_neg};

wire [13:0] Gx, Gy, Gx_neg, Gy_neg;
assign Gx_neg = -Gx;
assign Gy_neg = -Gy;

/* 
We opt not to use the Hypot function instead of calculating this large sum.
r = sqrt(x^2 + y^2)
= sqrt(x^2(1+(y/x)^2))
= |x| sqrt(1+(y/x)^2) <-- this will be screwy with integer logic.

Therefore we use the "naive" implementation.
*/

G_get Gx_get(
	.clock(clk),
	.data0x(gx_11),
	.data1x(gx_21),
	.data2x(gx_31),
	.data3x(gx_13),
	.data4x(gx_23),
	.data5x(gx_33),
	.result(Gx)
);

G_get Gy_get(
	.clock(clk),
	.data0x(gy_11),
	.data1x(gy_12),
	.data2x(gy_13),
	.data3x(gy_31),
	.data4x(gy_32),
	.data5x(gy_33),
	.result(Gy)
);

// These give unsigned values. Therefore we remove the MSB (sign bit).
wire [10:0] Gx_square = Gx[13] ? Gx_neg[12:2] : Gx[12:2];   // Fit the correct bit width. This is divided by 4 (as the sign bit is necessarily 0)
wire [10:0] Gy_square = Gy[13] ? Gy_neg[12:2] : Gy[12:2];

wire [21:0] Gx_2_tmp, Gy_2_tmp;

mult_1_11_u Gx_mult(.clock(clk), .dataa(Gx_square), .result(Gx_2_tmp));
mult_1_11_u Gy_mult(.clock(clk), .dataa(Gy_square), .result(Gy_2_tmp));

// Strategy: Saturating logic.
// We know that Gx_2_tmp and Gy_2_tmp are unsigned. Therefore if the MSB of the sum has overflowed
// We can consider the sum to have overflowed and this constitutes an 'edge'.
// Testing shows that this is a negligible loss.
wire [16:0] G_xy_add = Gx_2_tmp[15:0] + Gy_2_tmp[15:0];
wire [15:0] G_tmp = G_xy_add[16] ? 16'hFFFF : G_xy_add[15:0];

wire [7:0] G_sqrt;
sqrt_1 G_sqrt_mod(
    .clk(clk),
	.radical(G_tmp), .q(G_sqrt)
);

reg valid_reg_1, valid_reg_2, valid_reg_3, valid_reg_4;
reg [10:0] x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4;
always @ (posedge clk) begin
    out_gray <= G_sqrt;
    Gx_out <= Gx;
    Gy_out <= Gy;

    // Expected 5 cycle delay in result 
    valid_reg_1 <= valid_in;
    valid_reg_2 <= valid_reg_1;
    valid_reg_3 <= valid_reg_2;
    valid_reg_4 <= valid_reg_3;
    valid_out <= valid_reg_4;
    
    x_1 <= x;
    x_2 <= x_1;
    x_3 <= x_2;
    x_4 <= x_3;
    x_out <= x_4;

    y_1 <= y;
    y_2 <= y_1;
    y_3 <= y_2;
    y_4 <= y_3;
    y_out <= y_4;
    // No need to account for top and left edge of frame: we skip those calculations at that point anyway
    // (as data has not streamed in yet)
end

endmodule