/*
Author: LIM Tian Yi

Implements non-maximum suppression with an approximation of the arctan function.
Latency: 5 cycles.
*/

module non_max_suppress(
    input clk,
    input rst_n,

    input valid_in,
    input [7:0] in_gray,
    input [10:0] x,
    input [10:0] y,
    input [13:0] Gx_in,
    input [13:0] Gy_in,

    output reg valid_out,
    output reg [9:0] x_out,
    output reg [8:0] y_out,
    output reg [7:0] out_gray,   // output gray pixel

    input wen,
    input [7:0] data            // Tuning upper limit for thresholding
);

parameter THRESH = 8'd30;   // Try this for now
reg [7:0] line_thresh;

wire [35:0] datain = {Gx_in, Gy_in, in_gray}; // Also need to have the values for Gx and Gy, saved here.
wire [35:0] dout_1, dout_2, dout_3;
wire wen_1, wen_2, wen_3;

// Start of Convolution Implementation

// Instanstiate some memory
line_buf_36 buf1(.address(x), .clock(clk), .data(datain), .wren(wen_1), .q(dout_1));
line_buf_36 buf2(.address(x), .clock(clk), .data(datain), .wren(wen_2), .q(dout_2));
line_buf_36 buf3(.address(x), .clock(clk), .data(datain), .wren(wen_3), .q(dout_3));

// Logic for writing and reading. Tracks the active read buffer.
reg [2:0] currBuf;
always @ (posedge clk) begin
    if (~rst_n) begin
        currBuf <= 3'b001;
        line_thresh <= THRESH;
    end
    if (valid_in && (x==639)) begin
        currBuf <= {currBuf[1:0], currBuf[2]};  // shift register
    end

    if (wen) begin
        line_thresh <= data;
    end
end

assign wen_1 = currBuf[0] & valid_in;
assign wen_2 = currBuf[1] & valid_in;
assign wen_3 = currBuf[2] & valid_in;  // Whether it is read or write.

reg [35:0] g_11, g_12, g_13, g_21, g_22, g_23, g_31, g_32, g_33;
always @ (posedge clk) begin
    if (valid_in) begin
        g_12 <= g_13;
        g_11 <= g_12;
        g_22 <= g_23;
        g_21 <= g_22;
        g_32 <= g_33;
        g_31 <= g_32;

        g_33 <= datain;
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

// End of Convolution Implementation. Perform calculations here.

// Next, we need to calculate the gradient to enable non-maximum suppression.
// atan(Gy/Gx), normalised to 0/180 (x direction), 90/270 (y direction), 45/225 (rightward diagonal), 135/335 (leftward diagonal)

// See notes for more implementation on how this is done while skirting around atan

wire [13:0] Gx, Gy, Gx_reg;
wire [7:0] g_11_val, g_12_val, g_13_val, g_21_val, g_22_val, g_23_val, g_31_val, g_32_val, g_33_val;
assign {Gx, Gy, g_22_val} = g_22;
assign g_11_val = g_11[7:0];
assign g_12_val = g_12[7:0];
assign g_13_val = g_13[7:0];
assign g_21_val = g_21[7:0];
assign g_23_val = g_23[7:0];
assign g_31_val = g_31[7:0];
assign g_32_val = g_32[7:0];
assign g_33_val = g_33[7:0];

wire [13:0] div_res; // We know that the threshold numbers are 3.314, 22.5

// Try to use a slower divider?
wire [16:0] div_numer = {Gy, 3'b000};
div_2_17_14_s div_Gy_Gx (
    .clock(clk),
	.denom(Gx),	.numer(div_numer),
	.quotient(div_res)
);

reg [1:0] dir;  // direction with enum

// Div is undefined for Gx=0
// Need a reg for Gx because it is only valid in the next clock cycle
always @ (posedge clk) begin
    Gx_reg <= Gx;

    if (Gx_reg != 0) begin
        if (div_res > 0) begin
		if (div_res > 20) begin
                dir <= 2'b00;   // vertical
            end else if (div_res > 3) begin
                dir <= 2'b01;   // right diag
            end else begin
                dir <= 2'b11;   // Horizontal
            end
        end else begin
		if (div_res < -20) begin
                dir <= 2'b00;   // vertical
            end else if (div_res < -3) begin
                dir <= 2'b10;   // left diag
            end else begin
                dir <= 2'b11;   // Horizontal
            end
        end
    end else begin
        dir <= 2'b00;   // vertical
    end
end

// perform non-maximum supression
// as well as some basic thresholdling
// This turns the image into binary 0/255.
always @ (posedge clk) begin
    case (dir)
    2'b00: out_gray <= (g_22_val >= g_12_val && g_22_val >= g_32_val && g_22_val >= line_thresh) ? 8'hFF : 8'b0;  // vertical

    2'b01: out_gray <= (g_22_val >= g_13_val && g_22_val >= g_31_val && g_22_val >= line_thresh) ? 8'hFF : 8'b0;  // right diag

    2'b10: out_gray <= (g_22_val >= g_11_val && g_22_val >= g_33_val && g_22_val >= line_thresh) ? 8'hFF : 8'b0;  // left diag
       
    2'b11: out_gray <= (g_22_val >= g_21_val && g_22_val >= g_23_val && g_22_val >= line_thresh) ? 8'hFF : 8'b0;  // horizontal
    endcase
end

// output (modify validity)
reg valid_reg_1, valid_reg_2, valid_reg_3, valid_reg_4;
reg [9:0] x_1, x_2, x_3, x_4;
reg [8:0] y_1, y_2, y_3, y_4;
// Correct the offset inherrent in x,y due to convolution implementation
always @ (posedge clk) begin
    // Expected 3 cycle delay in result 
    valid_reg_1 <= valid_in;
    valid_reg_2 <= valid_reg_1;
    valid_reg_3 <= valid_reg_2;
    valid_reg_4 <= valid_reg_3;
    valid_out <= valid_reg_4;
    
    x_1 <= x;
    x_2 <= x_1;
    x_3 <= x_2;
    x_4 <= x_3;
    x_out <= x_4 - 10'd1;
    
    y_1 <= y;
    y_2 <= y_1;
    y_3 <= y_2;
    y_4 <= y_3;
    y_out <= y_4 - 9'd1;
    // No need to account for top and left edge of frame: we skip those calculations at that point anyway
    // (as data has not streamed in yet)
end

endmodule
