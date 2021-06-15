/*
Author: LIM Tian Yi with sample code and setup from Dr Ed Stott

Image processing IP Core connected in Qsys to the input from the camera
and containing the Colour and Line detection Pipeline.
*/

module EEE_IMGPROC(
	// global clock & reset
	input clk,
	input reset_n,
	
	// mm slave
	input s_chipselect,
	input s_read,
	input s_write,
	output reg [31:0] s_readdata,
	input [31:0] s_writedata,
	input [2:0] s_address,

	// stream sink
	input [23:0] sink_data,
	input sink_valid,
	output sink_ready,
	input sink_sop,
	input sink_eop,
	
	// streaming source
	output [23:0] source_data,
	output source_valid,
	input source_ready,
	output source_sop,
	output source_eop,
	
	// conduit
	input mode
);

parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 1;

// Detection parameters
// Modified at runtime, not exact! 
parameter SAT_THRESH_RED = 140;         // about 55%
parameter VAL_THRESH_RED = 100;         // about 40%
parameter SAT_THRESH_GREEN = 089;         // about 35%
parameter VAL_THRESH_GREEN = 064;         // about 25%
parameter SAT_THRESH_BLUE = 064;         // about 25%
parameter VAL_THRESH_BLUE = 025;         // about 10%
parameter SAT_THRESH_YELLOW = 140;         // about 55%
parameter VAL_THRESH_YELLOW = 100;         // about 40%
parameter SAT_THRESH_PURPLE = 115;         // about 45%
parameter VAL_THRESH_PURPLE = 064;         // about 25%_RED
parameter RED_THRESH_H = 235;       // 330 on the (0-360) hue range
parameter RED_THRESH_L = 28;        // 40   - center = 0
parameter GREEN_THRESH_H = 135;      // 190
parameter GREEN_THRESH_L = 71;      // 100  - center = 120
parameter BLUE_THRESH_H = 192;      // 270
parameter BLUE_THRESH_L = 138;      // 195  - center = 240
parameter YELLOW_THRESH_H = 64;     // 90
parameter YELLOW_THRESH_L = 28;     // 40   - center = 60
parameter PURPLE_THRESH_H = 235;    // 330
parameter PURPLE_THRESH_L = 191;    // 270  - center = 300

// Bounding box colours (Complementary)
parameter RED_BB_COL = 24'h00ffff;      // TEAL
parameter GREEN_BB_COL = 24'hff00ff;    // PURPLE
parameter BLUE_BB_COL = 24'hffff00;     // yellow
parameter YELLOW_BB_COL = 24'h0000ff;   // blue
parameter PURPLE_BB_COL = 24'h00ff00;   // green

// Colours
wire [7:0] red, green, blue, grey;
wire [7:0] red_out, green_out, blue_out;
wire sop, eop;
// HSV
wire [7:0] hue, saturation, value;

////////////////////////////////////////////////////////////////////////
wire valid_hsv_input, valid_hsv_output, rdy_in_out;

//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready), // connects outward
	.valid_in(sink_valid),  // connects to upstream
	.data_in({sink_data,sink_sop,sink_eop}),
	.ready_in(rdy_in_out),
	.valid_out(valid_hsv_input),
	.data_out({red, green, blue, sop, eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(rdy_in_out),
	.valid_in(valid_hsv_input),   // Fix: bypass HSV shenanigans
	.data_in({red_out, green_out, blue_out, sop, eop}),
	.ready_in(source_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop})
);

// instantiate RGB to HSV
rgb_to_hsv_pipe rgb2hsv(
    .clk(clk),
    .valid_in(valid_hsv_input),
    .red(red), .green(green), .blue(blue),
    .valid_out(valid_hsv_output),
    .hue(hue), .sat(saturation), .val(value)
);  // hardwire ready to 1!

wire sop_45, eop_45;
reg [1:0] p_01, p_12, p_23, p_34, p_45;
reg [10:0] y_01, y_12, y_23, y_34, y_45;
reg [10:0] x_01, x_12, x_23, x_34, x_45;

// Syncs EOP signals to account for delay from RGB2HSV Pipelining
always @(posedge clk) begin
    p_01 <= {sop, eop};
    p_12 <= p_01;
    p_23 <= p_12;
    p_34 <= p_23;
    p_45 <= p_34;

    y_01 <= y;
    y_12 <= y_01;
    y_23 <= y_12;
    y_34 <= y_23;
    y_45 <= y_34;

    x_01 <= x;
    x_12 <= x_01;
    x_23 <= x_12;
    x_34 <= x_23;
    x_45 <= x_34;
end
assign {sop_45, eop_45} = p_45;

// Instantiate one Colour Detection IP per colour of interest.
// "red" does not wrap around the colour wheel, but "pink" does
smooth_detect #(.SAT_THRESH(SAT_THRESH_RED), .VAL_THRESH(VAL_THRESH_RED),
                .HUE_THRESH_H(RED_THRESH_H), .HUE_THRESH_L(RED_THRESH_L),
                .IS_RED(1'b0) ) red_detector
(
    .clk(clk), .rst_n(reset_n),
    .x(x_45), .y(y_45),
    .hue(hue), .sat(saturation), .val(value), .sop(sop_45), .eop(eop_45),
    .valid(valid_hsv_output),
    .wen(red_detector_wen), .sel(red_detector_sel), .data(red_detector_data),
    .detected(red_detect),
    .max_x(right_red), .max_y(bottom_red), .min_x(left_red), .min_y(top_red)
);

smooth_detect #(.SAT_THRESH(SAT_THRESH_GREEN), .VAL_THRESH(VAL_THRESH_GREEN),
                .HUE_THRESH_H(GREEN_THRESH_H), .HUE_THRESH_L(GREEN_THRESH_L),
                .IS_RED(1'b0) ) green_detector
(
    .clk(clk), .rst_n(reset_n),
    .x(x_45), .y(y_45),
    .hue(hue), .sat(saturation), .val(value), .sop(sop_45), .eop(eop_45),
    .valid(valid_hsv_output),
    .wen(green_detector_wen), .sel(green_detector_sel), .data(green_detector_data),
    .detected(green_detect),
    .max_x(right_green), .max_y(bottom_green), .min_x(left_green), .min_y(top_green)
);

smooth_detect #(.SAT_THRESH(SAT_THRESH_BLUE), .VAL_THRESH(VAL_THRESH_BLUE),
                .HUE_THRESH_H(BLUE_THRESH_H), .HUE_THRESH_L(BLUE_THRESH_L),
                .IS_RED(1'b0) ) blue_detector
(
    .clk(clk), .rst_n(reset_n),
    .x(x_45), .y(y_45),
    .hue(hue), .sat(saturation), .val(value), .sop(sop_45), .eop(eop_45),
    .valid(valid_hsv_output),
    .wen(blue_detector_wen), .sel(blue_detector_sel), .data(blue_detector_data),
    .detected(blue_detect),
    .max_x(right_blue), .max_y(bottom_blue), .min_x(left_blue), .min_y(top_blue)
);

smooth_detect #(.SAT_THRESH(SAT_THRESH_YELLOW), .VAL_THRESH(VAL_THRESH_YELLOW),
                .HUE_THRESH_H(YELLOW_THRESH_H), .HUE_THRESH_L(YELLOW_THRESH_L),
                .IS_RED(1'b0) ) yellow_detector
(
    .clk(clk), .rst_n(reset_n),
    .x(x_45), .y(y_45),
    .hue(hue), .sat(saturation), .val(value), .sop(sop_45), .eop(eop_45),
    .valid(valid_hsv_output),
    .wen(yellow_detector_wen), .sel(yellow_detector_sel), .data(yellow_detector_data),
    .detected(yellow_detect),
    .max_x(right_yellow), .max_y(bottom_yellow), .min_x(left_yellow), .min_y(top_yellow)
);

smooth_detect #(.SAT_THRESH(SAT_THRESH_PURPLE), .VAL_THRESH(VAL_THRESH_PURPLE),
                .HUE_THRESH_H(PURPLE_THRESH_H), .HUE_THRESH_L(PURPLE_THRESH_L),
                .IS_RED(1'b1) ) purple_detector
(
    .clk(clk), .rst_n(reset_n),
    .x(x_45), .y(y_45),
    .hue(hue), .sat(saturation), .val(value), .sop(sop_45), .eop(eop_45),
    .valid(valid_hsv_output),
    .wen(purple_detector_wen), .sel(purple_detector_sel), .data(purple_detector_data),
    .detected(purple_detect),
    .max_x(right_purple), .max_y(bottom_purple), .min_x(left_purple), .min_y(top_purple)
);

wire [7:0] edge_out, bilat_out, nonmax_out;
wire [7:0] hough_r;
wire [5:0] hough_theta;   // output of edge detector
wire valid_edge_detect, valid_bilat, valid_nonmax, valid_hough;
// Bilat feeds into edge. So we need to delay x similarly
wire [10:0] x_bliat_edge, x_edge, x_nonmax;
wire [10:0] y_bliat_edge, y_edge, y_nonmax;
wire [13:0] Gx, Gy;

bilat_filt bilat (
    .clk(clk), .rst_n(reset_n),
    .valid_in(valid_hsv_input), .in_gray(grey), .x(x), .y(y),
    .valid_out(valid_bilat), .x_out(x_bliat_edge), .y_out(y_bliat_edge),
    .out_gray(bilat_out),   // output gray pixel
    .wen(bilat_wen), .data(bilat_data)
);

edge_detect edgy (
    .clk(clk), .rst_n(reset_n),
    .valid_in(valid_bilat), .in_gray(bilat_out), .x(x_bliat_edge), .y(y_bliat_edge),
    .valid_out(valid_edge_detect), .x_out(x_edge), .y_out(y_edge),
    .out_gray(edge_out),   // output gray pixel
    .Gx_out(Gx), .Gy_out(Gy)
);

non_max_suppress nonmax (
    .clk(clk), .rst_n(reset_n),
    .valid_in(valid_edge_detect), .in_gray(edge_out), .x(x_edge), .y(y_edge),
    .Gx_in(Gx), .Gy_in(Gy), .valid_out(valid_nonmax), .x_out(x_nonmax), .y_out(y_nonmax),
    .out_gray(nonmax_out),   // output gray pixel
    .wen(nonmax_wen), .data(nonmax_data)
);

wire nonmax_to_hough_data = nonmax_out[0];  // It is binary between 0, 255, just look at the LSB.
wire [7:0] hough_debug;
hough_transform hough (
    .clk(clk), .rst_n(reset_n),
    .x(x_nonmax), .y(y_nonmax),
    .canny_in(nonmax_to_hough_data), .valid_in(valid_nonmax),
    .valid_out(valid_hough),
    .r(hough_r), .theta(hough_theta),
    .wen(hough_wen), .data_in(hough_data),
    .int_out(hough_debug)
);

// detect areas of interest: red, green, blue, yellow, purple
wire red_detect, yellow_detect, green_detect, blue_detect, purple_detect;

// Highlight detected areas
wire [23:0] highlight;
assign grey = green[7:1] + red[7:2] + blue[7:2]; // Grey = green/2 + red/4 + blue/4
assign highlight =  red_detect ? {8'hff, 8'h0, 8'h0} :
                    yellow_detect ? {8'hff, 8'hff, 8'h0} :
                    green_detect ? {8'h0, 8'hff, 8'h0} :
                    blue_detect ? {8'h0, 8'h0, 8'hff} :
                    purple_detect ? {8'hff, 8'h0, 8'hff} : {grey, grey, grey};

// Show bounding box
wire [23:0] new_image;
wire bb_active_red, bb_active_green, bb_active_blue, bb_active_yellow, bb_active_purple;
wire [10:0] left_red, right_red, top_red, bottom_red;
wire [10:0] left_green, right_green, top_green, bottom_green;
wire [10:0] left_blue, right_blue, top_blue, bottom_blue;
wire [10:0] left_yellow, right_yellow, top_yellow, bottom_yellow;
wire [10:0] left_purple, right_purple, top_purple, bottom_purple;

assign bb_active_red = (x == left_red) | (x == right_red) | (y == top_red) | (y == bottom_red);
assign bb_active_green = (x == left_green) | (x == right_green) | (y == top_green) | (y == bottom_green);
assign bb_active_blue = (x == left_blue) | (x == right_blue) | (y == top_blue) | (y == bottom_blue);
assign bb_active_yellow = (x == left_yellow) | (x == right_yellow) | (y == top_yellow) | (y == bottom_yellow);
assign bb_active_purple = (x == left_purple) | (x == right_purple) | (y == top_purple) | (y == bottom_purple);

// Switch output pixels depending on mode switch
always @(*) begin
    case (reg_output)
    2'b00: begin
        new_image = bb_active_red ? RED_BB_COL :
                    bb_active_green ? GREEN_BB_COL :
                    bb_active_blue ? BLUE_BB_COL :
                    bb_active_yellow ? YELLOW_BB_COL :
                    bb_active_purple ? PURPLE_BB_COL : highlight;                                               // Colour detection
    end
    2'b01: new_image = valid_edge_detect ? {3{edge_out}} : {3{edge_buf}};                                       // Edge Detection
    2'b10: new_image = {8'b0, {2{hough_debug<<4}}} + ( valid_nonmax ? {3{nonmax_out}} : {3{nonmax_buf}} );      // Canny Edge Detection + Hough Overlay
    2'b11: new_image = {bilat_out, bilat_out, bilat_out};                                                       // Show Brightness Threshold
    endcase
end

// Have buffer such that output is the last valid output from detection IPs
reg[7:0] edge_buf, bilat_buf, nonmax_buf;
always @ (posedge clk) begin
    edge_buf <= valid_edge_detect ? edge_out : edge_buf;
    bilat_buf <= valid_bilat ? bilat_out : bilat_buf;
    nonmax_buf <= valid_nonmax ? nonmax_out : nonmax_buf;
end

// Don't modify the start-of-packet word - it's a packet descriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to get the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
    if (sop) begin
        packet_video <= (blue[3:0] == 3'h0);
		x <= 11'h0;
		y <= 11'h0;
	end	else if (valid_hsv_input) begin // Changed this to accomodate edge detector
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end else begin
			x <= x + 11'h1;
		end
	end
end

//Process bounding box at the end of the frame.
reg [4:0] msg_state;
reg [7:0] frame_count;

always@(posedge clk) begin
	if (eop & valid_hsv_input & packet_video) begin  
        //Ignore non-video packets
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
        // (This has been set to write _every_ frame)
		frame_count <= frame_count - 8'd1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 5'd1;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	// Cycle through message writer states once started
	if (msg_state != 5'd0 && msg_state < 5'd17) begin 
        msg_state <= msg_state + 4'b1;
    end else if (msg_state >= 5'd17) begin
        msg_state <= 5'd0;
    end
end
	
reg [10:0] hough_r_reg, hough_theta_reg;
always @(posedge clk) begin
    if (valid_hough) begin
        hough_r_reg <= {{3{hough_r[7]}}, hough_r};
        hough_theta_reg <= {{5{hough_theta[5]}}, hough_theta};
    end 
end
wire [10:0] wall_r = hough_r_reg;
wire [10:0] wall_theta = hough_theta_reg;
// Manual sign extension

reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define RED_MSG_ID "RBB"
`define GREEN_MSG_ID "GBB"
`define BLUE_MSG_ID "BBB"
`define YELLOW_MSG_ID "YBB"
`define PURPLE_MSG_ID "PBB"
`define WALL_MSG_ID "WBB"

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		5'd0: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
        5'd1: begin
            msg_buf_in = `RED_MSG_ID;
            msg_buf_wr = 1'b1;
        end
		5'd2: begin
			msg_buf_in = {5'b0, left_red, 5'b0, right_red};
			msg_buf_wr = 1'b1;
		end
        5'd3: begin
			msg_buf_in = {5'b0, top_red, 5'b0, bottom_red};
			msg_buf_wr = 1'b1;
		end
        5'd4: begin
            msg_buf_in = `GREEN_MSG_ID;
            msg_buf_wr = 1'b1;
        end
		5'd5: begin
			msg_buf_in = {5'b0, left_green, 5'b0, right_green};
			msg_buf_wr = 1'b1;
		end
        5'd6: begin
			msg_buf_in = {5'b0, top_green, 5'b0, bottom_green};
			msg_buf_wr = 1'b1;
		end
        5'd7: begin
            msg_buf_in = `BLUE_MSG_ID;
            msg_buf_wr = 1'b1;
        end
		5'd8: begin
			msg_buf_in = {5'b0, left_blue, 5'b0, right_blue};
			msg_buf_wr = 1'b1;
		end
        5'd9: begin
			msg_buf_in = {5'b0, top_blue, 5'b0, bottom_blue};
			msg_buf_wr = 1'b1;
		end
        5'd10: begin
            msg_buf_in = `YELLOW_MSG_ID;
            msg_buf_wr = 1'b1;
        end
		5'd11: begin
			msg_buf_in = {5'b0, left_yellow, 5'b0, right_yellow};
			msg_buf_wr = 1'b1;
		end
        5'd12: begin
			msg_buf_in = {5'b0, top_yellow, 5'b0, bottom_yellow};
			msg_buf_wr = 1'b1;
		end
        5'd13: begin
            msg_buf_in = `PURPLE_MSG_ID;
            msg_buf_wr = 1'b1;
        end
		5'd14: begin
			msg_buf_in = {5'b0, left_purple, 5'b0, right_purple};
			msg_buf_wr = 1'b1;
		end
		5'd15: begin
			msg_buf_in = {5'b0, top_purple, 5'b0, bottom_purple};
			msg_buf_wr = 1'b1;
		end
        5'd16: begin
            msg_buf_in = `WALL_MSG_ID;  // Only 2 needed for walls
            msg_buf_wr = 1'b1;
        end
        5'd17: begin
            msg_buf_in = {5'b0, wall_r, 5'b0, wall_theta};
		    msg_buf_wr = 1'b1;
      end
	endcase
end


//Output message FIFO
MSG_FIFO MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),      // the number of items inside
	.empty (msg_buf_empty)
);

/////////////////////////////////
///    Memory-mapped port     ///
/////////////////////////////////

// Addresses
`define REG_STATUS  0
`define READ_MSG    1
`define READ_ID    	2
`define REG_BBCOL	3
`define REG_THRESH  4
`define REG_OUTPUT  5

// Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused

// Threshold register:
// 31:16 - what to write to
// 15:8 - unused
// 7:0 - 8-bit threshold value

reg [1:0] reg_output;
reg [7:0] reg_status;

reg red_detector_wen, green_detector_wen, blue_detector_wen, yellow_detector_wen, purple_detector_wen;
reg [2:0] red_detector_sel, green_detector_sel, blue_detector_sel, yellow_detector_sel, purple_detector_sel;
reg [7:0] red_detector_data, green_detector_data, blue_detector_data, yellow_detector_data, purple_detector_data;

reg nonmax_wen, hough_wen, bilat_wen;
reg [7:0] nonmax_data, hough_data, bilat_data;

always @ (posedge clk) begin
	if (~reset_n) begin
		reg_status <= 8'b0;
        reg_output <= 0;    // display output

        red_detector_wen <= 0;
        green_detector_wen <= 0;
        blue_detector_wen <= 0;
        yellow_detector_wen <= 0;
        purple_detector_wen <= 0;
        nonmax_wen <= 0;

    end else begin
		if(s_chipselect & s_write) begin
            if (s_address == `REG_OUTPUT) reg_output <= s_writedata[1:0];

		    if (s_address == `REG_STATUS) reg_status <= s_writedata[7:0];

            if (s_address == `REG_THRESH) begin
                red_detector_data <= s_writedata[7:0];
                green_detector_data <= s_writedata[7:0];
                blue_detector_data <= s_writedata[7:0];
                yellow_detector_data <= s_writedata[7:0];
                purple_detector_data <= s_writedata[7:0];
                nonmax_data <= s_writedata[7:0];
                hough_data <= s_writedata[7:0];
                bilat_data <= s_writedata[7:0];

                case (s_writedata[31:16])
                0: begin    // Sat (all)
                    // Duplicate this for the other IPs 
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b000;

                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b000;

                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b000;

                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b000;

                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b000;
                end
                1: begin    // Value (all)
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b001;

                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b001;

                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b001;

                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b001;

                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b001;
                end
                2: begin    // Red_H
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b010;
                end
                3: begin    // Red_L
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b011;
                end
                4: begin    // Green_H
                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b010;
                end
                5: begin    // Green_L
                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b011;          
                end
                6: begin    // Blue_H
                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b010;
                end
                7: begin    // Blue_L
                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b011; 
                end
                8: begin    // Yellow_H
                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b010;
                end
                9: begin    // Yellow_L
                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b011;  
                end
                10: begin   // Purple_H
                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b010;
                end
                11: begin   // Purple_L
                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b011;     
                end
                12: begin   // Red_Sat
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b000;
                end
                13: begin   // Red_Val
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b001;
                end
                14: begin   // Green_sat
                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b000;
                end
                15: begin   // Green_val
                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b001;
                end
                16: begin   // Blue_sat
                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b000;
                end
                17: begin   // Blue_val
                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b001;
                end
                18: begin   // Yellow_sat
                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b000;
                end
                19: begin   // Yellow_val
                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b001;
                end
                20: begin   // Purple_Sat
                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b000;
                end
                21: begin   // Purple_val
                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b001;
                end
                22: begin   // size threshold
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b100;

                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b100;

                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b100;

                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b100;

                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b100;
                end
                23: begin   // prox threshold
                    red_detector_wen <= 1;
                    red_detector_sel <= 3'b101;

                    green_detector_wen <= 1;
                    green_detector_sel <= 3'b101;

                    blue_detector_wen <= 1;
                    blue_detector_sel <= 3'b101;

                    yellow_detector_wen <= 1;
                    yellow_detector_sel <= 3'b101;

                    purple_detector_wen <= 1;
                    purple_detector_sel <= 3'b101;
                end
                24: begin   // writing to non-max suppress for thresholding
                    nonmax_wen <= 1;
                end
                25: begin
                    hough_wen <= 1;
                end
                26: begin
                    bilat_wen <= 1;
                end

                default: begin  // end of write
                    red_detector_wen <= 0;
                    green_detector_wen <= 0;
                    blue_detector_wen <= 0;
                    yellow_detector_wen <= 0;
                    purple_detector_wen <= 0;
                    nonmax_wen <= 0;
                    hough_wen <= 0;
                    bilat_wen <= 0;
                end
                endcase
            end 
		end else begin
            red_detector_wen <= 0;
            green_detector_wen <= 0;
            blue_detector_wen <= 0;
            yellow_detector_wen <= 0;
            purple_detector_wen <= 0;
            nonmax_wen <= 0;
        end
	end
end

//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);

// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);

endmodule