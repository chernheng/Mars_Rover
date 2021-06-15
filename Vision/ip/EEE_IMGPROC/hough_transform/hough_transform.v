/*
Author: LIM Tian Yi

Implements the Hough Transform for Lines. Instanstiates an Image buffer and a Accumulator to detect r, theta parameters to describe lines.
*/

module hough_transform (
    input clk,
    input rst_n,
    input [9:0] x, input [8:0] y,
    input canny_in,
    input valid_in,

    output valid_out,
    output [7:0] r,
    output [5:0] theta,
    output [7:0] int_out,

    input wen,
    input [7:0] data_in
);

reg canny_in_reg;
always @(posedge clk) begin
    canny_in_reg <= valid_in ? canny_in : canny_in_reg;
end

reg [11:0] buf_addr;
reg buf_wen;
wire buf_dataout, buf_datain;
buffer buf_canny(             // Buffer is 20x120 (x * y)
    .clock(clk),
    .address(buf_addr), .data(buf_datain),
    .wren(buf_wen), .q(buf_dataout)
);
// Clear when asked to (state2), else OR with existing content
assign buf_datain = (state==2'd3) ? 1'b0 : ( buf_dataout || canny_in_reg );

reg [12:0] acc_addr;
reg acc_wen;
wire acc_wen_wire;
wire [7:0] acc_datain, acc_dataout;
accum acc(              // Acc is 121x41 (r*theta)
	.clock(clk),
	.address(acc_addr), .data(acc_datain),
	.wren(acc_wen_wire), .q(acc_dataout)
);
assign acc_wen_wire = (state==2'd0) ? (buf_dataout&&state_refresh&&(acc_addr!=acc_addr_calc)) : acc_wen;    // writing has special logic
assign acc_datain = (state==2'd2) ? 8'd0 : (acc_addr_ctr==0) ? acc_dataout + 8'd1 : acc_databuf + 8'd1;

// Long cases for Boxes to contain Hough Transform debug information
assign int_out = ( (y==9'd240 && (x<10'd121)) ||
                    (y==9'd281 && (x<10'd121)) ||
                    (y==9'd120 && (x>=10'd597 && x<10'd619)) ||
                    (y==9'd240 && (x>=10'd597 && x<10'd619)) ||
                    (x==10'd121 && (y>=9'd240 && y<9'd281)) ||
                    (x==10'd597 && (y>=9'd120 && y<9'd240)) ||
                    (x==10'd619 && (y>=9'd120 && y<9'd240)) ) ? 8'hFF : 
                (y>=9'd240 && y<9'd281 && x<10'd121) ? acc_dataout: 
                (y>=9'd120 && y<9'd240 && x>=10'd598 && x<10'd618) ? {4'd0, {4{buf_dataout}}} : 8'd0;

// Lookup tables for approximating the Sine/Cosine function (divided by 64)
// x_lut_64 = [41,39,37,35,33,32,30,28,26,23,21,19,17,15,13,11,8,6,4,2,0,-2,-4,-6,-8,-11,-13,-15,-17,-19,-21,-23,-26,-28,-30,-31,-33,-35,-37,-39,-41]
// y_lut_64 = [49,50,51,53,54,55,56,57,58,59,60,60,61,62,62,63,63,63,63,63,64,63,63,63,63,63,62,62,61,60,60,59,58,57,56,55,54,53,51,50,49]
wire[13:0] mult_x, mult_y;
// Contains ROM with relevant coefficients
cos_lut x_lut(theta_cnt_prev, clk, mult_x);
sin_lut y_lut(theta_cnt_prev, clk, mult_y);

wire [13:0] cos = {3'b0, x_cnt_prev} * mult_x;
wire [13:0] sin = {3'b0, y_cnt_prev} * mult_y;
wire [13:0] r_sum_bef = cos + sin;
wire [7:0] r_val;
round round_sum (r_sum_bef, r_val);     // Rounding

reg [1:0] state;
reg state_refresh;
// 0: Writing to accumulator, 1: getting result out, 2: waiting for stuff to happen, 3: Display and clear Buffer

reg [5:0] theta_cnt;
reg [6:0] r_cnt;
reg [4:0] x_cnt;
reg [6:0] y_cnt;    // counters iterate, track state
reg [10:0] x_cnt_prev, y_cnt_prev, r_cnt_prev, theta_cnt_prev;

parameter THRESH = 8;
reg [7:0] thresh_reg;
reg [7:0] max_val;  // highest accumulator value seen (so far)

reg [7:0] acc_databuf;
reg [7:0] acc_addr_ctr;
wire [12:0] acc_addr_calc = theta_cnt_prev*13'd121 + {5'b0, r_val};

always @ (posedge clk) begin
    if (~rst_n) begin
        // Reset all state
        theta_cnt <= 0;
        r_cnt <= 0;
        x_cnt <= 0;
        y_cnt <= 0;

        max_val <= 0;
        state <= 2'd2;

        buf_addr <= 0;
        buf_wen <= 0;

        thresh_reg <= THRESH;
		  
        r <= 0;
        theta <= 0;
    end else if (wen) begin
        thresh_reg <= data_in;
    end else begin
        // Display and clear Buffer
        if (state==2'd3) begin
            valid_out <= 0;

            if (y>=9'd120 && y<9'd240 && x>=10'd598 && x<10'd618) begin
                buf_addr <= ({3'b0, y-9'd120}*12'd20) + ({2'b0, x}-12'd598);
                buf_wen <= 0;
            end else if (y>=9'd120 && y<9'd240 && x>=10'd618 && x<10'd638) begin
                buf_addr <= ({3'b0, y-9'd120}*12'd20) + ({2'b0, x}-12'd618);
                buf_wen <= 1;
            end else begin
                buf_wen <= 0;
            end

            // At last pixel, go to Buffer-writing state
            if (y==9'd239 && x==10'd638) begin
                state <= 2'd2;
                acc_wen <= 0;
                buf_wen <= 0;
            end
        end
        // Write to buffer, Display and clear Acc
        if (state==2'd2) begin

            // Display and clear Acc
            if (y>=9'd240 && y<9'd281 && y<9'd281 && x<10'd121) begin
                acc_addr <= ({4'b0, y-9'd240}*13'd121) + ({3'b0, x});
                acc_wen <= 0;
            end else if (y>=9'd281 && y<9'd322 && y<9'd322 && x<10'd121) begin
                acc_addr <= ({4'b0, (y-9'd281)}*13'd121) + ({3'b0, x});   // y:theta, x:r
                acc_wen <= 1;
            end else begin
                acc_wen <= 0;
            end

            max_val <= 0;   // reset at the start of the frame
            valid_out <= 0;
            // Activate latter stage, because of the -1 in the previous stages

            // Crop off the bottom of the frame, otherwise false positive.
            if (x==10'd638 && y==9'd477) begin
                state <= 2'd0;
                state_refresh <= 0;
                buf_wen <= 0;
                acc_wen <= 0;
            end
            
            // Write to buffer
            if (y >= 9'd240 && x >= 10'd280 && x < 10'd360) begin
                // Perform compression and write to buffer
                if (valid_in) begin
                    buf_addr <= ((x-10'd280)>>2) + ( ((y-9'd240)>>1)*12'd20 );   // Buffer is 20*120
                    buf_wen <= (!buf_dataout && canny_in);  // Don't write unnecessarily
                end else begin
                    buf_wen <= 0;
                end
            end else begin
                buf_wen <= 0;
            end

        end

        // Write to Accumulator
        if (state == 2'd0) begin
            valid_out <= 0;
            buf_wen <= 0;

            if(state_refresh==0) state_refresh <= 1;    
            // To stop operations occuring at the initial state transition, when mem outputs have not stabilised.

            // Iterate through theta, x, y
            if (x_cnt < 5'd19) begin
                x_cnt <= x_cnt + 5'd1;
            end else begin
                x_cnt <= 5'd0;
                if (y_cnt < 7'd119) begin
                    y_cnt <= y_cnt + 7'd1;
                end else begin
                    y_cnt <= 7'd0;
                    if (theta_cnt < 6'd40) begin
                        theta_cnt <= theta_cnt + 6'd1;
                    end else begin
                        state <= 2'd1;
                        theta_cnt <= 6'd0;
                        acc_wen <= 0;
                        state_refresh <= 0;
                    end
                end
            end

            x_cnt_prev <= {6'b0, x_cnt};
            y_cnt_prev <= {4'b0, y_cnt};    // Delayed to match the address given in the calculation
            theta_cnt_prev <= {5'b0, theta_cnt};

            // Perform calculations here
            // Look at memory
            buf_addr <= {7'b0, x_cnt} + {6'b0, y_cnt}*12'd20;
            acc_addr <= acc_addr_calc;  // uses 'prev' values

            if (buf_dataout != 0 && state_refresh) begin
                // Calculate r = x*cos(theta) + y*sin(theta); acc[r][theta] += 1
                // "Look up Table"

                if (acc_addr != acc_addr_calc) begin
                    acc_addr_ctr <= 8'd0;
                end else begin
                    acc_addr_ctr <= acc_addr_ctr + 8'd1;
                end

                if (acc_addr_ctr==8'd0) begin
                    acc_databuf <= (acc_dataout==8'd255) ? 8'd255 : acc_dataout + 8'd1; // Reset the buffer
                end else begin
                    acc_databuf <= (acc_databuf==8'd255) ? 8'd255 : acc_databuf + 8'd1;
                end

            end else begin
                acc_wen <= 0;
            end

        // Obtain final result
        end else if (state==2'd1) begin
            acc_wen <= 0;
            if(state_refresh==0) state_refresh <= 1;    

            // Iterate through theta, r
            if (r_cnt < 7'd121) begin
                r_cnt <= r_cnt + 7'd1;
                valid_out <= 0;
            end else begin
                r_cnt <= 7'd0;
                if (theta_cnt < 6'd40) begin
                    theta_cnt <= theta_cnt + 6'd1;
                    valid_out <= 0;
                end else begin
                    theta_cnt <= 6'd0;
                    if (max_val == 0) begin
                        r <= -1;
                        theta <= -1;    // Error handling - throw "-1"
                    end
                    valid_out <= 1; // once per frame
                    state <= 2'd3;  // waiting for stuff to happen
                end
            end

            theta_cnt_prev <= {5'b0, theta_cnt};
            r_cnt_prev <= {4'd0, r_cnt};

            // Perform calculations here
            // look at buffer
            acc_addr <= theta_cnt*13'd121 + r_cnt;
            if ( (acc_dataout > thresh_reg) && (acc_dataout > max_val) && state_refresh) begin
                max_val <= acc_dataout;
                r <= r_cnt_prev[7:0];
                theta <= theta_cnt_prev[5:0];
            end

            if (acc_dataout==max_val) begin
                r <= -1;
                theta <= -1; // two conflicting data points: Reject this sample.
            end
        end
    end
end

endmodule