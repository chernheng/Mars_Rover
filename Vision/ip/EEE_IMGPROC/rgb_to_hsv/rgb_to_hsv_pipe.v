/*
Author: LIM Tian Yi

Pipelined, forked version of the RGB to HSV convertor.
Input: 8-bit RGB
Output: Hue scaled from 0-360 to 0-255
        Saturation scaled from 0-100% to 0-255
        Value scaled from 0-100% to 0-255
Latency: 5 cycles
*/
module rgb_to_hsv_pipe(
    input clk,

    input valid_in,
    input [7:0] red,
    input [7:0] green,
    input [7:0] blue,
    
    output reg valid_out,
    output reg [7:0] hue,
    output reg [7:0] sat,
    output reg [7:0] val
);

// Underscored numbers reflect the cycle at which the data is written at
reg [14:0] red_mult_1, green_mult_1, blue_mult_1;
reg valid_1, valid_2, valid_3, valid_4;
reg [8:0] delta_1, delta_2;
reg red_max_1, red_max_2, red_max_3, green_max_1, green_max_2, green_max_3, blue_max_1;
reg [15:0] min_color_shifted_1;
reg [8:0] sat_division_2, sat_division_3, sat_division_4;
reg [7:0] max_color_1, max_color_2, max_color_3, max_color_4;
reg subt_res_sgn_2, subt_res_sgn_3;
reg [14:0] subt_res_norm_2;  // normalised for subt_res sign
reg [8:0] hue_division_3;
reg [9:0] hue_added_4;

wire [8:0] sat_division_2_div, hue_division_3_div;
wire [7:0] sat_remain, hue_remain;

// Divison is clocked to reduce critical path.
// Dividers instantiated are Unsigned dividers to match previous code which used the unsigned '/' verilog operator
div sat_div(
	.clock(clk), .denom(max_color_1), .numer(min_color_shifted_1),
	.quotient(sat_division_2_div), .remain(sat_remain)
);

div hue_div(
	.clock(clk), .denom(delta_2), .numer(subt_res_norm_2),
	.quotient(hue_division_3_div), .remain(hue_remain)
);

always @(posedge clk) begin
    // STAGE 1 : DATA INPUT
        valid_1 <= valid_in;    // Pass on signals through the hardware pipeline

        // Calculate multiplied constants
        red_mult_1 <= red*15'd42;
        green_mult_1 <= green*15'd42;
        blue_mult_1 <= blue*15'd42;

        // HUE: calculate delta, max values
        delta_1 <= {1'b0, max_color - min_color};
        red_max_1 <= red_max;
        green_max_1 <= green_max;
        blue_max_1 <= blue_max;

        // SATURATION: Shift
        min_color_shifted_1 <= min_color << 8;    // this is unsigned
        max_color_1 <= max_color;
    
    // STAGE 2
        valid_2 <= valid_1;
        delta_2 <= delta_1;
        max_color_2 <= max_color_1;
        red_max_2 <= red_max_1;       
        green_max_2 <= green_max_1;

        // HUE: Division
        // =0 if delta = 0
        subt_res_norm_2 <= subt_res[14] ? -subt_res : subt_res;
        subt_res_sgn_2 <= subt_res[14];

        // SATURATION: Division (Happens in LPM function)

    // STAGE 3
        valid_3 <= valid_2;
        max_color_3 <= max_color_2;
        sat_division_3 <= {1'b0, sat_division_2_div[7:0]};  // changed this
        red_max_3 <= red_max_2;
        green_max_3 <= green_max_2;
        subt_res_sgn_3 <= subt_res_sgn_2;

        // HUE: Division (happens in LPM Function)

    // STAGE 4
        valid_4 <= valid_3;
        max_color_4 <= max_color_3;
        sat_division_4 <= sat_division_3;

        // HUE: Add value
        hue_added_4 <= hue_division_corrected + hue_add;

    // STAGE 5 : OUTPUT
        valid_out <= valid_4;        

        // HUE: Modulo
        if (hue_added_4[9]) begin             // MSB is high = hue_added is negative. Add 255
            hue <= hue_added_4 + 10'd255;
        end else if (hue_added_4[8]) begin    // Bit 8 = 256 is high. hue_added > 255. Subtract 255.
            hue <= hue_added_4 - 10'd255;
        end else begin                        // No modifications to hue_added necessary
            hue <= hue_added_4[7:0];
        end

        // SATURATION: Finish up
        sat <= 9'd256 - sat_division_4[7:0];

        // VALUE: output
        val <= max_color_4; // Moved here to make reading waveforms easier  
end

// Finding the max and min colors each cycle

// Combinatorial comparison between color inputs
wire red_gt_green = red >= green;
wire red_gt_blue = red >= blue;
wire green_gt_blue = green >= blue;

wire red_max = red_gt_green & red_gt_blue;
wire red_min = !red_gt_green & !red_gt_blue;
wire green_max = green_gt_blue & !red_gt_green;
wire green_min = !green_gt_blue & red_gt_green;
wire blue_max = !red_gt_blue & !green_gt_blue;
wire blue_min = red_gt_blue & green_gt_blue;

wire [7:0] max_color;
wire [7:0] min_color;

// mux instantitated to take in three one-hot values
onehot_mux max_mux(.in1(red), .in2(green), .in3(blue),
                    .sel1(red_max), .sel2(green_max), .sel3(blue_max),
                    .out(max_color)
);
onehot_mux min_mux(.in1(red), .in2(green), .in3(blue),
                    .sel1(red_min), .sel2(green_min), .sel3(blue_min),
                    .out(min_color)
);

// Combinatorial subtraction
wire [14:0] red_subt_res = green_mult_1 - blue_mult_1;
wire [14:0] green_subt_res = blue_mult_1 - red_mult_1;
wire [14:0] blue_subt_res = red_mult_1 - green_mult_1;
wire [14:0] subt_res;

onehot_mux #(.BUSWIDTH(15)) subt_res_mux (
                    .in1(red_subt_res), .in2(green_subt_res), .in3(blue_subt_res),
                    .sel1(red_max_1), .sel2(green_max_1), .sel3(blue_max_1),
                    .out(subt_res)
);

// Get complement of delta. This happens in parallel - not sure if it's the best
wire [8:0] hue_division_corrected = subt_res_sgn_3 ? {1'b1, ~hue_division_3_div+9'b1} : {1'b0, hue_division_3_div};
wire [9:0] hue_add = ( red_max_3 ? 10'd255 : green_max_3 ? 10'd85 : 10'd170 );  // additional term to add

endmodule