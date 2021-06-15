/*
Author: LIM Tian Yi

Colour Detection IP. Takes in HSV values from upstream core and performs thresholding based on runtime-configurable values.
Generates a bounding box based on the largest contiguous volume in the image. Uses two accumulators to reject noise in the frame.
Bounding box values have a one-frame delay; at 60fps this should be negligible.
*/

module smooth_detect (
    input clk,
    input rst_n,

    input [10:0] x,
    input [10:0] y,    // current x and y coordinates

    input [7:0] hue,
    input [7:0] sat,
    input [7:0] val,
    input sop, eop,
    // Don't attempt to do anything for SOP/EOP (or when inputs are not valid)
    // Reset counters on SOP
    // Relevant values are only clocked out on EOP
    input valid,    
    // Don't do anything if output is not valid
    
    input wen,  // Edit internal values for thresholding
    input [2:0] sel,    // see enum
    input [7:0] data,   // input data

    output wire detected,

    output reg [10:0] max_x,    // BB right
    output reg [10:0] max_y,    // BB bottom
    output reg [10:0] min_x,    // BB left
    output reg [10:0] min_y     // BB top
);

parameter SAT_THRESH = 8'd175;          // about 70%
parameter VAL_THRESH = 8'd175;          // same
parameter HUE_THRESH_H = 8'd248;        // 350 on the (0-360) hue range
parameter HUE_THRESH_L = 8'd7;          // 10
parameter IS_RED = 1'b1;                // is it red? *affects detection logic
parameter SIZE_THRESH = 200;            // should never be so small
parameter PROX_THRESH = 2;              // proximity between pixels to be counted as the same "item"

reg [3:0] prox_threshold;

// When user parameters change, do not perform any processing till the next packet.
reg wen_valid;

reg [7:0] sat_threshold, val_threshold, hue_threshold_h, hue_threshold_l;
reg [18:0] size_threshold;
wire sat_detect = (sat >= sat_threshold);
wire val_detect = (val >= val_threshold);
wire hue_detect = IS_RED ?
                    (hue >= hue_threshold_h || hue <= hue_threshold_l) : 
                    (hue <= hue_threshold_h && hue >= hue_threshold_l);
// red works a bit different due to wrapping around color wheel
wire detect = sat_detect && val_detect && hue_detect;
assign detected = detect;   // Assign output

// Two registers for keeping things in check
reg [10:0] first_x1, first_y1, last_x1, last_y1;
reg [10:0] first_x2, first_y2, last_x2, last_y2;
// Registers for keeping a running account of the number of pixels (size)
// log2(640*480)=18.22 bits->19-bit (although typically much smaller)
reg [18:0] size1, size2;

// Giving some allowance for pixels not being adjacent.
wire in_bin1x = (x - last_x1) <= prox_threshold;
wire in_bin1y = ((y - last_y1) <= prox_threshold) && (x <= last_x1+prox_threshold) && (x >= first_x1-prox_threshold);
wire in_bin2x = (x - last_x2) <= prox_threshold;
wire in_bin2y = ((y - last_y2) <= prox_threshold) && (x <= last_x2+prox_threshold) && (x >= first_x2-prox_threshold);
wire in_bin1 = in_bin1x || in_bin1y;
wire in_bin2 = in_bin2x || in_bin2y;

// Affects which bin to put next item into.
wire size_cmp = (size1 >= size2);

// Input can be glitchy, which results in BB's stuttering/appearing intermittently.
// To smooth out the output, give BB's a Time to Live
reg [2:0] last_seen_ctr;

always @ (posedge clk) begin
    if (~rst_n) begin
        // Reset
        first_x1 <= 0;
        first_y1 <= 0;
        last_x1 <= 0;
        last_y1 <= 0;
        
        first_x2 <= 0;
        first_y2 <= 0;
        last_x2 <= 0;
        last_y2 <= 0;
        
        size1 <= 0;
        size2 <= 0;

        wen_valid <= 0;

        last_seen_ctr <= 0;

        sat_threshold <= SAT_THRESH;
        val_threshold <= VAL_THRESH;
        hue_threshold_h <= HUE_THRESH_H;
        hue_threshold_l <= HUE_THRESH_L;
        size_threshold <= SIZE_THRESH;
        prox_threshold <= PROX_THRESH;
    end else begin

        if (eop) begin
            if (size1 > size_threshold || size2 > size_threshold) begin
                max_x <= (size_cmp) ? last_x1 : last_x2 ;
                max_y <= (size_cmp) ? last_y1 : last_y2 ;
                min_x <= (size_cmp) ? first_x1 : first_x2;
                min_y <= (size_cmp) ? first_y1 : first_y2;
                last_seen_ctr <= 0;
            end else begin
                if (last_seen_ctr==3'b111) begin
                    max_x <= -1;
                    max_y <= -1;
                    min_x <= -1;
                    min_y <= -1;
                end else begin                
                    last_seen_ctr <= last_seen_ctr + 3'd1;
                end
            end
        end
        
        if (wen) begin
            wen_valid <= 1; // Content has been modified, current vals don't make sense.

            case (sel)    // see enum
            3'b000: sat_threshold <= data;
            3'b001: val_threshold <= data;
            3'b010: hue_threshold_h <= data;
            3'b011: hue_threshold_l <= data;
            3'b100: size_threshold <= data;
            3'b101: prox_threshold <= data[3:0];
            endcase

        end else if (valid) begin
            if (sop) begin
                // Reset all
                first_x1 <= 0;
                first_y1 <= 0;
                last_x1 <= 0;
                last_y1 <= 0;
                
                first_x2 <= 0;
                first_y2 <= 0;
                last_x2 <= 0;
                last_y2 <= 0;
                
                size1 <= 0;
                size2 <= 0;

                wen_valid <= 0;
            end else if (detect && ~wen_valid) begin
                // 3 cases for in_bin1, in_bin2:
                if (in_bin1 && in_bin2) begin
                    // Merge two banks (to the larger one)
                    if (size_cmp) begin
                        size1 <= size1 + size2 + 19'd1;
                        size2 <= 0;

                        if (first_x2 < first_x1) begin
                            first_x1 <= first_x2;
                        end
                        if (last_x2 > last_x1) begin
                            last_x1 <= last_x2;
                        end
                        if (first_y2 < first_y1) begin
                            first_y1 <= first_y2;
                        end
                        if (last_y2 > last_y1) begin
                            last_y1 <= last_y2;
                        end

                    end else begin
                        size2 <= size1 + size2 + 19'd1;
                        size1 <= 0;

                        if (first_x1 < first_x2) begin
                            first_x2 <= first_x1;
                        end
                        if (last_x1 > last_x2) begin
                            last_x2 <= last_x1;
                        end
                        if (first_y1 < first_y2) begin
                            first_y2 <= first_y1;
                        end
                        if (last_y1 > last_y2) begin
                            last_y2 <= last_y1;
                        end

                    end
                end else if (in_bin1 || in_bin2) begin
                    // Only update one, and only one should be true anyway.
                    if (in_bin1) begin
                        first_x1 <= (x < first_x1) ? x : first_x1;
                        last_x1 <= (x > last_x1) ? x : last_x1;
                        last_y1 <= (y > last_y1) ? y : last_y1;
                        size1 <= size1 + 19'd1;
                    end else if (in_bin2) begin
                        first_x2 <= (x < first_x2) ? x : first_x2;
                        last_x2 <= (x > last_x2) ? x : last_x2;
                        last_y2 <= (y > last_y2) ? y : last_y2;
                        size2 <= size2 + 19'd1;
                    end

                end else begin
                    // Overwrite the smaller register
                    if (size_cmp) begin
                        first_x2 <= x;
                        first_y2 <= y;
                        last_x2 <= x;
                        last_y2 <= y;

                        size2 <= 1; // Overwrite 2
                    end else begin
                        first_x1 <= x;
                        first_y1 <= y;
                        last_x1 <= x;
                        last_y1 <= y;

                        size1 <= 1; // Overwrite 1
                    end
                end
            end
        end
    end
end

endmodule