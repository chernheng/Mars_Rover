// New version with 3 buffers for added stability.

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
    // Relevant values are only clocked out on EOP
    input valid,    // Of course don't do anything if output is not valid
    // (connect to HSV valid)

    input wen,  // Edit internal values for thresholding
    input [2:0] sel,    // see enum
    input [7:0] data,   // input data

    output wire detected,

    output reg [10:0] max_x,
    output reg [10:0] max_y,
    output reg [10:0] min_x,
    output reg [10:0] min_y // for BB
);

// parameter NUM_BUFS = 2;             // number of things to actively track 
/* Too hard to parameterise! Don't do it. */
parameter SAT_THRESH = 8'd175;         // about 70%
parameter VAL_THRESH = 8'd175;         // same
parameter HUE_THRESH_H = 8'd248;       // 350 on the (0-360) hue range
parameter HUE_THRESH_L = 8'd7;         // 10
parameter IS_RED = 1'b1;               // is it red? *affects detection logic

parameter SIZE_THRESH = 20; // should never be so small, tweak up after finding larger
parameter PROX_THRESH = 5;  // proximity between pixels to be counted as the same "item"
reg [3:0] prox_threshold;

// When user parameters change, do not perform any processing till the next pkt
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
assign detected = detect;

// Two registers for keeping things in check
reg [10:0] first_x1, first_y1, last_x1, last_y1;
reg [10:0] first_x2, first_y2, last_x2, last_y2;
reg [10:0] first_x3, first_y3, last_x3, last_y3;
reg [10:0] first_x4, first_y4, last_x4, last_y4;
// Registers for keeping a running account of the number of pixels (size)
// log2(640*480)=18.22 bits->19-bit
reg [18:0] size1, size2, size3, size4;

// Giving some allowance for pixels not being adjacent
wire in_bin1x = (x - last_x1) <= prox_threshold;
wire in_bin1y = (y == (last_y1+1) && (x <= last_x1+prox_threshold) && (x >= first_x1-prox_threshold) );
wire in_bin2x = (x - last_x2) <= prox_threshold;
wire in_bin2y = (y == (last_y2+1) && (x <= last_x2+prox_threshold) && (x >= first_x2-prox_threshold) );
wire in_bin3x = (x - last_x3) <= prox_threshold;
wire in_bin3y = (y == (last_y3+1) && (x <= last_x3+prox_threshold) && (x >= first_x3-prox_threshold) );
wire in_bin4x = (x - last_x4) <= prox_threshold;
wire in_bin4y = (y == (last_y4+1) && (x <= last_x4+prox_threshold) && (x >= first_x4-prox_threshold) );
wire in_bin1 = in_bin1x || in_bin1y;
wire in_bin2 = in_bin2x || in_bin2y;
wire in_bin3 = in_bin3x || in_bin3y;
wire in_bin4 = in_bin4x || in_bin4y;

// Detect which one is the offender
wire [2:0] in_num = {2'b00, in_bin1} + {2'b00, in_bin2} + {2'b00, in_bin3} + {2'b00, in_bin4};
wire in_1_2 = in_bin1 && in_bin2;
wire in_1_3 = in_bin1 && in_bin3;
wire in_1_4 = in_bin1 && in_bin4;
wire in_2_3 = in_bin2 && in_bin3;
wire in_2_4 = in_bin2 && in_bin4;
wire in_3_4 = in_bin3 && in_bin4;

wire larger_1_2 = size_1 >= size_2;
wire larger_1_3 = size_1 >= size_3;
wire larger_1_4 = size_1 >= size_4;
wire larger_2_3 = size_2 >= size_3;
wire larger_2_4 = size_2 >= size_4;
wire larger_3_4 = size_3 >= size_4;

// By summing these we get their sorted sizes.
wire [1:0] size_cmp_1 = {1'b0, larger_1_2} + {1'b0, larger_1_3} + {1'b0, larger_1_4};
wire [1:0] size_cmp_2 = {1'b0, larger_2_3} + {1'b0, larger_2_4} + {1'b0, ~larger_1_2};
wire [1:0] size_cmp_3 = {1'b0, larger_3_4} + {1'b0, ~larger_1_3} + {1'b0, ~larger_2_3};
wire [1:0] size_cmp_4 = {1'b0, ~larger_1_4} + {1'b0, ~larger_2_4} + {1'b0, ~larger_3_4};

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

        first_x3 <= 0;
        first_y3 <= 0;
        last_x3 <= 0;
        last_y3 <= 0;

        first_x4 <= 0;
        first_y4 <= 0;
        last_x4 <= 0;
        last_y4 <= 0;
        
        size1 <= 0;
        size2 <= 0;
        size3 <= 0;
        size4 <= 0;

        wen_valid <= 0;

        sat_threshold <= SAT_THRESH;
        val_threshold <= VAL_THRESH;
        hue_threshold_h <= HUE_THRESH_H;
        hue_threshold_l <= HUE_THRESH_L;
        size_threshold <= SIZE_THRESH;
        prox_threshold <= PROX_THRESH;
    end else begin

        if (eop) begin
            if (size1 > size_threshold || size2 > size_threshold || size3 > size_threshold || size4 > size_threshold) begin
                if (size_cmp_1 == 2'b11) begin
                    max_x <= last_x1;
                    max_y <= last_y1;
                    min_x <= first_x1;
                    min_y <= first_y1;
                end else if (size_cmp_2 == 2'b11) begin
                    max_x <= last_x2;
                    max_y <= last_y2;
                    min_x <= first_x2;
                    min_y <= first_y2;
                end else if (size_cmp_3 == 2'b11) begin
                    max_x <= last_x3;
                    max_y <= last_y3;
                    min_x <= first_x3;
                    min_y <= first_y3;
                end else if (size_cmp_4 == 2'b11) begin
                    max_x <= last_x4;
                    max_y <= last_y4;
                    min_x <= first_x4;
                    min_y <= first_y4;
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
            3'b101: prox_threshold <= data;
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

                first_x3 <= 0;
                first_y3 <= 0;
                last_x3 <= 0;
                last_y3 <= 0;

                first_x4 <= 0;
                first_y4 <= 0;
                last_x4 <= 0;
                last_y4 <= 0;
                
                size1 <= 0;
                size2 <= 0;
                size3 <= 0;
                size4 <= 0;

                wen_valid <= 0;
            end else if (detect && ~wen_valid) begin
                case (in_num):
                3'b000: begin   // none (overwrite last)
                    if (size_cmp_1==2'b00) begin
                        first_x1 <= x;
                        first_y1 <= y;
                        last_x1 <= x;
                        last_y1 <= y;
                        size1 <= 19'd1;
                    end else if (size_cmp_2==2'b00) begin
                        first_x2 <= x;
                        first_y2 <= y;
                        last_x2 <= x;
                        last_y2 <= y;
                        size2 <= 19'd1;
                    end else if (size_cmp_3==2'b00) begin
                        first_x3 <= x;
                        first_y3 <= y;
                        last_x3 <= x;
                        last_y3 <= y;
                        size3 <= 19'd1;
                    end else if (size_cmp_4==2'b00) begin
                        first_x4 <= x;
                        first_y4 <= y;
                        last_x4 <= x;
                        last_y4 <= y;
                        size4 <= 19'd1;
                    end
                end
                3'b001: begin   // one (add to it)
                    if (in_bin1) begin
                        first_x1 <= (x < first_x1) ? x : first_x1;
                        last_x1 <= (x > last_x1) ? x : last_x1;
                        last_y1 <= y;
                        size1 <= size1 + 19'd1;
                    end else if (in_bin2) begin
                        first_x2 <= (x < first_x2) ? x : first_x2;
                        last_x2 <= (x > last_x2) ? x : last_x2;
                        last_y2 <= y;
                        size2 <= size2 + 19'd1;
                    end else if (in_bin3) begin
                        first_x3 <= (x < first_x3) ? x : first_x3;
                        last_x3 <= (x > last_x3) ? x : last_x3;
                        last_y3 <= y;
                        size3 <= size3 + 19'd1;
                    end else if (in_bin4) begin
                        first_x4 <= (x < first_x4) ? x : first_x4;
                        last_x4 <= (x > last_x4) ? x : last_x4;
                        last_y4 <= y;
                        size4 <= size4 + 19'd1;
                    end
                end
                3'b010: begin   // two conflicting (merge, overwrite the smaller)
                wire in_1_2 = in_bin1 && in_bin2;
                    if (in_1_2) begin
                    end else if (in_1_3) begin
                    end else if (in_1_4) begin
                    end else if (in_2_3) begin
                    end else if (in_2_4) begin
                    end else if (in_3_4) begin
                    end
                end
                3'b011: begin   // three conflicting (merge smallest 2) -- shld not happen!!
                    
                end
                3'b100: begin   // four conflicting (merge smallest 2) -- shld not happen!!
                    
                end
                default: begin  // Error?
                    
                end
                endcase
                // 3 cases for in_bin1, in_bin2:
                if (in_bin1 && in_bin2) begin
                    // Merge two banks (to the larger one)
                    if (size_cmp) begin
                        size1 <= size1 + size2 + 1;
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
                        size2 <= size1 + size2 + 1;
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
                    last_x1 <= in_bin1 ? x : last_x1;
                    last_y1 <= in_bin1 ? y : last_y1;
                    last_x2 <= in_bin2 ? x : last_x2;
                    last_y2 <= in_bin2 ? y : last_y2;

                    size1 <= size1 + in_bin1;
                    size2 <= size2 + in_bin2;   // only one shld be active anyway

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