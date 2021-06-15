/* 
Author: LIM Tian Yi

Implements a simple LUT for coefficient of Gaussian kernel.
Outputs the numerator and denominator required given the input, an (unsigned) 4-bit number (contracted from possible 8-bit values)

obtain the kernel as from kernel-making python script.
Coefficients (numerator): 17 17 16 15 13 12 10 8 6 5 4 3 2 1 1 0
(and denominator is 256, bit-shift 8)

We only need 5 bits for this.
*/

module gauss_kernel(
    input [7:0] center_pixel,
    input [7:0] out_pixel,
    output [4:0] num
);

// Always take the positive value
wire [7:0] diff_8 = (center_pixel > out_pixel) ? (center_pixel-out_pixel) : (out_pixel-center_pixel);

// Take the MSBs as we further quantize down.
wire [4:0] diff = diff_8[7:3];

// LUT implementation.
always @ (*) begin
    case(diff)
    5'b00000: num=5'd17;   // 0
    5'b00001: num=5'd17;   // 1
    5'b00010: num=5'd16;   // 2
    5'b00011: num=5'd15;   // 3
    5'b00100: num=5'd13;   // 4     
    5'b00101: num=5'd12;   // 5
    5'b00110: num=5'd10;   // 6
    5'b00111: num=5'd8;    // 7
    5'b01000: num=5'd6;    // 8
    5'b01001: num=5'd5;    // 9
    5'b01010: num=5'd4;    // 10
    5'b01011: num=5'd3;    // 11
    5'b01100: num=5'd2;    // 12     
    5'b01101: num=5'd1;    // 13
    5'b01110: num=5'd1;    // 14
    5'b01111: num=5'd0;    // 15
    default: num=5'd0;

    endcase
end

endmodule