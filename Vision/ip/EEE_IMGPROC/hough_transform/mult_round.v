/*
Author: LIM Tian Yi

Does a simple rounding by looking at the 6th bit to instead of truncating the last 6 bits.
*/
module round (
    input [13:0] in,
    output [7:0] out
);
    
assign out = in[5] ? in[13:6]+8'd1 : in[13:6];

endmodule