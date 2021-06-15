/*
Author: LIM Tian Yi
*/

module onehot_mux(
    input [BUSWIDTH-1:0] in1,
    input [BUSWIDTH-1:0] in2,
    input [BUSWIDTH-1:0] in3,

    input sel1,
    input sel2,
    input sel3,

    output [BUSWIDTH-1:0] out
);
parameter BUSWIDTH = 8;

// Select the relavant item pointed by the mux
assign out = ( sel1 ? in1 : ( sel2 ? in2 : (sel3 ? in3 : {BUSWIDTH{1'd0}}) ) );

endmodule