// megafunction wizard: %LPM_DIVIDE%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: LPM_DIVIDE 

// ============================================================
// File Name: div_1_17_14_s.v
// Megafunction Name(s):
// 			LPM_DIVIDE
//
// Simulation Library Files(s):
// 			lpm
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 16.1.0 Build 196 10/24/2016 SJ Lite Edition
// ************************************************************


//Copyright (C) 2016  Intel Corporation. All rights reserved.
//Your use of Intel Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Intel Program License 
//Subscription Agreement, the Intel Quartus Prime License Agreement,
//the Intel MegaCore Function License Agreement, or other 
//applicable license agreement, including, without limitation, 
//that your use is for the sole purpose of programming logic 
//devices manufactured by Intel and sold by Intel or its 
//authorized distributors.  Please refer to the applicable 
//agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module div_1_17_14_s (
	clock,
	denom,
	numer,
	quotient,
	remain);

	input	  clock;
	input	[13:0]  denom;
	input	[16:0]  numer;
	output	[16:0]  quotient;
	output	[13:0]  remain;

	wire [16:0] sub_wire0;
	wire [13:0] sub_wire1;
	wire [16:0] quotient = sub_wire0[16:0];
	wire [13:0] remain = sub_wire1[13:0];

	lpm_divide	LPM_DIVIDE_component (
				.clock (clock),
				.denom (denom),
				.numer (numer),
				.quotient (sub_wire0),
				.remain (sub_wire1),
				.aclr (1'b0),
				.clken (1'b1));
	defparam
		LPM_DIVIDE_component.lpm_drepresentation = "SIGNED",
		LPM_DIVIDE_component.lpm_hint = "LPM_REMAINDERPOSITIVE=TRUE",
		LPM_DIVIDE_component.lpm_nrepresentation = "SIGNED",
		LPM_DIVIDE_component.lpm_pipeline = 1,
		LPM_DIVIDE_component.lpm_type = "LPM_DIVIDE",
		LPM_DIVIDE_component.lpm_widthd = 14,
		LPM_DIVIDE_component.lpm_widthn = 17;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "MAX 10"
// Retrieval info: PRIVATE: PRIVATE_LPM_REMAINDERPOSITIVE STRING "TRUE"
// Retrieval info: PRIVATE: PRIVATE_MAXIMIZE_SPEED NUMERIC "-1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: USING_PIPELINE NUMERIC "1"
// Retrieval info: PRIVATE: VERSION_NUMBER NUMERIC "2"
// Retrieval info: PRIVATE: new_diagram STRING "1"
// Retrieval info: LIBRARY: lpm lpm.lpm_components.all
// Retrieval info: CONSTANT: LPM_DREPRESENTATION STRING "SIGNED"
// Retrieval info: CONSTANT: LPM_HINT STRING "LPM_REMAINDERPOSITIVE=TRUE"
// Retrieval info: CONSTANT: LPM_NREPRESENTATION STRING "SIGNED"
// Retrieval info: CONSTANT: LPM_PIPELINE NUMERIC "1"
// Retrieval info: CONSTANT: LPM_TYPE STRING "LPM_DIVIDE"
// Retrieval info: CONSTANT: LPM_WIDTHD NUMERIC "14"
// Retrieval info: CONSTANT: LPM_WIDTHN NUMERIC "17"
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL "clock"
// Retrieval info: USED_PORT: denom 0 0 14 0 INPUT NODEFVAL "denom[13..0]"
// Retrieval info: USED_PORT: numer 0 0 17 0 INPUT NODEFVAL "numer[16..0]"
// Retrieval info: USED_PORT: quotient 0 0 17 0 OUTPUT NODEFVAL "quotient[16..0]"
// Retrieval info: USED_PORT: remain 0 0 14 0 OUTPUT NODEFVAL "remain[13..0]"
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @denom 0 0 14 0 denom 0 0 14 0
// Retrieval info: CONNECT: @numer 0 0 17 0 numer 0 0 17 0
// Retrieval info: CONNECT: quotient 0 0 17 0 @quotient 0 0 17 0
// Retrieval info: CONNECT: remain 0 0 14 0 @remain 0 0 14 0
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL div_1_17_14_s_bb.v TRUE
// Retrieval info: LIB_FILE: lpm
