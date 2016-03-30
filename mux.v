`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:32:02 10/15/2015 
// Design Name: 
// Module Name:    verilog 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module mux(
		input[3:0]a, //4 位输入 a
		input[3:0]b, //4 位输入 b
		input s, //1 位输入，当 s=0 时 r 的值等于 a，s=1 时 r 的值等于 b
		output[3:0] r//4 位输出，相应位的值由 abs 的值确定
			) ;
	assign r = s?b:a ;

endmodule
