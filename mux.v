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
		input[3:0]a, //4 λ���� a
		input[3:0]b, //4 λ���� b
		input s, //1 λ���룬�� s=0 ʱ r ��ֵ���� a��s=1 ʱ r ��ֵ���� b
		output[3:0] r//4 λ�������Ӧλ��ֵ�� abs ��ֵȷ��
			) ;
	assign r = s?b:a ;

endmodule
