`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:08:51 10/15/2015 
// Design Name: 
// Module Name:    ext 
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
module ext #(parameter WIDTH=16)(

				input [WIDTH-1:0] a, //�������ݣ����ݿ�ȿ��Ը�����Ҫ���ж���
				input sext, //sext ��ЧΪ������չ������ 0 ��չ
				output[31:0] b //32 λ������ݣ�
				);
			assign b = sext?{{(32 - WIDTH){a[WIDTH - 1]}}, a}:{27'b0, a};	
			
			
	

endmodule
