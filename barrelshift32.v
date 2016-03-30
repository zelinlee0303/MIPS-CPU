`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:52:50 11/05/2015 
// Design Name: 
// Module Name:    barrelshift32 
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
module barrelshifter32(
     input [31:0]a,
	  input [4:0]b,
	  input[1:0]aluc,
	  output reg[31:0]c
	  
    );
	 
	 integer i;
	 always @ (a or b or aluc)
	 begin
		c = a;
		for ( i = 0; i < b; i = i + 1 )
		begin
			case(aluc[1:0])
				2'b00:
					c = {c[31], c[31:1]};
				2'b01:
					c = {1'b0, c[31:1]};
				2'b10:
					c = {c[30:0], 1'b0};
				2'b11:
					c = {c[30:0], 1'b0};
				default:;
			endcase
		end
	end
endmodule
	 




