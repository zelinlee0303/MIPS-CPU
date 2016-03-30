`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:02:20 10/20/2015 
// Design Name: 
// Module Name:    precg 
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
module pcreg(
					input clk, 
					input rst,
					input ena,
					input [31:0] data_in,
					output reg [31:0] data_out
    );
	 
	
	 
			

     always @ (posedge clk)
	      begin
	         
			if ( rst == 1)
				 data_out = 0;
			else if ( ena == 1)
	             data_out = data_in;
			else
			   ;
	        end
	 
			
endmodule
