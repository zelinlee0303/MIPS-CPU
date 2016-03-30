`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:13:13 10/21/2015 
// Design Name: 
// Module Name:    ram 
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
module ram(
        input clk,
		  input ram_ena,
		  input wena,
		  input[4:0] addr,
		  input [31:0] data_in,
		  output [31:0]data_out
    );

      reg [31:0] ram[31:0];
		assign data_out = ram_ena?ram[addr]:32'bz;
		     always @ ( posedge clk )
			  begin
						if ( ram_ena == 1 && wena == 1 )
							ram[addr] = data_in;		  
				end	       

endmodule
