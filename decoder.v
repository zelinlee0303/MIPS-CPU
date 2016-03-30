`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:27:09 10/13/2015 
// Design Name: 
// Module Name:    decoder 
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
module decoder(
    input [ 2:0 ] data_in,
    input ena,
    output [ 7:0 ] data_out
    );
	 
		reg[7:0]data_temp;
		assign data_out = data_temp;
		always @( ena or data_in )  begin
			if ( ena == 1 )
				case ( data_in )
					3'b000:
						data_temp = 8'b11111110;
					3'b001:
						data_temp = 8'b11111101;
					3'b010:
						data_temp = 8'b11111011;
					3'b011:
						data_temp = 8'b11110111;
					3'b100:
						data_temp = 8'b11101111;
					3'b101:
						data_temp = 8'b11011111;
					3'b110:
						data_temp = 8'b10111111;
					3'b111:
						data_temp = 8'b01111111;
				endcase
			else
				data_temp = 8'b11111111;
		end


endmodule
