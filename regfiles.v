`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    03:55:56 10/28/2015 
// Design Name: 
// Module Name:    regfiles 
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
module regfiles(
			input clk,
			input rst,
			input we,
			input [4:0]raddr1,
			input [4:0]raddr2,
			input [4:0]waddr,
			input [31:0]wdata,
			output [31:0]rdata1,
			output [31:0]rdata2
    );
	 
    wire[31:0]regs[31:0];
	 wire[31:0]temp;
	 decoder decoder(
	 .data_in(waddr),
	 .ena(we),
	 .data_out(temp));
	 
	 
	 
	 pcreg pcreg_0(
	 .clk(clk),.rst(1),.ena(temp[0]),
	 .data_in(wdata),.data_out(regs[0]));
	 
	 pcreg pcreg_1(
	 .clk(clk),.rst(rst),.ena(temp[1]),
	 .data_in(wdata),.data_out(regs[1]));
	 
	 pcreg pcreg_2(
	 .clk(clk),.rst(rst),.ena(temp[2]),
	 .data_in(wdata),.data_out(regs[2]));
	 
	 pcreg pcreg_3(
	 .clk(clk),.rst(rst),.ena(temp[3]),
	 .data_in(wdata),.data_out(regs[3]));
	 
	 pcreg pcreg_4(
	 .clk(clk),.rst(rst),.ena(temp[4]),
	 .data_in(wdata),.data_out(regs[4]));
	 
	 pcreg pcreg_5(
	 .clk(clk),.rst(rst),.ena(temp[5]),
	 .data_in(wdata),.data_out(regs[5]));
	 
	 pcreg pcreg_6(
	 .clk(clk),.rst(rst),.ena(temp[6]),
	 .data_in(wdata),.data_out(regs[6]));
	 
	 pcreg pcreg_7(
	 .clk(clk),.rst(rst),.ena(temp[7]),
	 .data_in(wdata),.data_out(regs[7]));
	 
	 pcreg pcreg_8(
	 .clk(clk),.rst(rst),.ena(temp[8]),
	 .data_in(wdata),.data_out(regs[8]));
	 
	 pcreg pcreg_9(
	 .clk(clk),.rst(rst),.ena(temp[9]),
	 .data_in(wdata),.data_out(regs[9]));
	 
	 pcreg pcreg_10(
	 .clk(clk),.rst(rst),.ena(temp[10]),
	 .data_in(wdata),.data_out(regs[10]));
	 
	 pcreg pcreg_11(
	 .clk(clk),.rst(rst),.ena(temp[11]),
	 .data_in(wdata),.data_out(regs[11]));
	 
	 pcreg pcreg_12(
	 .clk(clk),.rst(rst),.ena(temp[12]),
	 .data_in(wdata),.data_out(regs[12]));
	 
	 pcreg pcreg_13(
	 .clk(clk),.rst(rst),.ena(temp[13]),
	 .data_in(wdata),.data_out(regs[13]));
	 
	 pcreg pcreg_14(
	 .clk(clk),.rst(rst),.ena(temp[14]),
	 .data_in(wdata),.data_out(regs[14]));
	 
	 pcreg pcreg_15(
	 .clk(clk),.rst(rst),.ena(temp[15]),
	 .data_in(wdata),.data_out(regs[15]));
	 
	 pcreg pcreg_16(
	 .clk(clk),.rst(rst),.ena(temp[16]),
	 .data_in(wdata),.data_out(regs[16]));
	 
	 pcreg pcreg_17(
	 .clk(clk),.rst(rst),.ena(temp[17]),
	 .data_in(wdata),.data_out(regs[17]));
	 
	 pcreg pcreg_18(
	 .clk(clk),.rst(rst),.ena(temp[18]),
	 .data_in(wdata),.data_out(regs[18]));
	 
	 pcreg pcreg_19(
	 .clk(clk),.rst(rst),.ena(temp[19]),
	 .data_in(wdata),.data_out(regs[19]));
	 
	 pcreg pcreg_20(
	 .clk(clk),.rst(rst),.ena(temp[20]),
	 .data_in(wdata),.data_out(regs[20]));
	 
	 pcreg pcreg_21(
	 .clk(clk),.rst(rst),.ena(temp[21]),
	 .data_in(wdata),.data_out(regs[21]));
	 
	 pcreg pcreg_22(
	 .clk(clk),.rst(rst),.ena(temp[22]),
	 .data_in(wdata),.data_out(regs[22]));
	 
	 pcreg pcreg_23(
	 .clk(clk),.rst(rst),.ena(temp[23]),
	 .data_in(wdata),.data_out(regs[23]));
	 
	 pcreg pcreg_24(
	 .clk(clk),.rst(rst),.ena(temp[24]),
	 .data_in(wdata),.data_out(regs[24]));
	 
	 pcreg pcreg_25(
	 .clk(clk),.rst(rst),.ena(temp[25]),
	 .data_in(wdata),.data_out(regs[25]));
	 
	 pcreg pcreg_26(
	 .clk(clk),.rst(rst),.ena(temp[26]),
	 .data_in(wdata),.data_out(regs[26]));
	 
	 pcreg pcreg_27(
	 .clk(clk),.rst(rst),.ena(temp[27]),
	 .data_in(wdata),.data_out(regs[27]));
	 
	 pcreg pcreg_28(
	 .clk(clk),.rst(rst),.ena(temp[28]),
	 .data_in(wdata),.data_out(regs[28]));
	 
	 pcreg pcreg_29(
	 .clk(clk),.rst(rst),.ena(temp[29]),
	 .data_in(wdata),.data_out(regs[29]));
	 
	 pcreg pcreg_30(
	 .clk(clk),.rst(rst),.ena(temp[30]),
	 .data_in(wdata),.data_out(regs[30]));
	 
	 pcreg pcreg_31(
	 .clk(clk),.rst(rst),.ena(temp[31]),
	 .data_in(wdata),.data_out(regs[31]));
	 
	 
	 mux mux_0(
	 .a(regs[0]),.b(regs[1]),.c(regs[2]),.d(regs[3]),.e(regs[4]),.f(regs[5]),.g(regs[6]),.h(regs[7]),.i(regs[8]),.j(regs[9]),
	 .k(regs[10]),.l(regs[11]),.m(regs[12]),.n(regs[13]),.o(regs[14]),.p(regs[15]),.q(regs[16]),.r(regs[17]),.s(regs[18]),
	 .t(regs[19]),.u(regs[20]),.v(regs[21]),.w(regs[22]),.x(regs[23]),.y(regs[24]),.z(regs[25]),.A(regs[26]),.B(regs[27]),
	 .C(regs[28]),.D(regs[29]),.E(regs[30]),.F(regs[31]),.addr(raddr1),.R(rdata1));
	 
	 
	 mux mux_1(
	 .a(regs[0]),.b(regs[1]),.c(regs[2]),.d(regs[3]),.e(regs[4]),.f(regs[5]),.g(regs[6]),.h(regs[7]),.i(regs[8]),.j(regs[9]),
	 .k(regs[10]),.l(regs[11]),.m(regs[12]),.n(regs[13]),.o(regs[14]),.p(regs[15]),.q(regs[16]),.r(regs[17]),.s(regs[18]),
	 .t(regs[19]),.u(regs[20]),.v(regs[21]),.w(regs[22]),.x(regs[23]),.y(regs[24]),.z(regs[25]),.A(regs[26]),.B(regs[27]),
	 .C(regs[28]),.D(regs[29]),.E(regs[30]),.F(regs[31]),.addr(raddr2),.R(rdata2));
	
	 

endmodule



module decoder(
	input [ 4:0 ] data_in,
	input ena,
	output [ 31:0 ] data_out
	);
	
		reg[ 31:0 ] data_temp;
		assign data_out = data_temp;
		always @( ena or data_in ) 
		begin
			if ( ena == 1 )
				case ( data_in )
					5'b00000:
						data_temp = 32'b00000000000000000000000000000001;
					5'b00001:
						data_temp = 32'b00000000000000000000000000000010;
					5'b00010:
						data_temp = 32'b00000000000000000000000000000100;
					5'b00011:
						data_temp = 32'b00000000000000000000000000001000;
					5'b00100:
						data_temp = 32'b00000000000000000000000000010000;
					5'b00101:
						data_temp = 32'b00000000000000000000000000100000;
					5'b00110:
						data_temp = 32'b00000000000000000000000001000000;
					5'b00111:
						data_temp = 32'b00000000000000000000000010000000;
					5'b01000:
						data_temp = 32'b00000000000000000000000100000000;
					5'b01001:
						data_temp = 32'b00000000000000000000001000000000;
					5'b01010:
						data_temp = 32'b00000000000000000000010000000000;
					5'b01011:
						data_temp = 32'b00000000000000000000100000000000;
					5'b01100:
						data_temp = 32'b00000000000000000001000000000000;
					5'b01101:
						data_temp = 32'b00000000000000000010000000000000;
					5'b01110:
						data_temp = 32'b00000000000000000100000000000000;
					5'b01111:
						data_temp = 32'b00000000000000001000000000000000;
				 	5'b10000:
						data_temp = 32'b00000000000000010000000000000000;
					5'b10001:
						data_temp = 32'b00000000000000100000000000000000;
					5'b10010:
						data_temp = 32'b00000000000001000000000000000000;
					5'b10011:
						data_temp = 32'b00000000000010000000000000000000;
					5'b10100:
						data_temp = 32'b00000000000100000000000000000000;
					5'b10101:
						data_temp = 32'b00000000001000000000000000000000;
					5'b10110:
						data_temp = 32'b00000000010000000000000000000000;
					5'b10111:
						data_temp = 32'b00000000100000000000000000000000;
					5'b11000:
						data_temp = 32'b00000001000000000000000000000000;
					5'b11001:
						data_temp = 32'b00000010000000000000000000000000;
					5'b11010:
						data_temp = 32'b00000100000000000000000000000000;	
					5'b11011:
						data_temp = 32'b00001000000000000000000000000000;
					5'b11100:
						data_temp = 32'b00010000000000000000000000000000;
					5'b11101:
						data_temp = 32'b00100000000000000000000000000000;	
					5'b11110:
						data_temp = 32'b01000000000000000000000000000000;
					5'b11111:
						data_temp = 32'b10000000000000000000000000000000;
				endcase
			else
				data_temp = 32'b00000000000000000000000000000000;
		end
	
endmodule


module mux(
		input[31:0]a, 
		input[31:0]b, 
		input[31:0]c, 
		input[31:0]d, 
		input[31:0]e, 
		input[31:0]f, 
		input[31:0]g, 
		input[31:0]h, 
		input[31:0]i, 
		input[31:0]j, 
		input[31:0]k, 
		input[31:0]l, 
		input[31:0]m, 
		input[31:0]n, 
		input[31:0]o, 
		input[31:0]p, 
		input[31:0]q, 
		input[31:0]r, 
		input[31:0]s, 
		input[31:0]t, 
		input[31:0]u, 
		input[31:0]v, 
		input[31:0]w, 
		input[31:0]x, 
		input[31:0]y, 
		input[31:0]z, 
		input[31:0]A, 
		input[31:0]B, 
		input[31:0]C, 
		input[31:0]D, 
		input[31:0]E, 
		input[31:0]F,
		input [4:0] addr,
		output[31:0] R
			) ;
		
		reg [31:0] data_temp;
		assign R = data_temp;
      always @ ( addr or a or b or c or d or e or f or g or h or i or j or k or l or m
                   or n or o or p or q or r or s or t or u or v or w or x or y or z or A or B or C or D or E or F )
		begin
		if ( 1 )
			case ( addr )
				5'b00000:
					data_temp = a;
				5'b00001:
					data_temp = b;
				5'b00010:
					data_temp = c;
				5'b00011:
					data_temp = d;
				5'b00100:
					data_temp = e;
				5'b00101:
					data_temp = f;
				5'b00110:
					data_temp = g;
				5'b00111:
					data_temp = h;
				5'b01000:
					data_temp = i;			
				5'b01001:
					data_temp = j;
				5'b01010:
					data_temp = k;
				5'b01011:
					data_temp = l;
				5'b01100:
					data_temp = m;
				5'b01101:
					data_temp = n;
			 	5'b01110:
					data_temp = o;
				5'b01111:
					data_temp = p;
				5'b10000:
					data_temp = q;
				5'b10001:
					data_temp = r;
				5'b10010:
					data_temp = s;
				5'b10011:
					data_temp = t;
				5'b10100:
					data_temp = u;
				5'b10101:
					data_temp = v;
				5'b10110:
					data_temp = w;
				5'b10111:
					data_temp = x;
				5'b11000:
					data_temp = y;
				5'b11001:
					data_temp = z;
				5'b11010:
					data_temp = A;	
				5'b11011:
					data_temp = B;
				5'b11100:
					data_temp = C;
				5'b11101:
					data_temp = D;	
				5'b11110:
					data_temp = E;
				5'b11111:
					data_temp = F;
			endcase
		end

endmodule




module pcreg(
					input clk, 
					input rst,
					input ena,
					input [31:0] data_in,
					output reg [31:0] data_out
    );
	 
     always @ (negedge clk)
	      begin
	         
			if ( rst == 1)
				 data_out = 0;
			else if ( ena == 1)
	             data_out = data_in;
			else
			   ;
	        end
	 
			
endmodule

