`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    02:12:06 11/17/2015 
// Design Name: 
// Module Name:    alu 
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
module alu(
	input[31:0] a,
	input[31:0] b,
	input[3:0] aluc,
	output[31:0] r,
	output zero,
	output carry,
	output negative,
	output overflow
    );
	
	 
	 //和或等运算结果
	 wire[31:0] d_and = a & b;
	 wire[31:0] d_or = a | b;
	 wire[31:0] d_xor = a ^ b;
	 wire[31:0] d_nor = ~(a | b);  
	 wire[31:0] d_r;  //和或等运算结果
	 mux4x32 mux4x32_d ( .a(d_and), .b(d_or), .c(d_xor), .d(d_nor), .addr(aluc[1:0]), .r(d_r) );
	 
	 
	 
	 //移位运算
	 wire[31:0] m_r;  //移位运算结果
	 wire m_c;   //移位运算进位
	 barrelshifter32 barrelshifter32 ( .a(a), .b(b), .aluc(aluc[1:0]), .c(m_r), .d(m_c) );
	 
	 
	 
	 //有无符号数加减
	 wire[31:0] as_r;
	 wire as_c;
	 wire as_o;
	 addsub32 addsub32 ( .a(a), .b(b), .aluc(aluc[1:0]), .c(as_c), .o(as_o), .s(as_r) );
	 
	 
	 //比较
	 wire[31:0] com_r;
	 wire com_c;
	 compare compare ( .a(a), .b(b), .aluc(aluc[1:0]), .c(com_c), .s(com_r) );
	 
	 //
	 wire[31:0] as_r1, d_r1, com_r1, m_r1;
	 assign as_r1 = as_r;
	 assign d_r1 = d_r;
	 assign com_r1 = com_r;
	 assign m_r1 = m_r;
	 
	 //输出结果r
	 mux4x32 mux4x32_r ( .a(as_r1), .b(d_r1), .c(com_r1), .d(m_r1), .addr(aluc[3:2]), .r(r) );
	 //输出overflow标志位
	 assign overflow = ( aluc[3:0] == 4'b0010 | aluc[3:0] == 4'b0011 ) ? as_o : (1'b0);
	 //输出carry标志位
	 reg cy;
	 assign carry = cy;
	 always @ (*)begin
	 if ( aluc[3:2] == 2'b11 )
		cy = m_c;
	 else if ( aluc[3:1] == 2'b000 )
		cy = as_c;
	 else if ( aluc[3:0] == 4'b1010 )
		cy = com_c;
	 else
		cy = 0;
	 end
	 //assign carry = ( aluc[3] & aluc[2] ) ? m_c : as_c; 
	 //输出negative标志位
	 assign negative = ( aluc[3:1] == 3'b000 ) ? (1'b0) : (r[31]);
	 //输出zero标志位
	 assign zero = ~|r;
	 
	 


endmodule




//mux4x32
module mux4x32(
	input[31:0] a,
	input[31:0] b,
	input[31:0] c,
	input[31:0] d,
	input[1:0] addr,
	output reg[31:0] r
	);
	always @ ( addr or a or b or c or d )
	begin
		case(addr)
			2'b00:
				r = a;
			2'b01:
				r = b;
			2'b10:
				r = c;
			2'b11:
				r = d;
		endcase
	end
endmodule

//add&sub
module addsub32(
	input[31:0] a,
	input[31:0] b,
	input[1:0] aluc,
	output c,  //carry
	output o, //overflow
	output [31:0] s
	);
   reg otemp, ctemp;
	reg [32:0] rtemp;
	assign s = rtemp[31:0];
	assign c = ctemp;
	assign o = otemp;
	always @(*)
	begin
		case(aluc[1:0])
		   //无符号加法
			2'b00:
			begin
				rtemp = {{1'b0}, a[31:0]};
				rtemp = rtemp + b;
				ctemp = rtemp[32];
			end
			//有符号加法
			2'b10:
			begin	
				rtemp = a + b;
				otemp =  (~a[31]&~b[31]&rtemp[31]) + (a[31]&b[31]&~rtemp[31]);
				rtemp = otemp ? 0 : rtemp;
			end
			//	无符号减法
			2'b01:
			begin
				rtemp = {{1'b0}, a[31:0]};
				rtemp = rtemp - b;
				ctemp = rtemp[32];
			end
			//有符号减法
			2'b11:
			begin	
				rtemp = a - b;
				otemp =  (~a[31]&~b[31]&rtemp[31]) + (a[31]&b[31]&~rtemp[31]);
				rtemp = otemp ? 0 : rtemp;
		   end
			default:;			
		endcase
	end
endmodule


//barrelshifter32
module barrelshifter32(
     input [31:0]a,
	  input [31:0]b,
	  input[1:0]aluc,
	  output [31:0]c,
	  output reg d	  
    );	
	reg[33:0] ctemp;
	assign c = ctemp[32:1];
	 always @ (*)
	 begin
		if ( aluc[1] == 1 )begin
			ctemp[33] = 0;
			ctemp[32:1] = b;
			ctemp[33:1] = ctemp[33:1] << a[4:0];
			d = ctemp[33];
		
		end else if ( aluc[1:0] == 2'b01 )begin
			ctemp[0] = 0;
			ctemp[32:1] = b;
			ctemp[32:0] = ctemp[32:0] >> a[4:0];
			d = ctemp[0];
		end else begin
			ctemp[0] = 0;
			ctemp[32:1] = b;
			ctemp[32:0] = $signed(ctemp[32:0]) >>> a[4:0];
			d = ctemp[0];
			end
	end
endmodule


//compare
module compare(
	input[31:0] a,
	input[31:0] b,
	input[1:0] aluc,
	output reg c,
	output [31:0] s
	);
	reg [31:0] r;
	assign s = r;
	always @(*)
	begin
		if ( aluc[1:0] == 2'b10 )begin
			r = (a < b) ? 1 : 0;
			c = r[0];
		end else if ( aluc[1:0] == 2'b11 )begin
			r = (a < b) ? 1 : 0;
			c = 0;
		end else begin
			r = {b[15:0], {16{1'b0}}};
			c = 0;
		end
	end
endmodule

