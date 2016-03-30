`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:35:07 12/12/2015 
// Design Name: 
// Module Name:    sccomp_dataflow 
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
module sccomp_dataflow(
	input clock,
	input resetn,
	//input mem_clk,
	output[31:0] inst,
	output[31:0] pc,
	output[31:0] aluout,
	output[31:0] memout
    );
	 wire[31:0] data;
	 wire wmem;
	 wire clk40;
	// hz newclk( .CLK_IN1(clock), .CLK_OUT1(clk40) );
	 sccpu_dataflow s( .clock(clock), .resetn(resetn), .inst(inst), .mem(memout), .pc(pc), .wmem(wmem), .alu(aluout), .data(data) );
	 dram d( .clk(clock), .data_in(data), .data_out(memout), .addr(aluout), .wena(wmem)/*, .ram_ena(~clk40)*/ );
	 imemip imem_ip ( .a(pc[11:2]), .spo(inst) ) ;
endmodule







//单周期cpu
module sccpu_dataflow(
	input[31:0] inst,
	input[31:0] mem,
	input clock,
	input resetn,
	output[31:0] pc,
	output[31:0] alu,
	output[31:0] data,
	output wmem
	);
	wire[31:0] p4, bpc, npc, adr, ra, alua, alub, res, alu_mem;
	wire[3:0] aluc;
	wire[4:0] reg_dest, wn;
	wire[1:0] pcsource;
	wire   zero, carry, negative, overflow, wreg, regrt, m2reg, shift, aluimm, jal, sext;
	wire[31:0] sa = {27'b0, inst[10:6]};
	sccu_dataflow cu( .op(inst[31:26]), .func(inst[5:0]), .z(zero), .wmem(wmem), .wreg(wreg), .regrt(regrt), .m2reg(m2reg),
								.aluc(aluc), .shift(shift), .aluimm(aluimm), .pcsource(pcsource), .jal(jal), .sext(sext) );
	wire e = sext & inst[15];
	wire[15:0] imm = {16{e}};
	wire[31:0] offset = {imm[13:0], inst[15:0], 2'b00};
	wire[31:0] immediate = {imm, inst[15:0]};
	//////////////////
	wire wreg1 = wreg & ~overflow;
	cla32 NPC ( .a(pc), .b(32'h4), .ci(1'b0), .r(p4) ); 
	cla32 br_adr ( .a(pc), .b(offset), .ci(1'b0), .r(adr) );
	wire[31:0] jpc = {p4[31:28], inst[25:0], 2'b00};
	mux2x32 alu_b ( .a(data), .b(immediate), .s(aluimm), .r(alub) );
	mux2x32 alu_a ( .a(ra), .b(sa), .s(shift), .r(alua) );
	alu al_unit ( .a(alua), .b(alub), .aluc(aluc), .r(alu), .zero(zero), .carry(carry), .negative(negative), .overflow(overflow) );
	mux2x32 result ( .a(alu), .b(mem), .s(m2reg), .r(alu_mem) );
	mux2x32 link ( .a(alu_mem), .b(p4+4), .s(jal), .r(res) );
	mux2x5 reg_wn ( .a(inst[15:11]), .b(inst[20:16]), .s(regrt), .r(reg_dest) );
	assign wn = reg_dest | {5{jal}};
	mux4x32 nextpc ( .a(p4), .b(adr), .c(ra), .d(jpc), .addr(pcsource), .r(npc) );
	dff32 ip ( .d(npc), .clk(clock), .clrn(resetn), .r(pc) );
	regfiles rf ( .clk(clock), .rst(resetn), .we(wreg1), .raddr1(inst[25:21]), .raddr2(inst[20:16]), 
		.waddr(wn), .wdata(res), .rdata1(ra), .rdata2(data) );
	
endmodule



//控制信号模块
module sccu_dataflow(
	input[5:0] op,
	input[5:0] func,
	input z,
	output wreg,
	output regrt,
	output jal, 
	output m2reg, 
	output shift,
	output aluimm,
	output sext,
	output wmem,
	output[3:0] aluc,
	output[1:0] pcsource
	);
	
	wire r_type = ~|op;
	wire i_add = r_type& func[5]& ~func[4]& ~func[3]& ~func[2]& ~func[1]& ~func[0];
	wire i_addu = r_type& func[5]& ~func[4]& ~func[3]& ~func[2]& ~func[1]& func[0];
	wire i_sub = r_type& func[5]& ~func[4]& ~func[3]& ~func[2]& func[1]& ~func[0];
	wire i_subu = r_type& func[5]& ~func[4]& ~func[3]& ~func[2]& func[1]& func[0];
	wire i_and = r_type& func[5]& ~func[4]& ~func[3]& func[2]& ~func[1]& ~func[0];
	wire i_or = r_type& func[5]& ~func[4]& ~func[3]& func[2]& ~func[1]& func[0];
	wire i_xor = r_type& func[5]& ~func[4]& ~func[3]& func[2]& func[1]& ~func[0];
	wire i_nor = r_type& func[5]& ~func[4]& ~func[3]& func[2]& func[1]& func[0];
	wire i_sll = r_type& ~func[5]& ~func[4]& ~func[3]& ~func[2]& ~func[1]& ~func[0];
	wire i_srl = r_type& ~func[5]& ~func[4]& ~func[3]& ~func[2]& func[1]& ~func[0];
	wire i_sra = r_type& ~func[5]& ~func[4]& ~func[3]& ~func[2]& func[1]& func[0];
	wire i_slt = r_type& func[5]& ~func[4]& func[3]& ~func[2]& func[1]& ~func[0];
	wire i_sltu = r_type& func[5]& ~func[4]& func[3]& ~func[2]& func[1]& func[0];
	wire i_sllv = r_type& ~func[5]& ~func[4]& ~func[3]& func[2]& ~func[1]& ~func[0];
	wire i_srlv = r_type& ~func[5]& ~func[4]& ~func[3]& func[2]& func[1]& ~func[0];
	wire i_srav = r_type& ~func[5]& ~func[4]& ~func[3]& func[2]& func[1]& func[0];
	wire i_jr = r_type& ~func[5]& ~func[4]& func[3]& ~func[2]& ~func[1]& ~func[0];
	wire i_addi = ~op[5]& ~op[4]& op[3]& ~op[2]& ~op[1]& ~op[0];
	wire i_addiu = ~op[5]& ~op[4]& op[3]& ~op[2]& ~op[1]& op[0];
	wire i_andi = ~op[5]& ~op[4]& op[3]& op[2]& ~op[1]& ~op[0];
	wire i_ori = ~op[5]& ~op[4]& op[3]& op[2]& ~op[1]& op[0];
	wire i_xori = ~op[5]& ~op[4]& op[3]& op[2]& op[1]& ~op[0];
	wire i_lw = op[5]& ~op[4]& ~op[3]& ~op[2]& op[1]& op[0];
	wire i_sw = op[5]& ~op[4]& op[3]& ~op[2]& op[1]& op[0];
	wire i_beq = ~op[5]& ~op[4]& ~op[3]& op[2]& ~op[1]& ~op[0];
	wire i_bne = ~op[5]& ~op[4]& ~op[3]& op[2]& ~op[1]& op[0];
	wire i_slti = ~op[5]& ~op[4]& op[3]& ~op[2]& op[1]& ~op[0];
	wire i_sltiu = ~op[5]& ~op[4]& op[3]& ~op[2]& op[1]& op[0];
	wire i_lui = ~op[5]& ~op[4]& op[3]& op[2]& op[1]& op[0];
	wire i_j = ~op[5]& ~op[4]& ~op[3]& ~op[2]& op[1]& ~op[0];
	wire i_jal = ~op[5]& ~op[4]& ~op[3]& ~op[2]& op[1]& op[0];
	
	
	assign wreg = i_add | i_addu | i_sub | i_subu | i_and | i_or | i_xor | i_nor | i_slt | i_sltu |
						i_sll | i_srl | i_sra | i_addi | i_addiu | i_andi | i_ori | i_xori |
						i_slti | i_sltiu | i_srav | i_sllv | i_srlv | i_lw | i_lui | i_jal;						
	assign regrt = i_addi | i_addiu | i_andi | i_ori | i_xori | i_slti | i_sltiu | i_lui | i_lw;					
	assign jal = i_jal;
	assign m2reg = i_lw;
	assign shift = i_sll | i_srl | i_sra;
	assign aluimm = i_addi | i_addiu | i_ori | i_xori | i_slti | i_sltiu | i_lw | i_sw | i_lui | i_andi;
	assign sext = i_addi | i_addiu| i_lw | i_sw | i_slti | i_beq | i_bne;
	assign aluc[3] = i_lui | i_slt | i_sltu | i_sll | i_srl | i_sra | i_slti | i_sltiu | i_srav | i_sllv | i_srlv;
	assign aluc[2] = i_and | i_andi | i_or | i_xor | i_nor | i_sra | i_sll | i_srl | i_ori | i_xori | i_srav | i_sllv | i_srlv | i_beq | i_bne;
	assign aluc[1] = i_add | i_sub | i_xor | i_nor | i_slt | i_sltu | i_sll | i_addi | i_xori | i_slti | i_sltiu | i_sllv | i_beq | i_bne | i_lw | i_sw;	
	assign aluc[0] = i_subu | i_sub | i_or | i_nor | i_slt | i_srl | i_ori | i_slti | i_srlv ;	
	assign wmem = i_sw;
	assign pcsource[1] = i_j | i_jr | i_jal;
	assign pcsource[0] = i_beq&z | i_bne&~z | i_j | i_jal;
endmodule
	


//DRAM
module dram(
		input clk,
		//input ram_ena,
		input wena,
		input[31:0] addr,
		input[31:0] data_in,
		output[31:0] data_out
		);
		wire ram_ena;
		assign ram_ena = ~clk;
		reg[31:0] ram[1023:0];
		assign data_out = ram_ena ? ram[addr[11:2]] : 32'bz;
		always @ ( negedge clk )begin
			if ( ram_ena == 1 && wena == 1 )
				ram[addr[11:2]] = data_in;
		end	
endmodule



//指令寄存器
module imem(
	input[31:0] a,
	output[31:0] inst
	);
	
	reg[31:0] ROM[1023:0];
	
	initial
		begin
			$readmemb ("test.txt", ROM);
		end
	assign inst = ROM[a[31:2]];
endmodule




//32位二选一数据选择器
module mux2x32(
	input[31:0] a,
	input[31:0] b,
	input s,
	output[31:0] r
	);
	assign r = s ? b : a;
endmodule


//5位二选一数据选择器
module mux2x5(
	input[4:0] a,
	input[4:0] b,
	input s,
	output[4:0] r
	);
	assign r = s ? b : a;
endmodule


//32位四选一数据选择器
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


//32位加法器
module cla32(
	input[31:0] a,
	input[31:0] b,
	input ci,
	output[31:0] r
	);
	assign r = a + b + {32{ci}};
endmodule


//D触发器(同步清零)
module dff32(
	input[31:0] d,
	input clk,
	input clrn,
	output[31:0] r
	);
	reg[31:0] result;
	assign r = result;
	always @ ( posedge clk )begin
		if ( clrn == 1 )
			result = 0;
		else
			result = d;
	end
endmodule
	




//寄存器堆
module regfiles(
	input clk,
	input rst,
	input we,
	input[4:0] raddr1,
	input[4:0] raddr2,
	input[4:0] waddr,
	input[31:0] wdata,
	output[31:0] rdata1,
	output[31:0] rdata2
	);
	wire[31:0]regs[31:0];
	wire[31:0]temp;
	decoder decoder(
	 .data_in(waddr),
	 .ena(we),
	 .data_out(temp));
	 
	 
	 pcreg pcreg_0(
	 .clk(clk),.rst(1'b1),.ena(temp[0]),
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

	

//alu运算器	
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




//add&sub
module addsub32(
	input[31:0] a,
	input[31:0] b,
	input[1:0] aluc,
	output c,  //carry
	output o, //overflow
	output [31:0] s
	);
   reg otemp;
	reg [32:0] rtemp;
	assign s = rtemp[31:0];
	assign c = rtemp[32];
	assign o = otemp;
	always @(*)
	begin
		case(aluc[1:0])
		   //无符号加法
			2'b00:
			begin
				rtemp = {{1'b0}, a[31:0]};
				rtemp = rtemp + b;
				otemp = 0;
//				ctemp = rtemp[32];
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
				otemp = 0;
//				ctemp = rtemp[32];
			end
			//有符号减法
			2'b11:
			begin	
				rtemp = a - b;
				otemp =  (a[31]&~b[31]&rtemp[31]) + (~a[31]&b[31]&~rtemp[31]);
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
		if ( aluc[1:0] == 2'b10 )
		begin
			r = (a < b) ? 1 : 0;
			c = r[0];
		end 
		
		else if ( aluc[1:0] == 2'b11 )
		begin
			if ( a[31] > b[31] )
			begin
				r = 1;
				c = 0;
			end
			else if ( a[31] + b[31] == 2 )
			begin
				r = (~a[30:0] > ~b[30:0]) ? 1 : 0;
				c = 0;
			end
			else if ( a[31] + b[31] == 0 )
			begin
				r = (a < b) ? 1 : 0;
				c = 0;
			end
			//if ( a[31] < b[31] )
			else
			begin
				r = 0;
				c = 0;
			end
		end 
		else 
		begin
			r = {b[15:0], {16{1'b0}}};
			c = 0;
		end
	end
endmodule
