// A single port ROM indeed
module instMemory #(parameter instructionW=32, parameter addrW=16) (
	output reg [(instructionW-1):0]inst,
	input [(addrW-1):0] pcVal,
	input sysCLK
);
	reg [instructionW-1:0] instructionROM[2**addrW-1:0];
	
	initial begin
		$readmemh("C:/Users/Faizan Ahmad/Documents/Education/Y2Ss 2019/Internship/RISCV-32BitCPU/VerilogHDL/rom_instruction_memory.txt", instructionROM);
	end

	always @ (posedge sysCLK) begin
		inst <= instructionROM[pcVal];
	end
endmodule


// DATA MEMORY
module dataMemory #(parameter dataW=32, parameter addrW=16) (
	output [(dataW-1):0] memDataR,
	input  [(dataW-1):0] memDataW,
	input  [(addrW-1):0] addrD,
	input  memRW, sysCLK
);
	genericRAM #(dataW, addrW) dmem(
		.Q(memDataR), .dataIN(memDataW), .addr(addrD), 
		.pCLK(sysCLK), .enWR(memRW)
	);

endmodule


/* Control Path
    ControlPathDriver #(controlWordWidth, parameter caddrW)(
        .PCSel(), .ImmSel(), .BrUn(), .ASel(), .BSel(), .ALUSel(),
        .MemRW(), .RegWEn(), .WBSel(), .inst32(), .BrEq(), .BrLt(), .sysCLK()
    );
*/
/*module ControlPath32v1 #(parameter controlWordWidth=16, parameter caddrW=16)(
    output [14:0] controlWord,
    input  [11:0] instrBits,
    input  sysCLK
);
    // preparing access location address
    wire [15:0] controlAddr;
    assign controlAddr[11:0]    = instrBits[11:0];
    assign controlAddr[12]      = instrBits[11]; //sign extension
    assign controlAddr[13]      = instrBits[11]; 
    assign controlAddr[14]      = instrBits[11];
    assign controlAddr[15]      = instrBits[11];

    // Accessing Storage
    reg [(controlWordWidth-1):0] controlWord;
    reg [(controlWordWidth-1):0] controlrom[((2**caddrW)-1):0];
	
	initial begin
		$readmemh("C:/Users/Faizan Ahmad/Documents/Education/Y2Ss 2019/Internship/RISCV-32BitCPU/VerilogHDL/rom_control_words_memory.txt", controlrom);
	end

	always @ (posedge sysCLK) begin
		controlWord <= controlrom[controlAddr];
	end
endmodule*/

module controlPathDriver #(parameter controlWordWidth=16, parameter caddrW=16) (
    output PCSel,
    output [2:0] ImmSel,
    output BrUn, ASel, BSel,
    output [3:0] ALUSel,
    output MemRW, RegWEn,
    output [1:0] WBSel,
    input  [31:0] inst32,
    input  BrEq, BrLt,
    input  sysCLK
);

	wire [14:0] rawControlWord;	// one word to drive 'em all
	wire [11:0] rawInstCodes;	// all the bits that matter
	assign rawInstCodes[11:0] = {inst32[30], inst32[14:12], inst32[6:2], BrEq, BrLt};

	ControlPath32v2 callCP(rawControlWord[14:0], rawInstCodes[11:0], sysCLK);

	// Preparing Controls to output
    assign PCSel        = rawControlWord[14];
    assign ImmSel[2:0]  = rawControlWord[13:11];
    assign BrUn         = rawControlWord[10];
    assign ASel         = rawControlWord[9];
    assign BSel         = rawControlWord[8];
    assign ALUSel[3:0]  = rawControlWord[7:4];
    assign MemRW        = rawControlWord[3];
    assign RegWEn       = rawControlWord[2];
    assign WBSel[1:0]   = rawControlWord[1:0];

endmodule

module ControlPath32v2 (
    output reg [14:0] controlWord,
    input  [11:0] instrBits,
    input  sysCLK
);
	/*-------------------------------
		L-Type		 0		00000
		I-Type		 4		00100
		Iw-Type		 6		00110
		S-Type		 8		01000
		R-Type		12		11000
		Rw-Type		14		01110
		SB-Type		24		11000
	-------------------------------*/
	
	initial controlWord = 15'b0;
	always@(posedge sysCLK) begin
		case(instrBits[6:2] /*OPCODE*/)
		//PCSel, ImmSel, BrUn, ASel, BSel, ALUSel, MemRW, RegWEn, WBSel
			5'b11000: controlWord <= {1'b0, 3'b000/*x*/, 1'b0/*X*/, 1'b0, 1'b0, 4'b0000, 1'b0, 1'b1, 1'b0}; //R
			5'b00100: controlWord <= {1'b0, 3'b000, 1'b0/*x*/, 1'b0, 1'b1, 4'b0000, 1'b0, 1'b1, 1'b0}; //I
			default: controlWord <= 15'b0;
		endcase
	end
endmodule

// register File
// a RAM with single port in, dual port out
module registerFile #(parameter dataWidth=32, parameter addrWidth=5) (
	output reg [(dataWidth-1):0] regDataA, regDataB,	// output RS1, RS2
	input  [(dataWidth-1):0] regDataD,					// input RD
	input  [(addrWidth-1):0] addrA, addrB, addrD,		// register Addresses
	input  regWEn, sysCLK
);
	// instantiate memory
	reg [(dataWidth-1):0]regFile[((2**addrWidth)-1):0];

	// regFile initializer
	initial begin
		$readmemh("C:/Users/Faizan Ahmad/Documents/Education/Y2Ss 2019/Internship/RISCV-32BitCPU/VerilogHDL/mem_register_file.txt", regFile);
	end

	always@(posedge sysCLK) begin
		regDataA <= regFile[addrA];
		regDataB <= regFile[addrB];
		if(regWEn && addrD != 0) begin // don't write to x0;
			regFile[addrD] <= regDataD;
		end
	end

endmodule

/* -----------------------------------------------------------------------
8-bit Register using D flops
Instantiation: registerNx #(width) name(.Q(), .D(), .regCLK(), .regRESN());
https://github.com/fayzanx/FPGA-VerilogHDL-Course/blob/cfc8c870b1298050ed9b8463b5b1c7bde273158c/B_memoryFlops.v
*/
module registerNx #(parameter regWidth=8)(
	output [regWidth-1 : 0]Q,
	input  [regWidth-1 : 0]D,
	input  regCLK, regRESN
);
	//parameter regWidth=8;
	genvar i;
	generate
		for(i=0; i<regWidth; i=i+1) begin: m
			mem_Dflippos df(Q[i], D[i], regCLK, regRESN);
		end //for
	endgenerate
endmodule

/* 
POSITIVE EDGE TRIGGERED - D FLIP FLOP
https://github.com/fayzanx/FPGA-VerilogHDL-Course/blob/cfc8c870b1298050ed9b8463b5b1c7bde273158c/B_memoryFlops.v
*/
module mem_Dflippos(
	output reg Q,
	input D, clk, resetN
);
	always@(posedge clk or negedge resetN)
	begin
		if(~resetN) begin
			Q <= 0;
		end //if
		else begin
			Q <= D;
		end
	end
endmodule

/*
A generic RAM module for memory without lpm
Instantiation: genericRAM #(dataW, addrW) ram_inst(.Q(), .dataIN(), .addr(), .pCLK(), .enWR());
https://github.com/fayzanx/FPGA-VerilogHDL-Course/blob/cfc8c870b1298050ed9b8463b5b1c7bde273158c/B_memoryFlops.v
*/
module genericRAM #(parameter dataW = 8, parameter addrW = 5) (
	output [(dataW-1):0]Q,
	input  [(dataW-1):0]dataIN,
	input  [(addrW-1):0]addr,
	input  pCLK, enWR
);
	reg [(dataW-1):0]storageData[((2**addrW)-1):0];
	reg [(addrW-1):0]storageAddr;
	always@(posedge pCLK) begin
		if(enWR == 1'b1) begin
			storageData[addr] <= dataIN;
		end
			storageAddr <= addr;
	end
	assign Q = storageData[storageAddr];
endmodule