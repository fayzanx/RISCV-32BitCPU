// A single port ROM indeed
module instMemory #(parameter instructionW=32, parameter addrW=16) (
	output reg [(instructionW-1):0]inst,
	input [(addrW-1):0] pcVal,
	input sysCLK
);
	reg [instructionW-1:0] rom[2**addrW-1:0];
	
	initial begin
		$readmemh("rom_instruction_memory.txt", rom);
	end

	always @ (posedge sysCLK) begin
		inst <= rom[pcVal];
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
		.Q(memDataR),
		.dataIN(memDataW), 
		.addr(addrD), 
		.pCLK(sysCLK), 
		.enWR(memRW)
	);

endmodule


/* Control Path
    ControlPath32 #(controlWordWidth, parameter caddrW)(
        .PCSel(), .ImmSel(), .BrUn(), .ASel(), .BSel(), .ALUSel(),
        .MemRW(), .RegWEn(), .WBSel(), .inst32(), .BrEq(), .BrLt(), .sysCLK()
    );
*/
module ControlPath32 #(parameter controlWordWidth=16, parameter caddrW=16)(
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
    // preparing access location address
    wire [15:0] controlAddr;
    assign controlAddr[11:0]    = {inst32[30], inst32[14:12], inst32[6:2], BrEq, BrLt};
    assign controlAddr[12]      = inst32[30]; //sign extension
    assign controlAddr[13]      = inst32[30]; 
    assign controlAddr[14]      = inst32[30];
    assign controlAddr[15]      = inst32[30];


    // Accessing Storage
    reg [(controlWordWidth-1):0] controlWord;
    reg [(controlWordWidth-1):0] controlrom[((2**caddrW)-1):0];
	
	initial begin
		$readmemh("rom_control_words_memory.txt", controlrom);
	end

	always @ (posedge sysCLK) begin
		controlWord <= controlrom[controlAddr];
	end

    // Preparing Controls to output
    assign PCSel        = controlWord[14];
    assign ImmSel[2:0]  = controlWord[13:11];
    assign BrUn         = controlWord[10];
    assign ASel         = controlWord[9];
    assign BSel         = controlWord[8];
    assign ALUSel[3:0]  = controlWord[7:4];
    assign MemRW        = controlWord[3];
    assign RegWEn       = controlWord[2];
    assign WBSel[1:0]   = controlWord[1:0];
    
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
	/*initial begin
		$readmemb("mem_register_file.txt", regFile);
	end*/

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