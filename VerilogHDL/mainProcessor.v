module mainProcessor(
        output [31:0] instr32,
        output [31:0] PC32,
        output [31:0] immidiate32b,
        output [31:0] dataRS1, dataRS2, aluOut, ALUinA, ALUinB,
        output [4:0]  rs1addr, rs2addr, rdaddr,
        input  [3:2]  KEY
);

    //-----------------
        wire mainCLOCK;
        assign mainCLOCK = ~KEY[3];
    //-----------------


    //wire [31:0] instr32;
    //wire [31:0] PC32;
    wire [15:0] PC16, PC16Next;
    assign PC32= {16'b0, PC16};

    // Control Path Outputs
    wire PCSel;
    wire [2:0] ImmSel;
    wire BrUn, ASel, BSel;
    wire [3:0] ALUSel;
    wire MemRW, RegWEn;
    wire [1:0] WBSel;

    // Comparator Outputs
    wire BrLt, BrEq;

    // WriteBack stage
    reg [31:0] writeBackData;

    // ---- INSTRUCTION FETCH ----
    programCounter #(16) myPC(
        .PC(PC16), .PCNext(PC16Next), .ALUout(aluOut), .PCSel(PCSel), .sysCLK(mainCLOCK), .pRST(~KEY[2])
    );

    instMemory #(32, 4) myIMEM(.inst(instr32), .pcVal(PC16[3:0]), .sysCLK(mainCLOCK));    
    // ---------------------------


    // ---- THE CONTROL PATH ----
    controlPathDriver #(16, 16) myCP(
        PCSel, ImmSel, BrUn, ASel, BSel, ALUSel, MemRW, RegWEn, WBSel, instr32, BrEq, BrLt, mainCLOCK
    );
    // ---------------------------



    // ---- INSTRUCTION DECODE ----
    // label different parts of the instruction
    wire [6:0] instOPCODE;
    //wire [4:0] rs1addr, rs2addr, rdaddr;
    wire [2:0] funct3;
    wire [6:0] funct7;
    splitInstruction getCodes(instOPCODE, rdaddr, rs1addr, rs2addr, funct3, funct7, instr32);

    // generate immidiates
    //wire [31:0] immidiate32b;
    immideateGen #(32) immInst(.imm32(immidiate32b), .instr(instr32), .immSelect(ImmSel));
    // ---------------------------


    // register value labels
    //wire [31:0] dataRS1, dataRS2;//, dataRD;
    registerFile #(32, 5) riscRegisters(
        .regDataA(dataRS1), .regDataB(dataRS2), .regDataD(writeBackData), .addrA(rs1addr),
        .addrB(rs2addr), .addrD(rdaddr), .regWEn(RegWEn), .sysCLK(mainCLOCK)
    );


    // ---- INSTRUCTION EXECUTE ----
    // ALU muxers
    //wire [31:0] ALUinA, ALUinB;
    mux2x1 #(32) aluinputMUX1(.M(ALUinA), .X(dataRS1), .Y(PC32), .Sel(ASel));
    mux2x1 #(32) aluinputMUX2(.M(ALUinB), .X(dataRS2), .Y(immidiate32b), .Sel(BSel));

    // feed data to ALU
    //wire [31:0] aluOut;
    mainALU #(32) testALU(
        .outALU(aluOut), .inALUa(ALUinA), .inALUb(ALUinB), .ALUSel(ALUSel)
    );

    // comparator
    comparatorNx #(32) compAB(
        .BrLt(BrLt), .BrEq(BrEq), .A(dataRS1), .B(dataRS2)
    );
    // ---------------------------


    // ---- MEMORY ----
    wire [31:0] dataRead;
    dataMemory #(32, 16) myDMEM(
        .memDataR(dataRead), .memDataW(dataRS2), .addrD(aluOut[15:0]), .memRW(MemRW), .sysCLK(mainCLOCK)
    );
    // ---------------------------


    // ---- DATA WRITE BACK STAGE ----
    always@(*) begin
        case(WBSel)
            2'b00: writeBackData = aluOut;
            2'b01: writeBackData = dataRead;
            2'b10: writeBackData = {16'b0, PC16Next};
            default: writeBackData = 32'h0;
        endcase
    end
    // ---------------------------

endmodule


// posedge counter with async reset
// programCounter #(pcWidth) myPC(.PC(), .PCNext(), .ALUout(), .PCSel(), .sysCLK(), .pRST());
module programCounter #(parameter counterWidth = 16)(
    output reg [(counterWidth-1):0]PC,
    output reg [(counterWidth-1):0]PCNext,
    input  [31:0]ALUout,
    input  PCSel,
    input  sysCLK, pRST
);
    //reg [(counterWidth-1):0] PCplus;
    initial PC = 0;

    always@(posedge sysCLK or posedge pRST) begin
        //PCplus = PC + 16'd1;
        if (pRST) begin
            PC <= 0;
        end else begin
            //case(PCSel)
                /*1'b0:*/ PC <= PC + 1; //PCNext = PC + 16'd1;
                //1'b1: /*PCNext =*/ PC <= ALUout[(counterWidth-1):0];
                //default: PCNext = 0;
            //endcase
            //PC <= PCNext;
        end
    end
endmodule


// comparator
// comparatorNx #(compWidth) (.BrLt(), .BrEq(), .A(), .B());
module comparatorNx #(parameter cW = 32) (
    output reg BrLt, BrEq,
    input  [(cW-1):0] A, B
);
    always@(*) begin
        BrLt = 0;
        BrEq = 0;
        if(A < B)
            BrLt = 1;
        if (A == B)
            BrEq = 1;
    end
endmodule



// immideate generator
// Instantiation: immideateGen #(instWidth) immInst(.imm32(), .instr(), .immSelect());
module immideateGen #(parameter instWidth = 32) (
    output reg [(instWidth-1):0] imm32,
    input  [(instWidth-1):0] instr,
    input  [2:0] immSelect
);
    // decoding inst into parts
    wire [6:0] opcode;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    wire [6:0] funct7;

    splitInstruction getParts(opcode, rd, rs1, rs2, funct3, funct7, instr);

    reg [31:0]imm_I, imm_S, imm_SB, immU;
    always@(*) begin
        case(immSelect)
            3'b000: imm32 = {20'b0, funct7, rs2}; //IMM_I
            3'b001: imm32 = {20'b0, funct7, rd}; //IMM_S
            3'b010: imm32 = {19'b0, funct7[6], rd[0], funct7[5:0], rd[4:1], 1'b0}; //IMM_SB
            3'b011: imm32 = {funct7, rs2, rs1, funct3, 12'b0}; //IMM_U
            default: imm32 = 32'hAAAAAAAA;
        endcase
    end
endmodule



// splitting the instruction
module splitInstruction (
    output [6:0] opcode,
    output [4:0] rd, rs1, rs2,
    output [2:0] funct3,
    output [6:0] funct7,
    input  [31:0] instr
);

    assign opcode[6:0] = instr[6:0];
    assign rd[4:0] = instr[11:7]; 
    assign funct3[2:0] = instr[14:12];
    assign rs1[4:0] = instr[19:15];
    assign rs2[4:0] = instr[24:20];
    assign funct7[6:0] = instr[31:25];

endmodule


/*  A Generalized 2x1 MUX
https://github.com/fayzanx/FPGA-VerilogHDL-Course/blob/cfc8c870b1298050ed9b8463b5b1c7bde273158c/B_muxers.v
    Instantiation: mux2x1 #(width) name(.M(), .X(), .Y(), .Sel());    */
module mux2x1 #(parameter dataW=16)(
    output reg [(dataW-1):0]M,
    input  [(dataW-1):0]X,
    input  [(dataW-1):0]Y,
    input  Sel
); 
    always@(*) begin
        case (Sel)
            1'b0: M = X;
            1'b1: M = Y;
            default: M = 0;
        endcase
    end
endmodule


module testBenchGen;
    /*  VERIFICATION -> PROCESSOR   */
    reg  clk;
    reg  resetPC;
    wire [31:0] instruction, RS1, RS2, OutALU, inALUa, inALUb, pc, immideate;
    wire [4:0] addrRS1, addrRS2, addrRD;
    mainProcessor testTheProcessor(
        instruction, pc, immideate, RS1, RS2, OutALU, inALUa, inALUb,
        addrRS1, addrRS2, addrRD,
        {clk, resetPC}
    );

    initial begin
        clk = 1'b0; //resetPC = 1'b0; #50;
        resetPC = 1'b1;
        forever #50 clk = ~clk;
    end
endmodule

/*  VERIFICATION -> ALU
------------------------
    wire [31:0]resAB;
    reg  [31:0]opA, opB;
    reg  [3:0]opSel;
    mainProcessor testProcessor(resAB, opA, opB, opSel);
    integer A, B, S;
	 initial begin
		 for(A = 1; A < 9; A=A+1) begin
			  for(B=1; B < 19; B=B+1) begin
					for(S=0; S<13; S=S+1) begin
						 opA = A; opB = B; opSel = S; #100;
					end
			  end
		 end
	 end */

/*  VERIFICATION -> IMEM
-------------------------
module testTheIMEM(
    output [31:0] INST,
    input  [15:0] INDEX,
    input  CLK
);
    instMemory #(32, 4) imemtest(INST, INDEX[3:0], CLK);
endmodule

module testBenchGen;
    reg  clk;
    reg [16:0]Index;
    wire [31:0]instr;
    testTheIMEM timem(instr, Index[3:0], clk);
    initial begin
        clk = 1'b0;
        forever #50 clk = ~clk;
    end
    integer i;
    initial begin
        for(i=0; i<15; i=i+1) begin
            #100; Index = i;
        end
    end
endmodule*/