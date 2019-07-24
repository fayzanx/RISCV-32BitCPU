module mainProcessor(
    output [31:0]S,
    input  [31:0]A, B,
    input  [3:0]ALUSel,
    input  mainCLOCK
);

    wire [31:0] instr32;
    wire [15:0] PC16;

    programCounter #(16) myPC(.PC(PC16), .sysCLK(mainCLOCK), .pRST(1'b0));
    instMemory #(32, 16) myIMEM(.inst(instr32), .pcVal(PC16), .sysCLK(mainCLOCK));    

    // decode the instruction
    wire [6:0] instOPCODE;
    wire [4:0] rs1addr, rs2addr, rdaddr;

    // label different parts of the instruction
    assign instOPCODE[6:0] = instr32[6:0];
    assign rdaddr[4:0] = instr32[11:7];
    assign rs1addr[4:0] = instr32[19:15];
    assign rs2addr[4:0] = instr32[24:20];

    // register value labels
    wire [31:0] dataRS1, dataRS2, dataRD;
    registerFile #(32, 5) riscRegisters(
        .regDataA(dataRS1), .regDataB(dataRS2), .regDataD(dataRD), .addrA(rs1addr),
        .addrB(rs2addr), .addrD(rdaddr), .regWEn(), .sysCLK(mainCLOCK)
    );

    // feed data to ALU
    wire [31:0] aluOut;
    mainALU #(32) testALU(
        .outALU(aluOut), .inALUa(dataRS1), .inALUb(dataRS2), .ALUSel()
    );

    // memory
    wire [31:0] dataRead;
    dataMemory #(32, 16) myDMEM(
        .memDataR(dataRead), .memDataW(dataRS2), .addrD(aluOut), .memRW(), .sysCLK(mainCLOCK)
    );

    // write back stage
    reg [31:0] writeBackData;
    always@() begin
        case(WBSel)
            2'b00: writeBackData = aluOut;
            2'b01: writeBackData = dataRead;
            //2'b10: writeBackData = PC+4
            default: writeBackData = 32'h0;
        endcase
    end

endmodule

// posedge counter with async reset
module programCounter #(parameter counterWidth = 16)(
    output reg [(counterWidth-1):0]PC,
    input  sysCLK, pRST
);
    always@(posedge sysCLK or posedge pRST) begin
        if (pRST) begin
            PC <= 1'b0;
        end else begin
            PC <= PC + 1'b1;
        end
    end
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
        if (Sel) begin
            M = Y;
        end
        else begin
            M = X;
        end
    end
endmodule

module testBenchGen;

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
	 end
*/