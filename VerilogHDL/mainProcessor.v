module mainProcessor(
    output [31:0]S,
    input  [31:0]A, B,
    input  [3:0]ALUSel
);

    instMemory #(32, 16) myIMEM(.inst(), .pcVal(), .sysCLK()); 
    
    programCounter #(16) myPC(.PC(), .sysCLK(), .pRST());

    mainALU #(32) testALU(S, A, B, ALUSel);

    registerFile #(32, 5) riscRegisters(
        .regDataA(), .regDataB(), .regDataD(), .addrA(), .addrB(),
        .addrD(), .regWEn(), .sysCLK()
    );


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