module mainProcessor(
    output [31:0]S,
    input  [31:0]A, B,
    input  [3:0]ALUSel
);
    mainALU #(32) testALU(S, A, B, ALUSel);
endmodule

// posedge counter with async reset
module programCounter #(parameter counterWidth = 16)(
    output reg [(counterWidth-1):0]PC,
    input  pCLK, pRST
);
    always@(posedge pCLK or posedge pRST) begin
        if (pRST) begin
            PC <= 1'b0;
        end else begin
            PC <= PC + 1'b1;
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