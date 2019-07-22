module mainALU #(parameter ALUw = 32) (
    output [(ALUw-1):0] outALU, //result
    input  [(ALUw-1):0] inALUa, inALUb, //operands
    input  [3:0] ALUSel //tells what operation to perform
);
    // temporary holders / nets
    reg  [(ALUw-1):0] finalALU;
    wire [(ALUw-1):0] opADD, opSUB, opMULh, opMULt;

    // computation operations
    ALU_addSub #(ALUw) calcSum(opADD, inALUa, inALUb, 1'b0);
    ALU_multiplyU #(ALUw) calcMul({opMULh, opMULt}, inALUa, inALUb);
    ALU_addSub #(ALUw) calcSub(opSUB, inALUa, inALUb, 1'b1);

    // a MUXer
    always@(*) begin
        case(ALUSel)
            4'b0000: /* 00 = ADD  */ finalALU = opADD;
            4'b0001: /* 01 = AND  */ finalALU = inALUa & inALUb;
            4'b0010: /* 02 = OR   */ finalALU = inALUa | inALUb;
            4'b0011: /* 03 = XOR  */ finalALU = inALUa ^ inALUb;
            4'b0100: /* 04 = SRL  */ finalALU = (inALUa >> inALUb[4:0]); 
            4'b0101: /* 05 = SRA  */ finalALU = ($signed(inALUa) >>> inALUb[4:0]);
            4'b0110: /* 06 = SLL  */ finalALU = (inALUa << inALUb[4:0]);
            4'b0111: /* 07 = SLT  */ finalALU = ({31'b0, ((inALUa < inALUb) ? 1'b1: 1'b0)});
            //4'b1000: /* 08 = DIV  */ // = A / B
            //4'b1001: /* 09 = MOD  */ // = A % B
            4'b1010: /* 10 = MULt */ finalALU = opMULt; // lower bits
            4'b1011: /* 11 = MULh */ finalALU = opMULh; // higher bits
            4'b1100: /* 12 = SUB  */ finalALU = opSUB;
            4'b1101: /* 13 = BSel */ finalALU = inALUb;
            //4'b1110: /* 14 = ADD  */
            //4'b1111: /* 15 = ADD  */
            default: /* XX = ADD  */ finalALU = 32'b0;
        endcase
    end

    // sending out the output
    assign outALU = finalALU;
endmodule

// for addition and subtraction
module ALU_addSub #(parameter adderW = 32)(
    output [(adderW-1):0]sumOUT,
    input  [(adderW-1):0]adderA, adderB,
    input  addOrSub
);
    reg [(adderW-1):0]addSubRes;
    always@(*) begin
        if (addOrSub == 1'b0) begin
            addSubRes = adderA + adderB;
        end else begin
            addSubRes = adderA - adderB;
        end 
    end
        assign sumOUT = addSubRes;
endmodule

// for multiplication (unsigned)
module ALU_multiplyU #(parameter mulW = 32) (
    output reg [((2*mulW)-1):0] mulOUT,
    input  [(mulW-1):0] mulA, mulB
);
    always@(*) begin
        mulOUT = mulA * mulB;
    end
endmodule