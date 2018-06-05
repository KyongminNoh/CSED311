`include "opcodes.v"    
`timescale 1ns/1ns


module Adder(A, B, AdderOut);
  input [`WORD_SIZE-1:0] A;
  input [`WORD_SIZE-1:0] B;
  output reg [`WORD_SIZE-1:0] AdderOut;

  always @(*) begin
    AdderOut <= A + B;
  end  
endmodule

module JumpAddressAdder(PC, Instruction, JumpAddressOut);
  input [`WORD_SIZE-1:0] PC;
  input [`WORD_SIZE-1:0] Instruction;
  output reg [`WORD_SIZE-1:0] JumpAddressOut;

  always @(*) begin
    JumpAddressOut <= { PC[15:12], Instruction[11:0] };
  end
endmodule

module TwoInputMux(A, B, MuxControl, MuxOut);
  input [`WORD_SIZE-1:0] A;
  input [`WORD_SIZE-1:0] B;
  input MuxControl;
  output reg [`WORD_SIZE-1:0] MuxOut;

  always @(*) begin
      if(MuxControl == 0) begin 
        MuxOut <= A;
      end
      else begin 
        MuxOut <= B;
      end
  end
endmodule

module ThreeInputMux(A, B, C, MuxControl, MuxOut);
  input [`WORD_SIZE-1:0] A;
  input [`WORD_SIZE-1:0] B;
  input [`WORD_SIZE-1:0] C;
  input [1:0] MuxControl;
  output reg [`WORD_SIZE-1:0] MuxOut;

  always @(*) begin
      if(MuxControl == 2'd0) begin 
        MuxOut <= A;
      end
      else if(MuxControl == 2'd1) begin 
        MuxOut <= B;
      end
      else begin
        MuxOut <= C;
      end
  end
endmodule

module FourInputMux(A, B, C, D, MuxControl, MuxOut);
  input [`WORD_SIZE-1:0] A;
  input [`WORD_SIZE-1:0] B;
  input [`WORD_SIZE-1:0] C;
  input [`WORD_SIZE-1:0] D;
  input [1:0] MuxControl;
  output reg [`WORD_SIZE-1:0] MuxOut;

  always @(*) begin
      if(MuxControl == 2'd0) begin 
        MuxOut <= A;
      end
      else if(MuxControl == 2'd1) begin 
        MuxOut <= B;
      end
      else if(MuxControl == 2'd2) begin
        MuxOut <= C;
      end
      else begin
        MuxOut <= D;
      end
  end
endmodule

module Four1bitInputMux(A, B, C, D, MuxControl, MuxOut);
  input A;
  input B;
  input C;
  input D;
  input [1:0] MuxControl;
  output reg MuxOut;

  always @(*) begin
      if(MuxControl == 2'd0) begin 
        MuxOut <= A;
      end
      else if(MuxControl == 2'd1) begin 
        MuxOut <= B;
      end
      else if(MuxControl == 2'd2) begin
        MuxOut <= C;
      end
      else begin
        MuxOut <= D;
      end
  end
endmodule

module SignExtend(A, SignExtendOut);
  input [7:0] A;
  output reg [`WORD_SIZE-1:0] SignExtendOut;

  always @(*) begin
    SignExtendOut <= {A[7], A[7], A[7], A[7], A[7], A[7], A[7], A[7],A[7:0]};
  end
endmodule


module ALU(A, B, ALUControl, ALUOut);
   input [15:0] A;
   input [15:0] B;
   input [5:0] ALUControl;
   output reg [15:0] ALUOut;                  


   always @(*)
   begin
      case(ALUControl)
        6'd0: // Addition
          ALUOut <= A+B; 
        6'd1: // Subtraction
          ALUOut <= A-B;
        6'd2: // AND
          ALUOut <= A & B;
        6'd3: // OR
          ALUOut <= A | B;
        6'd4: // NOT
          ALUOut <= ~(A);
        6'd5: // TCP
          ALUOut <= ~(A)+1;
        6'd6: // Logical left shift
          ALUOut <= A<<1;
        6'd7: // Logical right shift
          ALUOut <= A>>1;
        6'd8:
          ALUOut <= {B[7:0],8'h00};
        6'd9:
          ALUOut <= (A == B) ? 0 : 1;
        6'd10:
          ALUOut <= (A == B) ? 1 : 0;
        6'd11:
          ALUOut <= (A == 0) ? 0 : !A[15];
        6'd12:
          ALUOut <= (A == 0) ? 0 : A[15];
        6'd13:
          ALUOut <= A;
        6'd28:
          ALUOut <= A;
        6'd29:
          ALUOut <= 1;
      endcase
   end
endmodule

module ProgramCounter(Clk, Reset_N, PCIn, PCWrite, PCOut);
  input Clk;
  input Reset_N;
  input [`WORD_SIZE-1:0] PCIn;
  input PCWrite;
  output reg [`WORD_SIZE-1:0] PCOut;

  reg negUpdate;
  
  initial begin
    negUpdate <= 1'b0;
  end


  always @(posedge Clk) begin
    if(!Reset_N) begin
      PCOut <= 16'hffff;
    end
    else begin
      if(!PCWrite) begin
        PCOut <= PCIn;
      end
    end 
  end


endmodule

module RegisterFiles(Clk, RegWrite, WWD, RS, RT, RD, WD, R1, R2, output_port);
  input wire Clk;
  input RegWrite;
  input WWD;
  input [1:0] RS,RT,RD;
  input [`WORD_SIZE-1:0] WD;

  output [`WORD_SIZE-1:0] R1,R2;
  output reg[`WORD_SIZE-1:0] output_port;

  reg [15:0] RF_data[3:0];

  assign R1 = (RegWrite && RD == RS) ? WD : RF_data[RS];
  assign R2 = (RegWrite && RD == RT) ? WD : RF_data[RT];

  initial begin
      RF_data[0] <= 16'h0000;
      RF_data[1] <= 16'h0000;
      RF_data[2] <= 16'h0000;
      RF_data[3] <= 16'h0000;
  end

  always @(posedge Clk) begin
    if(RegWrite && !WWD) begin
      RF_data[RD] <= WD;
    end
    if(WWD) begin
      output_port <= WD;
    end
  end

endmodule

module IFIDRegister(Clk, PCPlus1In, InstructionIn, IFIDWrite, PredictResultIn, InstCacheMissIn, PCPlus1Out, InstructionOut, PredictResultOut, InstCacheMissOut);
  input wire Clk;
  input [`WORD_SIZE-1:0] PCPlus1In;
  input [`WORD_SIZE-1:0] InstructionIn;
  input IFIDWrite;
  input PredictResultIn;
  input InstCacheMissIn;
  output reg [`WORD_SIZE-1:0] PCPlus1Out;
  output reg [`WORD_SIZE-1:0] InstructionOut;
  output reg PredictResultOut;
  output reg InstCacheMissOut;

  always @(posedge Clk) begin
    if(!IFIDWrite) begin
      PCPlus1Out <= PCPlus1In;
      InstructionOut <= InstructionIn;
      PredictResultOut <= PredictResultIn;
      InstCacheMissOut <= InstCacheMissIn;
    end
  end
endmodule

module IDEXRegister(Clk, IDEXWrite, PCPlus1In, ControlSignalIn, PredictResultIn, AIn, BIn, ExtendedOpcodeIn, RSIn, RTIn, RDIn, PCPlus1Out, ControlSignalOut, PredictResultOut, Branch, BranchType, RegDst, ALUOp, ALUSrc, AOut, BOut, ExtendedOpcodeOut, RSOut, RTOut, RDOut);
  input wire Clk;
  input wire IDEXWrite;
  input [`WORD_SIZE-1:0] PCPlus1In;
  input [`WORD_SIZE-1:0] ControlSignalIn;
  input PredictResultIn;
  input [`WORD_SIZE-1:0] AIn;
  input [`WORD_SIZE-1:0] BIn;
  input [`WORD_SIZE-1:0] ExtendedOpcodeIn;
  input [1:0] RSIn;
  input [1:0] RTIn;
  input [1:0] RDIn;

  
  output reg [`WORD_SIZE-1:0] PCPlus1Out;
  output reg [6:0] ControlSignalOut;
  output reg PredictResultOut;
  output reg Branch;
  output reg [2:0] BranchType;
  output reg [1:0] RegDst;
  output reg [1:0] ALUOp;
  output reg ALUSrc;
  
  // output reg [5:0] ALUControl;

  output reg [`WORD_SIZE-1:0] AOut;
  output reg [`WORD_SIZE-1:0] BOut;
  output reg [`WORD_SIZE-1:0] ExtendedOpcodeOut;
  output reg [1:0] RSOut;
  output reg [1:0] RTOut;
  output reg [1:0] RDOut;


  always @(posedge Clk) begin
    if(!IDEXWrite) begin 
      PCPlus1Out <= PCPlus1In;
      PredictResultOut <= PredictResultIn;

      ControlSignalOut <= ControlSignalIn[15:9];
      Branch <= ControlSignalIn[8];
      BranchType <= ControlSignalIn[7:5];
      RegDst <= ControlSignalIn[4:3];
      ALUOp <= ControlSignalIn[2:1];
      ALUSrc <= ControlSignalIn[0];

      AOut <= AIn;
      BOut <= BIn;
      ExtendedOpcodeOut <= ExtendedOpcodeIn;
      RSOut <= RSIn;
      RTOut <= RTIn;
      RDOut <= RDIn;
    end
  end
endmodule


module EXMEMRegister(Clk, EXMEMWrite, ControlSignalIn, ALUOutIn, ALUInputBIn, RDIn, FlushIn, ControlSignalOut, MemRead, MerWrite, ALUOutOut, ALUInputBOut, RDOut, FlushOut);
  input wire Clk;
  input wire EXMEMWrite;
  input [6:0] ControlSignalIn;
  input [`WORD_SIZE-1:0] ALUOutIn;
  input [`WORD_SIZE-1:0] ALUInputBIn;
  input [1:0] RDIn;
  input FlushIn;
  output reg [4:0] ControlSignalOut;
  output reg MemRead;
  output reg MerWrite;
  output reg [`WORD_SIZE-1:0] ALUOutOut;
  output reg [`WORD_SIZE-1:0] ALUInputBOut;
  output reg [1:0] RDOut;
  output reg FlushOut;

  always @(posedge Clk) begin
    if(!EXMEMWrite) begin
      ControlSignalOut <= ControlSignalIn[6:2];
      MemRead <= ControlSignalIn[1];
      MerWrite <= ControlSignalIn[0];

      ALUOutOut <= ALUOutIn;
      ALUInputBOut <= ALUInputBIn;
      RDOut <= RDIn;

      FlushOut <= FlushIn;
    end
  end
endmodule

module MEMWBRegister(Clk, MEMWBWrite, ControlSignalIn, ALUOutIn, MemoryDataIn, RDIn, RegWrite, MemToReg, ALUOutOut, MemoryDataOut, RDOut, num_inst_plus, WWD, HLT);
  input wire Clk;
  input [4:0] ControlSignalIn;
  input [`WORD_SIZE-1:0] ALUOutIn;
  input [`WORD_SIZE-1:0] MemoryDataIn;
  input [1:0] RDIn;
  input MEMWBWrite;
  output reg RegWrite;
  output reg MemToReg;
  output reg [`WORD_SIZE-1:0] ALUOutOut;
  output reg [`WORD_SIZE-1:0] MemoryDataOut;
  output reg [1:0] RDOut;
  output reg num_inst_plus;
  output reg WWD;
  output reg HLT;


  always @(posedge Clk) begin
	if(!MEMWBWrite) begin
    RegWrite <= ControlSignalIn[1];
    MemToReg <= ControlSignalIn[0];

    ALUOutOut <= ALUOutIn;
    MemoryDataOut <= MemoryDataIn;
    RDOut <= RDIn;
    WWD <= ControlSignalIn[3];
    HLT <= ControlSignalIn[4];
  end
	end
  always @(posedge Clk) begin
    if(ControlSignalIn[2]&&!MEMWBWrite) begin
      num_inst_plus <= 1;
    end
    else begin
      num_inst_plus <= 0;
    end
  end
endmodule


module BranchCheckUnit(Opcode, BranchCheckOut);
  input wire [3:0] Opcode;
  output reg BranchCheckOut;

  always @(*) begin
    if(Opcode == 4'b0000 || Opcode == 4'b0001 || Opcode == 4'b0010 || Opcode == 4'b0011) begin
      BranchCheckOut <= 1'b1;
    end
    else begin
      BranchCheckOut <= 1'b0;
    end
  end

endmodule

module BranchHistoryTable(Clk, IF_PCPlus1, EX_PCPlus1, IsTaken, Predict);
  input wire Clk;
  input wire [`WORD_SIZE-3:0] IF_PCPlus1;
  input wire [`WORD_SIZE-3:0] EX_PCPlus1;
  input wire IsTaken;

  output reg Predict;

  reg [1:0] BHTEntry[199:0];
  integer i;

  initial begin
    for(i=0; i<200; i=i+1) begin
      BHTEntry[i] <= 2'b01;
    end
  end

  always @(*) begin
    if(BHTEntry[IF_PCPlus1] === 2'b00 || BHTEntry[IF_PCPlus1] === 2'b01) begin
      Predict <= 1'b0;
    end
    else if(BHTEntry[IF_PCPlus1] === 2'b10 || BHTEntry[IF_PCPlus1] === 2'b11) begin
      Predict <= 1'b1;
    end
  end

  always @(posedge Clk) begin
    if(IsTaken === 1'b0) begin
      if(BHTEntry[EX_PCPlus1] === 2'b00) begin
        BHTEntry[EX_PCPlus1] <= 2'b00;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b01) begin
        BHTEntry[EX_PCPlus1] <= 2'b00;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b10) begin
        BHTEntry[EX_PCPlus1] <= 2'b01;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b11) begin
        BHTEntry[EX_PCPlus1] <= 2'b10;
      end
    end
    else if(IsTaken === 1'b1) begin
      if(BHTEntry[EX_PCPlus1] === 2'b00) begin
        BHTEntry[EX_PCPlus1] <= 2'b01;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b01) begin
        BHTEntry[EX_PCPlus1] <= 2'b10;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b10) begin
        BHTEntry[EX_PCPlus1] <= 2'b11;
      end
      else if(BHTEntry[EX_PCPlus1] === 2'b11) begin
        BHTEntry[EX_PCPlus1] <= 2'b11;
      end
    end
  end

endmodule

module BranchTargetBuffer(Clk, IF_PCPlus1, EX_PCPlus1, EX_BTBValue, Update, IF_BTBValue);
  input wire Clk;
  input wire [`WORD_SIZE-3:0] IF_PCPlus1;
  input wire [`WORD_SIZE-3:0] EX_PCPlus1;
  input wire [`WORD_SIZE-1:0] EX_BTBValue;  
  input wire Update;

  output reg [`WORD_SIZE-1:0] IF_BTBValue;
  reg [`WORD_SIZE-1:0] BTBEntry[199:0];
  integer i;

  initial begin
    for(i=0; i<200; i=i+1) begin
      BTBEntry[i] <= 16'h0000;
    end
  end

  always @(*) begin
    IF_BTBValue <= BTBEntry[IF_PCPlus1];
  end

  always @(posedge Clk) begin
    if(Update === 1'b1) begin
      BTBEntry[EX_PCPlus1] <= EX_BTBValue;
    end
  end   

endmodule

module PredictCheckUnit(PredictResultIn, BranchAssert, ReturnAssert, BranchSrc, PCSrc2);
  input PredictResultIn;
  input BranchAssert;
  input ReturnAssert;

  output reg BranchSrc;
  output reg PCSrc2;



  always @(*) begin
    if(ReturnAssert === 1'b1) begin
      PCSrc2 <= 1'b1;
      BranchSrc <= 1'b1;
    end
    else begin
      if((PredictResultIn === 1'b0) && (BranchAssert === 1'b0)) begin
        PCSrc2 <= 1'b0;
        BranchSrc <= 1'b0;
      end
      else if((PredictResultIn === 1'b0) && (BranchAssert === 1'b1)) begin
        PCSrc2 <= 1'b1;
        BranchSrc <= 1'b1;
      end
      else if((PredictResultIn === 1'b1) && (BranchAssert === 1'b0)) begin
        PCSrc2 <= 1'b1;
        BranchSrc <= 1'b0;
      end
      else if((PredictResultIn === 1'b1) && (BranchAssert === 1'b1)) begin
        PCSrc2 <= 1'b0;
        BranchSrc <= 1'b1;
      end
      else begin
        PCSrc2 <= 1'b0;
        BranchSrc <= 1'b0;
      end
    end
  end

endmodule
