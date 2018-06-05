`include "opcodes.v"    
`timescale 1ns/1ns

module ControlUnit(Clk, Reset_N, Instruction, ControlSignal, BranchMuxSelector);
  input wire Clk;
	input wire Reset_N;
  input wire [`WORD_SIZE-1:0] Instruction;

  output [`WORD_SIZE-1:0] ControlSignal;
  output reg [1:0] BranchMuxSelector;

  parameter [2:0] R = 3'b000;
  parameter [2:0] I = 3'b001;
  parameter [2:0] LW = 3'b010;
  parameter [2:0] SW = 3'b011;
  parameter [2:0] BR = 3'b100;
  parameter [2:0] J = 3'b101;
  wire [3:0] opcode;
  reg [2:0] optype;

  reg HLT;
  reg WWD;
  reg NotStall;

  reg Branch;
  reg [2:0] BranchType;
  reg [1:0] RegDst;
  reg [1:0] ALUOp;
  reg ALUSrc;

  reg MemRead;
  reg MemWrite;

  reg RegWrite;
  reg MemToReg;

  assign ControlSignal = {HLT, WWD, NotStall, RegWrite, MemToReg, MemRead, MemWrite, Branch, BranchType, RegDst, ALUOp, ALUSrc};

  always @(*) begin
    if(Instruction[15:12] == 4'd0) begin
      BranchMuxSelector <= 2'b00;
    end
    else if(Instruction[15:12] == 4'd1) begin
      BranchMuxSelector <= 2'b01;
    end
    else if(Instruction[15:12] == 4'd2) begin
      BranchMuxSelector <= 2'b10;
    end
    else begin
      BranchMuxSelector <= 2'b11;
    end
  end

  assign opcode = Instruction[15:12];

  always @(*) begin
    if(!Reset_N) begin
        HLT <= 1'b0;
        WWD <= 1'b0;
        NotStall <= 1'b0;

        Branch <= 1'b0;
        BranchType <= 3'b000;
        RegDst <= 2'b00;
        ALUOp <= 2'b00;
        ALUSrc <= 1'b0;
        
        MemRead <= 1'b0;
        MemWrite <= 1'b0;

        RegWrite <= 1'b0;
        MemToReg <= 1'b0;
    end
    else begin
      if(opcode == 4'b1111 ) begin // R type
        if(Instruction[5:0] == 16'd25) begin  // JPR
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b1;
          BranchType <= 3'b100;
          RegDst <= 2'b00;
          ALUOp <= 2'b00;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b0;
          MemToReg <= 1'b0;
        end
        else if(Instruction[5:0] == 16'd26) begin  // JRL
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b1;
          BranchType <= 3'b100;
          RegDst <= 2'b10;
          ALUOp <= 2'b11;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b1;
          MemToReg <= 1'b0;  
        end
        else if(Instruction[5:0] == 16'd29) begin  // HLT
          HLT <= 1'b1;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b00;
          ALUOp <= 2'b00;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b0;
          MemToReg <= 1'b0;
        end
        else begin
          HLT <= 1'b0;
          if(Instruction[5:0] == 16'd28) WWD <= 1'b1;
          else WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b01;
          ALUOp <= 2'b11;
          ALUSrc <= 1'b0;          

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          if(Instruction[5:0] == 16'd28) RegWrite <= 1'b0;
          else RegWrite <= 1'b1;
          MemToReg <= 1'b0;
        end
      end
      else if(opcode == 4'b0100 || opcode == 4'b0101 || opcode == 4'b0110 ) begin // I type
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b00;
          if(opcode == `ADI_OP) begin
            ALUOp <= 2'b00;
          end
          else if(opcode == `ORI_OP) begin
            ALUOp <= 2'b01;
          end
          else if(opcode == `LHI_OP) begin
            ALUOp <= 2'b10;
          end
          ALUSrc <= 1'b1;


          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b1;
          MemToReg <= 1'b0;
      end 
      else if(opcode == 4'b0111) begin // LW
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b00;
          ALUOp <= 2'b00;
          ALUSrc <= 1'b1;

          MemRead <= 1'b1;
          MemWrite <= 1'b0;

          RegWrite <= 1'b1;
          MemToReg <= 1'b1;
      end
      else if(opcode == 4'b1000) begin // SW
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b00;
          ALUOp <= 2'b00;
          ALUSrc <= 1'b1;

          MemRead <= 1'b0;
          MemWrite <= 1'b1;

          RegWrite <= 1'b0;
          MemToReg <= 1'b0;
      end
      else if(opcode == 4'b0000 || opcode == 4'b0001 || opcode == 4'b0010 || opcode == 4'b0011) begin // Branch
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b1;
          BranchType <= { 1'b0, opcode[1:0] };
          RegDst <= 2'b00;
          ALUOp <= 2'b11;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b0;
          MemToReg <= 1'b0;  
      end
      else if(opcode == 4'b1001) begin // JMP
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b000;
          RegDst <= 2'b00;
          ALUOp <= 2'b00;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b0;
          MemToReg <= 1'b0;
      end
      else if(opcode == 4'b1010) begin // JAL
          HLT <= 1'b0;
          WWD <= 1'b0;
          NotStall <= 1'b1;

          Branch <= 1'b0;
          BranchType <= 3'b100;
          RegDst <= 2'b10;
          ALUOp <= 2'b11;
          ALUSrc <= 1'b0;

          MemRead <= 1'b0;
          MemWrite <= 1'b0;

          RegWrite <= 1'b1;
          MemToReg <= 1'b0;
      end

    end
  end

endmodule

module ALUControlUnit(Clk, ALUOp, Branch, BranchType, func, ALUControl);
  input wire Clk;
  input wire [1:0] ALUOp;
  input wire Branch;
  input wire [2:0] BranchType;
  input wire [5:0] func;

  output reg [5:0] ALUControl;

  always @(*) begin
    if(ALUOp == 2'b00) begin // add
      ALUControl <= 6'd0;
    end
    if(ALUOp == 2'b01) begin // or
      ALUControl <= 6'd3;
    end
    if(ALUOp == 2'b10) begin // lhi
      ALUControl <= 6'd8;
    end
    if(ALUOp == 2'b11) begin // R type
      if(Branch == 1'b1) begin
        if(BranchType == 3'b000)begin
          ALUControl <= 6'd9;
        end
        else if(BranchType == 3'b001)begin
          ALUControl <= 6'd10;
        end
        else if(BranchType == 3'b010)begin
          ALUControl <= 6'd11;
        end
        else if(BranchType == 3'b011)begin
          ALUControl <= 6'd12;
        end
        else if(BranchType == 3'b100)begin
          ALUControl <= 6'd13;
        end
      end
      else begin
        if(BranchType === 3'b100) ALUControl <= 6'd13;
        else ALUControl <= func;
      end
    end
  end
endmodule

module ForwardingUnit(Clk, JRL, ID_RS, ID_RT, EX_RegWrite, EX_RS, EX_RT, EX_RTMuxOut, MEM_RegWrite, MEM_RD, WB_RegWrite, WB_RD, ForwardA, ForwardB);
  input wire Clk;
  input wire JRL;
  input [1:0] ID_RS;
  input [1:0] ID_RT;
  input EX_RegWrite;
  input [1:0] EX_RS;
  input [1:0] EX_RT;
  input [1:0] EX_RTMuxOut;
  input MEM_RegWrite;
  input [1:0] MEM_RD;
  input WB_RegWrite;
  input [1:0] WB_RD;

  output reg [1:0] ForwardA;
  output reg [1:0] ForwardB;

  always @(*) begin
    if(MEM_RegWrite && (MEM_RD == EX_RS)) begin // add
      ForwardA <= 2'b10;
    end
    else if(WB_RegWrite && (WB_RD == EX_RS)) begin // add !(MEM_RegWrite && (MEM_RD != EX_RS)) && 
      ForwardA <= 2'b01;
    end
    else begin // or
      ForwardA <= 2'b00;
    end

    if(MEM_RegWrite && (MEM_RD == EX_RT)) begin // add
      ForwardB <= 2'b10;
    end
    else if(WB_RegWrite && (WB_RD == EX_RT)) begin // add !(MEM_RegWrite && (MEM_RD != EX_RT)) && 
      ForwardB <= 2'b01;
    end
    else begin // or
      ForwardB <= 2'b00;
    end

  end
endmodule

module HazardDetectionUnit(Clk, ID_RS, ID_RT, EX_MemRead, EX_RegWrite, EX_RS, EX_RT, Stall, IFIDWrite, PCWrite);
  input wire Clk;
  input [1:0] ID_RS;
  input [1:0] ID_RT;
  input EX_MemRead;
  input EX_RegWrite;
  input [1:0] EX_RS;
  input [1:0] EX_RT;

  output reg Stall;
  output reg IFIDWrite;
	output reg PCWrite;


  always @(*) begin
    if((EX_MemRead ) && ((EX_RT == ID_RS) || (EX_RT == ID_RT))) begin
      Stall <= 1'b1;
      IFIDWrite <= 1'b1;
      PCWrite <= 1'b1;
    end
    else begin
      Stall <= 1'b0;
      IFIDWrite <= 1'b0;
      PCWrite <= 1'b0;
    end
  end
endmodule
