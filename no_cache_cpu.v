`timescale 1ns/1ns
`include "opcodes.v"   

module nccpu(Clk, Reset_N, readM1, address1, data1_1, data1_2, data1_3, data1_4, readM2, writeM2, address2, data2_1, data2_2, data2_3, data2_4, num_inst, output_port, is_halted);
	input wire Clk;
	input wire Reset_N;

	output reg readM1;
	output wire [`WORD_SIZE-1:0] address1;
	output reg readM2;
	output reg writeM2;
	output wire [`WORD_SIZE-1:0] address2;

	input wire [`WORD_SIZE-1:0] data1_1;
	input wire [`WORD_SIZE-1:0] data1_2;
	input wire [`WORD_SIZE-1:0] data1_3;
	input wire [`WORD_SIZE-1:0] data1_4;
	inout wire [`WORD_SIZE-1:0] data2_1;
	inout wire [`WORD_SIZE-1:0] data2_2;
	inout wire [`WORD_SIZE-1:0] data2_3;
	inout wire [`WORD_SIZE-1:0] data2_4;

	output reg [`WORD_SIZE-1:0] num_inst;
	output wire [`WORD_SIZE-1:0] output_port;
	output wire is_halted;


	// TODO : Implement your pipelined CPU!


  wire num_inst_plus;

  reg readM1End;

  // IF
  wire [`WORD_SIZE-1:0] PCIn;
  wire [`WORD_SIZE-1:0] PCOut;
  wire [`WORD_SIZE-1:0] IF_PCPlus1;
  wire [`WORD_SIZE-1:0] IF_Instruction;
  wire [`WORD_SIZE-1:0] BranchPC;
  wire [`WORD_SIZE-1:0] SelectedPC;
  wire [`WORD_SIZE-1:0] NextPC;
  wire [`WORD_SIZE-1:0] JumpPC;
  wire [`WORD_SIZE-1:0] JumpAddr2;
	wire JMP;
  wire JAL;
  wire PCWrite;
	wire IF_PCWrite;
	wire IFIDWrite;
  wire IF_IFIDWrite;
	wire PCSrc1;
  wire PCSrc0;
  wire BranchCheckOut;
  wire IF_PredictResult;
  wire IsPredict;
  wire [`WORD_SIZE-1:0] IF_BTBValue;
  wire [`WORD_SIZE-1:0] PredictedPC;


  // ID
  wire [`WORD_SIZE-1:0] ID_PCPlus1;
  wire [`WORD_SIZE-1:0] ID_Instruction;
  wire [`WORD_SIZE-1:0] ID_ExtendedOffset;
  wire [5:0] ID_Opcode;
  wire [1:0] ID_RS;
  wire [1:0] ID_RT;
  wire [1:0] ID_RD;
  wire [7:0] ID_Offset;
  wire [5:0] ID_funcode;
  wire [`WORD_SIZE-1:0] ID_R1;
  wire [`WORD_SIZE-1:0] ID_R2;
	wire [`WORD_SIZE-1:0] ID_ControlSignal;
  wire [`WORD_SIZE-1:0] ID_SelectedSignal;
  wire IsFlush;
  wire ID_Flush;
	wire [1:0] ID_BranchType;
  wire ID_PredictResult;
	wire Stall;
  wire IDEXWrite;


  // EX
	wire [`WORD_SIZE-1:0] EX_PCPlus1;
  wire [6:0] EX_ControlSignal;
  wire EX_Branch;
  wire [2:0] EX_BranchType;
  wire [1:0] EX_RegDst;
  wire [1:0] EX_ALUOp;
	wire [5:0] EX_ALUFunc;
  wire EX_ALUSrc;
  wire [5:0] EX_ALUControl;
	wire EX_MemRead;
  wire EX_RegWrite;
  wire [`WORD_SIZE-1:0] EX_R1;
  wire [`WORD_SIZE-1:0] EX_R2;
  wire [`WORD_SIZE-1:0] EX_ExtendedOffset;
  wire [1:0] EX_RS;
  wire [1:0] EX_RT;
  wire [1:0] EX_RD;
  wire [`WORD_SIZE-1:0] MEMForwardingData;
  wire [`WORD_SIZE-1:0] WBForwardingData;
  wire [`WORD_SIZE-1:0] EX_ALUOut;
	wire [`WORD_SIZE-1:0] ForwardingMuxAOut;
  wire [`WORD_SIZE-1:0] ForwardingMuxBOut;
  wire [`WORD_SIZE-1:0] ALUInputA;
  wire [`WORD_SIZE-1:0] ALUInputB;
  wire [`WORD_SIZE-1:0] RTMuxOut;
  wire [1:0] ForwardA;
  wire [1:0] ForwardB;
	wire [1:0] EX_RTMuxOut;
	wire ReturnAssert;
	wire BranchAssert;
	wire EX_Flush;
	wire [`WORD_SIZE-1:0] EX_SelectedSignal;
  wire [`WORD_SIZE-1:0] ConditionalAddr;
  wire [`WORD_SIZE-1:0] ReturnAddr;
  wire [`WORD_SIZE-1:0] SelectedAddr;
  wire IsTaken;
  wire EX_PredictResult;
  wire PCSrc2;
  wire BranchSrc;
  wire EXMEMWrite; 

  // MEM
  wire [4:0] MEM_ControlSignal;
  wire [`WORD_SIZE-1:0] MEM_ALUOut;
  wire [`WORD_SIZE-1:0] MEM_ALUInputB;
  wire [1:0] MEM_RD;
  wire MEM_MemWrite;
  wire MEM_MemRead;
	wire MEM_RegWrite;
  wire MEMWBWrite;

  // WB
  wire [1:0] WB_WR;
  wire [`WORD_SIZE-1:0] WB_MemoryOut;
  wire [`WORD_SIZE-1:0] WB_ALUOut;
  wire [`WORD_SIZE-1:0] WB_WD;
  wire WB_RegWrite;
  wire WB_MemToReg;
  wire WB_WWD;
  wire WB_HLT;

	assign PCIn = NextPC;
  ProgramCounter PC(Clk, Reset_N, PCIn, IF_PCWrite, PCOut); 
  assign address1 = PCOut;
  assign IF_Instruction = data1_1;
  Adder PCAdder(address1, 1, IF_PCPlus1);  
	assign JMP = (IF_Instruction[15:12] === 4'd9) ? 1'b1 : 1'b0;
  assign JAL = (IF_Instruction[15:12] === 4'd10) ? 1'b1 : 1'b0;
	JumpAddressAdder JumpAddrAdder(IF_PCPlus1, IF_Instruction, JumpPC);
	assign PCSrc0 = (JMP || JAL ) ? 1'b1 : 1'b0;
  BranchCheckUnit BCU(IF_Instruction[15:12], BranchCheckOut);
  BranchHistoryTable BHT(Clk, IF_PCPlus1[13:0], EX_PCPlus1[13:0], IsTaken, IsPredict);
  assign IF_PredictResult = ((BranchCheckOut === 1'b1) && (IsPredict === 1'b1)) ? 1'b1 : 1'b0;
  assign PCSrc1 = IF_PredictResult;
  BranchTargetBuffer BTB(Clk, IF_PCPlus1[13:0], EX_PCPlus1[13:0], ConditionalAddr, IsTaken, IF_BTBValue);
  assign PredictedPC = IF_BTBValue;
  ThreeInputMux SelectedPCMux(IF_PCPlus1, JumpPC, PredictedPC, {PCSrc1, PCSrc0}, SelectedPC);
  TwoInputMux NextPCMux(SelectedPC, BranchPC, PCSrc2, NextPC);
  assign IF_PCWrite = (PCWrite === 1'b1 || readM1 === 1'b1) ? 1'b1 : 1'b0;
  assign IF_IFIDWrite = (IFIDWrite === 1'b1 || readM1 === 1'b1) ? 1'b1 : 1'b0;



	IFIDRegister IFID_Reg(Clk, IF_PCPlus1, IF_Instruction, IF_IFIDWrite, IF_PredictResult, ID_PCPlus1, ID_Instruction, ID_PredictResult); // IFID Register


  assign ID_Opcode = ID_Instruction[15:12];
  assign ID_Offset = ID_Instruction[7:0];
  assign ID_RS = ID_Instruction[11:10];
  assign ID_RT = ID_Instruction[9:8];
  assign ID_RD = ID_Instruction[7:6];
  assign ID_funcode = ID_Instruction[5:0];
  SignExtend ExtendOpcode(ID_Offset, ID_ExtendedOffset);
  RegisterFiles RF(Clk, WB_RegWrite, WB_WWD, ID_RS, ID_RT, WB_WR, WB_WD, ID_R1, ID_R2, output_port);
  assign ID_Flush = ((Stall === 1'b1) || (IsFlush === 1'b1)) ? 1'b1 : 1'b0;
  TwoInputMux ID_FlushMux(ID_ControlSignal, 16'd0, ID_Flush, ID_SelectedSignal);
  assign IDEXWrite = (readM1 === 1'b1) ? 1'b1 : 1'b0;

	IDEXRegister IDEX_Reg(Clk, IDEXWrite, ID_PCPlus1, ID_SelectedSignal, ID_PredictResult, ID_R1, ID_R2, ID_ExtendedOffset, ID_RS, ID_RT, ID_RD, EX_PCPlus1, EX_ControlSignal, EX_PredictResult, EX_Branch, EX_BranchType, EX_RegDst, EX_ALUOp, EX_ALUSrc, EX_R1, EX_R2, EX_ExtendedOffset, EX_RS, EX_RT, EX_RD);


  assign EX_ALUFunc = EX_ExtendedOffset[5:0];
  assign is_halted = (WB_HLT === 1'b1) ? 1 : 0;
  assign EX_MemRead = EX_ControlSignal[1];
  assign EX_RegWrite = EX_ControlSignal[3];
	Adder ConditionalAdder(EX_PCPlus1, EX_ExtendedOffset, ConditionalAddr);
  ThreeInputMux ForwardingMuxA(EX_R1, WBForwardingData, MEMForwardingData, ForwardA, ForwardingMuxAOut);
  ThreeInputMux ForwardingMuxB(EX_R2, WBForwardingData, MEMForwardingData, ForwardB, ForwardingMuxBOut);
  assign ReturnAddr = ForwardingMuxAOut;
  ThreeInputMux RTMux({14'd0, EX_RT}, {14'd0, EX_RD}, {14'd0, 2'b10}, EX_RegDst, RTMuxOut);
  assign EX_RTMuxOut = RTMuxOut[1:0];
	TwoInputMux ALUInputAMux(ForwardingMuxAOut, EX_PCPlus1, EX_BranchType[2], ALUInputA);
  TwoInputMux ALUInputBMux(ForwardingMuxBOut, EX_ExtendedOffset, EX_ALUSrc, ALUInputB);
  ALU EX_ALU(ALUInputA, ALUInputB, EX_ALUControl, EX_ALUOut);
	assign ReturnAssert = (EX_BranchType[2] & EX_Branch);
	assign BranchAssert = ((EX_ALUOut & EX_Branch === 1'b1) && (EX_BranchType[2] === 1'b0)) ? 1'b1 : 1'b0;
  assign IsTaken = ((BranchAssert === 1'b1) && (EX_BranchType[2] === 1'b0)) ? 1'b1 : 1'b0;
  PredictCheckUnit PCU(EX_PredictResult, BranchAssert, ReturnAssert, BranchSrc, PCSrc2);
	assign EX_Flush = (IsFlush === 1'b1) ? 1'b1 : 1'b0;
  TwoInputMux SelectAddrMux(ConditionalAddr, ReturnAddr, EX_BranchType[2], SelectedAddr);
  TwoInputMux BranchPCMux(EX_PCPlus1, SelectedAddr, BranchSrc, BranchPC);
	TwoInputMux EX_FlushMux({9'b0, EX_ControlSignal}, 16'd0, EX_Flush, EX_SelectedSignal);
  assign EXMEMWrite = (readM1 === 1'b1) ? 1'b1 : 1'b0;


  EXMEMRegister EXMEM_Reg(Clk, EXMEMWrite, EX_SelectedSignal[6:0], EX_ALUOut, ForwardingMuxBOut, RTMuxOut[1:0], PCSrc2, MEM_ControlSignal, MEM_MemRead, MEM_MemWrite, MEM_ALUOut, MEM_ALUInputB, MEM_RD, IsFlush);


  assign MEM_RegWrite = MEM_ControlSignal[1];
  assign address2 = MEM_ALUOut;
  assign data2_1 = (MEM_MemWrite) ? MEM_ALUInputB : 16'bz;
  assign MEMForwardingData = MEM_ALUOut;
  assign MEMWBWrite = (readM1 === 1'b1) ? 1'b1 : 1'b0;

  
  MEMWBRegister MEMWB_Reg(Clk, MEMWBWrite, MEM_ControlSignal, MEM_ALUOut, data2_1, MEM_RD, WB_RegWrite, WB_MemToReg, WB_ALUOut, WB_MemoryOut, WB_WR, num_inst_plus, WB_WWD, WB_HLT);


  TwoInputMux WBMux(WB_ALUOut, WB_MemoryOut, WB_MemToReg, WB_WD);
  assign WBForwardingData = WB_WD;
	

  always @(posedge Clk) begin
    if(!Reset_N) begin
      readM1 <= 0;
      readM1End <= 1;
      readM2 <= 0;
      writeM2 <= 0;
      num_inst <= 0;
    end
    else begin
      if(readM1End == 1) begin
        readM1 <= 0;
      end
      if(readM1End == 0) begin
        readM1 <= 1;
      end
      if(num_inst_plus) begin
        num_inst <= num_inst + 1;
      end      
    end
  end

  always @(negedge Clk) begin
      if(readM1 == 1)begin
        readM1End <= 1;
      end
      if(readM1 == 0)begin
        readM1End <= 0;
      end
  end

  always @(posedge MEM_MemRead) begin
    readM2 <= 1;
  end

  always @(negedge MEM_MemRead) begin
    readM2 <= 0;
  end

  always @(posedge MEM_MemWrite) begin
    writeM2 <= 1;
  end

  always @(negedge MEM_MemWrite) begin
    writeM2 <= 0;
  end


	ControlUnit controlUnit(Clk, Reset_N, ID_Instruction, ID_ControlSignal, ID_BranchType);
	ALUControlUnit aluControlUnit(Clk, EX_ALUOp, EX_Branch, EX_BranchType, EX_ALUFunc, EX_ALUControl);
	ForwardingUnit forwardingUnit(Clk, JRL, ID_RS, ID_RT, EX_RegWrite, EX_RS, EX_RT, EX_RTMuxOut, MEM_RegWrite, MEM_RD, WB_RegWrite, WB_WR, ForwardA, ForwardB);
	HazardDetectionUnit hazardDetectionUnit(Clk, ID_RS, ID_RT, EX_MemRead, EX_RegWrite, EX_RS, EX_RT, Stall, IFIDWrite, PCWrite);

endmodule



