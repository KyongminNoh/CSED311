`define WORD_SIZE 16	//	instead of 2^16 words to reduce memory
			//	requirements in the Active-HDL simulator 
`define BLOCK_SIZE 4
`define NUM_BLOCK 8
`define TAG_SIZE 12
`define NUM_SET 4

module Cache(clk, reset_n, mem_read_req, mem_write_req, mem_addr, mem_data_1, mem_data_2, mem_data_3, mem_data_4, resultData, cache_read_req, cache_write_req, is_cache_processing, req_addr, req_data);

//*******	PARAMETER	*******//
	input clk;
	wire clk;
	input reset_n;
	wire reset_n;

	input cache_read_req;
	wire cache_read_req;
	output reg mem_read_req;
	
	input cache_write_req;
	wire cache_write_req;
	output reg mem_write_req;
	
	input [`WORD_SIZE-1:0] req_addr;

	inout wire [`WORD_SIZE-1:0] mem_data_1;
	inout wire [`WORD_SIZE-1:0] mem_data_2;
	inout wire [`WORD_SIZE-1:0] mem_data_3;
	inout wire [`WORD_SIZE-1:0] mem_data_4;

	output wire [`WORD_SIZE-1:0] resultData; 


	inout [`WORD_SIZE-1:0] mem_addr;
	wire [`WORD_SIZE-1:0] mem_addr;

  input [`WORD_SIZE-1:0] req_data;	

	output is_cache_processing;
	reg is_cache_processing;


//*******	PARAMETER	*******//

//*******	LOCAL VARIABLE	*******//
	reg [`WORD_SIZE-1:0] block1[0:`NUM_BLOCK-1];
	reg [`WORD_SIZE-1:0] block2[0:`NUM_BLOCK-1];
	reg [`WORD_SIZE-1:0] block3[0:`NUM_BLOCK-1];
	reg [`WORD_SIZE-1:0] block4[0:`NUM_BLOCK-1];
	reg [3:0] RRPV[0:`NUM_BLOCK-1];
	reg [`TAG_SIZE-1:0] tag[0:`NUM_BLOCK-1];
	reg valid[0:`NUM_BLOCK-1];

	reg [`WORD_SIZE-1:0] outputData1;
	reg [`WORD_SIZE-1:0] outputAddr1;

	reg [`WORD_SIZE-1:0] writeData;
	
	reg [1:0] temp_block_idx;

	wire[3:0] line_idx;
	wire[1:0] block_idx;
	wire[13:0] addr_tag;
	wire[2:0] max_idx;

	reg [2:0] way;

	reg is_write;
	reg is_evict;
	
	reg [`WORD_SIZE-1:0] num_hit;
	reg [`WORD_SIZE-1:0] num_miss;
	
	reg [3:0] state;
	reg [3:0] next_state;

	parameter [3:0] IDLE=4'd0;
	parameter [3:0] HIT1=4'd1;
	parameter [3:0] HIT2=4'd2;
	parameter [3:0] FETCH0=4'd3;
	parameter [3:0] FETCH1=4'd4;
	parameter [3:0] FETCH2=4'd5;
	parameter [3:0] FETCH3=4'd6;
	parameter [3:0] FETCH4=4'd7;
	parameter [3:0] MISS = 4'd8;
	parameter [3:0] EVICT = 4'd9;

//*******	LOCAL VARIABLE	*******//
	
//*******	ASSIGNMENT	*******//
	assign resultData = outputData1;
	assign mem_data_1 = (!mem_read_req)?((mem_write_req)?block1[line_idx+way]:outputData1):`WORD_SIZE'bz;
	assign mem_data_2 = (!mem_read_req)?((mem_write_req)?block2[line_idx+way]:outputData1):`WORD_SIZE'bz;
	assign mem_data_3 = (!mem_read_req)?((mem_write_req)?block3[line_idx+way]:outputData1):`WORD_SIZE'bz;
	assign mem_data_4 = (!mem_read_req)?((mem_write_req)?block4[line_idx+way]:outputData1):`WORD_SIZE'bz;

	assign mem_addr = (is_cache_processing)?outputAddr1:((mem_write_req)?req_addr:`WORD_SIZE'bz);
	assign line_idx = 0;
	assign block_idx = req_addr[1:0];
	assign addr_tag = req_addr[15:2];
//*******	ASSIGNMENT	*******//
	MaxSelector MS(RRPV[0], RRPV[1], RRPV[2], RRPV[3], RRPV[4], RRPV[5], RRPV[6], RRPV[7], max_idx);

//******* Sequential Logic *******//
   always@(posedge clk) begin
     if(!reset_n) begin
       state = 4'd0;
		RRPV[0] <= 4'd14;
		tag[0] <= 14'bz;
		valid[0] <= 0;
		RRPV[1] <= 4'd14;
		tag[1] <= 14'bz;
		valid[1] <= 0;
		RRPV[2] <= 4'd14;
		tag[2] <= 14'bz;
		valid[2] <= 0;
		RRPV[3] <= 4'd14;
		tag[3] <= 14'bz;
		valid[3] <= 0;
		RRPV[4] <= 4'd14;
		tag[4] <= 14'bz;
		valid[4] <= 0;
		RRPV[5] <= 4'd14;
		tag[5] <= 14'bz;
		valid[5] <= 0;
		RRPV[6] <= 4'd14;
		tag[6] <= 14'bz;
		valid[6] <= 0;
		RRPV[7] <= 4'd14;
		tag[7] <= 14'bz;
		valid[7] <= 0;
     end
     else begin
       state = next_state;
     end
   end
//*********************************//

	always@(*) begin
	if(!reset_n) begin
		mem_read_req <= 1'b0;
		mem_write_req <= 1'b0;
		num_hit<=0;
		num_miss<=0;
	end
	else begin
	case (state)
	IDLE : begin
		is_cache_processing = 0;
		if(cache_read_req) begin
			is_write <=0;
			mem_write_req <=0;
		end
		else if(cache_write_req) begin
			is_write <= 1;
			writeData <= req_data;
			mem_write_req <=0;
		end
		
		if(cache_read_req || cache_write_req) begin
			if(addr_tag==tag[line_idx+0]) begin
				way = 3'd0;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]) begin
				way = 3'd1;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+2]) begin
				way = 3'd2;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+3]) begin
				way = 3'd3;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+4]) begin
				way = 3'd4;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+5]) begin
				way = 3'd5;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+6]) begin
				way = 3'd6;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+7]) begin
				way = 3'd7;
				next_state = HIT1;
			end
			else if(RRPV[line_idx]==4'd14) begin // miss
				way = 3'd0;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+1]==4'd14) begin // miss
				way = 3'd1;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+2]==4'd14) begin // miss
				way = 3'd2;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+3]==4'd14) begin // miss
				way = 3'd3;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+4]==4'd14) begin // miss
				way = 3'd4;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+5]==4'd14) begin // miss
				way = 3'd5;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+6]==4'd14) begin // miss
				way = 3'd6;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+7]==4'd14) begin // miss
				way = 3'd7;
				next_state = FETCH0;
			end
			else begin // evict
				way = max_idx;
				next_state = FETCH0;
			end
		end
		else
			next_state = IDLE;
	end
	HIT1 : begin
		if(cache_read_req) begin
			is_write <=0;
			mem_write_req <=0;
		end
		else if(cache_write_req) begin
			is_write <=1;
			writeData <= req_data;
			mem_write_req <=0;
		end
		if(cache_read_req || cache_write_req) begin
			if(addr_tag==tag[line_idx+0]) begin
				way = 3'd0;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]) begin
				way = 3'd1;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+2]) begin
				way = 3'd2;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+3]) begin
				way = 3'd3;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+4]) begin
				way = 3'd4;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+5]) begin
				way = 3'd5;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+6]) begin
				way = 3'd6;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+7]) begin
				way = 3'd7;
				next_state = HIT1;
			end
			else if(RRPV[line_idx]==4'd14) begin // miss
				way = 3'd0;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+1]==4'd14) begin // miss
				way = 3'd1;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+2]==4'd14) begin // miss
				way = 3'd2;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+3]==4'd14) begin // miss
				way = 3'd3;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+4]==4'd14) begin // miss
				way = 3'd4;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+5]==4'd14) begin // miss
				way = 3'd5;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+6]==4'd14) begin // miss
				way = 3'd6;
				next_state = FETCH0;
			end
			else if(RRPV[line_idx+7]==4'd14) begin // miss
				way = 3'd7;
				next_state = FETCH0;
			end
			else begin // evict
				way = max_idx;
				next_state = FETCH0;
			end
		end
		else
			next_state = IDLE;
	end
	FETCH0 : begin
		next_state = FETCH1;
		is_cache_processing = 1;
		
		outputAddr1 <= req_addr;
		temp_block_idx <= req_addr[1:0];
	end
	FETCH1 : begin
		next_state = FETCH2;
		is_cache_processing = 1;
	end
	FETCH2 : begin
		next_state = FETCH3;
		is_cache_processing = 1;
	end
	FETCH3 : begin
		next_state = FETCH4;
		is_cache_processing = 1;
	end
	FETCH4 : begin
		if(RRPV[line_idx+way] == 4'd14 && !valid[line_idx+way]) begin
			next_state = MISS;
			is_evict = 0;
			is_cache_processing = 1;
		end
		else begin
			next_state = EVICT;
			is_evict = 1;
			is_cache_processing = 1;
		end
	end
	MISS : begin
		next_state = IDLE;
		is_cache_processing =1;
	end
	EVICT : begin
		next_state = IDLE;
		is_cache_processing =1;
	end
	endcase
	end
	end

	always@(posedge clk) begin
	case(state)
	HIT1 : begin
		RRPV[line_idx+way] <= 4'd0;
		if(is_write) begin
			case(block_idx)
			2'b00 : begin
				block1[line_idx+way] <= writeData;
			end
			2'b01 : begin
				block2[line_idx+way] <= writeData;
			end
			2'b10 : begin
				block3[line_idx+way] <= writeData;
			end
			2'b11 : begin
				block4[line_idx+way] <= writeData;
			end
			endcase
		end
		else begin
			case(block_idx)
			2'b00 : begin
				outputData1 <= block1[line_idx+way];
			end
			2'b01 : begin
				outputData1 <= block2[line_idx+way];
			end
			2'b10 : begin
				outputData1 <= block3[line_idx+way];
			end
			2'b11 : begin
				outputData1 <= block4[line_idx+way];
			end
			endcase
		end
	end
	MISS : begin
		if(!is_write) begin
			case(temp_block_idx)
			2'b00 : begin
				outputData1<=block1[line_idx+way];
			end
			2'b01 : begin
				outputData1<=block2[line_idx+way];
			end
			2'b10 : begin
				outputData1<=block3[line_idx+way];
			end
			2'b11 : begin
				outputData1<=block4[line_idx+way];
			end
			endcase
		end
		else begin
			mem_write_req <= 0;
			case(temp_block_idx)
			2'b00 : begin
				block1[line_idx+way]<=writeData;
			end
			2'b01 : begin
				block2[line_idx+way]<=writeData;
			end
			2'b10 : begin
				block3[line_idx+way]<=writeData;
			end
			2'b11 : begin
				block4[line_idx+way]<=writeData;
			end
			endcase
		end
		tag[line_idx+way] <= mem_addr[15:2];
		RRPV[line_idx+way] <= 4'd13;
		valid[line_idx+way] <= 1;
	end
	EVICT : begin
		if(!is_write) begin
			case(temp_block_idx)
			2'b00 : begin
				outputData1<=block1[line_idx+way];
			end
			2'b01 : begin
				outputData1<=block2[line_idx+way];
			end
			2'b10 : begin
				outputData1<=block3[line_idx+way];
			end
			2'b11 : begin
				outputData1<=block4[line_idx+way];
			end
			endcase
		end
		else begin
			mem_write_req<=0;
			case(temp_block_idx)
			2'b00 : begin
				block1[line_idx+way]<=writeData;
			end
			2'b01 : begin
				block2[line_idx+way]<=writeData;
			end
			2'b10 : begin
				block3[line_idx+way]<=writeData;
			end
			2'b11 : begin
				block4[line_idx+way]<=writeData;
			end
			endcase
		end
		tag[line_idx+way] <= mem_addr[15:2];
		if(way!=0) RRPV[0] <= RRPV[0] +(4'd14 - RRPV[way]);
		if(way!=1) RRPV[1] <= RRPV[1] +(4'd14 - RRPV[way]);
		if(way!=2) RRPV[2] <= RRPV[2] +(4'd14 - RRPV[way]);
		if(way!=3) RRPV[3] <= RRPV[3] +(4'd14 - RRPV[way]);
		if(way!=4) RRPV[4] <= RRPV[4] +(4'd14 - RRPV[way]);
		if(way!=5) RRPV[5] <= RRPV[5] +(4'd14 - RRPV[way]);
		if(way!=6) RRPV[6] <= RRPV[6] +(4'd14 - RRPV[way]);
		if(way!=7) RRPV[7] <= RRPV[7] +(4'd14 - RRPV[way]);
		
		RRPV[line_idx+way] <= 4'd13;
		valid[line_idx+way] <= 1;
	end
	endcase
	end
	
	always@(negedge clk) begin
	case(state)
	IDLE : begin
	end
	HIT1 : begin
		num_hit <= num_hit+1;
	end
	HIT2 : begin
		num_hit <= num_hit+1;
	end
	FETCH0 : begin
		mem_read_req<=1;
	end
	FETCH1 : begin
	end
	FETCH2 : begin
	end
	FETCH3 : begin
	end
	FETCH4 : begin
		block1[line_idx+way] <= mem_data_1;
		block2[line_idx+way] <= mem_data_2;
		block3[line_idx+way] <= mem_data_3;
		block4[line_idx+way] <= mem_data_4;
		mem_read_req<=0;
		if(is_write&&is_evict) begin
			mem_write_req <=1;
			outputAddr1<= req_addr;
		end
		else begin
			outputAddr1 <= {outputAddr1[15:2], temp_block_idx};
		end
	end
	MISS : begin
		num_miss<=num_miss+1;
		end
	EVICT : begin
		num_miss<=num_miss+1;
	end
	endcase
	end
endmodule

module MaxSelector(inputData0, inputData1, inputData2, inputData3, inputData4, inputData5, inputData6, inputData7, maxIndex);
  input wire [3:0] inputData0;
  input wire [3:0] inputData1;
  input wire [3:0] inputData2;
  input wire [3:0] inputData3;
  input wire [3:0] inputData4;
  input wire [3:0] inputData5;
  input wire [3:0] inputData6;
  input wire [3:0] inputData7;
  
  output reg [2:0] maxIndex;

  wire [6:0] max1;
  wire [6:0] max2;
  wire [6:0] max3;
  wire [6:0] max4;
  wire [6:0] max5;
  wire [6:0] max6;
  wire [6:0] max7;

  reg [6:0] Data0;
  reg [6:0] Data1;
  reg [6:0] Data2;
  reg [6:0] Data3;
  reg [6:0] Data4;
  reg [6:0] Data5;
  reg [6:0] Data6;
  reg [6:0] Data7;

  assign max1 = (Data0[3:0] >= Data1[3:0]) ? Data0 : Data1;
  assign max2 = (max1[3:0] >= Data2[3:0]) ? max1 : Data2;
  assign max3 = (max2[3:0] >= Data3[3:0]) ? max2 : Data3;
  assign max4 = (max3[3:0] >= Data4[3:0]) ? max3 : Data4;
  assign max5 = (max4[3:0] >= Data5[3:0]) ? max4 : Data5;
  assign max6 = (max5[3:0] >= Data6[3:0]) ? max5 : Data6;
  assign max7 = (max6[3:0] >= Data7[3:0]) ? max6 : Data7;

  always @(*) begin
    Data0 <= {3'b000 , inputData0};
    Data1 <= {3'b001 , inputData1};
    Data2 <= {3'b010 , inputData2};
    Data3 <= {3'b011 , inputData3};
    Data4 <= {3'b100 , inputData4};
    Data5 <= {3'b101 , inputData5};
    Data6 <= {3'b110 , inputData6};
    Data7 <= {3'b111 , inputData7};

    maxIndex <= max7[6:4];
  end

endmodule
