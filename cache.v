`define WORD_SIZE 16	//	instead of 2^16 words to reduce memory
			//	requirements in the Active-HDL simulator 
`define BLOCK_SIZE 4
`define NUM_BLOCK 8
`define TAG_SIZE 12
`define NUM_SET 4

module Cache(clk, reset_n, mem_read_req, mem_write_req, mem_addr, mem_data, cache_read_req, cache_write_req, is_cache_processing, req_addr, req_data);

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

	inout mem_data;
	wire [`WORD_SIZE-1:0] mem_data;

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
	reg [`WORD_SIZE-1:0] latest[0:`NUM_BLOCK-1];
	reg [`TAG_SIZE-1:0] tag[0:`NUM_BLOCK-1];
	reg valid[0:`NUM_BLOCK-1];

	reg [`WORD_SIZE-1:0] outputData1;
	reg [`WORD_SIZE-1:0] outputAddr1;

	reg [`WORD_SIZE-1:0] writeData;
	
	reg [1:0] temp_block_idx;

	wire[3:0] line_idx;
	wire[1:0] block_idx;
	wire[11:0] addr_tag;

	reg way;

	reg is_write;
	
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
	assign mem_data = (!mem_read_req)?((mem_write_req)?writeData:outputData1):`WORD_SIZE'bz;
	assign mem_addr = (is_cache_processing)?outputAddr1:((mem_write_req)?req_addr:`WORD_SIZE'bz);
	assign line_idx = 2*req_addr[3:2];
	assign block_idx = req_addr[1:0];
	assign addr_tag = req_addr[15:4];
//*******	ASSIGNMENT	*******//

//******* Sequential Logic *******//
   always@(posedge clk) begin
     if(!reset_n) begin
       state = 4'd0;
		latest[0] <= 16'd0;
		tag[0] <= 12'd0;
		valid[0] <= 0;
		latest[1] <= 16'd0;
		tag[1] <= 12'd0;
		valid[1] <= 0;
		latest[2] <= 16'd0;
		tag[2] <= 12'd0;
		valid[2] <= 0;
		latest[3] <= 16'd0;
		tag[3] <= 12'd0;
		valid[3] <= 0;
		latest[4] <= 16'd0;
		tag[4] <= 12'd0;
		valid[4] <= 0;
		latest[5] <= 16'd0;
		tag[5] <= 12'd0;
		valid[5] <= 0;
		latest[6] <= 16'd0;
		tag[6] <= 12'd0;
		valid[6] <= 0;
		latest[7] <= 16'd0;
		tag[7] <= 12'd0;
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
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				next_state = FETCH0;
			end
		end
		else if(cache_write_req) begin
			is_write <= 1;
			writeData <= req_data;
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				mem_write_req <=1;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				mem_write_req <=1;
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				mem_write_req <=0;
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
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				next_state = FETCH0;
			end
		end
		else if(cache_write_req) begin
			is_write <=1;
			writeData <= req_data;
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				mem_write_req <=1;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				mem_write_req <=1;
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				mem_write_req <=0;
				next_state = FETCH0;
			end
		end
		else begin
			mem_write_req <=0;
			next_state = IDLE;
		end
	end
	HIT2 : begin
		if(cache_read_req) begin
			is_write<=0;
			mem_write_req <=0;
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				next_state = FETCH0;
			end
		end
		else if(cache_write_req) begin
			is_write<=1;
			writeData<=req_data;
			if(addr_tag==tag[line_idx]&&valid[line_idx]) begin
				mem_write_req <=1;
				next_state = HIT1;
			end
			else if(addr_tag==tag[line_idx+1]&&valid[line_idx+1]) begin
				mem_write_req <=1;
				next_state = HIT2;
			end
			else if(!valid[line_idx]) begin // miss
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(!valid[line_idx+1]) begin
				way = 1'b1;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else if(latest[line_idx] <= latest[line_idx+1]) begin
				way = 1'b0;
				mem_write_req <=0;
				next_state = FETCH0;
			end
			else begin
				way = 1'b1;
				mem_write_req <=0;
				next_state = FETCH0;
			end
		end
		else begin
			mem_write_req <=0;
			next_state = IDLE;
		end
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
		if(!valid[line_idx]) begin
			next_state = MISS;
			is_cache_processing = 1;
		end
		else if(!valid[line_idx+1]) begin
			next_state = MISS;
			is_cache_processing = 1;
		end
		else if(latest[line_idx] <= latest[line_idx+1]) begin
			next_state = EVICT;
			is_cache_processing = 1;
		end
		else begin
			next_state = EVICT;
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
		if(is_write) begin
			case(block_idx)
			2'b00 : begin
				block1[line_idx] <= writeData;
			end
			2'b01 : begin
				block2[line_idx] <= writeData;
			end
			2'b10 : begin
				block3[line_idx] <= writeData;
			end
			2'b11 : begin
				block4[line_idx] <= writeData;
			end
			endcase
		end
		else begin
			case(block_idx)
			2'b00 : begin
				outputData1 <= block1[line_idx];
			end
			2'b01 : begin
				outputData1 <= block2[line_idx];
			end
			2'b10 : begin
				outputData1 <= block3[line_idx];
			end
			2'b11 : begin
				outputData1 <= block4[line_idx];
			end
			endcase
		end
	end
	HIT2 : begin
		if(is_write) begin
			case(block_idx)
			2'b00 : begin
				block1[line_idx+1] <= writeData;
			end
			2'b01 : begin
				block2[line_idx+1] <= writeData;
			end
			2'b10 : begin
				block3[line_idx+1] <= writeData;
			end
			2'b11 : begin
				block4[line_idx+1] <= writeData;
			end
			endcase
		end
		else begin
			case(block_idx)
			2'b00 : begin
				outputData1 <= block1[line_idx+1];
			end
			2'b01 : begin
				outputData1 <= block2[line_idx+1];
			end
			2'b10 : begin
				outputData1 <= block3[line_idx+1];
			end
			2'b11 : begin
				outputData1 <= block4[line_idx+1];
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
		tag[line_idx+way] <= mem_addr[15:4];
		latest[line_idx+way] <= latest[line_idx+way] +1;
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
		tag[line_idx+way] <= mem_addr[15:4];
		latest[line_idx+way] <= latest[line_idx+way] +1;
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
		latest[line_idx] <= latest[line_idx] +1;
	end
	HIT2 : begin
		num_hit <= num_hit+1;
		latest[line_idx+1] <= latest[line_idx+1] +1;
	end
	FETCH0 : begin
		mem_read_req<=1;
		outputAddr1 <= {outputAddr1[15:2],2'b00};
	end
	FETCH1 : begin
		block1[line_idx+way] <= mem_data;
		outputAddr1 <= {outputAddr1[15:2],2'b01};
	end
	FETCH2 : begin
		block2[line_idx+way] <= mem_data;
		outputAddr1 <= {outputAddr1[15:2],2'b10};
	end
	FETCH3 : begin
		block3[line_idx+way] <= mem_data;
		outputAddr1 <= {outputAddr1[15:2],2'b11};
	end
	FETCH4 : begin
		block4[line_idx+way] <= mem_data;
		mem_read_req<=0;
		if(is_write) begin
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
