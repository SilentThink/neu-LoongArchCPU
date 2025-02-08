`include "head.h"
module mem_stage(
    input         clk           ,
    input         reset         ,
    // state
    input  [3:0]      state         ,
    output [3:0]      next_state    ,
    //from es
    input         es_to_ms_valid,
    input  [70:0] es_to_ms_bus  ,
    //to ws
    output        ms_to_ws_valid,
    output [69:0] ms_to_ws_bus  ,
    
    //from data-sram
    input  [31:0] data_sram_rdata
);

wire valid;
assign valid = (state == `STATE_MEM1) || (state == `STATE_MEM2);

reg  [70:0] es_to_ms_bus_r;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;

wire [31:0] mem_result;
wire [31:0] ms_final_result;


assign {ms_res_from_mem,  //70:70
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

always @(posedge clk) begin
    if (es_to_ms_valid) begin
        es_to_ms_bus_r <= es_to_ms_bus;
    end
end

assign ms_to_ws_valid = valid;
assign ms_to_ws_bus = {ms_gr_we       ,  //69:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };


assign next_state = (state == `STATE_MEM1)? `STATE_WB : `STATE_IF;

assign mem_result   = data_sram_rdata;
assign ms_final_result = ms_res_from_mem ? mem_result : ms_alu_result;

endmodule