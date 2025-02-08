`include "head.h"
module exe_stage(
    input                          clk           ,
    input                          reset         ,
    // state
    input  [3:0]                    state        ,
    output [3:0]                    next_state   ,
    //from ds
    input          ds_to_es_valid,
    input  [151:0] ds_to_es_bus  ,
    //to ms
    output         es_to_ms_valid,
    output [ 70:0] es_to_ms_bus  ,
    //to ws
    output         es_to_ws_valid,
    output [ 69:0] es_to_ws_bus  ,
    // data sram interface(write)
    output        data_sram_en   ,
    output [ 3:0] data_sram_we   ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata
);

wire valid;
assign valid = (state == `STATE_EX1) || (state == `STATE_EX2) || (state == `STATE_EX3) || (state == `STATE_EX4);

reg  [151:0] ds_to_es_bus_r;

wire [11:0] alu_op      ;
wire        es_load_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_4;
wire        res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire        es_mem_we;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire [31:0] es_pc;


assign {alu_op,
        es_load_op,
        src1_is_pc,
        src2_is_imm,
        src2_is_4,
        gr_we,
        es_mem_we,
        dest,
        imm,
        rj_value,
        rkd_value,
        es_pc,
        res_from_mem
       } = ds_to_es_bus_r;

always @(posedge clk) begin
    if (ds_to_es_valid) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;

assign es_to_ms_valid = (state == `STATE_EX3) || (state == `STATE_EX4);
assign es_to_ms_bus = {res_from_mem,  //70:70 1
                       gr_we       ,  //69:69 1
                       dest        ,  //68:64 5
                       alu_result  ,  //63:32 32
                       es_pc          //31:0  32
                      };

assign es_to_ws_valid = (state == `STATE_EX1) || (state == `STATE_EX2);
assign es_to_ws_bus = {gr_we       ,  //69:69 1
                       dest        ,  //68:64 5
                       alu_result  ,  //63:32 32
                       es_pc          //31:0  32
                      };
                
assign alu_src1 = src1_is_pc  ? es_pc  : rj_value;
assign alu_src2 = src2_is_imm ? imm : rkd_value;

assign next_state = (state == `STATE_EX1) ? `STATE_IF:
                    (state == `STATE_EX2) ? `STATE_WB:
                    (state == `STATE_EX3) ? `STATE_MEM1:
                    (state == `STATE_EX4) ? `STATE_MEM2:
                    `STATE_IF;

alu u_alu(
    .alu_op     (alu_op    ),
    .alu_src1   (alu_src1  ),
    .alu_src2   (alu_src2  ),
    .alu_result (alu_result)
    );

assign data_sram_en    = 1'b1;
assign data_sram_we    = es_mem_we && valid ? 4'hf : 4'h0;
assign data_sram_addr  = alu_result;
assign data_sram_wdata = rkd_value;

endmodule