`include "head.h"
module wb_stage(
    input  wire         clk           ,
    input  wire         reset         ,
    // state
    input  wire [3:0]        state         ,
    output wire [3:0]        next_state    ,
    //from ms
    input  wire         ms_to_ws_valid,
    input  wire [69:0]  ms_to_ws_bus  ,
    //from es
    input  wire         es_to_ws_valid,
    input  wire [69:0]  es_to_ws_bus  ,
    //to rf: for write back
    output wire         ws_to_rf_valid,
    output wire [37:0]  ws_to_rf_bus  ,
    //trace debug interface
    output wire [31:0] debug_wb_pc     ,
    output wire [ 3:0] debug_wb_rf_we  ,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);

wire valid;
assign valid = state == `STATE_WB;

reg  [69:0] to_ws_bus_r;
wire        ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
assign {ws_gr_we       ,  //69:69
        ws_dest        ,  //68:64
        ws_final_result,  //63:32
        ws_pc             //31:0
       } = to_ws_bus_r;

wire        rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;
assign ws_to_rf_valid = valid;
assign ws_to_rf_bus = {rf_we   ,  //37:37
                       rf_waddr,  //36:32
                       rf_wdata   //31:0
                      };

assign next_state = `STATE_IF;

always @(posedge clk) begin
    if (es_to_ws_valid) begin
        to_ws_bus_r <= es_to_ws_bus;
    end
    else if (ms_to_ws_valid) begin
        to_ws_bus_r <= ms_to_ws_bus;
    end
end

assign rf_we    = ws_gr_we && valid;
assign rf_waddr = ws_dest;
assign rf_wdata = ws_final_result;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_we    = {4{rf_we}};
assign debug_wb_rf_wnum  = ws_dest;
assign debug_wb_rf_wdata = ws_final_result;

endmodule