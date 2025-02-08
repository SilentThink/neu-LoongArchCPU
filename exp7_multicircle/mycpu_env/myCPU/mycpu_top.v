`include "head.h"

module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

// 多周期
reg [3:0] state;
wire [3:0] next_state_if;
wire [3:0] next_state_id;
wire [3:0] next_state_exe;
wire [3:0] next_state_mem;
wire [3:0] next_state_wb;


always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= `STATE_IF; // 初始状态
    end else begin
        // 根据实际需求选择 `next_state`
        case (state)
            `STATE_IF: state <= next_state_if;
            `STATE_ID: state <= next_state_id;
            `STATE_EX1,`STATE_EX2,`STATE_EX3,`STATE_EX4: state <= next_state_exe;
            `STATE_MEM1,`STATE_MEM2: state <= next_state_mem;
            `STATE_WB: state <= next_state_wb;
            default: state <= `STATE_IF;
        endcase
    end
end

// 通过总线连接各个模块缓存
wire [ 31:0] fs_to_ds_bus;
wire [151:0] ds_to_es_bus;
wire [ 70:0] es_to_ms_bus;
wire [ 69:0] es_to_ws_bus;
wire [ 69:0] ms_to_ws_bus;
wire [ 37:0] ws_to_rf_bus;
wire [ 32:0] br_bus;

wire fs_to_ds_valid;
wire ds_to_es_valid;
wire es_to_ms_valid;
wire es_to_ws_valid;
wire ms_to_ws_valid;

// IF stage
if_stage if_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    .state          (state          ),
    .next_state     (next_state_if  ),
    //brbus
    .br_bus         (br_bus         ),
    //outputs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    // inst sram interface
    .inst_sram_en   (inst_sram_en   ),
    .inst_sram_we   (inst_sram_we  ),
    .inst_sram_addr (inst_sram_addr ),
    .inst_sram_wdata(inst_sram_wdata)
    
);
// ID stage
id_stage id_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    // state
    .state          (state          ),
    .next_state     (next_state_id  ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs
    .br_bus         (br_bus         ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    .inst_sram_rdata(inst_sram_rdata)
);
// EXE stage
exe_stage exe_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    // state
    .state          (state          ),
    .next_state     (next_state_exe ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .es_to_ws_valid (es_to_ws_valid ),
    .es_to_ws_bus   (es_to_ws_bus   ),
    // data sram interface
    .data_sram_en   (data_sram_en   ),
    .data_sram_we   (data_sram_we  ),
    .data_sram_addr (data_sram_addr ),
    .data_sram_wdata(data_sram_wdata)
);
// MEM stage
mem_stage mem_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //state
    .state          (state          ),
    .next_state     (next_state_mem ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from data-sram
    .data_sram_rdata(data_sram_rdata)
);
// WB stage
wb_stage wb_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //state
    .state          (state          ),
    .next_state     (next_state_wb  ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from es
    .es_to_ws_valid (es_to_ws_valid ),
    .es_to_ws_bus   (es_to_ws_bus   ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //trace debug interface
    .debug_wb_pc      (debug_wb_pc      ),
    .debug_wb_rf_we   (debug_wb_rf_we   ),
    .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(debug_wb_rf_wdata)
);

endmodule