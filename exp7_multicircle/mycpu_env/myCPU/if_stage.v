`include "head.h"
module if_stage(
    input         clk,
    input         reset,
    input  [ 3:0] state,
    output [ 3:0] next_state,
    input  [32:0] br_bus,          // {br_taken, br_target}

    // to ds
    output        fs_to_ds_valid ,
    output [31:0] fs_to_ds_bus,    // {fs_pc}

    // inst sram interface
    output        inst_sram_en,
    output [ 3:0] inst_sram_we,
    output [31:0] inst_sram_addr,
    output [31:0] inst_sram_wdata
);
wire valid;
assign valid = state == `STATE_IF;

wire [31:0] seq_pc;
wire [31:0] nextpc;

wire         br_taken;
wire [ 31:0] br_target;
assign {br_taken, br_target} = br_bus;

// wire [31:0] fs_inst;
reg  [31:0] fs_pc;

assign fs_to_ds_valid = valid;
// assign fs_to_ds_bus = {fs_inst, nextpc};
assign fs_to_ds_bus = {nextpc};

assign seq_pc       = fs_pc + 3'h4;
assign nextpc       = br_taken ? br_target : seq_pc; 

assign next_state = reset? `STATE_IF:
                    valid? `STATE_ID:
                    `STATE_IF;

always @(posedge clk) begin
    if (reset) begin
        // fs_pc <= 32'h1c000000;
        fs_pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else if(valid) begin 
        fs_pc <= nextpc;
    end
end

assign inst_sram_en    = 1;
assign inst_sram_we    = 4'h0;
assign inst_sram_addr  = nextpc;
assign inst_sram_wdata = 32'b0;

// assign fs_inst         = inst_sram_rdata;

endmodule