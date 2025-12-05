`timescale 1ns / 1ps
`include "defines.sv"

module buffer (
    // Clock and Reset
    input wire clk,
    input wire rst_n,

    // Input Instruction
    input buf_inst_t buf_inst,
    input logic buf_inst_valid,

    // Outputs
    output logic [`MEM0_BITWIDTH-1:0] matrix_data,
    output logic [`MEM1_BITWIDTH-1:0] vector_data,
    input  logic [`MEM2_BITWIDTH-1:0] output_data
);

    logic [`MEM0_ADDR_WIDTH-1:0] mem0_addr;
    logic [`MEM0_BITWIDTH-1:0] mem0_output;
    logic [`MEM0_ADDR_WIDTH-1:0] mem0_addr_read;
    logic [`MEM1_ADDR_WIDTH-1:0] mem1_addr;
    logic [`MEM1_BITWIDTH-1:0] mem1_output;
    logic [`MEM1_BITWIDTH-1:0] vector_decoder_output;
    logic [`MEM1_BITWIDTH-1:0] vector_data_final;

    logic [`BUF_MEMB_OFFSET_BITWIDTH-1:0] memb_offset;
    logic [`MEM2_ADDR_WIDTH-1:0] mem2_addr;
    buf_inst_t buf_inst_delay;
    logic buf_inst_valid_delay;
    logic write_control_n;

    array #(
        .DW(`MEM0_BITWIDTH),
        .NW(`MEM0_DEPTH),
        .AW(`MEM0_ADDR_WIDTH)
    ) u_matrix_mem (
        .clk(clk),
        .cen('0),          
        .wen('1),         
        .gwen('1),          
        .a(mem0_addr),  
        .d(),         
        .q(mem0_output)       
    );

    array #(
        .DW(`MEM1_BITWIDTH),
        .NW(`MEM1_DEPTH),
        .AW(`MEM1_ADDR_WIDTH)
    ) u_vector_mem (
        .clk(clk),
        .cen('0),          
        .wen('1),         
        .gwen('1),         
        .a(mem1_addr),   
        .d(),          
        .q(mem1_output)       
    );

    // get values calculated already
    vector_decoder u_vector_decoder (
        .data_from_mem(mem1_output),
        .addr_from_controller(buf_inst.memb_offset),
        .addr_from_controller_reg(memb_offset),
        .mode(buf_inst_delay.mode),
        .data_to_pe(vector_decoder_output),
        .addr_to_mem(mem1_addr)
    );
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_inst_delay <= '0;
            buf_inst_valid_delay <= '0;
        end
        else begin
            buf_inst_delay <= buf_inst;
            buf_inst_valid_delay <= buf_inst_valid;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem0_addr_read <= '0;
        end else if (buf_inst_valid && (buf_inst.opcode == `BUF_READ)) begin
            mem0_addr_read <= buf_inst.mema_offset;
        end
    end
    assign mem0_addr = mem0_addr_read;
    assign matrix_data = mem0_output;
    
    logic read_enable, read_enable_delay;
    assign read_enable = buf_inst_valid && (buf_inst.opcode == `BUF_READ);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            read_enable_delay <= '0;
        else
            read_enable_delay <= read_enable;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            vector_data_final <= '0;
        else if (read_enable_delay)
            vector_data_final <= vector_decoder_output;
    end
    assign vector_data = vector_data_final;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            memb_offset <= '0;
        end

        else if (read_enable) begin 
            memb_offset <= buf_inst.memb_offset;
        end
    end

    array #(
        .DW(`MEM2_BITWIDTH),
        .NW(`MEM2_DEPTH),
        .AW(`MEM2_ADDR_WIDTH),
        .INITIALIZE_MEMORY(1)
    ) u_output_mem (
        .clk(clk),
        .cen('0),             
        .wen('0),             
        .gwen(write_control_n),
        .a(mem2_addr),      
        .d(output_data),    
        .q()         
    );

    assign write_control_n = buf_inst_valid_delay && 
    (buf_inst_delay.opcode == `BUF_WRITE) ? 1'b0 : 1'b1;

    always_comb begin
        mem2_addr = '0;
        if (buf_inst_valid_delay && (buf_inst_delay.opcode == `BUF_WRITE)) begin
            mem2_addr = buf_inst_delay.mema_offset;
        end
    end

endmodule

