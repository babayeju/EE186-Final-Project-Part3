`include "defines.sv"

module controller(

    // Clock and Reset
    input wire clk,
    input wire rst_n,

    // Input Instruction
    input  instruction_t inst,
    input  logic         inst_valid,
    output logic         inst_exec_begins,

    // Output Instructions to PEs/Buffer
    output pe_inst_t     pe_inst,
    output logic         pe_inst_valid,
    output buf_inst_t    buf_inst,
    output logic         buf_inst_valid

);

    typedef enum {IDLE, EXECUTING} fsm_t; 
    fsm_t state;
    fsm_t next_state;

    logic[`BUF_MEMA_OFFSET_BITWIDTH-1:0] current_mema_offset, next_mema_offset;
    logic[`BUF_MEMB_OFFSET_BITWIDTH-1:0] current_memb_offset, next_memb_offset;
    logic [`CONTROLLER_COUNT_BITWIDTH-1:0] current_count, next_count;

    instruction_t current_instruction;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            current_count <= '0;
            current_instruction <= '0;
            current_mema_offset <= '0;
            current_memb_offset <= '0;
        
        end else begin
            state <= next_state;
            current_count <= next_count;
            current_mema_offset <= next_mema_offset;
            current_memb_offset <= next_memb_offset;

            if (state == IDLE && inst_valid) begin
                current_instruction <= inst;
            end

        end
    end

    always_comb begin
        pe_inst = '0;
        buf_inst = '0;
        inst_exec_begins = 1'b0;
        pe_inst_valid = 1'b0;
        buf_inst_valid = 1'b0;
    
        next_state = state;
        next_count = current_count;
        next_mema_offset = current_mema_offset;
        next_memb_offset = current_memb_offset;

        case (state)
            IDLE: begin
                if (inst_valid) begin

                    next_mema_offset = inst.buf_instruction.mema_offset;
                    next_memb_offset = inst.buf_instruction.memb_offset;

                    next_count = inst.count;
                    next_state = EXECUTING;
                    inst_exec_begins = 1'b1;
                end
            end

            EXECUTING: begin

                next_mema_offset = current_mema_offset + current_instruction.mema_inc;
                next_memb_offset = current_memb_offset + current_instruction.memb_inc;  
                buf_inst.opcode = current_instruction.buf_instruction.opcode;
                buf_inst.mode = current_instruction.buf_instruction.mode;
                
                buf_inst.mema_offset = current_mema_offset;
                buf_inst.memb_offset = current_memb_offset;
                pe_inst = current_instruction.pe_instruction;

                pe_inst_valid = 1'b1;
                buf_inst_valid = 1'b1;

                if (current_count == 0) begin 
                    next_count = current_count;
                    next_state = IDLE;
                end else begin
                    next_count = current_count - 1;
                    next_state = EXECUTING;
                end
            end

            default: begin
                next_state = IDLE;
            end

        endcase
    end
endmodule
