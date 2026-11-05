`timescale  1ns/1ps
/*
Author: Jingling Hou

Module Description: Read only AXI Full wrapper for the accelector module
*/
module accelector_wrapper #(
    // Base address of targeted slave
    parameter  C_M_TARGET_SLAVE_BASE_ADDR = 32'h0000_0000, //DDR base addr
    // Burst length (beats per burst)
    parameter integer C_M_AXI_BURST_LEN   = 256,
    // Widths
    parameter integer C_M_AXI_ID_WIDTH    = 1,
    parameter integer C_M_AXI_ADDR_WIDTH  = 32,
    parameter integer C_M_AXI_DATA_WIDTH  = 16,
    // User‐side widths (unused here)
    parameter integer C_M_AXI_ARUSER_WIDTH = 0,
    parameter integer C_M_AXI_RUSER_WIDTH  = 0
) (
    // Control
    input  wire                      INIT_AXI_TXN,
    output wire                      TXN_DONE,
    output reg                       ERROR,
    // Clock / reset
    input  wire                      M_AXI_ACLK,
    input  wire                      M_AXI_ARESETN,

    // AXI Read Address Channel
    output wire [C_M_AXI_ID_WIDTH-1:0]   M_AXI_ARID,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR,
    output wire [7:0]                    M_AXI_ARLEN,
    output wire [2:0]                    M_AXI_ARSIZE,
    output wire [1:0]                    M_AXI_ARBURST,
    output wire                          M_AXI_ARLOCK,
    output wire [3:0]                    M_AXI_ARCACHE,
    output wire [2:0]                    M_AXI_ARPROT,
    output wire [3:0]                    M_AXI_ARQOS,
    output wire [C_M_AXI_ARUSER_WIDTH-1:0] M_AXI_ARUSER,
    output wire                          M_AXI_ARVALID,
    input  wire                          M_AXI_ARREADY,

    // AXI Read Data Channel
    input  wire [C_M_AXI_ID_WIDTH-1:0]     M_AXI_RID,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]   M_AXI_RDATA,
    input  wire [1:0]                      M_AXI_RRESP,
    input  wire                          M_AXI_RLAST,
    input  wire [C_M_AXI_RUSER_WIDTH-1:0]  M_AXI_RUSER,
    input  wire                          M_AXI_RVALID,
    output wire                          M_AXI_RREADY
);
    // Helper function declaration
    function integer clogb2(input integer bit_depth);
    // returns ceiling(log2(bit_depth))
    // used to calculate ARSIZE, the number byte of each beat within one burst
    // since DATA_WIDTH is 16 bits, ARSIZE should be 2^1
    begin
        for (clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1)
            bit_depth = bit_depth >> 1;
    end
    endfunction

    //width of beat counter of each burst
    localparam integer C_BURST_NUM = clogb2(C_M_AXI_BURST_LEN - 1);

    //FSM states
    localparam [1:0] IDLE = 2'b00, READ_ADDR = 2'b01, READ_DATA = 2'b10, DONE = 2'b11;

    //Internal Reg
    reg [1:0]                              state_read;
    reg                                    axi_arvalid_reg;
    reg                                    axi_rready_reg;
    reg [C_M_AXI_ADDR_WIDTH-1:0]           axi_araddr;
    reg [C_BURST_NUM:0]                    read_index;
    reg                                    init_ff, init_ff2;

    wire [C_BURST_NUM + 1: 0] burst_size_bytes = C_M_AXI_BURST_LEN * (C_M_AXI_DATA_WIDTH/8); //256 * 2

    // Handshake helpers
    wire init_pulse = INIT_AXI_TXN & ~init_ff2; //Note this will result in a one time transaction, if need continuous burst transaction, change init pulse generator
    wire rnext      = M_AXI_RVALID & axi_rready_reg; //Read Next 

    // AXI I/O assignments
    assign M_AXI_ARID    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign M_AXI_ARADDR  = C_M_TARGET_SLAVE_BASE_ADDR + axi_araddr;
    assign M_AXI_ARLEN   = C_M_AXI_BURST_LEN - 1;
    assign M_AXI_ARSIZE  = clogb2((C_M_AXI_DATA_WIDTH/8)-1);
    assign M_AXI_ARBURST = 2'b01;            // INCR
    assign M_AXI_ARLOCK  = 1'b0;
    assign M_AXI_ARCACHE = 4'b0010;
    assign M_AXI_ARPROT  = 3'b000;
    assign M_AXI_ARQOS   = 4'b0000;
    assign M_AXI_ARUSER  = 'b1;
    assign M_AXI_ARVALID = axi_arvalid_reg;
    assign M_AXI_RREADY  = axi_rready_reg;
    
    assign TXN_DONE = (state_read == DONE);

    // init pulse generate
    always @(posedge M_AXI_ACLK) begin
        if(!M_AXI_ARESETN)
        begin
            init_ff <= 'b0;
            init_ff2 <= 'b0;
        end
        else 
        begin
            init_ff <= INIT_AXI_TXN;
            init_ff2 <= init_ff; //Shift reg
        end
    end

    //FSM (Read Only AXI Full)
    //Only Read channels implemented

    //FSM design logic (Fully registered one process FSM):
    /*
    IDLE -> init pulse -> RADDR
    RADDR -> ARvalid && ARready handshake -> RDATA
    For this stage, Rready is asserted and the address is incremented by burst size
    RDATA -> Rvalid && Rready handshake -> check if Rlast
                                            -> if Rlast, reset read index and prepare for the next transaction
                                            -> if not Rlast, but read index reaches maximum, 
    For this stage, Rread is deasserted @ RLast high
    */

    reg [C_M_AXI_DATA_WIDTH - 1:0] read_buffer[0: C_M_AXI_BURST_LEN - 1];

    always @(posedge M_AXI_ACLK) begin
        if(!M_AXI_ARESETN || init_pulse)
        begin
            state_read <= IDLE;
            axi_arvalid_reg <= 'b0;
            axi_rready_reg <= 'b0;
            axi_araddr <= 0;
            read_index <= 0;
            ERROR <= 1'b0;
        end
        case(state_read)
            IDLE:
            begin
                if(init_pulse) begin
                    axi_arvalid_reg <= 1'b1;
                    axi_rready_reg <= 1'b0;
                    state_read <= READ_ADDR;
                    ERROR <= 1'b0;
                end
                else 
                begin
                    state_read <= IDLE;
                end
            end

            READ_ADDR:
            begin
                if(M_AXI_ARREADY && axi_arvalid_reg) begin
                    axi_arvalid_reg <= 1'b0; //ARvalid deasserted
                    axi_rready_reg <= 1'b1; //Rready asserted
                    axi_araddr <= axi_araddr + burst_size_bytes; //Increment address by burst size
                    state_read <= READ_DATA;
                end
                else
                    state_read <= state_read; //Stay in Read Addr state, waiting for slave to respond
            end

            READ_DATA:
            begin
                if(rnext) begin
                    //Data capture logic here
                    read_buffer[read_index] <= M_AXI_RDATA;

                    if(M_AXI_RLAST) begin
                        axi_rready_reg <= 1'b0; //Rready deasserted
                        axi_arvalid_reg <= 1'b1;
                        read_index <= 0;
                        state_read <= READ_ADDR;
                    end
                    else if (read_index == C_M_AXI_BURST_LEN)
                    begin
                        state_read <= IDLE;
                        //ERROR Rlast should be triggered before this clock cycle
                        ERROR <= 1'b1;
                    end
                    else begin
                        axi_rready_reg <= 1'b1; //Rready asserted
                        read_index <= read_index + 1;
                        state_read <= READ_DATA;
                    end
                end
                else begin
                    state_read <= IDLE;
                    //ERROR
                    ERROR <= 1'b1;
                end
            end


        endcase
    end
    
endmodule