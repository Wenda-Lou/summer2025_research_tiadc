`timescale 1ns/1ps

module tb ();

    logic m_axi_aclk, m_axi_arestn;

    localparam clock_cycle = 10; // 10ns

    typedef enum logic[1:0] {OKAY, EXOKAY, SLVERR, DECERR} rresp_state;

    always
    begin : clock_generator
        #(clock_cycle/2) m_axi_aclk <= ~m_axi_aclk;
    end

    initial begin
        m_axi_aclk <= 1'b0;
        m_axi_arestn = 1'b0;
        #15;
    end

    //DUT spec section
    // Burst length (beats per burst)
    parameter integer C_M_AXI_BURST_LEN   = 256;
    // Widths
    parameter integer C_M_AXI_ID_WIDTH    = 1;
    parameter integer C_M_AXI_ADDR_WIDTH  = 32;
    parameter integer C_M_AXI_DATA_WIDTH  = 16;
    // User‐side widths (unused here)
    parameter integer C_M_AXI_ARUSER_WIDTH = 0;
    parameter integer C_M_AXI_RUSER_WIDTH  = 0;

    //input logics
    logic init_txn, axi_arready;
    logic [C_M_AXI_ID_WIDTH - 1: 0] axi_rid;
    logic [C_M_AXI_DATA_WIDTH - 1: 0] axi_rdata;
    rresp_state axi_rresp;
    logic axi_rlast;
    logic [C_M_AXI_RUSER_WIDTH - 1:0] axi_ruser;
    logic axi_rvalid;

    //output logics
    logic txn_done, error;
    logic [C_M_AXI_ID_WIDTH-1:0]   M_AXI_ARID;
    logic [C_M_AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR;
    logic [7:0]                    M_AXI_ARLEN;
    logic [2:0]                    M_AXI_ARSIZE;
    logic [1:0]                    M_AXI_ARBURST;
    logic                          M_AXI_ARLOCK;
    logic [3:0]                    M_AXI_ARCACHE;
    logic [2:0]                    M_AXI_ARPROT;
    logic [3:0]                    M_AXI_ARQOS;
    logic [C_M_AXI_ARUSER_WIDTH-1:0] M_AXI_ARUSER;
    logic                          M_AXI_ARVALID;
    logic                          axi_rready;

    int i;
    logic [31:0] read_index;
    logic [C_M_AXI_DATA_WIDTH - 1:0] mem [0: C_M_AXI_BURST_LEN - 1];
    initial
    begin : assign_mem
        for (i = 0; i < C_M_AXI_BURST_LEN; i ++)
        begin
            mem[i] = 16'hbeef - i;
        end
        read_index = 0;
    end

    //assign input value
    assign init_txn = 1'b1;
    assign axi_rlast = (read_index == C_M_AXI_BURST_LEN - 1);
    assign axi_rresp = OKAY;
    assign axi_arready = 1'b1;
    assign axi_ruser = 'b1;
    assign axi_rvalid = 1'b1;
    assign axi_rid = 'b0;

    always @ (posedge m_axi_aclk)
    begin
        if (axi_rready)
        begin
            axi_rdata <= mem[read_index];
            read_index <= read_index + 1;
            if (axi_rlast)
                read_index <= 0;
        end
    end

    accelector_wrapper DUT (
    .INIT_AXI_TXN    (init_txn),
    .TXN_DONE        (txn_done),
    .ERROR           (error),
    // Clock / reset
    .M_AXI_ACLK      (m_axi_aclk),
    .M_AXI_ARESETN   (m_axi_arestn),

    // AXI Read Address Channel
    .M_AXI_ARID      (M_AXI_ARID),
    .M_AXI_ARADDR    (M_AXI_ARADDR),
    .M_AXI_ARLEN     (M_AXI_ARLEN),
    .M_AXI_ARSIZE    (M_AXI_ARSIZE),
    .M_AXI_ARBURST   (M_AXI_ARBURST),
    .M_AXI_ARLOCK    (M_AXI_ARLOCK),
    .M_AXI_ARCACHE   (M_AXI_ARCACHE),
    .M_AXI_ARPROT    (M_AXI_ARPROT),
    .M_AXI_ARQOS     (M_AXI_ARQOS),
    .M_AXI_ARUSER    (M_AXI_ARUSER),
    .M_AXI_ARVALID   (M_AXI_ARVALID),
    .M_AXI_ARREADY   (axi_arready),

    // AXI Read Data Channel
    .M_AXI_RID       (axi_rid),
    .M_AXI_RDATA     (axi_rdata),
    .M_AXI_RRESP     (axi_rresp),
    .M_AXI_RLAST     (axi_rlast),
    .M_AXI_RUSER     (axi_ruser),
    .M_AXI_RVALID    (axi_rvalid),
    .M_AXI_RREADY    (axi_rready)
    );
endmodule