`timescale 1ns/1ps

module tb_axi_lite_wrapper();

  // Parameters must match the DUT
  localparam C_M_START_DATA_VALUE     = 32'hAA000000;
  localparam C_M_TARGET_SLAVE_BASE_ADDR = 32'h40000000;
  localparam C_M_AXI_ADDR_WIDTH       = 32;
  localparam C_M_AXI_DATA_WIDTH       = 32;
  localparam C_M_TRANSACTIONS_NUM     = 1024;

  // Clock & reset
  reg  M_AXI_ACLK;
  reg  M_AXI_ARESETN;

  // Control inputs
  reg INIT_AXI_TXN;
  reg INIT_AXI_WRITE;
  reg INIT_AXI_READ;

  // DUT status outputs
  wire ERROR;
  wire TXN_DONE;

  // AXI master interface (from wrapper)
  wire [C_M_AXI_ADDR_WIDTH-1:0] M_AXI_AWADDR;
  wire [2:0]                    M_AXI_AWPROT;
  wire                          M_AXI_AWVALID;
  reg                           M_AXI_AWREADY;

  wire [C_M_AXI_DATA_WIDTH-1:0] M_AXI_WDATA;
  wire [C_M_AXI_DATA_WIDTH/8-1:0] M_AXI_WSTRB;
  wire                          M_AXI_WVALID;
  reg                           M_AXI_WREADY;

  reg  [1:0]                    M_AXI_BRESP;
  reg                           M_AXI_BVALID;
  wire                          M_AXI_BREADY;

  wire [C_M_AXI_ADDR_WIDTH-1:0] M_AXI_ARADDR;
  wire [2:0]                    M_AXI_ARPROT;
  wire                          M_AXI_ARVALID;
  reg                           M_AXI_ARREADY;

  reg  [C_M_AXI_DATA_WIDTH-1:0] M_AXI_RDATA;
  reg  [1:0]                    M_AXI_RRESP;
  reg                           M_AXI_RVALID;
  wire                          M_AXI_RREADY;

  // Instantiate the DUT

  axi_lite_wrapper dut (
  .m00_axi_read      (INIT_AXI_READ),
  .m00_axi_write     (INIT_AXI_WRITE),
  .m00_axi_init_axi_txn   (INIT_AXI_TXN),
  .m00_axi_error          (ERROR),
  .m00_axi_txn_done       (TXN_DONE),
  .m00_axi_aclk           (M_AXI_ACLK),
  .m00_axi_aresetn        (M_AXI_ARESETN),
  .m00_axi_awaddr         (M_AXI_AWADDR),
  .m00_axi_awprot         (M_AXI_AWPROT),
  .m00_axi_awvalid        (M_AXI_AWVALID),
  .m00_axi_awready        (M_AXI_AWREADY),
  .m00_axi_wdata          (M_AXI_WDATA),
  .m00_axi_wstrb          (M_AXI_WSTRB),
  .m00_axi_wvalid         (M_AXI_WVALID),
  .m00_axi_wready         (M_AXI_WREADY),
  .m00_axi_bresp          (M_AXI_BRESP),
  .m00_axi_bvalid         (M_AXI_BVALID),
  .m00_axi_bready         (M_AXI_BREADY),
  .m00_axi_araddr         (M_AXI_ARADDR),
  .m00_axi_arprot         (M_AXI_ARPROT),
  .m00_axi_arvalid        (M_AXI_ARVALID),
  .m00_axi_arready        (M_AXI_ARREADY),
  .m00_axi_rdata          (M_AXI_RDATA),
  .m00_axi_rresp          (M_AXI_RRESP),
  .m00_axi_rvalid         (M_AXI_RVALID),
  .m00_axi_rready         (M_AXI_RREADY)
  );

  // Simple AXI-Lite slave memory (256 words)
  reg [C_M_AXI_DATA_WIDTH-1:0] slave_mem [0:C_M_TRANSACTIONS_NUM];
  reg [C_M_AXI_ADDR_WIDTH-1:0] addr_reg;

  integer i;

  // Clock generation: 10ns period
  initial begin
    M_AXI_ACLK = 0;
  end
  
  always begin
    #1 M_AXI_ACLK <= ~M_AXI_ACLK;
  end 

  // Reset & test sequence
  initial begin
    // Initialize signals
    M_AXI_ARESETN   = 0;
    INIT_AXI_TXN    = 0;
    INIT_AXI_WRITE  = 0;
    INIT_AXI_READ   = 0;

    // Clear slave memory
    for (i = 0; i < C_M_TRANSACTIONS_NUM; i = i + 1)
      slave_mem[i] = 0;

    // Hold reset for 100ns
    #100;
    M_AXI_ARESETN = 1;

    // ===== 1) Issue a write burst =====
    // pulse_txn();
    pulse_write();

    // Wait enough time for C_M_TRANSACTIONS_NUM writes
    @ (posedge dut.axi_lite_wrapper_master_lite_v1_0_M00_AXI_inst.writes_done);
    #5; //Wait for a short period and let the last wdata be stored

    $display("\n--- Slave Mem Contents ---");
    check_slave_mem();

    // ===== 2) Issue a read burst =====
    // pulse_txn();
    pulse_read();

    // Wait enough time for reads
    @ (posedge dut.axi_lite_wrapper_master_lite_v1_0_M00_AXI_inst.reads_done);

    // ===== 3) Check results =====
    $display("\n---- Read Buffer Contents ----");
    // for (i = 0; i < C_M_TRANSACTIONS_NUM; i = i + 1) begin
    //   $display("  read_buffer[%0d] = 0x%08h  (expected = 0x%08h)",
    //             i,
    //             dut.axi_lite_wrapper_master_lite_v1_0_M00_AXI_inst.read_buffer[i],
    //             C_M_START_DATA_VALUE + i);
    // end
    check_read_buffer();

    if (ERROR)
      $display("\n>>>> ERROR flag is asserted in DUT!");
    else
      $display("\n>>>> NO ERROR");

    $display("TEST PASSED");
    $finish;
  end

  //Task: Check if slave mem reads the correct output
  task check_slave_mem;
  begin
    int i;
    for (i = 0; i < C_M_TRANSACTIONS_NUM; i = i + 1) begin
      assert (slave_mem[i] == C_M_START_DATA_VALUE + i) 
      else   
      begin
        $fatal(1, "@ slave mem index %0d, expecting %0h, received %0h", i, C_M_START_DATA_VALUE + i, slave_mem[i]);
        $finish;
      end
    end
    $display("Master Write Checked.\nWRITE OK");
  end
  endtask

  //Task: Check if read buffer reads the correct input
  task check_read_buffer;
  begin
    int i;
    for (i = 0; i < C_M_TRANSACTIONS_NUM; i = i + 1) begin
      assert (dut.axi_lite_wrapper_master_lite_v1_0_M00_AXI_inst.read_buffer[i] == slave_mem[i]) 
      else
      begin
        $fatal(1, "@ read buffer index %0d, expecting %0d, received %0d", i, slave_mem[i], dut.axi_lite_wrapper_master_lite_v1_0_M00_AXI_inst.read_buffer[i]);
        $finish;
      end
    end
    $display("Master Read Checked. \nREAD OK");
  end
  endtask

  // Task: pulse INIT_AXI_TXN for one cycle
  task pulse_txn;
    begin
      @ (posedge M_AXI_ACLK);
      INIT_AXI_TXN = 1;
      @ (posedge M_AXI_ACLK);
      INIT_AXI_TXN = 0;
    end
  endtask

  // Task: pulse INIT_AXI_WRITE for one cycle
  task pulse_write;
    begin
      @ (posedge M_AXI_ACLK);
      INIT_AXI_WRITE = 1;
      INIT_AXI_TXN = 1;
      @ (posedge M_AXI_ACLK);
      INIT_AXI_WRITE = 0;
      INIT_AXI_TXN = 0;

    end
  endtask

  // Task: pulse INIT_AXI_READ for one cycle
  task pulse_read;
    begin
      @ (posedge M_AXI_ACLK);
      INIT_AXI_READ = 1;
      INIT_AXI_TXN = 1;
      @ (posedge M_AXI_ACLK);
      INIT_AXI_READ = 0;
      INIT_AXI_TXN = 0;
    end
  endtask


  // Simple AXI-Lite Slave Handshaking Model

  // AWREADY: accept write address one cycle after AWVALID
  always @(posedge M_AXI_ACLK) begin
    if (!M_AXI_ARESETN)
      M_AXI_AWREADY <= 0;
    else if (M_AXI_AWVALID && !M_AXI_AWREADY)
      M_AXI_AWREADY <= 1;
    else
      M_AXI_AWREADY <= 0;
  end

  // WREADY: accept write data one cycle after WVALID
  always @(posedge M_AXI_ACLK) begin
    if (!M_AXI_ARESETN)
      M_AXI_WREADY <= 0;
    else if (M_AXI_WVALID && !M_AXI_WREADY)
      M_AXI_WREADY <= 1;
    else
      M_AXI_WREADY <= 0;
  end

  // BVALID/BRESP: once address+data handshake completes, respond OKAY
  always @(posedge M_AXI_ACLK) begin
    if (!M_AXI_ARESETN) begin
      M_AXI_BVALID <= 0;
      M_AXI_BRESP  <= 2'b00;
    end
    else if (M_AXI_AWREADY && M_AXI_AWVALID && M_AXI_WREADY && M_AXI_WVALID) begin
      addr_reg <= M_AXI_AWADDR;
      slave_mem[M_AXI_AWADDR[12:2]] <= M_AXI_WDATA;
      M_AXI_BVALID <= 1;
      M_AXI_BRESP  <= 2'b00; // OKAY
    end
    else if (M_AXI_BVALID && M_AXI_BREADY) begin
      M_AXI_BVALID <= 0;
    end
  end

  // ARREADY: accept read address
  always @(posedge M_AXI_ACLK) begin
    if (!M_AXI_ARESETN)
      M_AXI_ARREADY <= 0;
    else if (M_AXI_ARVALID && !M_AXI_ARREADY)
      M_AXI_ARREADY <= 1;
    else
      M_AXI_ARREADY <= 0;
  end

  // RVALID/RDATA/RRESP: return data from slave_mem
  always @(posedge M_AXI_ACLK) begin
    if (!M_AXI_ARESETN) begin
      M_AXI_RVALID <= 0;
      M_AXI_RRESP  <= 2'b00;
      M_AXI_RDATA  <= 0;
    end
    else if (M_AXI_ARREADY && M_AXI_ARVALID) begin
      addr_reg    <= M_AXI_ARADDR;
      M_AXI_RDATA <= slave_mem[M_AXI_ARADDR[12:2]];
      M_AXI_RRESP <= 2'b00; // OKAY
      M_AXI_RVALID<= 1;
    end
    else if (M_AXI_RVALID && M_AXI_RREADY) begin
      M_AXI_RVALID <= 0;
    end
  end

endmodule
