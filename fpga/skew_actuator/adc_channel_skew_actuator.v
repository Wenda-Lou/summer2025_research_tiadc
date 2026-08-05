`timescale 1ns / 1ps

/* Vectorized two-channel fractional-delay actuator.
 *
 * Input/output word order from jesd204_tpl_core:
 *   [127:64] ADC1/Channel B S3,S2,S1,S0
 *   [ 63: 0] ADC0/Channel A S3,S2,S1,S0
 *
 * Channel A receives a fixed one-sample delay. Channel B receives a
 * programmable 0..2 sample delay in Q8 units. Code 256 therefore preserves
 * the original B-A relationship, while codes below/above 256 advance/delay B
 * relative to A. The interpolation is causal and processes four chronological
 * samples per link clock.
 *
 * delay_q8_async is driven by a 10-bit output-only AXI GPIO. Its DATA
 * register is the software-visible delay register and resets to code 256.
 */
module adc_channel_skew_actuator_gpio (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 link_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET link_resetn, FREQ_HZ 325000000" *)
    input wire link_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 link_resetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire link_resetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)
    input wire [127:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *)
    input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *)
    output wire [127:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *)
    output wire m_axis_tvalid,
    input wire [9:0] delay_q8_async
);

    localparam [9:0] DELAY_RESET_Q8 = 10'd256;
    localparam [9:0] DELAY_MAX_Q8 = 10'd512;

    reg [9:0] delay_bus_sync1, delay_bus_sync2, delay_q8_link;
    always @(posedge link_clk) begin
        if (!link_resetn) begin
            delay_bus_sync1 <= DELAY_RESET_Q8;
            delay_bus_sync2 <= DELAY_RESET_Q8;
            delay_q8_link <= DELAY_RESET_Q8;
        end else begin
            delay_bus_sync1 <= delay_q8_async;
            delay_bus_sync2 <= delay_bus_sync1;
            delay_q8_link <= delay_bus_sync2 <= DELAY_MAX_Q8 ?
                delay_bus_sync2 : DELAY_MAX_Q8;
        end
    end

    wire signed [15:0] a0 = s_axis_tdata[15:0];
    wire signed [15:0] a1 = s_axis_tdata[31:16];
    wire signed [15:0] a2 = s_axis_tdata[47:32];
    wire signed [15:0] a3 = s_axis_tdata[63:48];
    wire signed [15:0] b0 = s_axis_tdata[79:64];
    wire signed [15:0] b1 = s_axis_tdata[95:80];
    wire signed [15:0] b2 = s_axis_tdata[111:96];
    wire signed [15:0] b3 = s_axis_tdata[127:112];
    reg signed [15:0] prev_a0, prev_a1, prev_a2, prev_a3;
    reg signed [15:0] prev_b0, prev_b1, prev_b2, prev_b3;
    reg [127:0] output_data_reg;
    reg output_valid_reg;
    assign m_axis_tdata = output_data_reg;
    assign m_axis_tvalid = output_valid_reg;

    function signed [15:0] b_history;
        input integer position;
        begin
            case (position)
                -3: b_history = prev_b1;
                -2: b_history = prev_b2;
                -1: b_history = prev_b3;
                 0: b_history = b0;
                 1: b_history = b1;
                 2: b_history = b2;
                 3: b_history = b3;
                default: b_history = 16'sd0;
            endcase
        end
    endfunction

    function signed [15:0] interpolate_q8;
        input signed [15:0] newer;
        input signed [15:0] older;
        input [7:0] fraction;
        reg signed [24:0] weighted;
        begin
            weighted = newer * (9'd256 - {1'b0, fraction}) + older * fraction;
            interpolate_q8 = weighted >>> 8;
        end
    endfunction

    integer coarse;
    reg [7:0] fraction;
    reg signed [15:0] bo0, bo1, bo2, bo3;
    always @(*) begin
        coarse = delay_q8_link[9:8];
        fraction = delay_q8_link[7:0];
        bo0 = interpolate_q8(b_history(0 - coarse), b_history(-1 - coarse), fraction);
        bo1 = interpolate_q8(b_history(1 - coarse), b_history(0 - coarse), fraction);
        bo2 = interpolate_q8(b_history(2 - coarse), b_history(1 - coarse), fraction);
        bo3 = interpolate_q8(b_history(3 - coarse), b_history(2 - coarse), fraction);
    end

    always @(posedge link_clk) begin
        if (!link_resetn) begin
            prev_a0 <= 16'sd0; prev_a1 <= 16'sd0;
            prev_a2 <= 16'sd0; prev_a3 <= 16'sd0;
            prev_b0 <= 16'sd0; prev_b1 <= 16'sd0;
            prev_b2 <= 16'sd0; prev_b3 <= 16'sd0;
            output_data_reg <= 128'd0;
            output_valid_reg <= 1'b0;
        end else begin
            output_valid_reg <= s_axis_tvalid;
            if (s_axis_tvalid) begin
                output_data_reg <= {
                    bo3, bo2, bo1, bo0,
                    a2, a1, a0, prev_a3
                };
                prev_a0 <= a0; prev_a1 <= a1;
                prev_a2 <= a2; prev_a3 <= a3;
                prev_b0 <= b0; prev_b1 <= b1;
                prev_b2 <= b2; prev_b3 <= b3;
            end
        end
    end
endmodule
