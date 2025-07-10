`timescale 1ns / 1ps

// -------------------------------------------------------------------------
// Testbench: tb_top_packetiser_mac_phy_fifo
// Description:
// - Generates write/read clocks
// - Applies reset
// - Drives ADC data and valid strobe
// - Monitors PHY interface outputs
// -------------------------------------------------------------------------
module tb_top_packetiser_mac_phy_fifo;

    // Testbench parameters
    localparam NUM_SAMPLES = 150;  // Number of ADC samples to send

    // Clock signals
    reg wr_clk = 0;  // Write-domain clock (packetiser)
    reg rd_clk = 0;  // Read-domain clock (MAC/PHY)

    // Reset signal
    reg rst = 1;    // Active-high reset

    // ADC input signals
    reg [13:0] adc_data  = 14'd0;  // 14-bit sample value
    reg        adc_valid = 1'b0;   // Sample valid strobe

    // DUT outputs
    wire [3:0] phy_txd;            // 4-bit PHY transmit data
    wire       phy_tx_en;          // PHY transmit enable
    wire       fifo_empty;         // FIFO empty indicator
    wire       fifo_full;          // FIFO full indicator
    wire [1:0] packetiser_state;   // Debug state from packetiser

    integer i; // Loop variable for stimulus

    // Instantiate the top-level DUT
   
    top_packetiser_mac_phy_fifo uut (
        .wr_clk           (wr_clk),
        .rd_clk           (rd_clk),
        .rst              (rst),
        .adc_data         (adc_data),
        .adc_valid        (adc_valid),
        .phy_txd          (phy_txd),
        .phy_tx_en        (phy_tx_en),
        .packetiser_state (packetiser_state),
        .fifo_empty       (fifo_empty),
        .fifo_full        (fifo_full)
    );

    // Clock generation: 50 MHz (20 ns period) for wr_clk
     always #10 wr_clk = ~wr_clk;

    
    // Clock generation: 25 MHz (40 ns period) for rd_clk
    always #20 rd_clk = ~rd_clk;

    
    // Stimulus process: Reset, send ADC samples, and monitor completion
    initial begin
        // Initialize inputs
        adc_data  = 14'd0;
        adc_valid = 1'b0;

        // Apply reset for a few cycles
        rst = 1'b1;
        repeat (10) @(posedge wr_clk);
        rst = 1'b0;

        // Wait additional cycles before starting
        repeat (5) @(posedge wr_clk);

        // Drive ADC samples with valid pulses
        for (i = 0; i < NUM_SAMPLES; i = i + 1) begin
            // Provide new sample and assert valid
            adc_data  <= i % 16384;  // Wrap within 14-bit range
            adc_valid <= 1'b1;
            @(posedge wr_clk);

            // Deassert valid for one cycle
            adc_valid <= 1'b0;
            @(posedge wr_clk);

            // Display progress occasionally
            if ((i + 1) % 50 == 0) begin
                $display("[%0t ns] Sent %0d ADC samples", $time, i + 1);
            end
        end

        // All samples sent, wait for transmission to finish
        $display("All samples sent, waiting for PHY transmission...");
        repeat (5000) @(posedge rd_clk);

        $display("Simulation complete at %0t ns", $time);
        $stop;
    end

    // Monitor process: Log key signals for debugging
    initial begin
        $monitor("Time %0t | rst=%b | adc_valid=%b | phy_tx_en=%b | phy_txd=%h | fifo_empty=%b | fifo_full=%b | state=%0d", \
                 $time, rst, adc_valid, phy_tx_en, phy_txd, fifo_empty, fifo_full, packetiser_state);
    end

endmodule
