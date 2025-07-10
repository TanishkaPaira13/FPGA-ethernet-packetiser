// -------------------------------------------------------------------------
// File: top_packetiser_mac_phy_fifo.v
// Project: FPGA-Based Radio Telescope Packetiser
// Description: Complete Verilog design for ADC sampling, packetisation,
//              Ethernet MAC framing, and PHY interface, with detailed comments.
// -------------------------------------------------------------------------

// -------------------------------------------------------------------------
// Simple UDP Packetiser Module
// - Captures NUM_SAMPLES of 14-bit ADC data
// - Constructs Ethernet/IPv4/UDP headers
// - Streams header + payload bytes
// - Outputs eth_tx_en, eth_tx_data, eth_tx_done signals
// -------------------------------------------------------------------------
module simple_packetiser #(
    parameter NUM_SAMPLES = 64,
    // Ethernet / IP / UDP configuration parameters
    parameter [47:0] DEST_MAC = 48'hFFFFFFFFFFFF,
    parameter [47:0] SRC_MAC  = 48'h002632F047BF,
    parameter [31:0] SRC_IP   = 32'h0ACD032F,
    parameter [31:0] DEST_IP  = 32'hFFFFFFFF,
    parameter [15:0] SRC_PORT = 16'h04D2,
    parameter [15:0] DEST_PORT= 16'h162E
)(
    input  wire        clk,            // Clock for packetiser domain
    input  wire        rst,            // Active-high reset
    input  wire [13:0] adc_data,       // 14-bit ADC sample input
    input  wire        adc_valid,      // Sample valid strobe

    output reg         eth_tx_en,      // Enable transmit
    output reg  [7:0]  eth_tx_data,    // Byte to transmit
    output reg         eth_tx_done,    // Packet completion strobe
    output     [1:0]   state_out       // Debug: current FSM state
);

// Expose internal state for debugging
assign state_out = state;

// FSM state encoding
localparam IDLE    = 2'b00;  // Sampling state
localparam HEADER  = 2'b01;  // Transmit header bytes
localparam PAYLOAD = 2'b10;  // Transmit payload bytes
localparam DONE    = 2'b11;  // Packet complete

reg [1:0] state;

// Counters for samples and bytes
reg [7:0]  sample_count;  // Number of samples collected
reg [15:0] byte_count;    // Byte index for output

// Memory to store captured samples (sign-extended to 16 bits)
reg [15:0] samples [0:NUM_SAMPLES-1];

// Header length definitions
localparam HEADER_SIZE   = 42; // Ethernet+IP+UDP header length
localparam SAMPLE_BYTES  = 2;  // Bytes per sample

// Derived lengths for IP and UDP
wire [15:0] udp_payload_size = NUM_SAMPLES * SAMPLE_BYTES;
wire [15:0] udp_length       = 8 + udp_payload_size;    // UDP header (8) + payload
wire [15:0] ip_length        = 20 + udp_length;         // IP header (20) + UDP

// Buffer to hold header bytes
reg [7:0] header [0:HEADER_SIZE-1];

// Main FSM: sample collection, header transmit, payload transmit
always @(posedge clk) begin
    if (rst) begin
        // Reset all signals and counters
        state         <= IDLE;
        eth_tx_en     <= 1'b0;
        eth_tx_done   <= 1'b0;
        sample_count  <= 8'd0;
        byte_count    <= 16'd0;

        // Initialize static header fields (length fields updated later)
        // Ethernet destination and source MAC
        header[0] <= DEST_MAC[47:40]; header[1] <= DEST_MAC[39:32];
        header[2] <= DEST_MAC[31:24]; header[3] <= DEST_MAC[23:16];
        header[4] <= DEST_MAC[15:8];  header[5] <= DEST_MAC[7:0];
        header[6] <= SRC_MAC[47:40];  header[7] <= SRC_MAC[39:32];
        header[8] <= SRC_MAC[31:24];  header[9] <= SRC_MAC[23:16];
        header[10]<= SRC_MAC[15:8];   header[11]<= SRC_MAC[7:0];
        header[12]<= 8'h08; // EtherType: IPv4
        header[13]<= 8'h00;

        // IP header: version, IHL, DSCP/ECN, flags, TTL, protocol
        header[14]<= 8'h45; // Version=4, IHL=5 words
        header[15]<= 8'h00; // DSCP/ECN
        header[18]<= 8'h00; header[19]<= 8'h00; // Identification
        header[20]<= 8'h40; header[21]<= 8'h00; // Flags=DF, Fragment offset
        header[22]<= 8'h40; // TTL=64
        header[23]<= 8'h11; // Protocol=UDP
        header[24]<= 8'h00; header[25]<= 8'h00; // Checksum placeholder

        // IP addresses
        header[26]<= SRC_IP[31:24]; header[27]<= SRC_IP[23:16];
        header[28]<= SRC_IP[15:8];  header[29]<= SRC_IP[7:0];
        header[30]<= DEST_IP[31:24];header[31]<= DEST_IP[23:16];
        header[32]<= DEST_IP[15:8]; header[33]<= DEST_IP[7:0];

        // UDP header: source & dest ports
        header[34]<= SRC_PORT[15:8]; header[35]<= SRC_PORT[7:0];
        header[36]<= DEST_PORT[15:8];header[37]<= DEST_PORT[7:0];
        header[40]<= 8'h00; header[41]<= 8'h00; // UDP checksum placeholder

    end else begin
        case (state)
           
            IDLE: begin
                eth_tx_en   <= 1'b0;
                eth_tx_done <= 1'b0;
                if (adc_valid && sample_count < NUM_SAMPLES) begin
                    // Sign-extend 14-bit sample to 16 bits
                    samples[sample_count] <= {{2{adc_data[13]}}, adc_data};
                    sample_count <= sample_count + 1;

                    // Once buffer full, compute length fields and move to HEADER
                    if (sample_count + 1 == NUM_SAMPLES) begin
                        header[16] <= ip_length[15:8]; header[17] <= ip_length[7:0];
                        header[38] <= udp_length[15:8];header[39] <= udp_length[7:0];
                        byte_count <= 16'd0;
                        state      <= HEADER;
                    end
                end
            end

           
            HEADER: begin
                eth_tx_en   <= 1'b1;
                eth_tx_done <= 1'b0;
                eth_tx_data <= header[byte_count];
                if (byte_count == HEADER_SIZE-1) begin
                    byte_count <= 16'd0;
                    state      <= PAYLOAD;
                end else begin
                    byte_count <= byte_count + 1;
                end
            end

           
            PAYLOAD: begin
                eth_tx_en   <= 1'b1;
                eth_tx_done <= 1'b0;
                if (byte_count[0] == 1'b0)
                    eth_tx_data <= samples[byte_count>>1][15:8];
                else
                    eth_tx_data <= samples[byte_count>>1][7:0];

                if (byte_count == NUM_SAMPLES*2-1)
                    state <= DONE;
                else
                    byte_count <= byte_count + 1;
            end

            DONE: begin
                eth_tx_en     <= 1'b0;
                eth_tx_done   <= 1'b1;
                sample_count  <= 8'd0;
                byte_count    <= 16'd0;
                state         <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule


// MAC Transmission Module
// - Implements Ethernet preamble, SFD, payload forwarding, and CRC32

module mac_tx (
    input  wire       clk,         // MAC clock domain
    input  wire       rst,         // Active-high reset
    input  wire [7:0] data_in,     // Byte from packetiser FIFO
    input  wire       data_valid,  // Data valid strobe
    input  wire       data_last,   // Last byte of frame indicator
    output reg  [7:0] mac_data,    // Transmitted byte to PHY interface
    output reg        mac_valid,   // Valid signal for mac_data
    output reg        mac_last     // Indicates last byte (end of frame)
);

// FSM states for MAC framing
localparam IDLE     = 3'd0;
localparam PREAMBLE = 3'd1;
localparam SFD      = 3'd2;
localparam PAYLOAD  = 3'd3;
localparam CRC      = 3'd4;

reg [2:0] state;
reg [2:0] preamble_cnt;   // Counter for 7-byte preamble
reg [31:0] crc;           // Running CRC32
reg [1:0]  crc_byte_cnt;  // Byte index for CRC output

// CRC32 calculation function (polynomial 0xEDB88320)
function [31:0] next_crc32_d8;
    input [7:0] data;
    input [31:0] crc_in;
    integer i;
    reg [31:0] crc_tmp;
begin
    crc_tmp = crc_in;
    for (i = 0; i < 8; i = i+1) begin
        if ((crc_tmp[0] ^ data[i]) == 1'b1)
            crc_tmp = (crc_tmp >> 1) ^ 32'hEDB88320;
        else
            crc_tmp = crc_tmp >> 1;
    end
    next_crc32_d8 = crc_tmp;
end
endfunction

always @(posedge clk) begin
    if (rst) begin
        state        <= IDLE;
        preamble_cnt<= 3'd0;
        crc          <= 32'hFFFFFFFF;
        crc_byte_cnt<= 2'd0;
        mac_data     <= 8'd0;
        mac_valid    <= 1'b0;
        mac_last     <= 1'b0;
    end else begin
        // Default outputs
        mac_valid <= 1'b0;
        mac_last  <= 1'b0;
        case (state)
          
            
            IDLE: begin
                if (data_valid) begin
                    state        <= PREAMBLE;
                    preamble_cnt<= 3'd0;
                    crc          <= 32'hFFFFFFFF;
                end
            end

            PREAMBLE: begin
                mac_data  <= 8'h55;
                mac_valid <= 1'b1;
                if (preamble_cnt == 3'd6)
                    state <= SFD;
                else
                    preamble_cnt <= preamble_cnt + 1;
            end

           
            SFD: begin
                mac_data  <= 8'hD5;
                mac_valid <= 1'b1;
                state     <= PAYLOAD;
            end

           
            PAYLOAD: begin
                if (data_valid) begin
                    mac_data  <= data_in;
                    mac_valid <= 1'b1;
                    crc       <= next_crc32_d8(data_in, crc);
                    if (data_last) begin
                        state        <= CRC;
                        crc          <= ~crc; // Finalize CRC
                        crc_byte_cnt <= 2'd0;
                    end
                end
            end

            CRC: begin
                mac_valid <= 1'b1;
                // Assert last on final CRC byte
                mac_last  <= (crc_byte_cnt == 2'd3);
                case (crc_byte_cnt)
                    2'd0: mac_data <= crc[7:0];
                    2'd1: mac_data <= crc[15:8];
                    2'd2: mac_data <= crc[23:16];
                    2'd3: mac_data <= crc[31:24];
                endcase
                if (crc_byte_cnt == 2'd3)
                    state <= IDLE;
                else
                    crc_byte_cnt <= crc_byte_cnt + 1;
            end
            default: state <= IDLE;
        endcase
    end
end
endmodule



// MII PHY Bus Interface
// - Converts 8-bit MAC data to 4-bit PHY nibbles (nibble swapping)
// - Controls phy_tx_en for the physical interface

module mii_phy_bus (
    input  wire       clk,        // PHY clock domain
    input  wire       rst,        // Reset
    input  wire [7:0] mac_data,   // Byte from mac_tx
    input  wire       mac_valid,  // Valid data strobe

    output reg  [3:0] phy_txd,    // 4-bit transmit data nibble
    output reg        phy_tx_en   // Transmit enable
);

reg nibble_sel;   // Select high or low nibble
reg [7:0] data_buf; // Buffer full byte for second nibble

always @(posedge clk) begin
    if (rst) begin
        phy_txd    <= 4'b0000;
        phy_tx_en  <= 1'b0;
        nibble_sel <= 1'b0;
        data_buf   <= 8'd0;
    end else begin
        if (mac_valid) begin
            if (!nibble_sel) begin
                data_buf   <= mac_data;
                phy_txd    <= mac_data[3:0]; // Lower nibble first
                phy_tx_en  <= 1'b1;
                nibble_sel <= 1'b1;
            end else begin
                phy_txd    <= data_buf[7:4]; // Higher nibble next
                phy_tx_en  <= 1'b1;
                nibble_sel <= 1'b0;
            end
        end else begin
            // Idle state: disable transmit
            phy_tx_en  <= 1'b0;
            phy_txd    <= 4'b0000;
            nibble_sel <= 1'b0;
        end
    end
end
endmodule



// Asynchronous FIFO for CDC between ADC (wr_clk) and MAC (rd_clk)
// - Uses Xilinx XPM FIFO primitive for clock-domain crossing

module adc_to_mac_cdc_fifo #(
    parameter DATA_WIDTH = 16,
    parameter FIFO_DEPTH = 1024
)(
    input  wire                   wr_clk, // Write domain clock
    input  wire                   rd_clk, // Read domain clock
    input  wire                   rst,    // Reset
    input  wire [DATA_WIDTH-1:0]  din,    // Data in (from packetiser)
    input  wire                   wr_en,  // Write enable
    output wire [DATA_WIDTH-1:0]  dout,   // Data out (to mac_tx)
    input  wire                   rd_en,  // Read enable
    output wire                   full,   // FIFO full flag
    output wire                   empty   // FIFO empty flag
);

// Instantiate Xilinx async FIFO primitive
xpm_fifo_async #(
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_WRITE_DEPTH(FIFO_DEPTH),
    .WRITE_DATA_WIDTH(DATA_WIDTH),
    .READ_DATA_WIDTH(DATA_WIDTH),
    .READ_MODE("fwft"),
    .CDC_SYNC_STAGES(2),
    .ECC_MODE("no_ecc"),
    .FIFO_READ_LATENCY(1),
    .FULL_RESET_VALUE(1)
) cdc_fifo_inst (
    .wr_clk    (wr_clk),
    .wr_en     (wr_en),
    .din       (din),
    .full      (full),
    .wr_rst_busy(),
    .sleep     (1'b0),
    .rd_clk    (rd_clk),
    .rd_en     (rd_en),
    .dout      (dout),
    .empty     (empty),
    .rd_rst_busy(),
    .rst       (rst)
);
endmodule


// Top-Level Integration: packetiser -> FIFO -> MAC -> PHY

module top_packetiser_mac_phy_fifo (
    input  wire        wr_clk_p,  // Differential write clock +
    input  wire        wr_clk_n,  // Differential write clock -
    input  wire        rd_clk,    // Read (MAC) clock
    input  wire        rst,       // Global reset
    input  wire [13:0] adc_data,  // ADC data input
    input  wire        adc_valid, // ADC data valid

    output wire [3:0]  phy_txd,    // PHY interface transmit data
    output wire        phy_tx_en,  // PHY interface transmit enable
    output wire [1:0]  packetiser_state, // Debug state
    output wire        fifo_full,  // FIFO full flag
    output wire        fifo_empty  // FIFO empty flag
);

// Internal signals
wire [7:0] packetiser_eth_tx_data;
wire       packetiser_eth_tx_en;
wire       packetiser_eth_tx_done;
wire       wr_clk;
reg        fifo_rd_en;
wire [7:0] fifo_dout;
wire [7:0] mac_data;
wire       mac_valid;
wire       mac_last;

// Differential to single-ended clock buffer
IBUFDS #(
    .DIFF_TERM("TRUE"),
    .IOSTANDARD("DIFF_HSTL_I_18")
) wr_clk_ibufds (
    .I(wr_clk_p),
    .IB(wr_clk_n),
    .O(wr_clk)
);

// Instantiate packetiser
simple_packetiser #(
    .NUM_SAMPLES(64)
) packetiser (
    .clk        (wr_clk),
    .rst        (rst),
    .adc_data   (adc_data),
    .adc_valid  (adc_valid),
    .eth_tx_en  (packetiser_eth_tx_en),
    .eth_tx_data(packetiser_eth_tx_data),
    .eth_tx_done(packetiser_eth_tx_done),
    .state_out  (packetiser_state)
);

// CDC FIFO between packetiser and MAC
adc_to_mac_cdc_fifo #(
    .DATA_WIDTH(8),
    .FIFO_DEPTH(2048)
) fifo_inst (
    .wr_clk(wr_clk),
    .rd_clk(rd_clk),
    .rst    (rst),
    .din    (packetiser_eth_tx_data),
    .wr_en  (packetiser_eth_tx_en & ~fifo_full),
    .dout   (fifo_dout),
    .rd_en  (fifo_rd_en),
    .full   (fifo_full),
    .empty  (fifo_empty)
);

// Instantiate MAC TX module
mac_tx mac_inst (
    .clk       (rd_clk),
    .rst       (rst),
    .data_in   (fifo_dout),
    .data_valid(~fifo_empty),
    .data_last (packetiser_eth_tx_done),
    .mac_data  (mac_data),
    .mac_valid (mac_valid),
    .mac_last  (mac_last)
);

// FIFO read enable logic: read when MAC ready
always @(posedge rd_clk) begin
    if (rst)
        fifo_rd_en <= 1'b0;
    else
        fifo_rd_en <= mac_valid & ~fifo_empty;
end

// PHY interface
mii_phy_bus phy_inst (
    .clk       (rd_clk),
    .rst       (rst),
    .mac_data  (mac_data),
    .mac_valid (mac_valid),
    .phy_txd   (phy_txd),
    .phy_tx_en (phy_tx_en)
);

endmodule
