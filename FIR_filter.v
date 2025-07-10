// -------------------------------------------------------------------------
// File: adc_fir_top.v
// Project: ADC Sign-Extension & 64-Tap FIR Filter
// Description: Implements ADC raw data sign-extension followed by a 64-tap
//              FIR filter using a binary-tree adder tree. Includes top-level
//              integration module.
// -------------------------------------------------------------------------

`timescale 1ns / 1ps

// -------------------------------------------------------------------------
// Module: ADC
// - Converts 14-bit unsigned/raw ADC data into 16-bit signed data
// - Sign-extends bit 13 into a 16-bit signed value
// -------------------------------------------------------------------------
module ADC (
    input  wire         clk,            // Clock input
    input  wire         rst,            // Active-high reset
    input  wire [13:0]  adc_data_raw,   // Raw 14-bit ADC data
    output reg  signed [15:0] adc_data_signed // Sign-extended output
);

    always @(posedge clk) begin
        if (rst) begin
            // On reset, clear output to zero
            adc_data_signed <= 16'sd0;
        end else begin
            // Sign-extend 14-bit data to 16 bits
            adc_data_signed <= {{2{adc_data_raw[13]}}, adc_data_raw};
        end
    end
endmodule


// -------------------------------------------------------------------------
// Module: FIR_filter
// - 64-tap FIR filter with pre-loaded Q1.15 coefficients
// - Implements delay line, multiplication, and binary-tree summation
// - Outputs 16-bit filtered signal (6.10 format after shift)
// -------------------------------------------------------------------------
module FIR_filter (
    input  wire         clk,           // Filter clock
    input  wire         rst,           // Active-high reset
    input  wire signed [15:0] noisy_signal,     // Signed input sample
    output reg  signed [15:0] filtered_signal   // Filtered output
);

  
    // Coefficient memory: 64 signed 16-bit values
   
    reg signed [15:0] coeff [0:63];
    initial begin
        // Initialize coefficients (hex Q1.15 format)
        coeff[0]  = 16'hFFF0; coeff[1]  = 16'h001B;
        coeff[2]  = 16'hFFE2; coeff[3]  = 16'h0016;
        coeff[4]  = 16'h0000; coeff[5]  = 16'hFFE1;
        coeff[6]  = 16'h003E; coeff[7]  = 16'hFFB4;
        coeff[8]  = 16'h0039; coeff[9]  = 16'h0000;
        coeff[10] = 16'hFFAD; coeff[11] = 16'h00A0;
        coeff[12] = 16'hFF43; coeff[13] = 16'h008A;
        coeff[14] = 16'h0000; coeff[15] = 16'hFF44;
        coeff[16] = 16'h0161; coeff[17] = 16'hFE67;
        coeff[18] = 16'h0124; coeff[19] = 16'h0000;
        coeff[20] = 16'hFE7B; coeff[21] = 16'h02D8;
        coeff[22] = 16'hFCB4; coeff[23] = 16'h0261;
        coeff[24] = 16'h0000; coeff[25] = 16'hFCA9;
        coeff[26] = 16'h0692; coeff[27] = 16'hF7C7;
        coeff[28] = 16'h06A8; coeff[29] = 16'h0000;
        coeff[30] = 16'hF01D; coeff[31] = 16'h4D7A;
        coeff[32] = 16'h4D7A; coeff[33] = 16'hF01D;
        coeff[34] = 16'h0000; coeff[35] = 16'h06A8;
        coeff[36] = 16'hF7C7; coeff[37] = 16'h0692;
        coeff[38] = 16'hFCA9; coeff[39] = 16'h0000;
        coeff[40] = 16'h0261; coeff[41] = 16'hFCB4;
        coeff[42] = 16'h02D8; coeff[43] = 16'hFE7B;
        coeff[44] = 16'h0000; coeff[45] = 16'h0124;
        coeff[46] = 16'hFE67; coeff[47] = 16'h0161;
        coeff[48] = 16'hFF44; coeff[49] = 16'h0000;
        coeff[50] = 16'h008A; coeff[51] = 16'hFF43;
        coeff[52] = 16'h00A0; coeff[53] = 16'hFFAD;
        coeff[54] = 16'h0000; coeff[55] = 16'h0039;
        coeff[56] = 16'hFFB4; coeff[57] = 16'h003E;
        coeff[58] = 16'hFFE1; coeff[59] = 16'h0000;
        coeff[60] = 16'h0016; coeff[61] = 16'hFFE2;
        coeff[62] = 16'h001B; coeff[63] = 16'hFFF0;
    end

    
    // Delay line registers for past samples
  
    reg signed [15:0] delayed_signal [0:63];

   
    // Scratch arrays for partial products and sums

    reg signed [31:0] prod     [0:63];     // 16x16 multiply -> 32-bit
    reg signed [32:0] sum_0    [0:31];     // Pairwise sums
    reg signed [33:0] sum_1    [0:15];
    reg signed [34:0] sum_2    [0:7];
    reg signed [35:0] sum_3    [0:3];
    reg signed [36:0] sum_4    [0:1];
    reg signed [37:0] sum_5;               // Final sum

    integer i;

    
    // Shift register: update delayed samples
   
    always @(posedge clk) begin
        if (rst) begin
            // Clear delay line on reset
            for (i = 0; i < 64; i = i+1)
                delayed_signal[i] <= 16'sd0;
        end else begin
            // Insert new sample at position 0, shift older samples
            delayed_signal[0] <= noisy_signal;
            for (i = 1; i < 64; i = i+1)
                delayed_signal[i] <= delayed_signal[i-1];
        end
    end

    // Multiply each delayed sample by its coefficient
   
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 64; i = i+1)
                prod[i] <= 32'sd0;
        end else begin
            for (i = 0; i < 64; i = i+1)
                prod[i] <= coeff[i] * delayed_signal[i];
        end
    end

 
    // Binary-tree adder stages: level 0 (32 sums)
  
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i+1)
                sum_0[i] <= 33'sd0;
        end else begin
            for (i = 0; i < 32; i = i+1)
                sum_0[i] <= prod[2*i] + prod[2*i+1];
        end
    end

    
    // Adder stage 1 (16 sums)
   
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 16; i = i+1)
                sum_1[i] <= 34'sd0;
        end else begin
            for (i = 0; i < 16; i = i+1)
                sum_1[i] <= sum_0[2*i] + sum_0[2*i+1];
        end
    end

    
    // Adder stage 2 (8 sums)
    
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 8; i = i+1)
                sum_2[i] <= 35'sd0;
        end else begin
            for (i = 0; i < 8; i = i+1)
                sum_2[i] <= sum_1[2*i] + sum_1[2*i+1];
        end
    end

  
    // Adder stage 3 (4 sums)
   
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 4; i = i+1)
                sum_3[i] <= 36'sd0;
        end else begin
            for (i = 0; i < 4; i = i+1)
                sum_3[i] <= sum_2[2*i] + sum_2[2*i+1];
        end
    end


    // Adder stage 4 (2 sums)
    
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 2; i = i+1)
                sum_4[i] <= 37'sd0;
        end else begin
            for (i = 0; i < 2; i = i+1)
                sum_4[i] <= sum_3[2*i] + sum_3[2*i+1];
        end
    end

 
    // Final sum stage

    always @(posedge clk) begin
        if (rst) begin
            sum_5 <= 38'sd0;
        end else begin
            sum_5 <= sum_4[0] + sum_4[1];
        end
    end

   
    // Output assignment: truncate and round (shift right by 22 bits)
    
    always @(posedge clk) begin
        if (rst) begin
            filtered_signal <= 16'sd0;
        end else begin
            // Take bits [37:22] for Q15 format output
            filtered_signal <= sum_5[37:22];
        end
    end
endmodule



// Module: top_level
// - Integrates ADC sign-extension and FIR filter into single interface

module top_level (
    input  wire         clk,           // System clock
    input  wire         rst,           // System reset
    input  wire [13:0]  adc_input,     // Raw ADC data
    output wire [15:0]  filtered_output // Filtered data output
);

    // Internal wires
    wire signed [15:0] adc_signed;
    wire signed [15:0] filtered_out;

    // Instantiate ADC sign-extension
    ADC adc_inst (
        .clk            (clk),
        .rst            (rst),
        .adc_data_raw   (adc_input),
        .adc_data_signed(adc_signed)
    );

    // Instantiate FIR filter
    FIR_filter filter_inst (
        .clk            (clk),
        .rst            (rst),
        .noisy_signal   (adc_signed),
        .filtered_signal(filtered_out)
    );

    // Connect filter output to top-level output
    assign filtered_output = filtered_out;

endmodule
