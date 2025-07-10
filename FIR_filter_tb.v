module top_level_tb;
  // Clock and reset
  reg clk_tb;
  reg rst_tb;

  // ADC input and filtered output signals
  reg signed [13:0] adc_input_tb;
  wire signed [15:0] filtered_output_tb;

  // Array to hold test values
  reg signed [15:0] test_values [0:99]; 
  integer i;

  // Instantiate DUT
  top_level uut(
    .clk(clk_tb),
    .adc_input(adc_input_tb),
    .filtered_output(filtered_output_tb),
    .rst(rst_tb)
  );

  // Clock generation: toggles every 10ns -> 50 MHz clock
  always #10 clk_tb = ~clk_tb;

  initial begin
    // Initialize signals
    clk_tb = 0;
    rst_tb = 1;
    adc_input_tb = 0;

    // Initialize 100 test values
    test_values[0] = 16'sh0064;  // 100
    test_values[1] = 16'sh00c8;  // 200
    test_values[2] = 16'shff6a;  // -150
    test_values[3] = 16'shfc18;  // -1000
    test_values[4] = 16'sh012c;  // 300
    test_values[5] = 16'sh0190;  // 400
    test_values[6] = 16'shfed4;  // -300
    test_values[7] = 16'sh0000;  // 0
    test_values[8] = 16'sh0258;  // 600
    test_values[9] = 16'shfe0c;  // -500
    test_values[10] = 16'sh00fa; // 250
    test_values[11] = 16'sh015e; // 350
    test_values[12] = 16'shfe3e; // -450
    test_values[13] = 16'sh02bc; // 700
    test_values[14] = 16'shfce0; // -800
    test_values[15] = 16'sh0384; // 900
    test_values[16] = 16'shfd44; // -700
    test_values[17] = 16'sh03e8; // 1000
    test_values[18] = 16'shff38; // -200
    test_values[19] = 16'sh0032; // 50
    test_values[20] = 16'shffce; // -50
    test_values[21] = 16'sh001e; // 30
    test_values[22] = 16'shffc4; // -60
    test_values[23] = 16'sh002d; // 45
    test_values[24] = 16'shffa6; // -90
    test_values[25] = 16'sh0096; // 150
    test_values[26] = 16'shfeb4; // -180
    test_values[27] = 16'sh00c8; // 200
    test_values[28] = 16'shfef4; // -220
    test_values[29] = 16'sh00f0; // 240
    test_values[30] = 16'shefec; // -260
    test_values[31] = 16'sh0118; // 280
    test_values[32] = 16'shecec; // -300
    test_values[33] = 16'sh0140; // 320
    test_values[34] = 16'shedac; // -340
    test_values[35] = 16'sh0168; // 360
    test_values[36] = 16'shea84; // -380
    test_values[37] = 16'sh0190; // 400
    test_values[38] = 16'she9bc; // -420
    test_values[39] = 16'sh01b8; // 440
    test_values[40] = 16'she88c; // -460
    test_values[41] = 16'sh01e0; // 480
    test_values[42] = 16'she70c; // -500
    test_values[43] = 16'sh0208; // 520
    test_values[44] = 16'she58c; // -540
    test_values[45] = 16'sh0230; // 560
    test_values[46] = 16'she40c; // -580
    test_values[47] = 16'sh0258; // 600
    test_values[48] = 16'she28c; // -620
    test_values[49] = 16'sh0280; // 640
    test_values[50] = 16'she10c; // -660
    test_values[51] = 16'sh02a8; // 680
    test_values[52] = 16'shdf8c; // -700
    test_values[53] = 16'sh02d0; // 720
    test_values[54] = 16'shde0c; // -740
    test_values[55] = 16'sh02f8; // 760
    test_values[56] = 16'shdc8c; // -780
    test_values[57] = 16'sh0320; // 800
    test_values[58] = 16'shdb0c; // -820
    test_values[59] = 16'sh0348; // 840
    test_values[60] = 16'shd98c; // -860
    test_values[61] = 16'sh0370; // 880
    test_values[62] = 16'shd80c; // -900
    test_values[63] = 16'sh0398; // 920
    test_values[64] = 16'shd68c; // -940
    test_values[65] = 16'sh03c0; // 960
    test_values[66] = 16'shd50c; // -980
    test_values[67] = 16'sh03e8; // 1000
    test_values[68] = 16'shd38c; // -1020
    test_values[69] = 16'sh0410; // 1040
    test_values[70] = 16'shd20c; // -1060
    test_values[71] = 16'sh0438; // 1080
    test_values[72] = 16'shd08c; // -1100
    test_values[73] = 16'sh0460; // 1120
    test_values[74] = 16'shcf0c; // -1140
    test_values[75] = 16'sh0488; // 1160
    test_values[76] = 16'shcd8c; // -1180
    test_values[77] = 16'sh04b0; // 1200
    test_values[78] = 16'shcc0c; // -1220
    test_values[79] = 16'sh04d8; // 1240
    test_values[80] = 16'shca8c; // -1260
    test_values[81] = 16'sh0500; // 1280
    test_values[82] = 16'shc90c; // -1300
    test_values[83] = 16'sh0528; // 1320
    test_values[84] = 16'shc78c; // -1340
    test_values[85] = 16'sh0550; // 1360
    test_values[86] = 16'shc60c; // -1380
    test_values[87] = 16'sh0578; // 1400
    test_values[88] = 16'shc48c; // -1420
    test_values[89] = 16'sh05a0; // 1440
    test_values[90] = 16'shc30c; // -1460
    test_values[91] = 16'sh05c8; // 1480
    test_values[92] = 16'shc18c; // -1500
    test_values[93] = 16'sh05f0; // 1520
    test_values[94] = 16'shc00c; // -1540
    test_values[95] = 16'sh0618; // 1560
    test_values[96] = 16'shbe8c; // -1580
    test_values[97] = 16'sh0640; // 1600
    test_values[98] = 16'shbd0c; // -1620
    test_values[99] = 16'sh0668; // 1640

    // Monitor output
    $monitor($time, " clk_tb=%b, rst_tb=%b, adc_input_tb=%d, filtered_output_tb=%d",
      clk_tb, rst_tb, adc_input_tb, filtered_output_tb);

    // Release reset after 40ns
    #40 rst_tb = 0;

    // Apply input samples every 20ns
    for (i = 0; i < 100; i = i + 1) begin
      #20 adc_input_tb = test_values[i];
    end

    // End simulation
    #4000 $finish;
  end
endmodule
