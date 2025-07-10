set_property IOSTANDARD LVCMOS18 [get_ports {adc_data[*]}]

set_property PACKAGE_PIN V17 [get_ports {adc_data[0]}]
set_property PACKAGE_PIN U17 [get_ports {adc_data[1]}]
set_property PACKAGE_PIN Y19 [get_ports {adc_data[2]}]
set_property PACKAGE_PIN W16 [get_ports {adc_data[3]}]
set_property PACKAGE_PIN Y16 [get_ports {adc_data[4]}]
set_property PACKAGE_PIN W15 [get_ports {adc_data[5]}]
set_property PACKAGE_PIN W14 [get_ports {adc_data[6]}]
set_property PACKAGE_PIN Y14 [get_ports {adc_data[7]}]
set_property PACKAGE_PIN W13 [get_ports {adc_data[8]}]
set_property PACKAGE_PIN V12 [get_ports {adc_data[9]}]
set_property PACKAGE_PIN V13 [get_ports {adc_data[10]}]
set_property PACKAGE_PIN T14 [get_ports {adc_data[11]}]
set_property PACKAGE_PIN T15 [get_ports {adc_data[12]}]
set_property PACKAGE_PIN V15 [get_ports {adc_data[13]}]

### ADC VALID and RESET
set_property PACKAGE_PIN N17 [get_ports adc_valid]
set_property IOSTANDARD LVCMOS18 [get_ports adc_valid]

set_property PACKAGE_PIN P18 [get_ports rst]
set_property IOSTANDARD LVCMOS18 [get_ports rst]

### ADC CLOCK (wr_clk)

set_property PACKAGE_PIN V20 [get_ports wr_clk_p]
set_property PACKAGE_PIN U19 [get_ports wr_clk_n]
set_property IOSTANDARD DIFF_HSTL_I_18 [get_ports wr_clk_p]
set_property IOSTANDARD DIFF_HSTL_I_18 [get_ports wr_clk_n]


create_clock -period 8.000 -name wr_clk [get_ports wr_clk_p]
set_input_delay -clock wr_clk 3.400 [get_ports adc_data[*]]

### PHY TX DATA

set_property IOSTANDARD LVCMOS25 [get_ports {phy_txd[*]}]
set_property PACKAGE_PIN G17 [get_ports {phy_txd[0]}]
set_property PACKAGE_PIN G18 [get_ports {phy_txd[1]}]
set_property PACKAGE_PIN H16 [get_ports {phy_txd[2]}]
set_property PACKAGE_PIN H17 [get_ports {phy_txd[3]}]

### PHY TX ENABLE

set_property IOSTANDARD LVCMOS25 [get_ports phy_tx_en]
set_property PACKAGE_PIN J18 [get_ports phy_tx_en]

### RD_CLK (Ethernet PHY clock domain)

set_property IOSTANDARD LVCMOS25 [get_ports rd_clk]
set_property PACKAGE_PIN K17 [get_ports rd_clk]


set_property IOSTANDARD LVCMOS25 [get_ports {packetiser_state[*]}]
set_property PACKAGE_PIN F16 [get_ports {packetiser_state[0]}]
set_property PACKAGE_PIN F17 [get_ports {packetiser_state[1]}]

set_property IOSTANDARD LVCMOS25 [get_ports fifo_full]
set_property PACKAGE_PIN G15 [get_ports fifo_full]

set_property IOSTANDARD LVCMOS25 [get_ports fifo_empty]
set_property PACKAGE_PIN H15 [get_ports fifo_empty]
