# FPGA-ethernet-packetiser

# Ethernet-Packet-transfer

This repository contains the complete HDL source code, design files, and documentation for an FPGA-based Radio Telescope system designed for radio interferometry. The project implements real-time signal acquisition, filtering, packetization, and Ethernet transmission of RF signals captured from a radio telescope antenna array.

This Radio Telescope system uses a Red Pitaya STEMlab 125-10 as the primary FPGA platform. The design directly interfaces with the onboard ADC to sample radio frequency signals received by the antenna. The system performs signal acquisition, digital signal processing (DSP) using a 64-tap FIR filter, and then streams the digitized data over Ethernet using the UDP protocol — fully implemented in Verilog on the FPGA fabric without an operating system.

Key Features: 
1.FPGA-based ADC interfacing for high-speed sampling.
2.Custom FIR Filter for real-time signal conditioning.
3.Finite State Machine (FSM) for efficient data packetization.
4.MAC Layer Implementation for Ethernet communication.
5.UDP Packet Generation and transmission using raw PHY-level signals (phy_txd, phy_tx_en).
6.Supports real-time streaming with minimal latency.

 Tools & Technologies
 
Hardware:
Red Pitaya STEMlab 125-10
FPGA (Xilinx Zynq 7010 SoC)

Languages & HDL:
Verilog HDL
Xilinx Vivado Design Suite

Protocols:
UDP/IP (implemented at MAC level)
PHY-level Ethernet signaling
