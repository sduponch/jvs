# JVSCore: FPGA JVS Host Protocol Implementation

[![Build Status](https://github.com/sduponch/jvs/actions/workflows/quartus-build.yml/badge.svg)](https://github.com/sduponch/jvs/actions/workflows/quartus-build.yml)
[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Live-brightgreen)](https://sduponch.github.io/jvs/)

⚠️ **ALPHA STATUS - PARTIAL COMMAND IMPLEMENTATION** ⚠️

> **🌐 [Latest Build Available](https://sduponch.github.io/jvs/)** - Automatic builds with FPGA statistics and direct downloads!

## What is JVS?

**JVS (JAMMA Video Standard)** is a digital communication protocol used in modern arcade systems to connect I/O boards (buttons, joysticks, coin mechanisms) to the main game board via RS485. It replaced the analog JAMMA standard with a more reliable digital interface supporting:

- **Multi-device chaining** - Up to 31 I/O boards on one bus
- **Digital communication** - No signal degradation or noise issues
- **Advanced features** - Analog inputs, coin counters, LED control, displays
- **Standardized protocol** - Universal compatibility across manufacturers

## Why This Project?

**Play your arcade cores on real arcade cabinets without modifications!**

This project enables you to:
- **Connect Analogue Pocket to JVS arcade cabinets** - Use real arcade controls
- **Preserve original hardware** - No cabinet modifications required
- **Universal compatibility** - Works with various JVS I/O boards
- **Native FPGA implementation** - No microcontroller needed for input emulation
- **Cost-effective solution** - Reliable and affordable JVS-to-JAMMA conversion interfaces available
- **Analog and digital output support** - Control LEDs, solenoids, motors, and other cabinet features
- **Light gun and IR support made easy** - Direct screen position input handling for shooting games
- **Future MiSTer support** - Expandable to other FPGA platforms
- **Plug-and-play experience** - Just connect and enjoy

## Hardware Requirements

### Essential Hardware
- **Analogue Pocket** - FPGA handheld console
- **Analogizer Extension** - FPGA expansion board ([RndMnkIII's Analogizer](https://github.com/RndMnkIII/Analogizer))
- **SNAC JVS Adapter** - Physical JVS connector interface (comming soon)

### JVS Cabinet Setup
- **JVS-compatible arcade cabinet** - Most modern arcade machines (post-1995)
- **JVS cable connection** - USB-like JVS cable or JVS adapter (e.g., Time Crisis 2 setup)
- **USB-C power supply** - Power for Analogizer (5V DC recommended)

This repository contains a complete JVS (JAMMA Video Standard) Master controller implementation for FPGA platforms, designed for connecting JVS arcade cabinets through RS485 communication.

## Work In Progress - Getting Latest Build

A test core has been developed in collaboration with **@RndMnkIII**. You can download the latest version on this page:

### Public Downloads (No GitHub Account Required)

**[Download Latest Build](https://sduponch.github.io/jvs/)** - Direct download, always up-to-date!

### Alternative (GitHub Account Required)
**📥 [GitHub Actions](https://github.com/sduponch/jvs/actions/workflows/quartus-build.yml?query=branch%3Amain+is%3Asuccess)** - Download `jvs-debugger-outputs` artifact

### Installation Steps:
1. **Pre-configure your Analog Pocket** with [AnalogizerConfigurator](https://github.com/RndMnkIII/AnalogizerConfigurator)
2. **Download** the latest build from the link above
3. **Extract** all files from the ZIP/folder
4. **Copy** the folders to your Analog Pocket SD card root
5. **Enjoy** - The core will appear in `Cores/RndMnkIII.JVS_Debugger/`

### What You Get:
- ✅ **Ready-to-use Analog Pocket core** in correct folder structure
- ✅ **Bit-reversed RBF file** (`jvs_debug.rbf_r`) for Analog Pocket compatibility
- ✅ **Complete package** with metadata and assets from `dist/` folder
- ✅ **Build info** with commit details and file listing

## Overview

This module implements a JVS (JAMMA Video Standard) Master controller that allows connecting JVS arcade cabinets to FPGA platforms. The implementation is optimized for gaming performance with partial command support.

The JVS protocol implementation is organized in a modular architecture:

- **RTL Layer** (`rtl/`) - Packages and definitions
- **Communication Layer** (`jvs_com.sv`) - Low-level UART and frame handling
- **Protocol Layer** (`jvs_ctrl.sv`) - High-level JVS command processing

## Architecture

```
jvs/
├── rtl/                    # RTL definitions and packages
│   ├── jvs_defs_pkg.sv    # JVS constants and command definitions
│   └── jvs_node_info_pkg.sv # Node information structures
├── jvs_com.sv            # Communication layer (UART + framing)
├── jvs_ctrl.sv           # Protocol layer (JVS state machine)
├── uart_tx.v             # UART transmitter
├── uart_rx.v             # UART receiver
└── jvs.qip              # Quartus IP Project
```

## Implementation Status

**ALPHA Version** - Partial implementation (~30-40% of JVS v3.0 specification)  
Based on JVS Specification v3.0 (25 pages, 49 commands total)

### ✅ Fully Implemented (10/49 commands)
- **Reset (F0)** - Double reset sequence with timing delays
- **Set Address (F1)** - Single device addressing
- **IO Identity (10)** - Device name string reading (up to 100 chars)
- **Command Revision (11)** - Format revision detection (BCD)
- **JVS Revision (12)** - Protocol version detection (BCD)
- **Communications Version (13)** - Communication system version
- **Feature Check (14)** - Complete capability parsing with all function codes
- **Switch Inputs (20)** - Digital buttons (Up to 2 players)
- **Coin Inputs (21)** - Multi-slot coin counter parsing (14-bit counters + condition status)
- **Analog Inputs (22)** - Multi-channel 16-bit analog data with proper channel parsing
- **Generic Output 1 (32)** - Digital GPIO control (3-byte format for Time Crisis 4 recoil)
- **JVS escape sequences** - D0 DF → E0, D0 CF → D0 (Untested)
- **RS485 timing control** - Setup/hold delays

### 🟡 Partially Implemented (2/49 commands)
- **Screen Position Inputs (25)** - Basic X/Y coordinates (16-bit each)
- **Keycode Inputs (24)** - Command sent, response skipped
- **Misc Switch Inputs (26)** - Command sent, response skipped

### ❌ Not Implemented (37/49 commands)
- **Main ID (15)** - Send main board identification to device
- **Rotary Inputs (23)** - Rotary encoder data (parsed but ignored)
- **Remaining Payout (2E)** - Medal hopper status and count
- **Data Retransmit (2F)** - Checksum error recovery
- **Coin management**: COINDEC (30), COININC (35), PAYINC (31), PAYDEC (36)
- **Advanced outputs**: OUTPUT2 (37), OUTPUT3 (38), ANLOUT (33), CHAROUT (34)
- **Communication changes**: COMMCHG (F2)
- **Manufacturer-specific commands** (Taito TypeX, Namco, CyberLead LED)
- **Multi-device addressing** (supports single device only)
- **Dynamic baud rate changes**
- **Error recovery mechanisms**

## Hardware Compatibility

### Compatible I/O Boards
- **NAJV2** (Tekken 7)
- **NAJV** (Time Crisis 4) 
- **TAITO CORP Ver2.0** (Viewlix)

### Incompatible
- "No Brand;NAOMI Converter98701;ver2.0" (frames ignored)

## Architecture Details

### Modular Architecture (Refactored)
- **jvs_com module** - Low-level communication layer with UART/RS485/framing with **Command FIFO system** to supports chained commands (e.g., SWINP+COININP+ANLINP)
- **jvs_ctrl module** - High-level JVS protocol state machine and data parsing (will be moved to specialised module)
- **Node information management** - Device capabilities tracking
- **SystemVerilog packages** - Modern constant definitions with jvs_defs_pkg

### Protocol Compliance
- **Physical Layer**: RS-485 at 115200 baud (8N1) ✅
- **Link Layer**: SYNC(0xE0) + NODE + LENGTH + DATA + CHECKSUM ✅ (provided by jvscom)
- **Escape sequences**: D0 DF → E0, D0 CF → D0 ✅ (untested)
- **Address assignment**: Master=0x00, Slaves=0x01-0x1F ✅ (infrastructure ready)
- **Initialization**: Double reset + sequential addressing ✅
- **Multi-device chaining**: Infrastructure present but single device only

## API Interface Examples

### Simple Command (Single command per frame)
```systemverilog
// Send RESET command
com_tx_data <= CMD_RESET;        // Command byte
com_tx_cmd_push <= 1'b1;         // Push as command (stores in FIFO)
com_tx_data <= CMD_RESET_ARG;    // Argument byte  
com_tx_data_push <= 1'b1;        // Push as data
com_commit <= 1'b1;              // Transmit frame
```

### Chained Commands (Multiple commands per frame)
```systemverilog
// Send SWINP + COININP + ANLINP in single frame
com_tx_data <= CMD_SWINP;        // Command 1
com_tx_cmd_push <= 1'b1;         // Store in FIFO[0]
com_tx_data <= players;          // SWINP arguments
com_tx_data_push <= 1'b1;

com_tx_data <= CMD_COININP;      // Command 2  
com_tx_cmd_push <= 1'b1;         // Store in FIFO[1]
com_tx_data <= coin_slots;
com_tx_data_push <= 1'b1;

com_tx_data <= CMD_ANLINP;       // Command 3
com_tx_cmd_push <= 1'b1;         // Store in FIFO[2] 
com_tx_data <= channels;
com_tx_data_push <= 1'b1;

com_commit <= 1'b1;              // Transmit chained frame
```

### Sequential RX Processing
```systemverilog
// Process chained response
if (com_rx_complete) begin
    case (com_src_cmd)           // Current command from FIFO
        CMD_SWINP: begin
            // Process switch data with com_rx_byte/com_rx_next
            com_src_cmd_next <= 1'b1;  // Advance to CMD_COININP
        end
        CMD_COININP: begin
            // Process coin data
            com_src_cmd_next <= 1'b1;  // Advance to CMD_ANLINP  
        end
        CMD_ANLINP: begin
            // Process analog data
            // FIFO automatically depleted
        end
    endcase
end
```

## Usage

Include in your Quartus project:
```tcl
set_global_assignment -name QIP_FILE [file join $::quartus(qip_path) jvs/jvs.qip]
```

## 🚀 Roadmap & Future Improvements

### Architecture Enhancements
- **🔗 Chained I/O Board Support** - Full multi-device addressing (up to 31 devices)
- **📦 Split jvs_ctrl Module** - Create dedicated `jvs_cmd.sv` for I/O command parsing
- **⏰ NCO Clock Generation** - Numerically Controlled Oscillator for precise UART timing
- **⚡ FPGA Resource Optimization** - Reduce logic utilization and improve timing

### Protocol Completeness
- **🔧 Error Recovery** - Checksum error handling and retransmission
- **⏱️ Timeout Management** - Device response timeouts and recovery mechanisms
- **🪙 Advanced Coin Management** - COINDEC, COININC, PAYINC, PAYDEC commands
- **🎮 Extended I/O Support** - Rotary encoders, screen position, keycode inputs
- **📺 Display Output** - Character display control (CHAROUT)

### Performance & Compatibility
- **🏭 Multi-platform Support** - MiSTer, DE10-Nano adaptations
- **📡 Dynamic Baud Rate** - Runtime communication speed changes

## Author

**Totaly FuRy - Sebastien DUPONCHEEL** (sduponch on GitHub)  
Project: Analogizer JVS Controller  
Status: Alpha - Partial Implementation  
Date: 2025

## Credits

Special thanks to **@RndMnkIII** for creating the core JVS_Debugger module and his work on the [Analogizer project](https://github.com/RndMnkIII/Analogizer).