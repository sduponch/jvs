# JVS Protocol Implementation

⚠️ **ALPHA STATUS - PARTIAL COMMAND IMPLEMENTATION** ⚠️

This repository contains a JVS (JAMMA Video Standard) Master controller implementation for FPGA platforms, designed for connecting JVS arcade cabinets through RS485 communication.

## Overview

This module implements a JVS (JAMMA Video Standard) Master controller that allows connecting JVS arcade cabinets to FPGA platforms. The implementation is optimized for gaming performance with partial command support.

The JVS protocol implementation is organized in a modular architecture:

- **RTL Layer** (`rtl/`) - Base packages and definitions
- **Communication Layer** (`jvscom/`) - Low-level UART and frame handling  
- **Protocol Layer** - High-level JVS command processing

## Architecture

```
jvs/
├── rtl/                    # RTL definitions
│   ├── JVS_pkg.sv         # Base JVS package
│   └── jvs_node_info_pkg.sv # Node information structures
├── jvscom/                # Communication layer
│   ├── jvs_com.sv         # Frame encapsulation/decapsulation
│   ├── uart_rx.v          # UART receiver
│   └── uart_tx.v          # UART transmitter
├── jvs_controller.sv      # Main JVS protocol controller
└── jvs.qip               # Quartus project file
```

## Implementation Status

**ALPHA Version** - Partial implementation (~30-40% of JVS v3.0 specification)  
Based on JVS Specification v3.0 (25 pages, 49 commands total)

### ✅ Fully Implemented (8/49 commands)
- **Reset (F0)** - Double reset sequence with timing delays
- **Set Address (F1)** - Single device addressing
- **IO Identity (10)** - Device name string reading (up to 100 chars)
- **Command Revision (11)** - Format revision detection (BCD)
- **JVS Revision (12)** - Protocol version detection (BCD)
- **Communications Version (13)** - Communication system version
- **Feature Check (14)** - Complete capability parsing with all function codes
- **Switch Inputs (20)** - Digital buttons (2 players, 13 buttons each)
- **Analog Inputs (22)** - Multi-channel 16-bit analog data
- **Generic Output 1 (32)** - Digital GPIO control (3-byte format)
- **JVS escape sequences** - D0 DF → E0, D0 CF → D0
- **RS485 timing control** - Setup/hold delays

### 🟡 Partially Implemented (4/49 commands)
- **Coin Inputs (21)** - Command sent, status parsed but coin data ignored
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
- **jvs_com module** - Low-level communication layer with UART/RS485/framing
- **jvs_controller module** - High-level JVS protocol state machine
- **Command FIFO system** - Supports chained commands (e.g., SWINP+COININP+ANLINP)
- **Sequential API interface** - Optimized TX/RX with 25 signals vs 2048 array-based
- **Node information management** - Device capabilities tracking

### Protocol Compliance
- **Physical Layer**: RS-485 at 115200 baud (8N1) ✅
- **Link Layer**: SYNC(0xE0) + NODE + LENGTH + DATA + CHECKSUM ✅
- **Escape sequences**: D0 DF → E0, D0 CF → D0 ✅
- **Address assignment**: Master=0x00, Slaves=0x01-0x1F ✅
- **Initialization**: Double reset + sequential addressing ✅
- **Multi-device chaining**: Infrastructure present but single device only

### Hardware Requirements
- **External MAX485** or equivalent RS485 transceiver
- **Proper 120Ω termination** for reliable communication
- **JVS-compatible arcade cabinet**
- **SENSE line connection** for proper device chaining (unused in single mode)

## Technical Features

- **RS485 Communication** with proper timing (10μs setup, 30μs hold)
- **Escape Sequence Handling** (D0 DF → E0, D0 CF → D0)
- **Frame Validation** with checksum verification
- **Multi-node Support** (up to 8 devices)
- **Configurable UART** baud rates

## Modular Architecture Details

### jvs_com Module (Communication Layer)
- **UART TX/RX** with configurable baud rates and proper RS485 timing
- **Frame encapsulation/decapsulation** with checksum validation
- **Escape sequence processing** (D0 DF → E0, D0 CF → D0) automatically
- **Command FIFO tracking** for chained command responses
- **Sequential API interface** with tx_data_push/tx_cmd_push and rx_byte/rx_next

### jvs_controller Module (Protocol Layer)  
- **JVS protocol state machine** - Initialization sequence and command handling
- **Device capability parsing** - Feature detection and node information storage
- **Input/output data processing** - Button, analog, coin, and screen position data
- **Chained command support** - Single frame with multiple commands (SWINP+COININP+ANLINP)
- **SNAC interface compatibility** - Direct integration with Analogue Pocket SNAC

### Interface Benefits
- **25 signals vs 2048** - Massive interconnect reduction compared to array-based design
- **DMA-like transfers** - Sequential byte reading with rx_remaining counter
- **Command tracking** - src_cmd_count and src_cmd_next for robust parsing
- **Portability ready** - Clean separation for MiSTer and other FPGA platforms

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

## Simulation Support

Use the following macro for simulation without JVS device:
```tcl
set_global_assignment -name VERILOG_MACRO "USE_DUMMY_JVS_DATA=1"
```

## Author

**Totaly FuRy - Sebastien DUPONCHEEL** (sduponch on GitHub)  
Project: Analogizer JVS Controller  
Status: Alpha - Partial Implementation  
Date: 2025

## Credits

Special thanks to **@RndMnkIII** for creating the core JVS_Debugger module and his work on the [Analogizer project](https://github.com/RndMnkIII/Analogizer).