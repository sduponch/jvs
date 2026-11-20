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
- **Debug** (`jvs_bus_logger.sv`, `scripts/jvs_dump.py`) - Bus capture and decoding
- **Simulation** (`simu/`) - Regression tests, including replays of real hardware captures

## Architecture

```
jvs/
├── rtl/                    # RTL definitions and packages
│   ├── jvs_defs_pkg.sv    # JVS constants and command definitions
│   └── jvs_node_info_pkg.sv # Node information structures
├── jvs_com.sv            # Communication layer (UART + framing)
├── jvs_ctrl.sv           # Protocol layer (JVS state machine)
├── jvs_bus_logger.sv     # Bus capture (both lines, SENSE/LINK, DIR, us timestamps)
├── uart_tx.v             # UART transmitter
├── uart_rx.v             # UART receiver
├── jvs.qip               # Quartus IP Project
├── scripts/jvs_dump.py   # Capture decoder / replay script export
└── simu/                 # Simulation (Questa or Verilator)
    ├── tb_smart_device.sv     # init, polling, outputs, link recovery vs a board model
    ├── tb_capture_replay.sv   # replay of a real bus capture
    ├── smart_jvs_device.sv    # synthetic I/O board model
    ├── replay_jvs_device.sv   # capture replay model
    └── captures/              # real captures (regression vectors)
```

## Implementation Status

**ALPHA Version** - Partial implementation (~30-40% of JVS v3.0 specification)  
Based on JVS Specification v3.0 (25 pages, 49 commands total)

### ✅ Fully Implemented (11/49 commands)
- **Reset (F0)** - Double reset sequence with timing delays
- **Set Address (F1)** - Single device addressing
- **IO Identity (10)** - Device name string reading (up to 100 chars)
- **Command Revision (11)** - Format revision detection (BCD)
- **JVS Revision (12)** - Protocol version detection (BCD)
- **Communications Version (13)** - Communication system version
- **Feature Check (14)** - Complete capability parsing with all function codes
- **Switch Inputs (20)** - Digital buttons (up to 4 players across chained nodes)
- **Coin Inputs (21)** - Multi-slot coin counters (14-bit counter + condition), raw status on `coin_status`
- **Analog Inputs (22)** - Up to 8 channels per node, 16-bit; per node copy on `node_analog`
- **Generic Output 1 (32)** - Digital outputs, sent to every node declaring outputs with ceil(outputs/8) data bytes
- **Analog Output (33)** - `output_analog` channels, sent to every node declaring analog outputs
- **JVS escape sequences** - E0 → D0 DF, D0 → D0 CF in both directions (validated in simulation)
- **RS485 timing control** - Setup/hold delays (measured on hardware: 10 us / 30 us)

### 🟡 Partially Implemented (2/49 commands)
- **Screen Position Inputs (25)** - Basic X/Y coordinates (16-bit each)
- **Keycode Inputs (24)** - Command sent, response skipped
- **Misc Switch Inputs (26)** - Command sent, response skipped

### ❌ Not Implemented (36/49 commands)
- **Main ID (15)** - Send main board identification to device
- **Rotary Inputs (23)** - Rotary encoder data (parsed but ignored)
- **Remaining Payout (2E)** - Medal hopper status and count
- **Data Retransmit (2F)** - Checksum error recovery
- **Coin management**: COINDEC (30), COININC (35), PAYINC (31), PAYDEC (36)
- **Advanced outputs**: OUTPUT2 (37), OUTPUT3 (38), CHAROUT (34)
- **Communication changes**: COMMCHG (F2)
- **Manufacturer-specific commands** (Taito TypeX, Namco, CyberLead LED)
- **Dynamic baud rate changes**
- **Checksum error recovery** (timeouts are recovered: unanswered commands are flushed and the init sequence restarts)

## Hardware Compatibility

### Compatible I/O Boards
- **SEGA 838-13683B** (`SEGA ENTERPRISES,LTD.;I/O 838-13683B ;Ver1.07;99/06`) - single board and two chained boards, hardware captures in `simu/captures/`
- **NAJV2** (Tekken 7)
- **NAJV** (Time Crisis 4)
- **TAITO CORP Ver2.0** (Viewlix)
- **"No Brand;NAOMI Converter98701;ver2.0"** - previously listed as incompatible; its oscilloscope capture (`simu/captures/naomi_tekken6_io_board_2.jvs`) now replays identically in simulation, hardware confirmation pending

### Known limitations
- Up to `MAX_JVS_NODES` (2) chained nodes, 4 players, 4 coin slots, 8 analog channels on the shared outputs
- Push buttons 9 and 10 are only kept for P1 (forced to 0 for P2..P4)
- Analogizer JVS SNAC adapter: the LINK comparator does not follow the SENSE line, keep `HAS_LINK_DETECT=0`

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
- **Address assignment**: Master=0x00, Slaves=0x01-0x1F ✅
- **Initialization**: Double reset + sequential addressing ✅
- **Multi-device chaining**: ✅ up to `MAX_JVS_NODES` nodes (SENSE driven addressing)

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

### SENSE / LINK lines and bus state

`jvs_ctrl` reads the JVS SENSE line through two digital inputs:

| Port | Meaning | When not available |
|---|---|---|
| `i_sense` | `1` = all devices addressed (SENSE ≈ 0 V), `0` = address needed or no device | required |
| `i_link` | `1` = a device is present on the SENSE line (SENSE < ~3 V) | leave `HAS_LINK_DETECT=0`, tie to `1'b1` |

Both inputs are synchronized and debounced (`LINE_DEBOUNCE_MS`, 10 ms by default).
With `HAS_LINK_DETECT=1`, the controller and `jvs_com` are held in reset while no
device is present: nothing is transmitted on an empty bus, and the full init
sequence restarts on every hot-plug.

`o_bus_state` (`jvs_bus_state_t`, see `rtl/jvs_node_info_pkg.sv`) gives a high level view of the link:

| State | Meaning |
|---|---|
| `JVS_NO_DEVICE` | LINK inactive, nothing plugged (only with `HAS_LINK_DETECT=1`) |
| `JVS_ADDRESSING` | Startup delay, bus reset and address assignment |
| `JVS_INIT` | Reading node identity, revisions and features |
| `JVS_READY` | Input polling running |
| `JVS_ERROR` | 3+ consecutive timeouts: a device is expected but does not answer |
| `JVS_LINK_LOST` | SENSE lost while polling, waiting for the device to come back |

**Link supervision with SENSE only.** Once polling has started with SENSE reporting
"all addressed" (`i_sense=1`), a drop of `i_sense` is treated as an unplug:
the controller stops transmitting, flushes `jvs_com`, releases all inputs and
enters `JVS_LINK_LOST`. If SENSE comes back within `RELINK_TIMEOUT_MS` (3 s by
default), the device still holds its address and polling resumes immediately;
otherwise a full init sequence (reset, addressing, identification) restarts.
Boards that never pull SENSE to "addressed" are not supervised (no false link loss).

While nobody answers SETADDR, the controller periodically restarts from the bus
reset, so a device plugged later (possibly still holding an old address) is
always reset before being addressed.

> On the Analogizer JVS SNAC adapter, the LINK comparator does not follow the
> SENSE line reliably: keep `HAS_LINK_DETECT=0` and rely on SENSE supervision.

```systemverilog
jvs_ctrl #(.MASTER_CLK_FREQ(50_000_000), .HAS_LINK_DETECT(1)) jvs_inst (
    ...
    .i_sense(sense_addressed),
    .i_link(device_present),
    .o_bus_state(jvs_bus_state),
    ...
);
```

### Debug and per node outputs

| Port | Content |
|---|---|
| `node_analog` | 8 analog channels of every node: node n channel c at `[(n*8 + c)*16 +: 16]` |
| `coin_status` | raw `{condition[1:0], counter[13:0]}` of global coin slots 0..3 |

Players, coin slots and analog channels of chained nodes are stored one after the other on the shared outputs (`playerN_input_switch`, `coin*`, `analog_chN`).

## Bus capture

`jvs_bus_logger.sv` decodes both serial lines independently of the controller and records every byte, the SENSE (`setup_n`) / LINK changes and the RS485 DIR changes with a microsecond timestamp (format in the module header: `JVSDUMP2`, 8-word header then 2 words per entry). In ring mode (`ring`) it keeps the most recent entries, which lets a host core sniff a bus driven by another host.

`scripts/jvs_dump.py` decodes a capture (`.bin`, or `.hex` written by the simulation):
```bash
python3 scripts/jvs_dump.py jvs_dump.bin          # frames, checksums, RS485 timing
python3 scripts/jvs_dump.py jvs_dump.bin --raw    # every entry
python3 scripts/jvs_dump.py jvs_dump.bin --replay x.replay   # replay script for simu/
python3 scripts/jvs_dump.py capture.jvs --replay x.replay --prepend-resets --drop <frame>
                                                  # text capture (oscilloscope, one frame per line)
```

## Simulation

```bash
cd simu
make regression                 # smart device test + replay of every capture (Questa)
make regression SIM=verilator   # same with Verilator (used by the CI)
make smart                      # tb_smart_device only
make replay REPLAY=captures/sega_838-13683B_x2.replay
```

* `tb_smart_device`: identification, input polling, digital outputs and link recovery (the board stops answering, then answers again) against `smart_jvs_device.sv`, with automatic `[CHECK] PASS/FAIL` lines.
* `tb_capture_replay`: plays a real capture back against the controller (request driven): every frame sent by the controller is compared byte by byte with the captured one, and the decoded values (identity, revisions, switches, coins, analog channels) are checked per node.
* `FAST=1` (default) defines `JVS_SIM_FAST`, which shortens the startup / reset delays and the answer timeout: the full regression runs in about 20 s.
* Adding a regression vector: copy a capture to `simu/captures/<name>.bin` (Pocket dump) or `<name>.jvs` (text capture, with optional conversion options in `<name>.opts`). The replay script is generated by the Makefile.
* Questa (Intel FPGA Starter Edition) needs `SALT_LICENSE_SERVER` pointing to the license file (default `~/.altera.quartus/questa_lic.dat`).

The CI (`.github/workflows/quartus-build.yml`) runs `make regression SIM=verilator` in a pinned Verilator docker image before the Quartus build.

## 🚀 Roadmap & Future Improvements

### Architecture Enhancements
- **🔗 Chained I/O Board Support** - More than `MAX_JVS_NODES` (2) devices (up to 31)
- **📦 Split jvs_ctrl Module** - Create dedicated `jvs_cmd.sv` for I/O command parsing
- **⏰ NCO Clock Generation** - Numerically Controlled Oscillator for precise UART timing
- **⚡ FPGA Resource Optimization** - Reduce logic utilization and improve timing

### Protocol Completeness
- **🔧 Error Recovery** - Checksum error handling and retransmission
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
Date: 2025-2026

## Credits

Special thanks to **@RndMnkIII** for creating the core JVS_Debugger module and his work on the [Analogizer project](https://github.com/RndMnkIII/Analogizer).