# JVS Module Changelog

All notable changes to the JVS protocol implementation will be documented in this file.

## [Unreleased]

### Added
- `jvs_bus_state_t` enum in `jvs_node_info_pkg` and `o_bus_state` output on `jvs_ctrl`
  (NO_DEVICE / ADDRESSING / INIT / READY / ERROR / LINK_LOST)
- `i_link` input and `HAS_LINK_DETECT` parameter: no traffic while no device is plugged,
  full re-initialization on hot-plug
- 2-FF synchronization and debounce (`LINE_DEBOUNCE_MS`) on SENSE and LINK inputs
- SENSE based link supervision: `JVS_LINK_LOST` state on unplug while polling, resume
  if the device comes back addressed, full re-init after `RELINK_TIMEOUT_MS`
- SETADDR without answer now restarts from the bus reset (hot-plug of a device
  holding a stale address)
- `node_analog` output: the 8 analog channels of every node (debug/display)
- `coin_status` output: raw {condition, counter} of each coin slot (debug/display)
- `jvs_bus_logger`: decodes both serial lines and records every byte plus SETUP / LINK
  changes with a microsecond timestamp (jvs_dump format, see the module header), and
  RS485 DIR changes (type 04) to check the driver enable / release timing
- `jvs_bus_logger` ring mode (`ring` input) and `JVSDUMP2` format (8-word header: flags,
  oldest entry index): used by the core sniffer mode to keep the most recent traffic
- `jvs_bus_logger` `sync_start` input / `waiting` output: nothing is recorded before the
  first SYNC byte (E0), so a capture armed before the host is powered on starts on a
  frame boundary
- `scripts/jvs_dump.py` reads JVSDUMP1/2, reorders ring captures, and in sniffer captures
  tells host requests from board answers by their destination node (replayable too)
- `scripts/jvs_dump.py` reports the measured RS485 timing (DIR on -> first TX byte,
  last TX byte -> DIR off)
- `scripts/jvs_dump.py`: decodes a capture frame by frame (checksums checked) and exports
  a replay script (`--replay`)
- Simulation `tb_capture_replay`: replays a real hardware capture against the controller
  (request driven), checks that every frame sent is identical to the captured one and
  that the decoded values match, per node. Reference captures in `simu/captures/`
  (`sega_838-13683B.bin`, `sega_838-13683B_x2.bin` with 2 chained boards) and
  `naomi_tekken6_io_board_2.jvs`: oscilloscope capture of a NAOMI host with the
  "No Brand;NAOMI Converter98701;ver2.0" board, previously listed as incompatible: its
  742 frames now replay identically. Text captures (`.jvs`, one frame per line) are
  converted with `jvs_dump.py --replay` (`--prepend-resets`, `--drop`, see `<name>.opts`)
- Simulation `tb_smart_device` (was test_03): identification, input polling, digital
  outputs and link recovery against the smart device model, with distinctive values and
  automatic PASS/FAIL checks
- `simu/Makefile`: `make smart`, `make replay`, `make regression` (smart + every capture),
  `SIM=questa|verilator`, `FAST=1` (`JVS_SIM_FAST`: shortened startup / reset delays,
  full regression in ~20 s). Obsolete tests and helper files removed
- ANLOUT (33): `output_analog` input on `jvs_ctrl` (8 x 16-bit channels), sent to every
  node declaring analog outputs with the declared channel count (e.g. the cabinet top
  light ramp driven by NAOMI hosts on channel 1)
- Simulation: the smart device model checks the OUTPUT1 / ANLOUT data it receives, escapes
  its answers and decodes escaped requests per the spec (analog CH7 = E0D0 and ANLOUT
  channel 0 = E000 exercise both directions)
- CI: `simulation-regression` job (Verilator, pinned docker image) runs before the
  Quartus build

### Fixed
- RX byte stream (validated in simulation):
  - `jvs_com`: `rx_read_idx` was incremented once too many when presenting the first
    REPORT byte, so the first `rx_next` skipped the first data byte of every answer
  - `jvs_ctrl` STATE_RX_NEXT used the stale `rx_ready` of the previous byte during its
    first cycle and could return before the next byte was read
  - Both bugs partially compensated each other depending on each parser's timing:
    REPORT byte stored in place of the first character of the node name, CMD/JVS/COM
    revisions read as 0x01, analog channels shifted by one (last channel lost), coin
    slot 2 never parsed. Parsers re-aligned: the dispatcher checks the REPORT byte, coin
    and analog parsers use the same structure as the switch parser and resume the
    dispatcher on the next REPORT byte without an extra read
- `jvs_com` TX escape: a data byte equal to E0 or D0 stalled the transmitter (it waited
  for a BRAM read already consumed). Now sent as D0 DF / D0 CF with the checksum computed
  on the unescaped byte, as required by the spec
- `jvs_com` port widths used localparams declared later in the module (rejected by Questa)
- No recovery after answer timeouts: the commands of unanswered frames stayed in the
  `jvs_com` command FIFO (later answers parsed with the wrong commands, FIFO eventually
  full) and polling timeouts never re-initialized the bus. `jvs_com` is now flushed on
  every timeout, and while polling a timeout restarts the full init sequence when SENSE
  reports "not addressed" (device reset / power cycled) or after 3 consecutive timeouts
- JVSREV (12) and COMMVER (13) requests never loaded the answer timeout: WAIT_RX timed
  out immediately while the frame was still being transmitted (hidden until timeouts
  started to flush `jvs_com`)
- OUTPUT1 (32) is sent to every node declaring digital outputs (was single player boards
  only) with ceil(outputs / 8) data bytes (was always 3); its REPORT-only answer is parsed
  with the same alignment rule as the other commands
- Analog channels were re-centered (0x8000) on every analog poll, making all analog
  outputs glitch to center for a few microseconds every polling cycle
- Chained nodes overwrote each other's analog channels: channels are now stored one
  after the other (node 2 channels follow node 1 channels), like players and coins
- Addressing stops at `MAX_JVS_NODES` (node_count could exceed the node storage)
- Next node info initialization wrote at index addr+1 instead of addr (out of bounds)
- `player_count` was not cleared on reset and kept growing on each re-initialization
- `timeout_retry_count` now saturates instead of wrapping around after 255 timeouts

### Known issues
- Switch inputs: push buttons 9 and 10 (bits 12/13) are only kept for P1; they are
  forced to 0 for P2..P4 (boards with more than 8 buttons, e.g. 15 on a chained 838-13683B)
- Simulation does not cover digital/analog outputs, rotary, keycode, screen position and
  misc inputs yet (the simulated board does not declare them)

## [0.2.0-dev] - previous unreleased work

### Added
- Modular architecture with jvs_com communication layer
- Standalone submodule structure for GitHub distribution
- Comprehensive README.md with implementation status
- RTL package organization in dedicated rtl/ directory
- Optimized API interface for jvs_com module
- Development files (TODO.md, CHANGELOG.md, .gitignore)

### Changed
- Split communication logic into separate jvs_com module
- Moved packages to rtl/ subdirectory for better organization
- Updated QIP file structure for modular compilation
- Refined API naming for intuitive usage:
  - TX Path: tx_data, tx_data_push, dst_node, commit, tx_ready
  - RX Path: rx_data, rx_valid, src_node, src_cmd, rx_complete, rx_error
- Simplified interface by removing redundant tx_done signal

### API Design
- Sequential data push interface (tx_data + tx_data_push)
- CMD echo mechanism for RX correlation (src_cmd)
- Automatic frame encapsulation (SYNC + NODE + LENGTH + DATA + CHECKSUM)
- Clean separation: jvs_com handles transport, jvs_controller handles protocol

## [0.1.0-alpha] - 2025-09-10

### Added
- Initial JVS Master controller implementation
- Support for 8/49 JVS commands (fully implemented)
- Partial support for 4/49 JVS commands
- RS485 communication with proper timing (10μs/30μs)
- Escape sequence handling (D0 DF → E0, D0 CF → D0)
- Frame validation with checksum verification
- Multi-node infrastructure (single device only)
- Hardware compatibility with NAJV2, NAJV, TAITO CORP Ver2.0

### Implemented Commands
- Reset (F0) - Double reset sequence with timing delays
- Set Address (F1) - Single device addressing
- IO Identity (10) - Device name string reading (up to 100 chars)
- Command Revision (11) - Format revision detection (BCD)
- JVS Revision (12) - Protocol version detection (BCD)
- Communications Version (13) - Communication system version
- Feature Check (14) - Complete capability parsing
- Switch Inputs (20) - Digital buttons (2 players, 13 buttons each)
- Analog Inputs (22) - Multi-channel 16-bit analog data
- Generic Output 1 (32) - Digital GPIO control (3-byte format)

### Known Issues
- Incompatible with "No Brand;NAOMI Converter98701;ver2.0" I/O boards
- Multi-device addressing not implemented
- 37/49 JVS commands not implemented
- No error recovery mechanisms

---

## Format
This changelog follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

### Categories
- **Added** for new features
- **Changed** for changes in existing functionality
- **Deprecated** for soon-to-be removed features
- **Removed** for now removed features
- **Fixed** for any bug fixes
- **Security** for vulnerability fixes