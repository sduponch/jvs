# JVS Module Changelog

All notable changes to the JVS protocol implementation will be documented in this file.

## [Unreleased]

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