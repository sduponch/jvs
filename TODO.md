# JVS Development TODO

## Current Status
✅ **COMPLETED**
- ✅ Basic JVS module structure created (`jvs/`, `jvs/rtl/`, `jvs/jvscom/`)
- ✅ jvs_com communication layer interface designed
- ✅ Optimized API interface with intuitive naming:
  - TX: `tx_data`, `tx_data_push`, `dst_node`, `commit`, `tx_ready`
  - RX: `rx_data`, `rx_valid`, `src_node`, `src_cmd`, `rx_complete`, `rx_error`
- ✅ jvs_controller protocol layer (ALPHA - ~30-40% JVS commands)
- ✅ Standalone submodule structure ready for GitHub
- ✅ Complete documentation (README.md, CHANGELOG.md, TODO.md)
- ✅ QIP file organization for Quartus integration
- ✅ RTL package structure (`JVS_pkg.sv`, `jvs_node_info_pkg.sv`)

## Next Development Phases

### Phase 1: jvs_com Implementation
- [ ] Implement TX logic for new API (tx_data_push → commit workflow)
- [ ] Implement RX logic with CMD echo (src_cmd output)
- [ ] Internal buffer management for sequential data push
- [ ] UART integration with new interface
- [ ] RS485 timing control (10μs/30μs delays)

### Phase 2: jvs_controller Integration
- [ ] Refactor jvs_controller.sv to use jvs_com interface
- [ ] Remove redundant UART/RS485 code from jvs_controller
- [ ] Adapt state machines for new API workflow
- [ ] Test communication layer integration
- [ ] Verify frame encapsulation/decapsulation

### Phase 2: Protocol Completion (BETA)
- [ ] Implement missing JVS commands (priority list)
  - [ ] Main ID (15) - Board identification
  - [ ] Rotary Inputs (23) - Complete implementation
  - [ ] Coin management commands (30, 31, 35, 36)
  - [ ] Advanced output commands (37, 38, 33, 34)
- [ ] Error recovery mechanisms
- [ ] Multi-device addressing support
- [ ] Dynamic baud rate changes

### Phase 3: Advanced Features (RELEASE)
- [ ] Manufacturer-specific command support
  - [ ] Taito TypeX extensions
  - [ ] Namco specific commands
  - [ ] CyberLead LED control
- [ ] Performance optimizations
- [ ] Memory usage optimization
- [ ] Timing analysis and improvements

### Phase 4: Platform Portability
- [ ] MiSTer platform support
- [ ] Generic FPGA platform abstraction
- [ ] SNAC interface generalization
- [ ] Platform-specific optimizations

## Testing & Validation

### Hardware Testing
- [ ] NAJV2 (Tekken 7) - Extended testing
- [ ] NAJV (Time Crisis 4) - Gun support validation
- [ ] TAITO CORP Ver2.0 (Viewlix) - Multi-game testing
- [ ] Additional I/O board compatibility testing

### Software Testing
- [ ] Simulation test bench development
- [ ] Automated testing framework
- [ ] Regression testing suite
- [ ] Performance benchmarking

## Documentation
- [ ] API documentation for jvs_com interface
- [ ] Integration guide for other projects
- [ ] Troubleshooting guide
- [ ] Hardware setup documentation

## Notes
- Keep ALPHA status until Phase 2 completion
- Maintain backward compatibility during refactoring
- Document all breaking changes
- Regular testing with real hardware

---
*Last updated: 2025-09-10*