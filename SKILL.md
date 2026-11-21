---
name: jvs-core-integration
description: How to add JVS (this library, jvs_ctrl) to an Analogue Pocket openFPGA core that uses the Analogizer SNAC module (openFPGA_Pocket_Analogizer_SNAC.sv) — pins, SNAC type, tie-offs, button mapping, the JVS profile data slot, guns and recoil, board timings, and how to verify the port. Use it before porting JVS to a new core (NES, SNES, Genesis, arcade cores) or when a ported core's JVS does not work.
---

# Integrating JVS into an openFPGA Analogizer core

Written from the ports to JVS_Debugger and openfpga-NES-Analogizer (branch `jvs`) and from real board tests
(Sega 838-13683B). Follow the steps in order; each pitfall below cost a debugging session once.

## 0. Before writing anything

- **Start from the target core's upstream `master`**, not from an old local WIP. An old port overwrote the
  core's own SNAC changes (the NES Zapper support was silently reverted). Keep any WIP on a branch.
- **Check the area.** `jvs_ctrl` + `jvs_com` cost about **1,450 ALMs** in the NES core (1,960 in JVS_Debugger,
  which also uses the node name BRAM and the bus logger). Profiles, guns and recoil add about **500 ALMs**.
  Take the target's last `*.fit.summary`; above ~90 % of 18,480 ALMs, plan for a seed sweep.
  Old copies of this library (Sept. 2025: `jvs_com` alone 2,580 ALMs) must not be reused: take `bus_state`.
- **Licence.** This repository has no licence file yet; most target cores are GPL-3.0. Add a compatible
  licence here before a core that includes it is published.

## 1. Bring the library in

- Add it as a **git submodule** at `analogizer/jvs`, branch `bus_state`:
  `git submodule add -b bus_state https://github.com/sduponch/jvs.git analogizer/jvs`
- Include `jvs/jvs.qip` **first** in `analogizer/analogizer.qip` (the packages `rtl/jvs_defs_pkg.sv` and
  `rtl/jvs_node_info_pkg.sv` must be analysed before the files that import them):
  `set_global_assignment -name QIP_FILE [file join $::quartus(qip_path) jvs/jvs.qip]`
- `jvs_bus_logger.sv` is in the qip; Quartus prunes it when it is not instantiated.

## 2. The SNAC controller type and the pins

- SNAC type **`GC_JVS = 5'h14` (20)**, written by the Analogizer Configurator in `analogizer.bin`.
  With `bridge_endian_little = 0` the bridge gives file byte 0 in `bridge_wr_data[31:24]`, and the Analogizer
  tops read the configuration **little endian: file byte 0 = bits 7:0** (type [4:0], Analogizer on [5],
  assignment [9:6], video [13:10], blank [14]). JVS, Analogizer on, SNAC P1/P2 → P1/P2, RGBS = `b4 00 00 00`.
  A file reading `00 00 xx xx` is all off for the core (JVS never starts): check it before debugging the bus.
  **Check which file is read:** the slot's parameters `0x1000000` (bits 25:24 = 1) select platform index 1 of
  `core.json`'s `platform_ids` (`["nes", "analogizer"]`): the Pocket reads `Assets/analogizer/common/analogizer.bin`,
  not `Assets/<platform>/common/` — one config shared by every Analogizer core.
  The core's own "Enable Analogizer" menu entry must be on as well (`core_top` uses it to take the SNAC pads).
- **Conf. A pins** (the JVS SNAC adapter's wiring, hardware is final):

  | Signal | SNAC | Pocket pin | Direction |
  |---|---|---|---|
  | UART TX (DI) | OUT1 | `cart_tran_bank1[6]` | out |
  | RS485 DIR (DE, active high) | OUT2 | `cart_tran_bank1[7]` | out |
  | UART RX (RO) | IN4 | `cart_tran_bank0[7]` | in |
  | SENSE | IN7 | `cart_tran_bank0[5]` | in |
  | LINK (active low, optional) | IO3 | `cart_tran_bank0[4]` | in |

- **Pitfall — Conf. B.** Many cores compute `conf_AB = (snac_game_cont_type >= 16)`. Type 20 would then use
  the Conf. B pins and nothing works. Exclude it:
  `wire conf_AB = (snac_game_cont_type >= 5'd16) && (snac_game_cont_type != 5'd20);`
  (JVS_Debugger: a `case` that lists 20 with the Conf. A types.)
- Bank 0 must be an **input** in JVS mode (the SNAC module's Conf. A branch already does it).
- If the core gates analog joystick data on the SNAC type (e.g. NES `snac_is_analog` = PSX analog types),
  add 20 there too, or the profile axes never reach the core.

## 3. Instantiating `jvs_ctrl`

```systemverilog
jvs_ctrl #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ), .HAS_LINK_DETECT(0)) jvs_inst (
    .i_clk(i_clk), .i_rst(reset_on_change || vendor_loading_s[1]), .i_ena(jvs_ena), .i_stb(1'b0),
    .i_uart_rx(SNAC_IN4), .o_uart_tx(JVS_UART_TX), .i_sense(SNAC_IN7), .i_link(~SNAC_IO3_A),
    .o_rx485_dir(JVS_485_DIR), .o_bus_state(),
    .player1_input_switch(jvs_sw1), .player2_input_switch(jvs_sw2), ...
    .node_analog(jvs_node_analog),                 // MAX_JVS_NODES*8 channels of 16 bits
    .coin1(jvs_coin1), .coin2(jvs_coin2), ...
    .p1_output_digital(p_output_digital[0]), ...   // 24'd0 without outputs
    .output_analog(128'd0), .output_node(8'd0),
    .jvs_data_ready(), .jvs_nodes(), .node_name_rd_data(), .node_name_rd_addr('0),
    .vendor_rd_addr(vendor_rd_addr), .vendor_rd_data(vendor_rd_data)   // 8'd0 = no profile
);
```

- **`MASTER_CLK_FREQ` must be the real frequency of `i_clk`** (NES: 42,954,496 NTSC / 42,562,736 PAL; the
  UART divisor is `MASTER_CLK_FREQ / 115200`). The UART counters are sized from `CLKS_PER_BIT`; a fixed 9-bit
  counter once froze the UART on the first bit at 96 MHz (jtcores CPS) — never narrow them again.
- `jvs_ena <= (game_cont_type == GC_JVS);` in its own `always`: the SNAC module's type `case` does not clear
  the other enables when the type changes.
- **Pitfall:** no trailing comma after the last port (an old port did not compile because of it).
- `vendor_rd_data = 0` means NULL pointers: standard init, no MAINID, no vendor frame.
- Unused outputs left open; unused inputs tied to 0.

## 4. Buttons

- `playerN_input_switch` is already in the **Pocket key layout**: bits 3:0 D-pad (up, down, left, right),
  4 push 1, 5 push 2, 6 push 3, 7 push 4, 8..13 push 5..10, 14 SERVICE, 15 START.
- Remap per core in the SNAC module (NES: push 1 → B, push 2 → A, push 3/4 → B/A turbo, so that the panel's
  left button is the pad's left button).
- **Coins:** `coinN` is high for one polling cycle only (~10 ms), shorter than a frame. **Stretch it**
  (100 ms) before mapping it to SELECT, or games miss it.

## 5. The JVS profile data slot (`jvs_vendor.bin`)

Same 256-byte table as JVS_Debugger, built by `scripts/jvs_vendor.py` (its docstring is the format):
MAINID + vendor frames per node (0x00..0xD7), player axes (0xD8..0xDF), trigger masks (0xE8..0xEF),
OUTPUT1 base and recoil masks (0xF0..0xFF).

- **`data.json`** (both `dist/` and `pkg/` copies when a core has two):
  ```json
  { "name": "JVS Profile", "id": 24, "required": false, "filename": "jvs_vendor.bin",
    "parameters": "0x203", "extensions": ["bin"], "size_maximum": "0x100", "address": "0x50000000" }
  ```
  `0x203` = bit 0 reloadable from the Pocket menu, bit 1 core-specific folder (`Assets/<platform>/<core id>/`),
  bit 9 "persist browsed filename": the profile picked in the menu is loaded again at the next core start
  ("Reset All to Defaults" forgets it). Each reloadable slot adds a line to the core's menu (see the 16-entry
  limit in section 6).
  **Check that the id and the address are free in the core** (bridge address decode in `core_top.sv`:
  NES uses 0x1, 0x2, 0x4, 0xF7, 0xF8 — 0x50 is free).
- Decode in the Analogizer top (`clk_74a`):
  `wire jvs_profile_wr = bridge_wr && bridge_addr[31:24] == 8'h50 && bridge_addr[23:8] == 0;`
  and pass `bridge_addr[7:2]`, `bridge_wr_data` to the SNAC module.
- **Byte order:** the bridge gives file byte 0 in `bridge_wr_data[31:24]`; read byte `a` of a word as
  `word[8*(3 - a[1:0]) +: 8]`. If the core has `bridge_endian_little`, swap when it is set (JVS_Debugger).
- Store it in a **dual-clock M10K** (write `clk_74a`, read `i_clk`, data 2 cycles after `vendor_rd_addr`).
  Keep the fixed tail (0xD8..0xFF = **40 bytes = 10 words**, word addresses 0x36..0x3F) in registers too, and
  re-register it in `i_clk` (quasi-static). Pitfall: an array of 8 words for it does not elaborate.
- **Check what the core does on any data slot load.** Many cores (NES: `core_top.sv` `is_downloading`, then
  `nes_top.sv`) reset the console and clear the save RAM at the end of every download but a few known slots: a
  profile reloaded from the menu then restarts the game and can wipe a battery save. Keep the profile's slot out
  of that logic (NES: `if (dataslot_requestwrite && dataslot_requestwrite_id != 16'd24) is_downloading <= 1;`).
  JVS_Debugger has no such logic, so a profile load there does not reset anything.
- **Chip32 cores load their data slots themselves.** With `"chip32_vm"` in `core.json`, the Pocket loads only
  what the Chip32 program asks with `loadf`: a new slot in `data.json` is **never loaded** until the program
  loads it too (NES: the profile never arrived — no axes, no trigger, no recoil). Add to the program's slot
  loads (NES `loader/nes_loader.asm`, after the Analogizer config):
  ```
  ld r1,#jvs_profile_dataslot
  queryslot r1
  jp nz,jvs_profile_done
  loadf r1
  jvs_profile_done:
  ```
  Assembler: ARM9/bass branch `devel` (add `#include <stdexcept>` to `nall/arithmetic/natural.hpp` with a
  recent GCC) and `architectures/chip32.vm.arch` from open-fpga/bass-chip32, next to the `bass` binary.
  Rebuild the unmodified source first and check it is byte-identical to the shipped `.bin` (the NES source
  lacked the `seek2()` / `read2()` macros: copies of `seek()` / `read()` with their own labels).
- **Chip32 cores** (`"chip32_vm"` in `core.json`, e.g. the NES core's `nes_loader.bin`, which picks the bitstream):
  reloading any data slot from the menu restarts the whole core, bitstream included, and the Pocket does not
  write the interact settings again. Registers that hold a menu setting are back at their initial value: give
  the ones the JVS needs a working default (NES: `reg analogizer_ena = 1;`, else "Enable Analogizer" has to be
  toggled before the SNAC / JVS is used again). Persist the profile choice (bit 9) so the restart happens once.
- **Reload while running:** hold `jvs_ctrl` in reset during the transfer and 10 ms after the last word
  (`vendor_loading`), so that the init restarts with the new table.
- Ship profiles in `dist/Assets/<platform>/<core id>/`: an empty `jvs_vendor.bin` (default) and examples
  (`jvs_vendor.py --p1-x 1:0 --p1-y 1:1 --dout-base "00 00 00" --recoil-trigger "02 00"` for a generic gun:
  trigger masks are written only when an output option is given).

## 6. Axes, guns and recoil

- Axis byte: bit 7 defined, bits 6..4 node − 1, bit 3 inverted, bits 2..0 channel; value
  `node_analog[(node*8 + ch)*16 +: 16]`, inverted with `~`. **`MAX_JVS_NODES` is 2**: axes on nodes 3..8 are
  undefined.
- Trigger masks are in **JVS order** (switch bytes 1, 2); compare them with the switches put back in that order
  (`jvs_order()` in the SNAC modules), not with the Pocket layout.
- Recoil: console games never drive one, so the core makes it from the trigger. OUTPUT1 = base | player's
  recoil mask while the pulse is on; all zero without an output profile (output wiring is cabinet specific).
  Give the user a menu choice, decoded in the Analogizer top in `clk_74a`, `synch_3` into `i_clk`.
  **The Pocket shows 16 `interact.json` entries at most, and each user-reloadable data slot takes a menu line
  too; the last entries are silently dropped** (the documentation only mentions a combined limit of 20 above 4
  reloadable slots, the firmware is stricter). NES had exactly 16 entries: adding the reloadable "JVS Profile"
  slot dropped the last two (Use Zapper, D-Pad Aim Speed), and a 17th entry broke more. Count entries + reloadable
  slots first, leave margin, drop the disabled section titles (`"type": "action", "enabled": false`) when room is
  needed (NES: 16 → 13), and fold new choices into an existing `list` (one entry whatever its options, up to 16).
  **Changing an entry's type (check → list) needs a new `id`:** the Pocket keeps `{id, type, val}` in
  `Settings/<core>/Interact/_core/interact_persist.json`, and an old `check` value under a `list` id gives
  "load error in interact" (Genesis: "Lightgun" check 70 + "Show Crosshair" 71 merged into list 73). NES: "Use Zapper" (0x304) = 0 off, 1 emulated, 2 SNAC Zapper, 3 emulated + recoil,
  4 emulated + recoil auto; `core_top.sv` maps 1/3/4 to the emulated Zapper, the Analogizer top 3/4 to the
  recoil modes:
  - **Off**: never (the profile's base value only).
  - **Single** (default): one 50 ms pulse per trigger press (rising edge).
  - **Auto** (machine gun): from the press and while the trigger is held, 50 ms on / 150 ms off (5 shots a
    second); one counter per player reloaded with 200 ms at the press and whenever it runs out while held,
    the output on during its first 50 ms. A new press restarts the cycle at once.
  Pick a free interact address: check the `casex (bridge_addr[11:0])` of the core's `core_top.sv`.
- Gun to the core: scale the 16-bit axes to the core's own range. NES: the profile's P1 axes drive the left
  stick absolutely (X = `axis[15:8]` 0..255, Y = `axis*240 >> 16`), the trigger drives A, and the core's
  "Use Zapper → Emulated Zapper (Stick)" aims and fires with them.

## 7. Board timings that matter (tested)

- Init in `jvs_ctrl`: 6 s after power-up, reset `F0 D9`, 1 s, reset again, **3 s**, then SETADDR. A
  **Sega 838-13683B ignores a SETADDR sent 100 ms after the second reset** (seen on MIRLO64: TX blinks, RX never).
- **≥ 10 ms between an answer and the next poll** for the 838-13683B (`POLLING_INTERVAL_DELAY`).
- Namco NA-JV: vendor frames 10 ms apart; it ignores a frame sent 0.5 ms after the previous answer.
- RS485 DIR ~10 µs before the first start bit, ~30 µs after the last stop bit; leave the bus quiet a few tens
  of µs after an answer (the board's driver trails its last stop bit).
- 838-13683B: 2 players × 11 switches, 2 coin slots, 8 analog channels (unconnected ones float around
  0x0030..0x0180), 8 outputs, no vendor frame needed.

## 8. Verify before handing it over

- Build **every variant** of the core (NES: `NTSC_SET1`, `NTSC_SET2`, `PAL_SET1`, `PAL_SET2` through
  `generate.tcl`), each in its own copy of the tree when run in parallel (`generate.tcl` rewrites the `.qsf`).
- Check **all** timing analyses, not one line:
  `grep '^Slack' output_files/*.sta.summary | awk '$3<0'` must print nothing.
- Some cores never close timing upstream (Neo Geo: 12 of 104 analyses negative, SDRAM inputs at −8 ns, Y/C and
  scandoubler paths). Then build the **unmodified upstream with the same Quartus** and compare: same clocks,
  same count, same magnitude, and list the worst paths (`get_timing_paths ... -npaths 400` in a `quartus_sta -t`
  script) to check that none goes through `jvs_*` or the SNAC module.
- Read the SNAC module's synthesis warnings (truncations, "never read", stuck registers).
- Bitstream: bit-reverse every byte of the `.rbf` (`reverse_bits.exe`, or any equivalent script) into the
  core's `.rev` / `.rbf_r`.
- Simulation of this library: `simu/` (`make regression`, Questa or Verilator); replay real captures
  (`simu/captures`, `scripts/jvs_dump.py`).
- On the Pocket: an adapter LED pattern of one TX blink, a pause, two quick blinks, repeating, with RX dark,
  is the init looping on a board that does not answer — compare the init delays above before suspecting
  clocks. The JVS Debugger core in sniffer mode shows what is really on the bus.
