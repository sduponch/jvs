#!/usr/bin/env python3
"""Decode a jvs_dump.bin capture written by the JVS Debugger core (Pocket B button).

File layout (32-bit big-endian words):
  word 0..1 : b"JVSDUMP2" (b"JVSDUMP1": 4-word header, words 4..7 below absent)
  word 2    : number of entries
  word 3    : timestamp unit in ns (1000 = microseconds)
  word 4    : flags: bit 0 = sniffer / ring mode, bit 1 = ring wrapped
  word 5    : index of the oldest entry (ring mode: entries are stored circularly)
  word 6..7 : reserved
  then 2 words per entry:
    timestamp (units since capture start)
    type[31:24] byte[23:16] lines[15:8] seq[7:0]
      type : 1 = TX byte (Pocket -> I/O board), 2 = RX byte (I/O board -> Pocket),
             3 = SETUP/LINK change, 4 = RS485 DIR change (byte = new DIR level)
      lines: bit0 = SNAC IN7 level (0 = SETUP LED lit, all devices addressed when 1)
             bit1 = SNAC IO3 level (0 = LINK LED lit)
             bit2 = RS485 driver enable (DIR)

Usage:
  jvs_dump.py jvs_dump.bin                 # frame by frame listing
  jvs_dump.py jvs_dump.bin --raw           # one line per captured entry
  jvs_dump.py sim_jvs_dump.hex             # capture written by the simulation (simu/ test_03)
  jvs_dump.py jvs_dump.bin --replay x.replay   # replay script for simu/ tb_capture_replay
  jvs_dump.py capture.jvs --replay x.replay [--prepend-resets] [--drop E00103210227]
                                           # same from a text capture (oscilloscope decode)
"""
import argparse
import os
import struct
import sys

TYPE_TX, TYPE_RX, TYPE_LINES, TYPE_DIR = 1, 2, 3, 4
BIT_US = 1e6 / 115200                # one bit at 115200 bauds (~8.68 us)
# A byte is time stamped when the UART decoder validates it, in the middle of its stop bit:
# its start bit began 9.5 bits earlier and its stop bit ends 0.5 bit later.
DV_AFTER_START_US = 9.5 * BIT_US
DV_BEFORE_END_US = 0.5 * BIT_US


DUMP_INFO = {"sniffer": False}       # header information of the last dump read


def read_dump(path):
    data = open(path, "rb").read()
    if path.endswith(".hex"):                 # simulation dump: one 32-bit hex word per line
        data = b"".join(struct.pack(">I", int(l, 16)) for l in data.decode().split() if l.strip())
    if len(data) < 16 or data[:7] != b"JVSDUMP" or data[7:8] not in (b"1", b"2"):
        sys.exit(f"{path}: not a JVS dump (bad magic)")
    count, unit_ns = struct.unpack(">II", data[8:16])
    hdr, flags, oldest = 16, 0, 0
    if data[7:8] == b"2":
        flags, oldest = struct.unpack(">II", data[16:24])
        hdr = 32
    DUMP_INFO["sniffer"] = bool(flags & 1)
    DUMP_INFO["wrapped"] = bool(flags & 2)
    entries = []
    for n in range(count):
        off = hdr + 8 * n
        if off + 8 > len(data):
            print(f"warning: file truncated after {n} of {count} entries", file=sys.stderr)
            break
        t, w = struct.unpack(">II", data[off:off + 8])
        entries.append({
            "t_us": t * unit_ns / 1000.0,
            "type": w >> 24,
            "byte": (w >> 16) & 0xFF,
            "lines": (w >> 8) & 0xFF,
            "seq": w & 0xFF,
        })
    if DUMP_INFO["wrapped"] and 0 < oldest < len(entries):
        entries = entries[oldest:] + entries[:oldest]   # ring buffer: oldest entry first
    return entries


def lines_str(lines):
    setup = "SETUP:on " if not (lines & 1) else "SETUP:off"
    link = "LINK:on " if not (lines & 2) else "LINK:off"
    return f"{setup} {link} DIR:{(lines >> 2) & 1}"


def unescape(raw):
    """JVS escape: D0 xx -> xx + 1 (D0 DF -> E0, D0 CF -> D0)."""
    out, esc = [], False
    for b in raw:
        if esc:
            out.append((b + 1) & 0xFF)
            esc = False
        elif b == 0xD0:
            esc = True
        else:
            out.append(b)
    return out


def frames(entries):
    """Group TX / RX bytes into JVS frames: E0 starts a frame, which is complete once
    node + length + `length` bytes have been received. Yields events in time order."""
    cur = {TYPE_TX: None, TYPE_RX: None}
    for e in entries:
        if e["type"] == TYPE_LINES:
            yield ("lines", e)
            continue
        if e["type"] == TYPE_DIR:
            yield ("dir", e)
            continue
        d = e["type"]
        if e["byte"] == 0xE0:                 # SYNC: start of a new frame
            if cur[d] is not None:            # previous one never completed
                yield ("frame", cur[d])
            cur[d] = {"dir": d, "t_us": e["t_us"], "t_end": e["t_us"], "raw": [0xE0]}
            continue
        if cur[d] is None:                    # noise / bytes outside of a frame
            yield ("stray", e)
            continue
        cur[d]["raw"].append(e["byte"])
        cur[d]["t_end"] = e["t_us"]
        body = unescape(cur[d]["raw"][1:])      # without the E0 SYNC
        if len(body) >= 2 and len(body) >= 2 + body[1]:
            yield ("frame", cur[d])
            cur[d] = None
    for d in (TYPE_TX, TYPE_RX):
        if cur[d] is not None:
            yield ("frame", cur[d])


def is_complete(frame):
    body = unescape(frame["raw"][1:])
    return len(body) >= 2 and len(body) >= 2 + body[1]


def describe(frame):
    body = unescape(frame["raw"][1:])      # node, len, payload..., checksum
    text = " ".join(f"{b:02X}" for b in body)
    note = ""
    if len(body) >= 2:
        node, length = body[0], body[1]
        payload = body[2:]
        if len(payload) < length:
            note = f"  (incomplete: {len(payload)}/{length})"
        else:
            payload = payload[:length]
            csum = (node + length + sum(payload[:-1])) & 0xFF
            note = "" if payload and csum == payload[-1] else f"  (BAD checksum, expected {csum:02X})"
    return f"E0 {text}{note}"


def is_request(frame):
    """Host request (destination != 00). In sniffer mode every frame comes on the receive
    line, so requests and answers are told apart by their destination node."""
    if not DUMP_INFO["sniffer"]:
        return frame["dir"] == TYPE_TX
    body = unescape(frame["raw"][1:])
    return len(body) > 0 and body[0] != 0x00


def bin_events(entries):
    """Replay events from a jvs_dump.bin capture: ("L", in7, io3) / ("T", raw) / ("R", gap_us, raw)."""
    ev, last_tx = [], None
    for kind, item in frames(entries):
        if kind == "lines":
            if not DUMP_INFO["sniffer"]:              # not connected when sniffing another host
                ev.append(("L", item["lines"] & 1, (item["lines"] >> 1) & 1))
        elif kind == "frame" and is_complete(item):
            if is_request(item):
                ev.append(("T", item["raw"]))
                last_tx = item
            else:
                gap = 100 if last_tx is None else max(50, int(item["t_us"] - last_tx["t_end"]))
                ev.append(("R", gap, item["raw"]))
    return ev


RESET_FRAME = [0xE0, 0xFF, 0x03, 0xF0, 0xD9, 0xCB]


def text_events(path, prepend_resets=False, drop=(), gap_us=300):
    """Replay events from a text capture (oscilloscope decode, one frame per line in hex,
    request then answer). Requests (destination != 00) become T, answers R. There is no
    SETUP/LINK information: SETUP is "not addressed" at start and "addressed" right after
    the last SETADDR request. `drop` lists request frames (hex) the controller does not
    send: they are skipped with their answer."""
    drop = {d.upper().replace(" ", "") for d in drop}
    lines = [l.strip().upper().replace(" ", "") for l in open(path) if l.strip()]
    ev = [("L", 0, 1)]
    if prepend_resets:
        ev += [("T", RESET_FRAME), ("T", RESET_FRAME)]
    skip_answer = False
    for l in lines:
        try:
            raw = list(bytes.fromhex(l))
        except ValueError:
            continue                                   # not a frame
        if len(raw) < 4 or raw[0] != 0xE0:
            continue
        body = unescape(raw[1:])
        if len(body) < 2 or len(body) < 2 + body[1]:
            continue                                   # truncated / garbled decode
        if (sum(body[:1 + body[1]]) & 0xFF) != body[1 + body[1]]:
            continue                                   # bad checksum (scope decode error)
        if body[0] != 0x00:                            # request
            if l in drop:
                skip_answer = True
                continue
            skip_answer = False
            ev.append(("T", raw))
            if body[0] == 0xFF and len(body) > 2 and body[2] == 0xF1:
                ev.append(("L", 1, 1))                 # SETADDR: device addressed
        else:                                          # answer
            if skip_answer:
                skip_answer = False
                continue
            ev.append(("R", gap_us, raw))
    return ev


def export_replay(events, path, src_name):
    """Write a replay script for the simulation (simu/replay_jvs_device.sv):
         L <IN7> <IO3>           SETUP / LINK line levels from now on
         T <bytes...>            frame the controller is expected to send (raw, as on the wire)
         R <delay_us> <bytes...> frame to send back <delay_us> after the end of the last T
         E <key> <values...>     expected decoded values (checked by the testbench),
                                 per node: N<addr>_<key>, plus NODES (addressed nodes)
    """
    out = [f"# JVS replay script generated from {src_name}"]
    last_tx = None
    expect = {}
    for e in events:
        if e[0] == "L":
            out.append(f"L {e[1]} {e[2]}")
        elif e[0] == "T":
            out.append("T " + " ".join(f"{b:02X}" for b in e[1]))
            last_tx = e[1]
            body = unescape(e[1][1:])
            if len(body) >= 4 and body[0] == 0xFF and body[2] == 0xF1:      # SETADDR <addr>
                expect["NODES"] = f"{max(int(expect.get('NODES', '0')), body[3])}"
        else:
            out.append(f"R {e[1]} " + " ".join(f"{b:02X}" for b in e[2]))
            if last_tx is not None:
                decode_answer(unescape(last_tx[1:]), unescape(e[2][1:]), expect)
    for key, vals in expect.items():
        out.append(f"E {key} {vals}")
    open(path, "w").write("\n".join(out) + "\n")
    return len(out), expect


def decode_answer(req, ans, expect):
    """Collect expected values from a request / answer pair (unescaped bodies)."""
    cmds = req[2:-1]                          # node, len, commands..., checksum
    data = ans[2:-1]                          # node, len, STATUS, reports/data..., checksum
    if not cmds or not data or data[0] != 0x01 or req[0] == 0xFF:
        return
    n = f"N{req[0]}_"                         # values are stored per node address
    p = 1                                     # after STATUS
    i = 0
    while i < len(cmds) and p < len(data):
        c = cmds[i]
        if data[p] != 0x01:                   # REPORT not normal: stop
            return
        p += 1
        if c == 0x10:                         # IO identity: name
            name = bytes(data[p:data.index(0, p)]) if 0 in data[p:] else bytes(data[p:])
            expect[n + "NAME"] = name.hex().upper()
            return
        if c in (0x11, 0x12, 0x13):
            expect[n + {0x11: "CMDREV", 0x12: "JVSREV", 0x13: "COMVER"}[c]] = f"{data[p]:02X}"
            return
        if c == 0x20:                         # switches: system byte + players x bytes
            players, nbytes = cmds[i + 1], cmds[i + 2]
            p += 1
            for pl in range(players):
                expect[f"{n}SW{pl}"] = " ".join(f"{b:02X}" for b in data[p:p + nbytes])
                p += nbytes
            i += 3
        elif c == 0x21:                       # coins: 2 bytes per slot
            for s in range(cmds[i + 1]):
                expect[f"{n}COIN{s}"] = f"{data[p]:02X}{data[p + 1]:02X}"
                p += 2
            i += 2
        elif c == 0x22:                       # analog: 2 bytes per channel
            for ch in range(cmds[i + 1]):
                expect[f"{n}ANALOG{ch}"] = f"{data[p]:02X}{data[p + 1]:02X}"
                p += 2
            i += 2
        elif c == 0x32:                       # OUTPUT1: REPORT only
            i += 2 + cmds[i + 1]
        else:
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--raw", action="store_true", help="list every captured entry")
    ap.add_argument("--replay", metavar="OUT", help="write a replay script for the simulation")
    ap.add_argument("--prepend-resets", action="store_true",
                    help="text capture: add the controller's 2 RESET frames at the start")
    ap.add_argument("--drop", action="append", default=[], metavar="FRAME",
                    help="text capture: skip this request frame (hex) and its answer")
    args = ap.parse_args()

    if args.dump.endswith(".jvs"):                    # text capture (oscilloscope decode)
        if not args.replay:
            sys.exit("text captures (.jvs) are only supported with --replay")
        ev = text_events(args.dump, args.prepend_resets, args.drop)
        n, expect = export_replay(ev, args.replay, os.path.basename(args.dump))
        print(f"{args.replay}: {n} lines, expected values: {', '.join(expect)}")
        return

    entries = read_dump(args.dump)
    if DUMP_INFO["sniffer"]:
        print("sniffer capture (whole bus seen on the receive line)"
              + (", ring buffer wrapped" if DUMP_INFO["wrapped"] else ""))
    if args.replay:
        n, expect = export_replay(bin_events(entries), args.replay, os.path.basename(args.dump))
        print(f"{args.replay}: {n} lines, expected values: {', '.join(expect)}")
        return
    print(f"{len(entries)} entries")
    if args.raw:
        for e in entries:
            kind = {TYPE_TX: "TX", TYPE_RX: "RX", TYPE_LINES: "LN", TYPE_DIR: "DR"}.get(e["type"], f"?{e['type']}")
            print(f"{e['t_us']:12.0f} us  {kind}  {e['byte']:02X}  {lines_str(e['lines'])}")
        return
    setup_us, hold_us = [], []        # DIR on -> first TX byte start, last TX byte end -> DIR off
    dir_on_t, last_tx_end = None, None
    for kind, item in frames(entries):
        if kind == "dir":
            on = item["byte"] & 1
            print(f"{item['t_us']:12.0f} us  --  DIR {'on  (driver enabled)' if on else 'off (driver released)'}")
            if on:
                dir_on_t = item["t_us"]
            elif last_tx_end is not None:
                hold_us.append(item["t_us"] - (last_tx_end + DV_BEFORE_END_US))
                last_tx_end = None
            continue
        if kind == "frame" and item["dir"] == TYPE_TX:
            if dir_on_t is not None:
                setup_us.append(item["t_us"] - DV_AFTER_START_US - dir_on_t)
                dir_on_t = None
            last_tx_end = item["t_end"]
        if kind == "lines":
            print(f"{item['t_us']:12.0f} us  --  {lines_str(item['lines'])}")
        elif kind == "stray":
            d = "TX" if item["type"] == TYPE_TX else "RX"
            print(f"{item['t_us']:12.0f} us  {d}  stray byte {item['byte']:02X}")
        else:
            if DUMP_INFO["sniffer"]:
                d = "HOST  ->" if is_request(item) else "BOARD <-"
            else:
                d = "TX ->" if item["dir"] == TYPE_TX else "RX <-"
            print(f"{item['t_us']:12.0f} us  {d} {describe(item)}")
    if setup_us or hold_us:
        def stats(v):
            return f"min {min(v):.0f} us, max {max(v):.0f} us, avg {sum(v)/len(v):.0f} us ({len(v)} frames)" if v else "n/a"
        print(f"RS485 DIR on -> first start bit : {stats(setup_us)}")
        print(f"RS485 last stop bit -> DIR off  : {stats(hold_us)}")
        print("  (+/- 1 bit: timestamps come from the UART decoder, 1 us resolution)")


if __name__ == "__main__":
    main()
