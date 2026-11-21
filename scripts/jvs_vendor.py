#!/usr/bin/env python3
"""Build the JVS vendor init table (jvs_vendor.bin) read by jvs_ctrl (vendor_rd_* ports).

Some I/O boards only work after host specific frames sent after the feature check, e.g. the
Namco NA-JV (Time Crisis 4) drives its outputs only after MAINID and the Namco command 70.
Nothing of this is hardcoded in jvs_ctrl: the table gives, per node, what to send.

Table layout (256 bytes):
  0x00..0x0F  header, 2 bytes per node n = 1..8:
                [2(n-1)]   MAINID pointer        (00 = NULL: no MAINID frame)
                [2(n-1)+1] vendor frames pointer (00 = NULL: no vendor frame)
  MAINID entry : 00 terminated string, sent as MAINID (15) <string> 00
  vendor entry : frames [L][L bytes: command + data] ... [00], one JVS frame each
  0xD8..0xDF  player analog axes (read by the core): P1 X, P1 Y, P2 X ... P4 Y, one byte each,
              bit 7 = defined, bits 6..4 = node - 1, bit 3 = inverted, bits 2..0 = channel.
              Drives the player joystick (pN_joy_state) and gun position:
              --p1-x 1:2 --p1-y 1:3 (node:CH), ":i" to invert the axis (--p1-x 1:0:i)
  0xE0..0xE7  reserved (0)
  0xE8..0xFF  digital output profile (read by the core, not by jvs_ctrl):
                0xE8, 0xEA, 0xEC, 0xEE: recoil trigger mask of P1, P2, P3, P4, 2 bytes each in
                           JVS order (SWINP player bytes 1, 2; 08 00 = switch LEFT): the
                           recoil pulse starts when any masked switch is pressed
                0xF0       flags, bit 0 = profile valid
                0xF1..0xF3 OUTPUT1 (32) base value, bytes 1..3
                0xF4, 0xF7, 0xFA, 0xFD: recoil mask of P1, P2, P3, P4 (3 bytes each):
                           OUTPUT1 bits set during the player gun recoil pulse
              no profile: OUTPUT1 = 00 00 00, no recoil

Default table (MAINID of the adapter sent to every node, no vendor frame):
  jvs_vendor.py --node all --mainid "TFURY.COM; JVS Adapter V1.0" -o jvs_vendor.bin

Example (Time Crisis 4, Namco NA-JV on node 1, recoil solenoid on output 1, lamps A0):
  jvs_vendor.py --node 1 --mainid "namco ltd.;S246;VerA;JPN" --frame "70 18 50 4C 14 40" \\
                --dout-base "00 A0 00" --dout-recoil-mask "80 00 00" -o jvs_vendor.bin
An empty table (no --node) disables everything (standard JVS init).
"""
import argparse
import sys

TABLE_SIZE = 256
HEADER_SIZE = 16
DATA_END = 0xD8           # 0xD8..0xDF: player axes, 0xE0..0xE7: reserved, 0xE8..: outputs
AXIS_BASE = 0xD8
TRIG_BASE = 0xE8
DOUT_BASE = 0xF0
TRIGGER_LEFT = bytes([0x08, 0x00])   # default recoil trigger: switch LEFT (Time Crisis 4 wiring)
MAX_NODES = 8
MAINID_MAX = 100          # jvs_ctrl bound, including the 00 terminator


def build(nodes, dout=None, axes=None):
    """nodes: {node: {"mainid": str or None, "frames": [bytes, ...]}},
    dout: (base 3 bytes, [recoil mask 3 bytes] x 4, [trigger mask 2 bytes] x 4) or None."""
    table = bytearray(TABLE_SIZE)
    ptr = HEADER_SIZE
    placed = {}               # identical entries are stored once, shared by the nodes

    def put(data):
        nonlocal ptr
        if data in placed:
            return placed[data]
        if ptr + len(data) > DATA_END:
            sys.exit(f"vendor table full ({DATA_END - HEADER_SIZE} data bytes)")
        start = ptr
        table[ptr:ptr + len(data)] = data
        ptr += len(data)
        placed[data] = start
        return start

    for node in sorted(nodes):
        cfg = nodes[node]
        if cfg["mainid"] is not None:
            s = cfg["mainid"].encode("ascii") + b"\x00"
            if len(s) > MAINID_MAX:
                sys.exit(f"node {node}: MAINID longer than {MAINID_MAX - 1} characters")
            table[2 * (node - 1)] = put(s)
        if cfg["frames"]:
            data = bytearray()
            for f in cfg["frames"]:
                if not 1 <= len(f) <= 255:
                    sys.exit(f"node {node}: frame length must be 1..255 bytes")
                data += bytes([len(f)]) + f
            table[2 * (node - 1) + 1] = put(bytes(data) + b"\x00")
    for a, v in (axes or {}).items():          # a = 2 * player + (0 X, 1 Y), v = (node, CH, inverted)
        table[AXIS_BASE + a] = 0x80 | ((v[0] - 1) << 4) | (0x08 if v[2] else 0) | v[1]
    if dout is not None:
        table[DOUT_BASE] = 0x01
        table[DOUT_BASE + 1:DOUT_BASE + 4] = dout[0]
        for p, mask in enumerate(dout[1]):
            table[DOUT_BASE + 4 + 3 * p:DOUT_BASE + 7 + 3 * p] = mask
        for p, trig in enumerate(dout[2]):
            table[TRIG_BASE + 2 * p:TRIG_BASE + 2 + 2 * p] = trig
    return bytes(table)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", action="append", default=[],
                    help='node address 1..8, or "all"; the following --mainid / --frame apply to it')
    ap.add_argument("--mainid", action="append", default=[], help="MAINID string of the last --node")
    ap.add_argument("--frame", action="append", default=[],
                    help='vendor frame of the last --node, hex bytes "70 18 50 4C 14 40" (repeatable)')
    ap.add_argument("--dout-base", help='OUTPUT1 base value, bytes 1..3, e.g. "00 A0 00"')
    ap.add_argument("--dout-recoil-mask", help='OUTPUT1 bits set during the P1 gun recoil pulse, e.g. "80 00 00"')
    for n in (2, 3, 4):
        ap.add_argument(f"--dout-recoil-mask-p{n}", help=f"same for P{n}")
    ap.add_argument("--recoil-trigger", help='P1 recoil trigger mask, JVS order (player bytes 1, 2), '
                    'default "08 00" (switch LEFT) when the player has a recoil mask')
    for n in (2, 3, 4):
        ap.add_argument(f"--recoil-trigger-p{n}", help=f"same for P{n}")
    for n in (1, 2, 3, 4):
        for axis in ("x", "y"):
            ap.add_argument(f"--p{n}-{axis}", help=f'P{n} {axis.upper()} axis: "node:CH[:i]", e.g. "1:2", '
                            f'"1:0:i" = inverted (node 1..8, CH 0..7)')
    ap.add_argument("-o", "--output", required=True, help="output file (.bin, or .hex with --hex)")
    ap.add_argument("--hex", action="store_true", help="one hex byte per line ($readmemh, simulation)")
    args = ap.parse_args()

    # Keep the command line order: each --mainid / --frame belongs to the --node before it
    nodes, cur = {}, None
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--node":
            v = argv[i + 1]
            cur = list(range(1, MAX_NODES + 1)) if v == "all" else [int(v)]
            if not all(1 <= n <= MAX_NODES for n in cur):
                sys.exit(f"--node must be 1..{MAX_NODES} or all")
            for n in cur:
                nodes.setdefault(n, {"mainid": None, "frames": []})
            i += 2
        elif a in ("--mainid", "--frame"):
            if cur is None:
                sys.exit(f"{a} needs a --node before it")
            for n in cur:
                if a == "--mainid":
                    nodes[n]["mainid"] = argv[i + 1]
                else:
                    nodes[n]["frames"].append(bytes(int(b, 16) for b in argv[i + 1].split()))
            i += 2
        else:
            i += 2 if a in ("-o", "--output", "--dout-base", "--dout-recoil-mask", "--dout-recoil-mask-p2",
                            "--dout-recoil-mask-p3", "--dout-recoil-mask-p4", "--recoil-trigger",
                            "--recoil-trigger-p2", "--recoil-trigger-p3", "--recoil-trigger-p4") \
                 or (a.startswith("--p") and a[-2:] in ("-x", "-y")) else 1

    dout = None
    masks = (args.dout_recoil_mask, args.dout_recoil_mask_p2, args.dout_recoil_mask_p3, args.dout_recoil_mask_p4)
    if args.dout_base or any(masks):
        def three(v):
            b = bytes(int(x, 16) for x in (v or "00 00 00").split())
            if len(b) != 3:
                sys.exit("--dout-base / --dout-recoil-mask*: 3 bytes (OUTPUT1 bytes 1..3)")
            return b
        def trigger(v, mask):
            if v is None:
                return TRIGGER_LEFT if any(mask) else bytes(2)
            b = bytes(int(x, 16) for x in v.split())
            if len(b) != 2:
                sys.exit("--recoil-trigger*: 2 bytes (player switch bytes 1, 2 in JVS order)")
            return b
        recoil = [three(m) for m in masks]
        trig_args = (args.recoil_trigger, args.recoil_trigger_p2, args.recoil_trigger_p3, args.recoil_trigger_p4)
        dout = (three(args.dout_base), recoil, [trigger(t, m) for t, m in zip(trig_args, recoil)])
    axes = {}
    for n in range(4):
        for k, axis in enumerate(("x", "y")):
            v = getattr(args, f"p{n + 1}_{axis}")
            if v:
                f = v.split(":")
                if len(f) not in (2, 3) or (len(f) == 3 and f[2] != "i"):
                    sys.exit(f'--p{n + 1}-{axis}: "node:CH" or "node:CH:i"')
                node, ch = int(f[0]), int(f[1])
                if not (1 <= node <= MAX_NODES and 0 <= ch <= 7):
                    sys.exit(f"--p{n + 1}-{axis}: node 1..{MAX_NODES}, CH 0..7")
                axes[2 * n + k] = (node, ch, len(f) == 3)
    if axes:
        print("player axes: " + ", ".join(f"P{a // 2 + 1} {'XY'[a % 2]} = node {v[0]} CH{v[1]}{' inverted' if v[2] else ''}"
                                          for a, v in sorted(axes.items())))
    table = build(nodes, dout, axes)
    if args.hex:
        open(args.output, "w").write("".join(f"{b:02X}\n" for b in table))
    else:
        open(args.output, "wb").write(table)
    used = [n for n in sorted(nodes)]
    print(f"{args.output}: {TABLE_SIZE} bytes, nodes {used if used else 'none'}, "
          f"digital output profile: {'base ' + dout[0].hex(' ') + ' / recoil (mask, trigger) ' + ', '.join(f'P{i + 1} ' + m.hex(' ') + ' / ' + t.hex(' ') for i, (m, t) in enumerate(zip(dout[1], dout[2]))) if dout else 'none'}")


if __name__ == "__main__":
    main()
