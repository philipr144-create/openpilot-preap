#!/usr/bin/env python3

import os
import sys
import time
import traceback

from panda import Panda

# ============================================================
# CONFIG
# ============================================================

DBC = "/data/openpilot/opendbc/dbc/tesla_preap.dbc"

WATCH = {
    0x068: "MCU_locationStatus2",
    0x218: "MCU_chassisControl",
    0x238: "UI_driverAssistRoadSign",
    0x2B8: "UI_radarMapData",
    0x2C8: "UI_roadCurvature",
    0x2D8: "UI_csaOfframpCurvature",
    0x2E8: "UI_csaRoadCurvature",
    0x338: "UI_status",
    0x3C8: "UI_driverAssistMapData",
    0x3D8: "MCU_locationStatus",
    0x428: "UI_telemetryControl",
}

RESET = "\033[0m"
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
MAGENTA = "\033[95m"
DIM = "\033[2m"
RED = "\033[91m"


# ============================================================
# LOAD OPENPILOT DBC PARSER
# ============================================================

def load_dbc():

    print()
    print("=" * 72)
    print("        TESLA PRE-AP CAN / NAV DECODER")
    print("=" * 72)
    print()

    print("DBC:")
    print(" ", DBC)

    if not os.path.exists(DBC):
        print(f"{RED}DBC NOT FOUND{RESET}")
        sys.exit(1)

    # openpilot normally uses opendbc.dbc
    try:
        from opendbc.can.dbc import DBC

        db = DBC(DBC)

        print(f"{GREEN}DBC loaded{RESET}")
        return db

    except Exception as e:
        print()
        print(f"{YELLOW}Openpilot DBC loader unavailable:{RESET}")
        print(repr(e))

    return None


# ============================================================
# FALLBACK RAW DBC READER
# ============================================================

def parse_dbc_messages():

    messages = {}

    try:
        with open(DBC, "r", errors="ignore") as f:

            for line in f:

                if not line.startswith("BO_ "):
                    continue

                parts = line.split()

                if len(parts) < 4:
                    continue

                try:
                    address = int(parts[1])
                except Exception:
                    continue

                name = parts[2].rstrip(":")

                try:
                    length = int(parts[3])
                except Exception:
                    length = 8

                messages[address] = {
                    "name": name,
                    "length": length,
                    "signals": [],
                }

            print(
                f"{GREEN}DBC message table: "
                f"{len(messages)} messages{RESET}"
            )

    except Exception as e:
        print(f"{RED}Could not read DBC: {e}{RESET}")

    return messages


# ============================================================
# RAW DBC SIGNAL PARSER
#
# This gives us useful decoding even without cantools.
# ============================================================

def parse_signals():

    messages = {}

    current = None

    try:
        with open(DBC, "r", errors="ignore") as f:

            for line in f:

                line = line.strip()

                if line.startswith("BO_ "):

                    parts = line.split()

                    if len(parts) >= 4:

                        try:
                            address = int(parts[1])
                        except Exception:
                            current = None
                            continue

                        name = parts[2].rstrip(":")

                        messages[address] = {
                            "name": name,
                            "signals": [],
                        }

                        current = address

                    continue

                if current is None:
                    continue

                if not line.startswith("SG_ "):
                    continue

                try:

                    # SG_ SignalName : start|length@endian sign
                    left, right = line.split(":", 1)

                    signal_name = left.split()[1]

                    fields = right.strip().split()

                    start_len = fields[0]

                    start = int(start_len.split("|")[0])

                    length = int(
                        start_len.split("|")[1]
                        .split("@")[0]
                    )

                    endian_sign = start_len.split("@")[1]

                    endian = int(endian_sign[0])

                    sign = endian_sign[1]

                    # scaling
                    factor = 1.0
                    offset = 0.0

                    if len(fields) > 1:

                        scale = fields[1]

                        if scale.startswith("("):

                            scale = scale.strip("()")

                            if "," in scale:

                                a, b = scale.split(",", 1)

                                factor = float(a)

                                offset = float(b)

                    messages[current]["signals"].append({
                        "name": signal_name,
                        "start": start,
                        "length": length,
                        "endian": endian,
                        "signed": sign == "-",
                        "factor": factor,
                        "offset": offset,
                    })

                except Exception:
                    # Ignore DBC constructs we don't understand.
                    pass

    except Exception as e:

        print(
            f"{RED}DBC signal parsing failed: "
            f"{repr(e)}{RESET}"
        )

    return messages


# ============================================================
# SIGNAL DECODER
# ============================================================

def decode_signal(data, signal):

    start = signal["start"]
    length = signal["length"]

    if length <= 0:
        return None

    value = int.from_bytes(
        data,
        byteorder="little",
        signed=False,
    )

    # Most of the Tesla DBC signals we care about are
    # straightforward Intel/little-endian fields.
    if signal["endian"] == 1:

        mask = (1 << length) - 1

        raw = (value >> start) & mask

    else:

        # Motorola fallback.
        raw = 0

        for i in range(length):

            bit = start - i

            if bit < 0:
                break

            byte = bit // 8

            bit_in_byte = bit % 8

            if byte >= len(data):
                break

            raw <<= 1

            raw |= (
                data[byte] >> bit_in_byte
            ) & 1

    if signal["signed"]:

        sign_bit = 1 << (length - 1)

        if raw & sign_bit:
            raw -= 1 << length

    return (
        raw * signal["factor"]
        + signal["offset"]
    )


# ============================================================
# PRINT FRAME
# ============================================================

def print_frame(address, data, dbc):

    name = WATCH.get(
        address,
        dbc.get(address, {}).get(
            "name",
            "UNKNOWN"
        )
    )

    print()

    print(
        f"{CYAN}0x{address:03X}{RESET} "
        f"{GREEN}{name}{RESET}"
    )

    print(
        f"  {DIM}RAW: "
        f"{data.hex(' ').upper()}{RESET}"
    )

    msg = dbc.get(address)

    if not msg:
        return

    signals = msg.get("signals", [])

    if not signals:
        return

    for signal in signals:

        try:

            value = decode_signal(
                data,
                signal
            )

            if value is None:
                continue

            # Keep the display sane.
            if isinstance(value, float):

                if abs(value) < 0.0001:
                    value = 0.0

                text = f"{value:.4f}"

            else:
                text = str(value)

            print(
                f"    {MAGENTA}"
                f"{signal['name']:<35}"
                f"{RESET} {text}"
            )

        except Exception:
            pass


# ============================================================
# MAIN
# ============================================================

def main():

    dbc = parse_signals()

    print()
    print("Watching:")
    print()

    for address, name in WATCH.items():

        signal_count = len(
            dbc.get(address, {})
               .get("signals", [])
        )

        print(
            f"  0x{address:03X} "
            f"{name:<38}"
            f"{signal_count:>3} signals"
        )

    print()
    print("=" * 72)
    print("Opening Panda...")
    print("=" * 72)

    try:

        panda = Panda()

    except Exception as e:

        print()
        print(f"{RED}PANDA OPEN FAILED{RESET}")
        print(repr(e))
        return

    print(
        f"{GREEN}Panda connected{RESET}"
    )

    # --------------------------------------------------------
    # Receive counters
    # --------------------------------------------------------

    counts = {
        address: 0
        for address in WATCH
    }

    last_values = {}

    print()
    print(
        f"{YELLOW}"
        "Listening for Tesla CAN traffic..."
        f"{RESET}"
    )

    print(
        f"{DIM}"
        "Ctrl-C to stop."
        f"{RESET}"
    )

    print()

    try:

        while True:

            try:

                packets = panda.can_recv()

                if not packets:
                    time.sleep(0.001)
                    continue

                for packet in packets:

                    # Panda returns:
                    #
                    # address, dat, src
                    #
                    # depending on Panda version it may
                    # contain additional fields.

                    if len(packet) < 2:
                        continue

                    address = packet[0]

                    data = packet[1]

                    if not isinstance(address, int):
                        continue

                    if not isinstance(data, (bytes, bytearray)):
                        continue

                    address &= 0x1FFFFFFF

                    if address not in WATCH:
                        continue

                    counts[address] += 1

                    # ------------------------------------------------
                    # Only print when payload changes.
                    # This prevents the terminal from becoming
                    # completely unusable.
                    # ------------------------------------------------

                    data_key = bytes(data)

                    if (
                        last_values.get(address)
                        == data_key
                    ):
                        continue

                    last_values[address] = data_key

                    print_frame(
                        address,
                        data_key,
                        dbc
                    )

            except Exception as e:

                print(
                    f"{RED}"
                    f"CAN packet error: "
                    f"{repr(e)}"
                    f"{RESET}"
                )

                time.sleep(0.01)

    except KeyboardInterrupt:

        print()
        print()
        print("=" * 72)
        print("MONITOR STOPPED")
        print("=" * 72)

        print()
        print("Frames received:")

        for address, name in WATCH.items():

            print(
                f"  0x{address:03X} "
                f"{name:<38}"
                f"{counts[address]}"
            )

        print()


if __name__ == "__main__":
    main()
