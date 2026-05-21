#!/usr/bin/env python3

import argparse
import statistics
import sys
import time
from pymodbus.client import ModbusTcpClient


GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"


# ============================================================
# Keyence PLC Model Definitions
# ============================================================
#
# Each entry describes the Modbus address space and default
# device-name mapping for a specific Keyence KV-series model.
#
# Modbus area -> Keyence device mapping (default KV STUDIO config):
#
#   Coils (FC01/FC05)            -> Internal Relay (R)
#   Discrete Inputs (FC02)       -> External Input Relay (X / MR depending on model)
#   Holding Registers (FC03/FC06)-> Data Memory (DM)
#   Input Registers (FC04)       -> Analog/High-speed counter input (AT / CTH)
#
# NOTE: The exact mapping MUST be verified in KV STUDIO
#       Unit Editor -> Modbus TCP settings for your project.
#       These are common defaults, not guaranteed values.
#
# Fields:
#   label      : Human-readable model name shown in CLI help
#   relay_dev  : Device prefix for coil (bit) addresses  (e.g. "R")
#   input_dev  : Device prefix for discrete input addresses (e.g. "MR")
#   dm_dev     : Device prefix for holding register addresses (e.g. "DM")
#   ir_dev     : Device prefix for input register addresses (e.g. "AT")
#   max_relay  : Maximum usable relay address (for range warnings)
#   max_dm     : Maximum usable DM address (for range warnings)
#   notes      : Model-specific caveats

KEYENCE_MODELS = {
    # ── Flagship / large-scale controllers ─────────────────────────────────
    "keyence-kv8000": {
        "label":      "KV-8000 (flagship, built-in EtherNet/IP + Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  99999,
        "max_dm":     99999,
        "notes": (
            "KV-8000 supports up to 32 CPU units in a multiprocessor config. "
            "Modbus TCP server is built-in (no additional option board required). "
            "R0–R99999 mapped to coils by default; DM0–DM99999 to holding registers."
        ),
    },
    "keyence-kv7500": {
        "label":      "KV-7500 (high-speed motion + Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  99999,
        "max_dm":     99999,
        "notes": (
            "KV-7500 includes built-in Modbus TCP server. "
            "Supports synchronized motion control (up to 64 axes via EtherCAT). "
            "R and DM address spaces same as KV-8000."
        ),
    },
    "keyence-kv7300": {
        "label":      "KV-7300 (high-speed, compact flagship)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  99999,
        "max_dm":     99999,
        "notes": (
            "KV-7300 is the compact version of the KV-7500 without motion expansion. "
            "Built-in Modbus TCP. Address space mirrors KV-7500."
        ),
    },

    # ── Mid-range controllers ───────────────────────────────────────────────
    "keyence-kv5500": {
        "label":      "KV-5500 (mid-range, built-in Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  59999,
        "max_dm":     59999,
        "notes": (
            "KV-5500 has built-in Ethernet with Modbus TCP server. "
            "R0–R59999 for coils; DM0–DM59999 for holding registers. "
            "Supports up to 4,096 I/O points."
        ),
    },
    "keyence-kv5000": {
        "label":      "KV-5000 (mid-range, requires KV-EP21V option for Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  59999,
        "max_dm":     59999,
        "notes": (
            "KV-5000 requires the KV-EP21V Ethernet option unit for Modbus TCP. "
            "Address space same as KV-5500 once the option is installed."
        ),
    },
    "keyence-kv3000": {
        "label":      "KV-3000 (mid-range, requires KV-EP21V option for Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  39999,
        "max_dm":     32767,
        "notes": (
            "KV-3000 requires the KV-EP21V Ethernet option unit for Modbus TCP. "
            "R0–R39999 for coils; DM0–DM32767 for holding registers."
        ),
    },

    # ── Compact/entry-level controllers ────────────────────────────────────
    "keyence-kv1000": {
        "label":      "KV-1000 (compact, requires KV-EP21V option for Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  19999,
        "max_dm":     19999,
        "notes": (
            "KV-1000 requires the KV-EP21V for Modbus TCP. "
            "R0–R19999 for coils; DM0–DM19999 for holding registers. "
            "Up to 512 I/O expansion points."
        ),
    },
    "keyence-kv700": {
        "label":      "KV-700 (compact, requires KV-EP21V option for Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  9999,
        "max_dm":     9999,
        "notes": (
            "KV-700 requires the KV-EP21V for Modbus TCP. "
            "Reduced address space: R0–R9999 / DM0–DM9999."
        ),
    },
    "keyence-kvp16": {
        "label":      "KV-P16 (compact, 16-point base unit, requires KV-EP21V)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  9999,
        "max_dm":     4095,
        "notes": (
            "KV-P16 is a minimal base unit with 16 built-in I/O. "
            "Requires KV-EP21V for Modbus TCP. "
            "DM space limited to DM0–DM4095."
        ),
    },

    # ── KV Nano series ──────────────────────────────────────────────────────
    "keyence-kv-n14": {
        "label":      "KV Nano KV-N14 (14-point, NPN/PNP, Modbus TCP via KV-EP21V or built-in depending on revision)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  3999,
        "max_dm":     3999,
        "notes": (
            "KV-N14 is the smallest Nano unit (8 inputs / 6 outputs). "
            "R0–R3999; DM0–DM3999. "
            "Modbus TCP requires KV-EP21V or the NP series option depending on firmware version."
        ),
    },
    "keyence-kv-n24": {
        "label":      "KV Nano KV-N24 (24-point, expandable)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  3999,
        "max_dm":     3999,
        "notes": (
            "KV-N24 has 14 inputs / 10 outputs, expandable via KV-E series. "
            "Same address space as KV-N14."
        ),
    },
    "keyence-kv-c16": {
        "label":      "KV Nano KV-C16 (16-point, transistor output)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  3999,
        "max_dm":     3999,
        "notes": (
            "KV-C16 is a Nano unit with 8 inputs / 8 transistor outputs. "
            "Address space same as KV-N14/N24."
        ),
    },
    "keyence-kv-c24": {
        "label":      "KV Nano KV-C24 (24-point, transistor output, expandable)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  3999,
        "max_dm":     3999,
        "notes": (
            "KV-C24 has 14 inputs / 10 transistor outputs. "
            "Expandable with KV-E units. Address space same as KV-N series."
        ),
    },

    # ── KV-NP series (newer Nano with built-in Ethernet) ───────────────────
    "keyence-kv-np20": {
        "label":      "KV Nano KV-NP20 (20-point, built-in Ethernet + Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  3999,
        "max_dm":     3999,
        "notes": (
            "KV-NP20 is the Nano series with built-in Ethernet (no option board needed). "
            "12 inputs / 8 outputs. Modbus TCP server supported natively."
        ),
    },

    # ── KV-XH / High-performance series ────────────────────────────────────
    "keyence-kv-xh16ml": {
        "label":      "KV-XH16ML (high-speed motion, 16-axis, built-in Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  99999,
        "max_dm":     99999,
        "notes": (
            "KV-XH16ML is a high-performance motion controller supporting up to 16 EtherCAT axes. "
            "Built-in Modbus TCP. Address space same as KV-8000/7500."
        ),
    },
    "keyence-kv-xh04ml": {
        "label":      "KV-XH04ML (4-axis motion variant, built-in Modbus TCP)",
        "relay_dev":  "R",
        "input_dev":  "MR",
        "dm_dev":     "DM",
        "ir_dev":     "AT",
        "max_relay":  99999,
        "max_dm":     99999,
        "notes": (
            "KV-XH04ML is the 4-axis compact version of the XH series. "
            "Built-in Modbus TCP. Full KV-8000 address space."
        ),
    },
}

# All profile choices: openplc, generic, + every Keyence model key
ALL_PROFILES = ["generic", "openplc"] + sorted(KEYENCE_MODELS.keys())


# ============================================================
# Colors / Formatting
# ============================================================

def color_value(value):
    try:
        numeric_value = int(value)
    except (ValueError, TypeError):
        return str(value)

    if numeric_value > 0:
        return f"{GREEN}{numeric_value}{RESET}"

    return str(numeric_value)


def color_bool(value):
    numeric = 1 if value else 0
    return color_value(numeric)


# ============================================================
# Argument parsing helpers
# ============================================================

def parse_bool(value):
    value = str(value).lower().strip()

    if value in ["1", "true", "on", "yes"]:
        return True

    if value in ["0", "false", "off", "no"]:
        return False

    raise argparse.ArgumentTypeError("Use true/false, 1/0, on/off")


def parse_csv_ints(value):
    try:
        return [int(x.strip(), 0) for x in value.split(",") if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError("Use comma-separated integers, example: 10,20,30")


def parse_csv_bools(value):
    return [parse_bool(x.strip()) for x in value.split(",") if x.strip()]


# ============================================================
# Modbus compatibility helpers
# ============================================================

def connect_client(host, port, timeout):
    client = ModbusTcpClient(host=host, port=port, timeout=timeout)

    if not client.connect():
        print(f"{RED}[!] Cannot connect to {host}:{port}{RESET}")
        sys.exit(1)

    return client


def modbus_call(func, unit, **kwargs):
    """
    Compatible with different pymodbus versions.
    Some versions use:
      device_id
      slave
      unit
    """

    for unit_key in ["device_id", "slave", "unit"]:
        try:
            return func(**kwargs, **{unit_key: unit})
        except TypeError:
            continue

    try:
        return func(**kwargs)
    except TypeError:
        address = kwargs.get("address")
        count = kwargs.get("count")
        value = kwargs.get("value")
        values = kwargs.get("values")

        for unit_key in ["device_id", "slave", "unit"]:
            try:
                if count is not None:
                    return func(address, count, **{unit_key: unit})
                if value is not None:
                    return func(address, value, **{unit_key: unit})
                if values is not None:
                    return func(address, values, **{unit_key: unit})
            except TypeError:
                continue

        if count is not None:
            return func(address, count)

        if value is not None:
            return func(address, value)

        if values is not None:
            return func(address, values)

        raise


def is_error(result):
    return result is None or result.isError()


# ============================================================
# Address hints
# ============================================================

def openplc_qx_hint(address):
    byte = address // 8
    bit = address % 8
    return f"%QX{byte}.{bit}"


def openplc_ix_hint(address):
    byte = address // 8
    bit = address % 8
    return f"%IX{byte}.{bit}"


def openplc_hr_hint(address):
    if address >= 1024:
        return f"%MW{address - 1024}"
    return f"%QW{address}"


def keyence_model_hint(profile, area, address):
    """
    Build a device-name hint for a specific Keyence model profile.
    Falls back to generic if the profile is not in KEYENCE_MODELS.
    """
    model = KEYENCE_MODELS.get(profile)
    if model is None:
        return generic_hint(area, address)

    max_relay = model["max_relay"]
    max_dm    = model["max_dm"]

    if area == "coils":
        dev = model["relay_dev"]
        if address > max_relay:
            return f"{dev}{address} {YELLOW}(exceeds {dev} max {max_relay}){RESET}"
        return f"{dev}{address}"

    if area == "di":
        dev = model["input_dev"]
        return f"{dev}{address}"

    if area == "hr":
        dev = model["dm_dev"]
        if address > max_dm:
            return f"{dev}{address} {YELLOW}(exceeds {dev} max {max_dm}){RESET}"
        return f"{dev}{address}"

    if area == "ir":
        dev = model["ir_dev"]
        return f"{dev}{address}"

    return generic_hint(area, address)


def generic_hint(area, address):
    if area == "coils":
        return f"Coil {address}"
    if area == "di":
        return f"DI {address}"
    if area == "hr":
        return f"HR {address}"
    if area == "ir":
        return f"IR {address}"
    return str(address)


def get_hint(profile, area, address):
    if profile == "openplc":
        if area == "coils":
            return openplc_qx_hint(address)
        if area == "di":
            return openplc_ix_hint(address)
        if area == "hr":
            return openplc_hr_hint(address)
        if area == "ir":
            return f"IR{address}"

    if profile in KEYENCE_MODELS:
        return keyence_model_hint(profile, area, address)

    return generic_hint(area, address)


def area_title(area):
    if area == "coils":
        return "COILS / FC01"
    if area == "di":
        return "DISCRETE INPUTS / FC02"
    if area == "hr":
        return "HOLDING REGISTERS / FC03"
    if area == "ir":
        return "INPUT REGISTERS / FC04"
    return area


def print_model_info(profile):
    """Print model metadata when a Keyence profile is selected."""
    model = KEYENCE_MODELS.get(profile)
    if model is None:
        return
    print(f"{BLUE}[i] Model   : {model['label']}{RESET}")
    print(f"{BLUE}[i] Coils   : {model['relay_dev']}0–{model['relay_dev']}{model['max_relay']} (internal relays){RESET}")
    print(f"{BLUE}[i] DI      : {model['input_dev']}0 (input relays, check KV STUDIO mapping){RESET}")
    print(f"{BLUE}[i] HR      : {model['dm_dev']}0–{model['dm_dev']}{model['max_dm']} (data memory){RESET}")
    print(f"{BLUE}[i] IR      : {model['ir_dev']}0 (analog/high-speed inputs, check KV STUDIO mapping){RESET}")
    print(f"{YELLOW}[i] Note    : {model['notes']}{RESET}")


# ============================================================
# Printing functions
# ============================================================

def print_bits(result, start_address, count, profile, area):
    if is_error(result):
        print(f"{RED}[!] Modbus error: {result}{RESET}")
        return

    bits = result.bits[:count]

    for i, bit in enumerate(bits):
        address = start_address + i
        hint = get_hint(profile, area, address)
        value = color_bool(bit)

        print(f"address {address:<6} hint={hint:<40} value={value}")


def print_registers(result, start_address, profile, area):
    if is_error(result):
        print(f"{RED}[!] Modbus error: {result}{RESET}")
        return

    for i, value in enumerate(result.registers):
        address = start_address + i
        hint = get_hint(profile, area, address)
        value_colored = color_value(value)

        print(f"address {address:<6} hint={hint:<40} value={value_colored}")


# ============================================================
# Actions
# ============================================================

def check_connection(args):
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{GREEN}[+] TCP connection successful: {args.host}:{args.port}{RESET}")
    print(f"{BLUE}[i] Profile: {args.profile}{RESET}")
    print_model_info(args.profile)

    result = modbus_call(
        client.read_holding_registers,
        args.unit,
        address=args.address,
        count=1
    )

    if is_error(result):
        print(f"{YELLOW}[!] TCP is open, but Modbus read failed with unit {args.unit}{RESET}")
        print(f"{BLUE}[i] Try: discover-unit or use another test address with --address{RESET}")
    else:
        print(f"{GREEN}[+] Modbus response OK with unit {args.unit}{RESET}")
        print_registers(result, args.address, args.profile, "hr")

    client.close()


def discover_unit(args):
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{BLUE}[*] Discovering Unit ID from {args.start_unit} to {args.end_unit}{RESET}")
    print(f"{BLUE}[i] Test address: Holding Register {args.address}{RESET}")
    print_model_info(args.profile)

    found = []

    for unit in range(args.start_unit, args.end_unit + 1):
        result = modbus_call(
            client.read_holding_registers,
            unit,
            address=args.address,
            count=1
        )

        if not is_error(result):
            value = result.registers[0]
            print(f"{GREEN}[+] Unit {unit} responded | HR {args.address} = {value}{RESET}")
            found.append(unit)
        elif args.verbose:
            print(f"[-] Unit {unit} no valid response")

    if not found:
        print(f"{RED}[!] No Unit ID found{RESET}")
    else:
        print(f"{GREEN}[+] Found units: {found}{RESET}")

        if len(found) > 1:
            print(f"{YELLOW}[!] Multiple Unit IDs responded.{RESET}")
            print(f"{YELLOW}[i] This often means the Modbus TCP server ignores Unit ID.{RESET}")
            print(f"{YELLOW}[i] Use --unit 1 unless your Keyence/PLC documentation says otherwise.{RESET}")

    client.close()


def read_coils(args):
    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.read_coils,
        args.unit,
        address=args.address,
        count=args.count
    )

    print_bits(result, args.address, args.count, args.profile, "coils")
    client.close()


def read_discrete_inputs(args):
    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.read_discrete_inputs,
        args.unit,
        address=args.address,
        count=args.count
    )

    print_bits(result, args.address, args.count, args.profile, "di")
    client.close()


def read_holding_registers(args):
    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.read_holding_registers,
        args.unit,
        address=args.address,
        count=args.count
    )

    print_registers(result, args.address, args.profile, "hr")
    client.close()


def read_input_registers(args):
    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.read_input_registers,
        args.unit,
        address=args.address,
        count=args.count
    )

    print_registers(result, args.address, args.profile, "ir")
    client.close()


def write_coil(args):
    if not args.allow_write:
        print(f"{RED}[!] Write blocked.{RESET}")
        print(f"{YELLOW}[i] Add --allow-write if this is your authorized lab/PLC and you know the address.{RESET}")
        return

    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.write_coil,
        args.unit,
        address=args.address,
        value=args.value
    )

    if is_error(result):
        print(f"{RED}[!] Write coil failed: {result}{RESET}")
    else:
        hint = get_hint(args.profile, "coils", args.address)
        numeric_value = 1 if args.value else 0

        print(
            f"{GREEN}[+] Coil written successfully | "
            f"address={args.address} hint={hint} value={numeric_value}{RESET}"
        )

    client.close()


def write_coils(args):
    if not args.allow_write:
        print(f"{RED}[!] Write blocked.{RESET}")
        print(f"{YELLOW}[i] Add --allow-write if this is your authorized lab/PLC and you know the addresses.{RESET}")
        return

    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.write_coils,
        args.unit,
        address=args.address,
        values=args.values
    )

    if is_error(result):
        print(f"{RED}[!] Write coils failed: {result}{RESET}")
    else:
        print(f"{GREEN}[+] Coils written successfully | start_address={args.address} values={args.values}{RESET}")

    client.close()


def write_register(args):
    if not args.allow_write:
        print(f"{RED}[!] Write blocked.{RESET}")
        print(f"{YELLOW}[i] Add --allow-write if this is your authorized lab/PLC and you know the address.{RESET}")
        return

    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.write_register,
        args.unit,
        address=args.address,
        value=args.value
    )

    if is_error(result):
        print(f"{RED}[!] Write register failed: {result}{RESET}")
    else:
        hint = get_hint(args.profile, "hr", args.address)

        print(
            f"{GREEN}[+] Register written successfully | "
            f"address={args.address} hint={hint} value={args.value}{RESET}"
        )

    client.close()


def write_registers(args):
    if not args.allow_write:
        print(f"{RED}[!] Write blocked.{RESET}")
        print(f"{YELLOW}[i] Add --allow-write if this is your authorized lab/PLC and you know the addresses.{RESET}")
        return

    client = connect_client(args.host, args.port, args.timeout)

    result = modbus_call(
        client.write_registers,
        args.unit,
        address=args.address,
        values=args.values
    )

    if is_error(result):
        print(f"{RED}[!] Write registers failed: {result}{RESET}")
    else:
        print(f"{GREEN}[+] Registers written successfully | start_address={args.address} values={args.values}{RESET}")

    client.close()


# ============================================================
# Turn ON / OFF / Toggle helpers
# ============================================================

def parse_csv_addresses(value):
    """Accept comma-separated integers: '0,1,5,10'"""
    try:
        return [int(x.strip(), 0) for x in value.split(",") if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError("Use comma-separated integers, e.g. 0,1,5")


def resolve_addresses(args):
    """
    Build a flat list of coil addresses from the three ways a user can
    specify them:
      --addresses 0,3,7        explicit list
      --address 0 --count 8   contiguous block (legacy write-coil style)
      --from 0 --to 7         inclusive range shorthand
    """
    if hasattr(args, "addresses") and args.addresses:
        return args.addresses

    if hasattr(args, "addr_from") and args.addr_from is not None:
        return list(range(args.addr_from, args.addr_to + 1))

    # fall back to single --address
    count = getattr(args, "count", 1)
    return list(range(args.address, args.address + count))


def _write_guard(args):
    if not args.allow_write:
        print(f"{RED}[!] Write blocked.{RESET}")
        print(f"{YELLOW}[i] Add --allow-write to confirm this is your authorized PLC.{RESET}")
        return False
    return True


def _coil_state_line(address, value, profile, ok):
    hint  = get_hint(profile, "coils", address)
    state = f"{GREEN}ON {RESET}" if value else f"{RED}OFF{RESET}"
    mark  = f"{GREEN}[+]{RESET}" if ok else f"{RED}[!]{RESET}"
    return f"{mark} address {address:<6} hint={hint:<40} → {state}"


def turn_on(args):
    if not _write_guard(args):
        return

    addresses = resolve_addresses(args)
    client    = connect_client(args.host, args.port, args.timeout)

    print(f"{BLUE}[*] Turning ON {len(addresses)} coil(s)...{RESET}")

    ok_count = err_count = 0
    for addr in addresses:
        result = modbus_call(client.write_coil, args.unit, address=addr, value=True)
        ok     = not is_error(result)
        ok_count  += int(ok)
        err_count += int(not ok)
        print(_coil_state_line(addr, True, args.profile, ok))

    _print_summary(ok_count, err_count)
    client.close()


def turn_off(args):
    if not _write_guard(args):
        return

    addresses = resolve_addresses(args)
    client    = connect_client(args.host, args.port, args.timeout)

    print(f"{BLUE}[*] Turning OFF {len(addresses)} coil(s)...{RESET}")

    ok_count = err_count = 0
    for addr in addresses:
        result = modbus_call(client.write_coil, args.unit, address=addr, value=False)
        ok     = not is_error(result)
        ok_count  += int(ok)
        err_count += int(not ok)
        print(_coil_state_line(addr, False, args.profile, ok))

    _print_summary(ok_count, err_count)
    client.close()


def toggle(args):
    """Read current coil state then flip it."""
    if not _write_guard(args):
        return

    addresses = resolve_addresses(args)
    client    = connect_client(args.host, args.port, args.timeout)

    print(f"{BLUE}[*] Toggling {len(addresses)} coil(s)...{RESET}")

    ok_count = err_count = 0
    for addr in addresses:
        # Read current state
        r = modbus_call(client.read_coils, args.unit, address=addr, count=1)
        if is_error(r):
            print(f"{RED}[!] address {addr:<6} could not read current state — skipped{RESET}")
            err_count += 1
            continue

        current  = r.bits[0]
        new_val  = not current
        result   = modbus_call(client.write_coil, args.unit, address=addr, value=new_val)
        ok       = not is_error(result)
        ok_count  += int(ok)
        err_count += int(not ok)

        hint     = get_hint(args.profile, "coils", addr)
        old_str  = f"{GREEN}ON{RESET}"  if current else f"{RED}OFF{RESET}"
        new_str  = f"{GREEN}ON{RESET}"  if new_val else f"{RED}OFF{RESET}"
        mark     = f"{GREEN}[+]{RESET}" if ok      else f"{RED}[!]{RESET}"
        print(f"{mark} address {addr:<6} hint={hint:<40} {old_str} → {new_str}")

    _print_summary(ok_count, err_count)
    client.close()


def _print_summary(ok, err):
    total = ok + err
    if err == 0:
        print(f"{GREEN}[+] Done — {ok}/{total} coil(s) written successfully.{RESET}")
    else:
        print(f"{YELLOW}[!] Done — {ok}/{total} OK, {err}/{total} failed.{RESET}")


def add_plc_switch_arguments(parser):
    """
    Three mutually exclusive ways to specify target coil addresses.
    Only one group is required; if none is given argparse will still
    accept --address (legacy single-coil) via the fallback in resolve_addresses.
    """
    grp = parser.add_mutually_exclusive_group(required=True)
    grp.add_argument(
        "--address", type=int, metavar="ADDR",
        help="Single coil address."
    )
    grp.add_argument(
        "--addresses", type=parse_csv_addresses, metavar="A,B,C",
        help="Comma-separated list of coil addresses, e.g. 0,3,7"
    )
    grp.add_argument(
        "--from", type=int, dest="addr_from", metavar="START",
        help="Start of an inclusive address range (use with --to)."
    )
    parser.add_argument(
        "--to", type=int, dest="addr_to", metavar="END",
        help="End of address range (required with --from)."
    )
    parser.add_argument(
        "--count", type=int, default=1,
        help="Number of consecutive coils when using --address (default: 1)."
    )


def validate_plc_switch_args(args):
    """Extra cross-field validation argparse cannot express."""
    if args.addr_from is not None and args.addr_to is None:
        raise SystemExit(f"{RED}[!] --from requires --to{RESET}")
    if args.addr_from is not None and args.addr_to < args.addr_from:
        raise SystemExit(f"{RED}[!] --to must be >= --from{RESET}")


def read_area(client, area, unit, address, count):
    if area == "coils":
        return modbus_call(client.read_coils, unit, address=address, count=count)

    if area == "di":
        return modbus_call(client.read_discrete_inputs, unit, address=address, count=count)

    if area == "hr":
        return modbus_call(client.read_holding_registers, unit, address=address, count=count)

    if area == "ir":
        return modbus_call(client.read_input_registers, unit, address=address, count=count)

    raise ValueError("Unknown area")


def discover_area(client, args, area):
    print()
    print(f"{BLUE}[*] Discovering {area_title(area)} from {args.start} to {args.end}{RESET}")

    current = args.start
    displayed = 0
    valid_blocks = 0
    error_blocks = 0

    while current <= args.end:
        count = min(args.block_size, args.end - current + 1)
        result = read_area(client, area, args.unit, current, count)

        if is_error(result):
            error_blocks += 1
            current += count
            continue

        valid_blocks += 1

        if area in ["coils", "di"]:
            values = result.bits[:count]

            for i, value in enumerate(values):
                address = current + i
                numeric_value = 1 if value else 0

                if args.show_all or numeric_value > 0:
                    hint = get_hint(args.profile, area, address)
                    print(f"address {address:<6} hint={hint:<40} value={color_value(numeric_value)}")
                    displayed += 1

        else:
            values = result.registers

            for i, value in enumerate(values):
                address = current + i

                if args.show_all or value > 0:
                    hint = get_hint(args.profile, area, address)
                    print(f"address {address:<6} hint={hint:<40} value={color_value(value)}")
                    displayed += 1

        current += count

        if args.delay > 0:
            time.sleep(args.delay)

    print(f"{BLUE}[i] Valid blocks: {valid_blocks}, error blocks: {error_blocks}, displayed values: {displayed}{RESET}")


def discover_all(args):
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{GREEN}[+] Connected to {args.host}:{args.port}{RESET}")
    print(f"{BLUE}[i] Profile: {args.profile}{RESET}")
    print(f"{BLUE}[i] Discovery is read-only.{RESET}")
    print_model_info(args.profile)

    if args.profile in KEYENCE_MODELS:
        print(f"{YELLOW}[i] Keyence note: exact device mapping depends on KV STUDIO Modbus settings.{RESET}")
        print(f"{YELLOW}[i] Hints below show the default device names; verify in KV STUDIO Unit Editor.{RESET}")

    for area in ["coils", "di", "hr", "ir"]:
        discover_area(client, args, area)

    client.close()


def discover_one_area(args, area):
    client = connect_client(args.host, args.port, args.timeout)
    discover_area(client, args, area)
    client.close()


def monitor(args):
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{BLUE}[*] Monitoring started. Press CTRL+C to stop.{RESET}")

    try:
        while True:
            print()
            print(time.strftime(f"{BOLD}[%H:%M:%S]{RESET}"))

            result = read_area(client, args.area, args.unit, args.address, args.count)

            if args.area in ["coils", "di"]:
                print_bits(result, args.address, args.count, args.profile, args.area)
            else:
                print_registers(result, args.address, args.profile, args.area)

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print(f"\n{YELLOW}[*] Monitoring stopped{RESET}")

    client.close()


# ============================================================
# Benchmark — latency / throughput measurement (read-only)
# ============================================================

MODBUS_EXCEPTION_NAMES = {
    1: "Illegal Function",
    2: "Illegal Data Address",
    3: "Illegal Data Value",
    4: "Slave Device Failure",
    5: "Acknowledge",
    6: "Slave Device Busy",
    8: "Memory Parity Error",
    10: "Gateway Path Unavailable",
    11: "Gateway Target Device Failed to Respond",
}


def _exception_code(result):
    """Extract Modbus exception code from an error result, or None."""
    if result is None:
        return None
    try:
        return result.exception_code
    except AttributeError:
        pass
    try:
        # pymodbus 3.x
        return result.function_code & 0x7F if result.isError() else None
    except AttributeError:
        return None


def _exc_label(code):
    if code is None:
        return "timeout/no response"
    name = MODBUS_EXCEPTION_NAMES.get(code, "Unknown")
    return f"Exception {code:02d}: {name}"


def benchmark(args):
    client = connect_client(args.host, args.port, args.timeout)

    area_names = {
        "coils": "read_coils       (FC01)",
        "di":    "read_disc_inputs (FC02)",
        "hr":    "read_holding_reg (FC03)",
        "ir":    "read_input_reg   (FC04)",
    }

    print(f"{GREEN}[+] Connected to {args.host}:{args.port}{RESET}")
    print(f"{BLUE}[i] Benchmark — {area_names[args.area]}{RESET}")
    print(f"{BLUE}[i] address={args.address}  count={args.count}  "
          f"requests={args.requests}  delay={args.delay}s{RESET}")
    print(f"{BLUE}[i] This is a read-only test. No data is written.{RESET}")
    print()

    latencies = []
    errors     = 0
    start_wall = time.perf_counter()

    for i in range(1, args.requests + 1):
        t0     = time.perf_counter()
        result = read_area(client, args.area, args.unit, args.address, args.count)
        t1     = time.perf_counter()
        ms     = (t1 - t0) * 1000

        if is_error(result):
            errors += 1
            mark    = f"{RED}ERR{RESET}"
        else:
            latencies.append(ms)
            mark = f"{GREEN}OK {RESET}"

        # Progress line — overwrite in place
        print(f"\r  [{mark}] {i:>{len(str(args.requests))}}/{args.requests}  "
              f"last={ms:7.2f} ms", end="", flush=True)

        if args.delay > 0:
            time.sleep(args.delay)

    wall = time.perf_counter() - start_wall
    client.close()

    print("\n")
    ok = len(latencies)

    if not latencies:
        print(f"{RED}[!] All {errors} requests failed — no latency data.{RESET}")
        return

    mn  = min(latencies)
    mx  = max(latencies)
    avg = statistics.mean(latencies)
    med = statistics.median(latencies)
    sd  = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
    rps = args.requests / wall if wall > 0 else 0

    col = lambda v: f"{GREEN}{v}{RESET}"

    print(f"{BOLD}{'─'*50}{RESET}")
    print(f"{BOLD}  BENCHMARK RESULTS{RESET}")
    print(f"{'─'*50}")
    print(f"  Total requests : {args.requests}")
    print(f"  Successful     : {col(ok)}")
    print(f"  Errors         : {RED}{errors}{RESET}" if errors else f"  Errors         : 0")
    print(f"  Wall time      : {wall:.3f} s")
    print(f"  Throughput     : {col(f'{rps:.1f}')} req/s")
    print(f"{'─'*50}")
    print(f"  Min latency    : {col(f'{mn:.2f}')} ms")
    print(f"  Avg latency    : {col(f'{avg:.2f}')} ms")
    print(f"  Median latency : {col(f'{med:.2f}')} ms")
    print(f"  Max latency    : {col(f'{mx:.2f}')} ms")
    print(f"  Std deviation  : {sd:.2f} ms")
    print(f"{'─'*50}")

    # Percentile buckets
    buckets = [(p, sorted(latencies)[int(len(latencies) * p / 100)]) for p in (50, 90, 95, 99)]
    for pct, val in buckets:
        bar_len = min(int(val / mx * 30), 30) if mx > 0 else 0
        bar = "█" * bar_len
        print(f"  p{pct:<3}           : {val:7.2f} ms  {BLUE}{bar}{RESET}")
    print(f"{'─'*50}")


# ============================================================
# Fuzz boundary — edge-address and edge-count probing (read-only)
# ============================================================

def fuzz_boundaries(args):
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{GREEN}[+] Connected to {args.host}:{args.port}{RESET}")
    print(f"{BLUE}[i] Fuzz boundary test — area={args.area}  unit={args.unit}{RESET}")
    print(f"{BLUE}[i] Read-only. Tests edge addresses and edge counts.{RESET}")
    print(f"{BLUE}[i] Checks whether the PLC returns valid data or correct exception codes.{RESET}")
    print()

    # Protocol maximums per spec
    MAX_COIL_COUNT = 2000
    MAX_REG_COUNT  = 125
    is_bit_area    = args.area in ("coils", "di")
    max_count      = MAX_COIL_COUNT if is_bit_area else MAX_REG_COUNT

    mid_addr = args.max_address // 2

    # (label, address, count)
    cases = [
        ("addr=0            count=1          (min address)",         0,                    1),
        (f"addr=0            count={max_count:<5}          (max valid count)", 0,          max_count),
        (f"addr=0            count={max_count+1:<5}          (count overflow)",  0,        max_count + 1),
        (f"addr=0            count=0          (zero count)",          0,                    0),
        (f"addr={mid_addr:<6}          count=1          (mid address)",     mid_addr,      1),
        (f"addr={args.max_address-1:<6}          count=1          (max-1 address)",  args.max_address - 1, 1),
        (f"addr={args.max_address:<6}          count=1          (max address)",    args.max_address,     1),
        (f"addr={args.max_address:<6}          count=2          (address+count overflow)", args.max_address, 2),
        ("addr=65535         count=1          (protocol ceiling)",   65535,                1),
        ("addr=65534         count=2          (crosses 65535)",      65534,                2),
    ]

    ok = errors = exc = 0

    header = f"  {'Case':<55} {'Status':<14} {'Detail'}"
    print(f"{BOLD}{header}{RESET}")
    print(f"  {'─'*110}")

    for label, addr, count in cases:
        t0     = time.perf_counter()
        result = read_area(client, args.area, args.unit, addr, max(count, 1))
        ms     = (time.perf_counter() - t0) * 1000

        if count == 0:
            # pymodbus won't send a count=0 request — mark as skipped
            status = f"{YELLOW}SKIPPED{RESET}"
            detail = "count=0 rejected client-side (per spec: illegal)"
        elif is_error(result):
            code   = _exception_code(result)
            exc   += 1
            errors += 1
            status = f"{YELLOW}EXCEPTION{RESET}"
            detail = _exc_label(code)
        else:
            ok    += 1
            status = f"{GREEN}VALID   {RESET}"
            detail = f"{ms:.1f} ms"

        print(f"  {label:<55} {status:<23} {detail}")

    print(f"  {'─'*110}")
    print(f"\n{BLUE}[i] Valid responses: {ok}  |  Exceptions: {exc}  |  Other errors: {errors - exc}{RESET}")
    print(f"{BLUE}[i] Exceptions are expected for out-of-range addresses/counts on a conformant device.{RESET}")
    client.close()


# ============================================================
# Exception test — verify PLC returns correct Modbus exception codes
# ============================================================

def exception_test(args):
    """
    Sends deliberately out-of-spec requests and checks the PLC returns
    the correct Modbus exception code for each.

    No writes are performed unless --allow-write is set (write-to-read-only tests).
    """
    client = connect_client(args.host, args.port, args.timeout)

    print(f"{GREEN}[+] Connected to {args.host}:{args.port}{RESET}")
    print(f"{BLUE}[i] Exception response test — unit={args.unit}{RESET}")
    if args.allow_write:
        print(f"{YELLOW}[i] --allow-write set: write-to-read-only tests included.{RESET}")
    else:
        print(f"{BLUE}[i] Write-to-read-only tests skipped (add --allow-write to include).{RESET}")
    print()

    # Each case: (description, func, kwargs, expected_exception_code or None for "any error")
    cases = []

    # ── FC03 / FC01 boundary violations ────────────────────────────────────
    cases += [
        (
            "FC03  HR read  addr=0      count=126  (count > 125)",
            client.read_holding_registers,
            dict(address=0, count=126),
            3,   # Illegal Data Value
        ),
        (
            "FC03  HR read  addr=65535  count=1    (addr at ceiling)",
            client.read_holding_registers,
            dict(address=65535, count=1),
            2,   # Illegal Data Address  (most PLCs)
        ),
        (
            "FC03  HR read  addr=65534  count=2    (addr+count > 65535)",
            client.read_holding_registers,
            dict(address=65534, count=2),
            2,
        ),
        (
            "FC04  IR read  addr=65535  count=1    (addr at ceiling)",
            client.read_input_registers,
            dict(address=65535, count=1),
            2,
        ),
        (
            "FC01  Coil read  addr=65535  count=1  (addr at ceiling)",
            client.read_coils,
            dict(address=65535, count=1),
            2,
        ),
        (
            "FC01  Coil read  addr=0  count=2001   (count > 2000)",
            client.read_coils,
            dict(address=0, count=2001),
            3,
        ),
        (
            "FC02  DI read    addr=65535  count=1  (addr at ceiling)",
            client.read_discrete_inputs,
            dict(address=65535, count=1),
            2,
        ),
    ]

    # ── Write-to-read-only areas (FC02, FC04) ───────────────────────────────
    if args.allow_write:
        cases += [
            (
                "FC05  Write coil  addr=65535           (addr at ceiling)",
                client.write_coil,
                dict(address=65535, value=False),
                2,
            ),
            (
                "FC06  Write HR    addr=65535           (addr at ceiling)",
                client.write_register,
                dict(address=65535, value=0),
                2,
            ),
        ]

    PASS_COL = f"{GREEN}PASS{RESET}"
    FAIL_COL = f"{RED}FAIL{RESET}"
    SKIP_COL = f"{YELLOW}N/A {RESET}"

    passed = failed = na = 0

    header = f"  {'Test':<57} {'Expect':>8}   {'Got':>8}   {'Result'}"
    print(f"{BOLD}{header}{RESET}")
    print(f"  {'─'*100}")

    for desc, func, kwargs, expected_code in cases:
        result = modbus_call(func, args.unit, **kwargs)

        if not is_error(result):
            # PLC accepted an out-of-spec request — note it but don't crash
            got_str    = f"{YELLOW}ACCEPTED{RESET}"
            expect_str = f"Exc {expected_code:02d}" if expected_code else "any error"
            verdict    = f"{YELLOW}NOTE {RESET}"
            na        += 1
        else:
            code    = _exception_code(result)
            got_str = f"Exc {code:02d}" if code is not None else "timeout"

            if expected_code is None:
                # Any error is fine
                verdict = PASS_COL
                passed += 1
                expect_str = "any error"
            elif code == expected_code:
                verdict    = PASS_COL
                passed    += 1
                expect_str = f"Exc {expected_code:02d}"
            else:
                verdict    = FAIL_COL
                failed    += 1
                expect_str = f"Exc {expected_code:02d}"

        expected_label = (
            f"Exc {expected_code:02d} ({MODBUS_EXCEPTION_NAMES.get(expected_code, '?')})"
            if expected_code else "any error"
        )
        print(f"  {desc:<57} {expect_str:>8}   {got_str:>8}   {verdict}")
        print(f"  {'':>57}   {BLUE}{expected_label}{RESET}")
        print()

    print(f"  {'─'*100}")
    total = passed + failed + na
    print(f"\n{BOLD}  Results: {GREEN}{passed} PASS{RESET}  /  {RED}{failed} FAIL{RESET}  /  {YELLOW}{na} NOTE{RESET}  (of {total} tests){RESET}")

    if failed == 0 and na == 0:
        print(f"{GREEN}[+] All tests passed — PLC exception handling is conformant.{RESET}")
    elif failed > 0:
        print(f"{RED}[!] {failed} test(s) returned unexpected exception codes.{RESET}")
        print(f"{YELLOW}[i] This may indicate non-standard Modbus implementation on this device.{RESET}")
    if na > 0:
        print(f"{YELLOW}[i] {na} test(s) — PLC accepted out-of-spec requests without error.{RESET}")
        print(f"{YELLOW}[i] Some devices silently clamp or ignore bad addresses; verify in KV STUDIO.{RESET}")

    client.close()


# ============================================================
# --examples output
# ============================================================

EXAMPLES_TEXT = f"""
{BOLD}modbus-tool.py — Usage Examples{RESET}
{'─' * 60}

{BOLD}{BLUE}CONNECTIVITY{RESET}

  # Verify TCP + Modbus reachability (generic)
  python3 modbus-tool.py --host 192.168.1.10 check

  # Verify with a specific Keyence model profile
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 check

  # Use a non-standard port or timeout
  python3 modbus-tool.py --host 192.168.1.10 --port 1502 --timeout 5.0 check

  # Discover which Modbus Unit ID the PLC responds on
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 discover-unit

  # Narrow the Unit ID scan range
  python3 modbus-tool.py --host 192.168.1.10 discover-unit --start-unit 1 --end-unit 10

{'─' * 60}
{BOLD}{BLUE}READ — COILS (FC01){RESET}

  # Read 1 coil at address 0
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 read-coils --address 0

  # Read 64 coils starting at address 0  (KV-7500: R0–R63)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 read-coils --address 0 --count 64

  # KV Nano — read 16 coils  (KV-N24: R0–R15)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-n24 --unit 1 read-coils --address 0 --count 16

  # OpenPLC — read 8 coils starting at %QX0.0
  python3 modbus-tool.py --host 192.168.1.10 --profile openplc --unit 1 read-coils --address 0 --count 8

{'─' * 60}
{BOLD}{BLUE}READ — DISCRETE INPUTS (FC02){RESET}

  # Read 8 discrete inputs at address 0  (KV: MR0–MR7)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 read-discrete-inputs --address 0 --count 8

  # OpenPLC — read 8 inputs starting at %IX0.0
  python3 modbus-tool.py --host 192.168.1.10 --profile openplc --unit 1 read-discrete-inputs --address 0 --count 8

{'─' * 60}
{BOLD}{BLUE}READ — HOLDING REGISTERS (FC03){RESET}

  # Read DM0–DM9 on a KV-8000
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 read-holding-registers --address 0 --count 10

  # Read DM0–DM19 on a KV-5500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 read-holding-registers --address 0 --count 20

  # Read DM0–DM9 on a KV-3000
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv3000 --unit 1 read-holding-registers --address 0 --count 10

  # Read DM0–DM9 on a KV Nano (N14/N24/C16/C24/NP20 — same DM space)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-n24 --unit 1 read-holding-registers --address 0 --count 10

  # OpenPLC — read %MW0–%MW9  (HR address 1024–1033)
  python3 modbus-tool.py --host 192.168.1.10 --profile openplc --unit 1 read-holding-registers --address 1024 --count 10

  # OpenPLC — read %QW0–%QW9  (HR address 0–9)
  python3 modbus-tool.py --host 192.168.1.10 --profile openplc --unit 1 read-holding-registers --address 0 --count 10

{'─' * 60}
{BOLD}{BLUE}READ — INPUT REGISTERS (FC04){RESET}

  # Read AT0–AT3 (analog/HS counter inputs) on a KV-7500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 read-input-registers --address 0 --count 4

  # Read 4 input registers on generic profile
  python3 modbus-tool.py --host 192.168.1.10 --unit 1 read-input-registers --address 0 --count 4

{'─' * 60}
{BOLD}{YELLOW}WRITE — COILS (FC05 / FC15)  [require --allow-write]{RESET}

  # Write TRUE to coil 0 — KV-8000 R0  (authorized lab only)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 --allow-write write-coil --address 0 --value true

  # Write FALSE to coil 5 — KV Nano R5
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-n24 --unit 1 --allow-write write-coil --address 5 --value false

  # Write multiple coils at once — R0=ON, R1=OFF, R2=ON
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 --allow-write write-coils --address 0 --values true,false,true

  # Accepted boolean formats: true/false  1/0  on/off  yes/no
  python3 modbus-tool.py --host 192.168.1.10 --unit 1 --allow-write write-coil --address 0 --value 1

{'─' * 60}
{BOLD}{YELLOW}WRITE — HOLDING REGISTERS (FC06 / FC16)  [require --allow-write]{RESET}

  # Write 1234 to DM0 on a KV-5500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 --allow-write write-register --address 0 --value 1234

  # Write 0xABCD to DM10 on a KV-8000 (hex accepted)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 --allow-write write-register --address 10 --value 0xABCD

  # Write multiple registers — DM0=100, DM1=200, DM2=300
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv3000 --unit 1 --allow-write write-registers --address 0 --values 100,200,300

  # OpenPLC — write 42 to %MW0  (HR address 1024)
  python3 modbus-tool.py --host 192.168.1.10 --profile openplc --unit 1 --allow-write write-register --address 1024 --value 42

{'─' * 60}
{BOLD}{YELLOW}TURN ON / TURN OFF / TOGGLE  [require --allow-write]{RESET}

  Three ways to specify which coils to switch:

  {BOLD}① Single coil  --address ADDR{RESET}
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 --allow-write turn-on  --address 0
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 --allow-write turn-off --address 0

  {BOLD}② Multiple consecutive coils  --address ADDR --count N{RESET}
    # Turn on R0–R7 (8 coils) — KV-1000
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv1000 --unit 1 --allow-write turn-on  --address 0 --count 8
    # Turn off R0–R7 — KV-7500
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 --allow-write turn-off --address 0 --count 8

  {BOLD}③ Explicit non-contiguous list  --addresses A,B,C{RESET}
    # Turn on R0, R3, R7 — KV-5500
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 --allow-write turn-on  --addresses 0,3,7
    # Turn off R0, R1, R2 — KV-7500
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 --allow-write turn-off --addresses 0,1,2

  {BOLD}④ Inclusive range  --from START --to END{RESET}
    # Turn on R0–R15 — KV-8000
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 --allow-write turn-on  --from 0 --to 15
    # Turn off R0–R15 — KV Nano
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-n24 --unit 1 --allow-write turn-off --from 0 --to 15

  {BOLD}Toggle  (reads current state then writes the opposite){RESET}
    # Flip single coil — KV-3000 R5
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv3000 --unit 1 --allow-write toggle --address 5
    # Flip R0, R2, R4 — KV-5500
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 --allow-write toggle --addresses 0,2,4
    # Flip R0–R3 range — KV-7300
    python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7300 --unit 1 --allow-write toggle --from 0 --to 3

{'─' * 60}
{BOLD}{BLUE}DISCOVERY (read-only scan){RESET}

  # Full discovery — all 4 Modbus areas — KV-8000, show non-zero only
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 discover-all --start 0 --end 500

  # Full discovery — show every address including zeros
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 discover-all --start 0 --end 200 --show-all

  # Discover only holding registers (DM) on a KV-7300
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7300 --unit 1 discover-holding-registers --start 0 --end 100 --show-all

  # Discover only coils on a KV Nano
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-np20 --unit 1 discover-coils --start 0 --end 64 --show-all

  # Slow scan with inter-block delay (avoids overwhelming the PLC)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv1000 --unit 1 discover-all --start 0 --end 300 --delay 0.1

  # Large block size for faster scanning on reliable networks
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 discover-holding-registers --start 0 --end 1000 --block-size 50

{'─' * 60}
{BOLD}{BLUE}MONITOR (live continuous polling){RESET}

  # Poll DM0–DM4 every second — KV-5500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 monitor --area hr --address 0 --count 5

  # Poll R0–R7 coils every 500 ms — KV-8000
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 monitor --area coils --address 0 --count 8 --interval 0.5

  # Poll discrete inputs every 2 s — KV Nano
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-c24 --unit 1 monitor --area di --address 0 --count 8 --interval 2.0

  # Monitor analog inputs (AT) every second — KV-7500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 monitor --area ir --address 0 --count 4

  # Press CTRL+C to stop monitoring

{'─' * 60}
{BOLD}{BLUE}BENCHMARK — latency / throughput (read-only){RESET}

  # 100 HR reads, default settings — KV-8000
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 benchmark

  # 500 requests, read 10 registers each — KV-5500
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 benchmark --area hr --address 0 --count 10 --requests 500

  # Coil read benchmark — KV-7500, 200 requests, 50 ms pacing
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7500 --unit 1 benchmark --area coils --address 0 --count 8 --requests 200 --delay 0.05

  # Analog input benchmark — KV Nano
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-np20 --unit 1 benchmark --area ir --address 0 --count 4 --requests 100

  # Slow-paced test to measure steady-state latency
  python3 modbus-tool.py --host 192.168.1.10 --unit 1 benchmark --requests 50 --delay 0.5

{'─' * 60}
{BOLD}{BLUE}FUZZ BOUNDARIES — edge-address / edge-count probing (read-only){RESET}

  # HR boundary probe — KV-8000 (max address 99999)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 fuzz-boundaries --area hr --max-address 99999

  # Coil boundary probe — KV-5500 (max R59999)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 fuzz-boundaries --area coils --max-address 59999

  # Nano HR probe — max address 3999
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv-n24 --unit 1 fuzz-boundaries --area hr --max-address 3999

  # Generic device — probe protocol ceiling (65535)
  python3 modbus-tool.py --host 192.168.1.10 --unit 1 fuzz-boundaries --area hr --max-address 65535

  # Discrete input boundary probe
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv7300 --unit 1 fuzz-boundaries --area di --max-address 99999

{'─' * 60}
{BOLD}{BLUE}EXCEPTION TEST — verify Modbus exception code compliance{RESET}

  # Read-only exception tests — KV-8000
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv8000 --unit 1 exception-test

  # Include write-boundary tests — KV-5500 (authorized lab only)
  python3 modbus-tool.py --host 192.168.1.10 --profile keyence-kv5500 --unit 1 --allow-write exception-test

  # Generic device — check standard exception responses
  python3 modbus-tool.py --host 192.168.1.10 --unit 1 exception-test

  # What is tested:
  #   FC03 count > 125         → expects Exception 03 (Illegal Data Value)
  #   FC03 addr=65535          → expects Exception 02 (Illegal Data Address)
  #   FC03 addr+count > 65535  → expects Exception 02
  #   FC04 addr=65535          → expects Exception 02
  #   FC01 addr=65535          → expects Exception 02
  #   FC01 count > 2000        → expects Exception 03
  #   FC02 addr=65535          → expects Exception 02
  #   (--allow-write) FC05 addr=65535  → expects Exception 02
  #   (--allow-write) FC06 addr=65535  → expects Exception 02

{'─' * 60}
{BOLD}{BLUE}KEYENCE MODEL QUICK REFERENCE{RESET}

  Profile                        Coils (R)     HR (DM)       Modbus TCP
  ─────────────────────────────────────────────────────────────────────
  keyence-kv8000                 R0–R99999     DM0–DM99999   Built-in
  keyence-kv7500                 R0–R99999     DM0–DM99999   Built-in
  keyence-kv7300                 R0–R99999     DM0–DM99999   Built-in
  keyence-kv-xh16ml              R0–R99999     DM0–DM99999   Built-in
  keyence-kv-xh04ml              R0–R99999     DM0–DM99999   Built-in
  keyence-kv5500                 R0–R59999     DM0–DM59999   Built-in
  keyence-kv5000                 R0–R59999     DM0–DM59999   KV-EP21V required
  keyence-kv3000                 R0–R39999     DM0–DM32767   KV-EP21V required
  keyence-kv1000                 R0–R19999     DM0–DM19999   KV-EP21V required
  keyence-kv700                  R0–R9999      DM0–DM9999    KV-EP21V required
  keyence-kvp16                  R0–R9999      DM0–DM4095    KV-EP21V required
  keyence-kv-n14                 R0–R3999      DM0–DM3999    KV-EP21V / built-in*
  keyence-kv-n24                 R0–R3999      DM0–DM3999    KV-EP21V / built-in*
  keyence-kv-c16                 R0–R3999      DM0–DM3999    KV-EP21V / built-in*
  keyence-kv-c24                 R0–R3999      DM0–DM3999    KV-EP21V / built-in*
  keyence-kv-np20                R0–R3999      DM0–DM3999    Built-in

  * KV-EP21V required unless firmware supports native Ethernet

  {YELLOW}Always verify address mapping in KV STUDIO → Unit Editor → Modbus TCP.{RESET}
"""


def print_examples():
    print(EXAMPLES_TEXT)


# ============================================================
# Parser
# ============================================================

def add_discovery_arguments(parser):
    parser.add_argument("--start", type=int, default=0, help="Start address")
    parser.add_argument("--end", type=int, default=100, help="End address")
    parser.add_argument("--block-size", type=int, default=20, help="Read block size")
    parser.add_argument("--show-all", action="store_true", help="Show zero/FALSE values too")
    parser.add_argument("--delay", type=float, default=0.0, help="Delay between blocks in seconds")


def build_keyence_profiles_help():
    lines = ["  Keyence model profiles:"]
    for key, model in sorted(KEYENCE_MODELS.items()):
        lines.append(f"    {key:<30} {model['label']}")
    return "\n".join(lines)


def build_parser():
    keyence_help = build_keyence_profiles_help()

    # Handle --examples before argparse enforces --host as required
    if "--examples" in sys.argv:
        print_examples()
        sys.exit(0)

    parser = argparse.ArgumentParser(
        prog="modbus-tool.py",
        description="Generic Modbus TCP CLI tool for authorized OpenPLC / Keyence / PLC labs",
        epilog=f"""
Tip: run  python3 modbus-tool.py --examples  for the full usage reference.

Examples:

  Check connection:
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv8000 check

  Discover Unit ID:
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv5500 discover-unit

  KV-8000 read-only discovery:
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv8000 --unit 1 discover-all --start 0 --end 200 --show-all

  KV-5500 read holding registers (DM0..DM19):
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv5500 --unit 1 read-holding-registers --address 0 --count 20

  KV-7500 read coils (R0..R63):
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv7500 --unit 1 read-coils --address 0 --count 64

  KV Nano write coil (authorized lab only):
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv-n24 --unit 1 --allow-write write-coil --address 0 --value true

  KV-3000 write register (authorized lab only):
    python3 modbus-tool.py --host 192.168.100.9 --profile keyence-kv3000 --unit 1 --allow-write write-register --address 0 --value 1234

  OpenPLC read %MW0:
    python3 modbus-tool.py --host 192.168.100.9 --profile openplc --unit 1 read-holding-registers --address 1024 --count 10

Profiles:

  generic:
    Basic Modbus hints only.

  openplc:
    Adds OpenPLC hints:
      %QX0.0 = Coil 0
      %IX0.0 = Discrete Input 0
      %QW0   = Holding Register 0
      %MW0   = Holding Register 1024

{keyence_help}

  Default Keyence device mapping (configurable in KV STUDIO Unit Editor):
    Coils (FC01/FC05)            -> Internal Relay  (R device)
    Discrete Inputs (FC02)       -> Input Relay      (MR device)
    Holding Registers (FC03/FC06)-> Data Memory      (DM device)
    Input Registers (FC04)       -> Analog/HS input  (AT device)

  IMPORTANT: Always verify the Modbus address table in KV STUDIO before
  reading or writing. Do not assume OpenPLC %MW offset 1024 for Keyence.
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        "--examples",
        action="store_true",
        help="Print detailed usage examples for every action and Keyence model, then exit."
    )
    parser.add_argument("--host", required=True, help="Target IP address")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP port")
    parser.add_argument("--unit", type=int, default=1, help="Modbus Unit ID / Slave ID")
    parser.add_argument("--timeout", type=float, default=3.0, help="Connection timeout")
    parser.add_argument(
        "--profile",
        choices=ALL_PROFILES,
        default="generic",
        metavar="PROFILE",
        help=(
            "Address hint profile. Choices: "
            + ", ".join(ALL_PROFILES)
            + " (see --help for full list)"
        )
    )
    parser.add_argument(
        "--allow-write",
        action="store_true",
        help="Enable write actions. Without this, write actions are blocked."
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")

    sub = parser.add_subparsers(dest="action", required=True)

    p = sub.add_parser("check", help="Check TCP and Modbus connectivity")
    p.add_argument("--address", type=int, default=0, help="Holding register address used for test read")

    p = sub.add_parser("discover-unit", help="Discover Modbus Unit ID")
    p.add_argument("--start-unit", type=int, default=1, help="Start Unit ID")
    p.add_argument("--end-unit", type=int, default=20, help="End Unit ID")
    p.add_argument("--address", type=int, default=0, help="Holding register address used for test")

    p = sub.add_parser("read-coils", help="Read coils / FC01")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--count", type=int, default=1)

    p = sub.add_parser("read-discrete-inputs", help="Read discrete inputs / FC02")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--count", type=int, default=1)

    p = sub.add_parser("read-holding-registers", help="Read holding registers / FC03")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--count", type=int, default=1)

    p = sub.add_parser("read-input-registers", help="Read input registers / FC04")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--count", type=int, default=1)

    p = sub.add_parser("write-coil", help="Write one coil / FC05")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--value", type=parse_bool, required=True)

    p = sub.add_parser("write-coils", help="Write multiple coils / FC15")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--values", type=parse_csv_bools, required=True)

    p = sub.add_parser("write-register", help="Write one holding register / FC06")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--value", type=int, required=True)

    p = sub.add_parser("write-registers", help="Write multiple holding registers / FC16")
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--values", type=parse_csv_ints, required=True)

    p = sub.add_parser("discover-all", help="Read-only discovery for all Modbus areas")
    add_discovery_arguments(p)

    p = sub.add_parser("discover-coils", help="Read-only discovery for coils")
    add_discovery_arguments(p)

    p = sub.add_parser("discover-discrete-inputs", help="Read-only discovery for discrete inputs")
    add_discovery_arguments(p)

    p = sub.add_parser("discover-holding-registers", help="Read-only discovery for holding registers")
    add_discovery_arguments(p)

    p = sub.add_parser("discover-input-registers", help="Read-only discovery for input registers")
    add_discovery_arguments(p)

    p = sub.add_parser(
        "turn-on",
        help="Turn ON one or more coils / FC05  [requires --allow-write]",
        description=(
            "Write TRUE to one or more Modbus coils.\n"
            "Specify targets with --address, --addresses, or --from/--to."
        )
    )
    add_plc_switch_arguments(p)

    p = sub.add_parser(
        "turn-off",
        help="Turn OFF one or more coils / FC05  [requires --allow-write]",
        description=(
            "Write FALSE to one or more Modbus coils.\n"
            "Specify targets with --address, --addresses, or --from/--to."
        )
    )
    add_plc_switch_arguments(p)

    p = sub.add_parser(
        "toggle",
        help="Read then flip coil state / FC01+FC05  [requires --allow-write]",
        description=(
            "Read the current coil state and write its inverse.\n"
            "Specify targets with --address, --addresses, or --from/--to."
        )
    )
    add_plc_switch_arguments(p)

    p = sub.add_parser(
        "benchmark",
        help="Latency / throughput measurement via repeated reads (read-only)",
        description="Send N read requests and report min/avg/max/stdev latency and req/s."
    )
    p.add_argument("--area", choices=["coils", "di", "hr", "ir"], default="hr",
                   help="Modbus area to read (default: hr)")
    p.add_argument("--address", type=int, default=0, help="Start address (default: 0)")
    p.add_argument("--count",   type=int, default=1,
                   help="Registers/coils per request (default: 1)")
    p.add_argument("--requests", type=int, default=100,
                   help="Total number of requests to send (default: 100)")
    p.add_argument("--delay", type=float, default=0.0,
                   help="Delay between requests in seconds (default: 0)")

    p = sub.add_parser(
        "fuzz-boundaries",
        help="Edge-address and edge-count read probing — checks exception handling",
        description=(
            "Send reads at boundary addresses (0, max-1, max, 65535) and boundary counts "
            "(1, max_valid, max_valid+1). Reports valid responses and Modbus exception codes. "
            "Read-only."
        )
    )
    p.add_argument("--area", choices=["coils", "di", "hr", "ir"], default="hr",
                   help="Modbus area to probe (default: hr)")
    p.add_argument("--max-address", type=int, default=9999,
                   help="Highest address to include in boundary tests (default: 9999)")

    p = sub.add_parser(
        "exception-test",
        help="Verify PLC returns correct Modbus exception codes for bad requests",
        description=(
            "Sends deliberately out-of-spec requests (overflow addresses, overflow counts) "
            "and checks each response against the expected Modbus exception code (EC01–EC04). "
            "With --allow-write also tests write-boundary cases."
        )
    )
    # no extra args needed beyond the globals; --allow-write enables write cases

    p = sub.add_parser("monitor", help="Monitor values continuously")
    p.add_argument("--area", choices=["coils", "di", "hr", "ir"], required=True)
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--interval", type=float, default=1.0)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.action == "check":
        check_connection(args)

    elif args.action == "discover-unit":
        discover_unit(args)

    elif args.action == "read-coils":
        read_coils(args)

    elif args.action == "read-discrete-inputs":
        read_discrete_inputs(args)

    elif args.action == "read-holding-registers":
        read_holding_registers(args)

    elif args.action == "read-input-registers":
        read_input_registers(args)

    elif args.action == "write-coil":
        write_coil(args)

    elif args.action == "write-coils":
        write_coils(args)

    elif args.action == "write-register":
        write_register(args)

    elif args.action == "write-registers":
        write_registers(args)

    elif args.action == "discover-all":
        discover_all(args)

    elif args.action == "discover-coils":
        discover_one_area(args, "coils")

    elif args.action == "discover-discrete-inputs":
        discover_one_area(args, "di")

    elif args.action == "discover-holding-registers":
        discover_one_area(args, "hr")

    elif args.action == "discover-input-registers":
        discover_one_area(args, "ir")

    elif args.action == "turn-on":
        validate_plc_switch_args(args)
        turn_on(args)

    elif args.action == "turn-off":
        validate_plc_switch_args(args)
        turn_off(args)

    elif args.action == "toggle":
        validate_plc_switch_args(args)
        toggle(args)

    elif args.action == "benchmark":
        benchmark(args)

    elif args.action == "fuzz-boundaries":
        fuzz_boundaries(args)

    elif args.action == "exception-test":
        exception_test(args)

    elif args.action == "monitor":
        monitor(args)


if __name__ == "__main__":
    main()
