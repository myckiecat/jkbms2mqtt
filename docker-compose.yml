#!/usr/bin/env python3
"""
jkbms2mqtt - Puente multi-batería JK BMS (v19) -> MQTT con autodiscovery Home Assistant.

Lee uno o varios JK BMS a través de un gateway RS485->TCP (EW11) por Modbus TCP,
y publica los datos en MQTT con descubrimiento automático en Home Assistant.

Cada batería se publica como un dispositivo HA independiente bajo el topic:
    jkbms/<nombre_bateria>/state         (JSON con todos los campos)
    jkbms/<nombre_bateria>/availability  (online/offline)

Config por variables de entorno (ver docker-compose.yml).

Diseñado para JK BMS con protocolo Modbus (UART1 = "001 - JK BMS RS485").
"""

import os
import sys
import json
import time
import struct
import signal
import logging
import socket

import paho.mqtt.client as mqtt
from pymodbus.client import ModbusTcpClient

# ----------------------------------------------------------------------------
# Configuración desde entorno
# ----------------------------------------------------------------------------
MODBUS_IP   = os.getenv("MODBUS_IP", "192.168.1.100")
MODBUS_PORT = int(os.getenv("MODBUS_PORT", "502"))

MQTT_HOST = os.getenv("MQTT_HOST", "192.168.1.207")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")

# Lista de baterías: "nombre:unit_id,nombre2:unit_id2"
BATTERIES_RAW = os.getenv("BATTERIES", "battery_1:1")

# Intervalo de sondeo en segundos
POLL_INTERVAL = int(os.getenv("POLL_INTERVAL", "10"))

# Modo de lectura: "block" (rápido, bloques grandes) o "single" (registro a registro,
# más lento pero compatible con EW11 que limitan a 125 registros por petición)
READ_MODE = os.getenv("READ_MODE", "single").lower()

# Prefijo de discovery de HA
HA_DISCOVERY_PREFIX = os.getenv("HA_DISCOVERY_PREFIX", "homeassistant")

DEBUG = os.getenv("DEBUG", "false").lower() in ("1", "true", "yes")

logging.basicConfig(
    level=logging.DEBUG if DEBUG else logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("jkbms2mqtt")

# ----------------------------------------------------------------------------
# Mapa de registros JK BMS (Modbus holding registers)
# Direcciones base del protocolo JK BMS RS485. Los cell voltages, battery voltage,
# current, power, SOC, temperaturas y balance current están validados contra el
# stack original que funciona. Los campos "extra" (SOH, ciclos, capacidad,
# alarmas, FET) provienen del mapa Modbus del JK v19 y conviene validarlos con
# DEBUG=true en la primera ejecución.
# ----------------------------------------------------------------------------

# Bloque 1: lectura principal. Base 0x1200, longitud suficiente para cubrir celdas + agregados.
BLOCK1_START = 0x1200
BLOCK1_COUNT = 144  # 16 celdas + resistencias + agregados

# Bloque 2: información de estado / SOC / alarmas.
BLOCK2_START = 0x1280
BLOCK2_COUNT = 80

# Offsets relativos dentro del área leída (en nº de registros de 16 bits desde 0x1200).
# NOTA: estos offsets siguen el layout del JK BMS v19. Ajustar si DEBUG revela desfase.
REG = {
    "cell_voltages_start": 0x1200,   # 16 celdas, 1 registro (uint16, mV) cada una
    "cell_count": 16,
    "cell_res_start":      0x1220,   # resistencias de conexión, 1 reg (uint16, mOhm/100) cada una
    "battery_voltage":     0x1290,   # uint32 (2 regs), mV
    "battery_current":     0x1298,   # int32 (2 regs), mA (con signo)
    "battery_power":       0x129C,   # uint32 (2 regs), mW
    "temp_mos":            0x1280,   # int16, 0.1 C
    "temp_1":              0x1282,   # int16, 0.1 C
    "temp_2":              0x1284,   # int16, 0.1 C
    "balance_current":     0x12A0,   # int16, mA
    "soc":                 0x12A2,   # uint16, %
    "remaining_capacity":  0x12A4,   # uint32 (2 regs), mAh
    "nominal_capacity":    0x12A8,   # uint32 (2 regs), mAh
    "cycle_count":         0x12AC,   # uint32 (2 regs)
    "soh":                 0x12B0,   # uint16, %
    "alarms":              0x12B2,   # uint32 (2 regs), bitfield
    "fet_state":           0x12B6,   # uint16, bitfield (bit0 chg, bit1 dsg, bit2 bal)
}


# ----------------------------------------------------------------------------
# Utilidades de decodificación
# ----------------------------------------------------------------------------
def u16(regs, idx):
    return regs[idx]

def s16(regs, idx):
    v = regs[idx]
    return v - 0x10000 if v >= 0x8000 else v

def u32(regs, idx):
    return (regs[idx] << 16) | regs[idx + 1]

def s32(regs, idx):
    v = (regs[idx] << 16) | regs[idx + 1]
    return v - 0x100000000 if v >= 0x80000000 else v


def parse_batteries(raw):
    """Convierte 'name:1,name2:2' en [('name',1),('name2',2)]."""
    out = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            log.warning("Entrada de batería mal formada (falta ':'): %r", part)
            continue
        name, sid = part.split(":", 1)
        try:
            out.append((name.strip(), int(sid.strip())))
        except ValueError:
            log.warning("Unit id no numérico en %r", part)
    return out


# ----------------------------------------------------------------------------
# Lectura Modbus
# ----------------------------------------------------------------------------
import inspect

# pymodbus renombró el argumento de unidad: <=3.6 usa "slave", >=3.7 usa "device_id".
# Detectamos cuál acepta esta versión una sola vez.
def _detect_unit_kwarg():
    try:
        sig = inspect.signature(ModbusTcpClient.read_holding_registers)
        params = sig.parameters
        if "slave" in params:
            return "slave"
        if "device_id" in params:
            return "device_id"
    except (ValueError, TypeError):
        pass
    # Fallback: intentamos slave (versiones antiguas más comunes en la práctica)
    return "slave"

UNIT_KWARG = _detect_unit_kwarg()
log.info("pymodbus usa el argumento de unidad: '%s'", UNIT_KWARG)


def _read_holding(client, address, count, unit_id):
    """Lee holding registers de forma compatible con cualquier versión de pymodbus."""
    kwargs = {"address": address, "count": count, UNIT_KWARG: unit_id}
    return client.read_holding_registers(**kwargs)


def read_registers_block(client, unit_id):
    """Lee en dos bloques grandes. Devuelve dict {abs_addr: value} o None si falla."""
    regs = {}
    for start, count in ((BLOCK1_START, BLOCK1_COUNT), (BLOCK2_START, BLOCK2_COUNT)):
        rr = _read_holding(client, start, count, unit_id)
        if rr.isError():
            log.error("[unit %s] Error leyendo bloque 0x%04X x%d: %s",
                      unit_id, start, count, rr)
            return None
        for i, val in enumerate(rr.registers):
            regs[start + i] = val
    return regs


def read_registers_single(client, unit_id):
    """Lee registro a registro solo las direcciones necesarias. Más lento pero
    compatible con EW11 que limitan el tamaño de petición."""
    regs = {}
    # celdas + resistencias contiguas
    needed = []
    needed += [REG["cell_voltages_start"] + i for i in range(REG["cell_count"])]
    needed += [REG["cell_res_start"] + i for i in range(REG["cell_count"])]
    # agregados (algunos uint32 => 2 registros)
    singles = [
        (REG["battery_voltage"], 2), (REG["battery_current"], 2), (REG["battery_power"], 2),
        (REG["temp_mos"], 1), (REG["temp_1"], 1), (REG["temp_2"], 1),
        (REG["balance_current"], 1), (REG["soc"], 1),
        (REG["remaining_capacity"], 2), (REG["nominal_capacity"], 2),
        (REG["cycle_count"], 2), (REG["soh"], 1), (REG["alarms"], 2), (REG["fet_state"], 1),
    ]
    for addr, n in singles:
        for k in range(n):
            needed.append(addr + k)

    for addr in needed:
        rr = _read_holding(client, addr, 1, unit_id)
        if rr.isError():
            log.error("[unit %s] Error leyendo registro 0x%04X: %s", unit_id, addr, rr)
            return None
        regs[addr] = rr.registers[0]
    return regs


def rel(regs, abs_addr, base=BLOCK1_START):
    """Acceso por dirección absoluta con lista basada en base. Devuelve índice list."""
    # Construimos una lista contigua desde el mínimo para poder usar u16/u32 por índice.
    return regs[abs_addr]


def decode(regs):
    """Construye el diccionario de estado a partir del dict {addr: value}."""
    def g(addr):
        return regs.get(addr)

    def g16(addr):
        v = regs.get(addr)
        return None if v is None else (v - 0x10000 if v >= 0x8000 else v)

    def g32(addr):
        hi, lo = regs.get(addr), regs.get(addr + 1)
        if hi is None or lo is None:
            return None
        v = (hi << 16) | lo
        return v

    def g32s(addr):
        v = g32(addr)
        if v is None:
            return None
        return v - 0x100000000 if v >= 0x80000000 else v

    cells = []
    for i in range(REG["cell_count"]):
        mv = g(REG["cell_voltages_start"] + i)
        if mv:
            cells.append(round(mv / 1000.0, 3))
    resistances = []
    for i in range(REG["cell_count"]):
        r = g(REG["cell_res_start"] + i)
        if r is not None:
            resistances.append(round(r / 1000.0, 3))

    data = {}
    if cells:
        data["cell_voltages"] = cells
        data["cell_min"] = min(cells)
        data["cell_max"] = max(cells)
        data["cell_delta_mv"] = round((max(cells) - min(cells)) * 1000, 1)
        data["cell_avg"] = round(sum(cells) / len(cells), 3)
    if resistances:
        data["cell_resistances"] = resistances

    bv = g32(REG["battery_voltage"])
    if bv is not None:
        data["battery_voltage"] = round(bv / 1000.0, 3)
    bc = g32s(REG["battery_current"])
    if bc is not None:
        data["battery_current"] = round(bc / 1000.0, 3)
    bp = g32(REG["battery_power"])
    if bp is not None:
        data["battery_power"] = round(bp / 1000.0, 1)

    for key, addr, scale in (
        ("temp_mos", REG["temp_mos"], 0.1),
        ("temp_1", REG["temp_1"], 0.1),
        ("temp_2", REG["temp_2"], 0.1),
    ):
        v = g16(addr)
        if v is not None:
            data[key] = round(v * scale, 1)

    bal = g16(REG["balance_current"])
    if bal is not None:
        data["balance_current"] = round(bal / 1000.0, 3)

    soc = g(REG["soc"])
    if soc is not None:
        data["soc"] = soc

    rc = g32(REG["remaining_capacity"])
    if rc is not None:
        data["remaining_capacity_ah"] = round(rc / 1000.0, 2)
    nc = g32(REG["nominal_capacity"])
    if nc is not None:
        data["nominal_capacity_ah"] = round(nc / 1000.0, 2)

    cyc = g32(REG["cycle_count"])
    if cyc is not None:
        data["cycle_count"] = cyc
    soh = g(REG["soh"])
    if soh is not None:
        data["soh"] = soh

    alarms = g32(REG["alarms"])
    if alarms is not None:
        data["alarms_raw"] = alarms
        data["alarm_active"] = alarms != 0

    fet = g(REG["fet_state"])
    if fet is not None:
        data["charging"] = bool(fet & 0x01)
        data["discharging"] = bool(fet & 0x02)
        data["balancing"] = bool(fet & 0x04)

    return data


# ----------------------------------------------------------------------------
# HA autodiscovery
# ----------------------------------------------------------------------------
# (sensor_key, nombre, unidad, device_class, state_class, icon)
SENSOR_DEFS = [
    ("battery_voltage", "Voltage", "V", "voltage", "measurement", None),
    ("battery_current", "Current", "A", "current", "measurement", None),
    ("battery_power", "Power", "W", "power", "measurement", None),
    ("soc", "SOC", "%", "battery", "measurement", None),
    ("soh", "SOH", "%", None, "measurement", "mdi:heart-pulse"),
    ("cell_min", "Cell Min", "V", "voltage", "measurement", None),
    ("cell_max", "Cell Max", "V", "voltage", "measurement", None),
    ("cell_avg", "Cell Avg", "V", "voltage", "measurement", None),
    ("cell_delta_mv", "Cell Delta", "mV", "voltage", "measurement", None),
    ("temp_mos", "Temp MOS", "°C", "temperature", "measurement", None),
    ("temp_1", "Temp 1", "°C", "temperature", "measurement", None),
    ("temp_2", "Temp 2", "°C", "temperature", "measurement", None),
    ("balance_current", "Balance Current", "A", "current", "measurement", None),
    ("remaining_capacity_ah", "Remaining Capacity", "Ah", None, "measurement", "mdi:battery-50"),
    ("nominal_capacity_ah", "Nominal Capacity", "Ah", None, None, "mdi:battery"),
    ("cycle_count", "Cycle Count", None, None, "total_increasing", "mdi:counter"),
]

BINARY_SENSOR_DEFS = [
    ("charging", "Charging", "battery-charging"),
    ("discharging", "Discharging", "battery-arrow-down"),
    ("balancing", "Balancing", "scale-balance"),
    ("alarm_active", "Alarm", "alert"),
]


def publish_discovery(client, name):
    dev = {
        "identifiers": [f"jkbms_{name}"],
        "name": f"JK BMS {name}",
        "manufacturer": "JIKONG",
        "model": "JK-BMS v19",
    }
    base_state = f"jkbms/{name}/state"
    avail = f"jkbms/{name}/availability"

    for key, label, unit, dclass, sclass, icon in SENSOR_DEFS:
        uid = f"jkbms_{name}_{key}"
        cfg = {
            "name": label,
            "unique_id": uid,
            "object_id": uid,
            "state_topic": base_state,
            "value_template": f"{{{{ value_json.{key} }}}}",
            "availability_topic": avail,
            "device": dev,
        }
        if unit:
            cfg["unit_of_measurement"] = unit
        if dclass:
            cfg["device_class"] = dclass
        if sclass:
            cfg["state_class"] = sclass
        if icon:
            cfg["icon"] = icon
        topic = f"{HA_DISCOVERY_PREFIX}/sensor/{uid}/config"
        client.publish(topic, json.dumps(cfg), retain=True)

    for key, label, icon in BINARY_SENSOR_DEFS:
        uid = f"jkbms_{name}_{key}"
        cfg = {
            "name": label,
            "unique_id": uid,
            "object_id": uid,
            "state_topic": base_state,
            "value_template": f"{{{{ 'ON' if value_json.{key} else 'OFF' }}}}",
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability_topic": avail,
            "device": dev,
            "icon": f"mdi:{icon}",
        }
        topic = f"{HA_DISCOVERY_PREFIX}/binary_sensor/{uid}/config"
        client.publish(topic, json.dumps(cfg), retain=True)

    log.info("[%s] Discovery HA publicado (%d sensores, %d binarios)",
             name, len(SENSOR_DEFS), len(BINARY_SENSOR_DEFS))


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
running = True

def handle_signal(signum, frame):
    global running
    log.info("Señal %s recibida, cerrando...", signum)
    running = False

signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


def main():
    batteries = parse_batteries(BATTERIES_RAW)
    if not batteries:
        log.error("No hay baterías configuradas. Revisa BATTERIES=%r", BATTERIES_RAW)
        sys.exit(1)

    log.info("Baterías: %s | EW11 %s:%s | MQTT %s:%s | modo lectura: %s",
             batteries, MODBUS_IP, MODBUS_PORT, MQTT_HOST, MQTT_PORT, READ_MODE)

    mqttc = mqtt.Client(client_id="jkbms2mqtt")
    if MQTT_USER:
        mqttc.username_pw_set(MQTT_USER, MQTT_PASS)
    for name, _ in batteries:
        mqttc.will_set(f"jkbms/{name}/availability", "offline", retain=True)

    try:
        mqttc.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    except Exception as e:
        log.error("No se pudo conectar a MQTT: %s", e)
        sys.exit(1)
    mqttc.loop_start()

    for name, _ in batteries:
        publish_discovery(mqttc, name)

    read_fn = read_registers_block if READ_MODE == "block" else read_registers_single

    modbus = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT, timeout=5)

    while running:
        if not modbus.connected:
            if not modbus.connect():
                log.error("No se pudo conectar al EW11 %s:%s, reintento en %ss",
                          MODBUS_IP, MODBUS_PORT, POLL_INTERVAL)
                time.sleep(POLL_INTERVAL)
                continue

        for name, unit_id in batteries:
            try:
                regs = read_fn(modbus, unit_id)
                if not regs:
                    mqttc.publish(f"jkbms/{name}/availability", "offline", retain=True)
                    continue
                data = decode(regs)
                if DEBUG:
                    log.debug("[%s] %s", name, json.dumps(data, ensure_ascii=False))
                mqttc.publish(f"jkbms/{name}/state", json.dumps(data), retain=False)
                mqttc.publish(f"jkbms/{name}/availability", "online", retain=True)
            except Exception as e:
                log.exception("[%s] Error en ciclo: %s", name, e)
                mqttc.publish(f"jkbms/{name}/availability", "offline", retain=True)

        time.sleep(POLL_INTERVAL)

    modbus.close()
    for name, _ in batteries:
        mqttc.publish(f"jkbms/{name}/availability", "offline", retain=True)
    mqttc.loop_stop()
    mqttc.disconnect()
    log.info("Terminado.")


if __name__ == "__main__":
    main()
