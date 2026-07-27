# jkbms2mqtt

Puente **multi-batería** JK BMS (v19) → MQTT con autodiscovery para Home Assistant.

Lee uno o varios JK BMS a través de un gateway RS485→TCP (EW11) por **Modbus TCP** y
publica cada batería como un **dispositivo independiente** en Home Assistant.

## Características

- Multi-batería vía `BATTERIES=nombre:unit_id,nombre2:unit_id2`
- Topics separados por batería: `jkbms/<nombre>/state` (no colisiona con stacks que usen `jkbms/state`)
- Autodiscovery HA por batería (sensores + binarios)
- Campos calculados: min/max/avg celda, delta en mV
- Dos modos de lectura: `block` (rápido) o `single` (compatible con EW11 que limitan el tamaño de petición)
- Se ejecuta como usuario no root

## Campos publicados

Voltaje, corriente, potencia, SOC, SOH, celdas individuales, min/max/avg/delta,
resistencias de celda, temperaturas (MOS, T1, T2), corriente de balanceo,
capacidad restante/nominal, ciclos, alarmas (bitfield) y estado de FET
(charging / discharging / balancing).

> **Nota de validación:** los campos básicos (celdas, voltaje, corriente, potencia,
> SOC, temperaturas, balanceo) están validados contra el stack original. Los campos
> "extra" (SOH, ciclos, capacidad, alarmas, FET) provienen del mapa Modbus del JK v19
> y **conviene verificarlos con `DEBUG=true`** en la primera ejecución por si hay
> desfase de direcciones según firmware.

## Despliegue en Portainer (Web editor, sin GitHub)

1. Portainer (VM 200) → **Stacks → Add stack → Web editor**
2. Pega el contenido de `docker-compose.yml`
3. Como usa `build: .`, necesitas que Portainer tenga acceso a los ficheros
   (`Dockerfile`, `jkbms2mqtt.py`, `requirements.txt`). Si despliegas por Web editor
   sin repo, usa en su lugar el método **Repository** apuntando a tu GitHub, o
   construye la imagen aparte y referencia `image:`.
4. Ajusta variables (broker, IP EW11) y despliega.

## Despliegue por repositorio Git

1. Sube los 5 ficheros a tu repo (`myckiecat/jkbms2mqtt`)
2. Portainer → **Stacks → Add stack → Repository**
3. URL del repo + `docker-compose.yml` como compose path
4. Deploy

## Validación (orden recomendado)

1. Arranca con `BATTERIES=battery_1:1` y `DEBUG=true`
2. Revisa logs: ¿lecturas OK o el EW11 rechaza los bloques? (si falla → `READ_MODE=single`)
3. Comprueba el JSON publicado: ¿valores razonables en todos los campos?
4. Confirma que aparecen los sensores en HA (dispositivo "JK BMS battery_1")
5. **Solo entonces** añade la esclava: `BATTERIES=battery_1:1,battery_2:2`
   y verifica que la batería 2 responde por el bus (slave id 2 a través de la maestra)

## Variables de entorno

| Variable | Por defecto | Descripción |
|---|---|---|
| `MODBUS_IP` | `192.168.1.100` | IP del EW11 |
| `MODBUS_PORT` | `502` | Puerto Modbus TCP |
| `MQTT_HOST` | `192.168.1.207` | Broker MQTT |
| `MQTT_PORT` | `1883` | Puerto MQTT |
| `MQTT_USER` / `MQTT_PASS` | vacío | Credenciales MQTT (si aplica) |
| `BATTERIES` | `battery_1:1` | Lista `nombre:unit_id` separada por comas |
| `READ_MODE` | `block` | `block` o `single` |
| `POLL_INTERVAL` | `10` | Segundos entre lecturas |
| `DEBUG` | `false` | Log detallado + JSON en logs |
| `HA_DISCOVERY_PREFIX` | `homeassistant` | Prefijo discovery HA |

## Aviso sobre el bus RS485

El EW11 y el bus RS485 son de **un solo interlocutor a la vez**. No ejecutes varios
lectores simultáneos contra el mismo EW11 (ni este contenedor duplicado, ni junto al
stack original) o tendrás timeouts y lecturas cruzadas. El Cerbo habla por CAN, que es
un canal distinto, así que ese no interfiere con el Modbus.
