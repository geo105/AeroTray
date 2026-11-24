# Probar el ToF aislado con ESP32-C3 Super Mini

Desconectar los cuatro cables del sensor del carrito antes de conectarlos al C3.
Con USB desconectado, conectar por las etiquetas del módulo:

| ToF | ESP32-C3 Super Mini |
|---|---|
| VCC / VIN | 3V3 |
| GND | GND |
| SDA | GPIO4 (4) |
| SCL | GPIO5 (5) |

Usar cables cortos y alimentar únicamente el C3 por USB. Esta prueba no usa
los GPIO23/22 del ESP32 clásico. El programa configura `Wire.begin(4,5,100000)`.
INT/GPIO1 del sensor queda libre. En un breakout con pull-up de XSHUT, dejar
XSHUT/SHDN libre; si ese pin está conectado a tierra, el sensor no responderá.
La existencia del pull-up depende del módulo: verificar su modelo antes de
atribuir una ausencia de respuesta a una avería.

En Arduino IDE: placa **ESP32C3 Dev Module**, **USB CDC On Boot = Enabled**,
Monitor serie **115200**. Abrir `prueba_tof_esp32c3.ino` y seleccionar el puerto
del C3, no el CP2102 `/dev/ttyUSB0` que corresponde al dron.

El programa busca direcciones I2C cada tres segundos si no logra iniciar:

- `Responde 0x29`: existe comunicación I2C con el módulo.
- `0xC0=0xEE`: corresponde al identificador esperado de VL53L0X.
- Otro modelo: informa la identificación y no lo configura como VL53L0X.
- CSV con `valida=1`: distancia aceptada por la API, en milímetros. Apuntar
  a una superficie a 10–30 cm y moverla para comprobar cambios coherentes.
- Dirección detectada pero sin medidas válidas: revisar diagnóstico de
  inicialización/rango, superficie, obstrucción y alimentación.
- Sin respuesta I2C: revisar las cuatro conexiones, alimentación y XSHUT.
  No basta para concluir que el sensor está averiado.

## Diagnóstico realizado el 10 de noviembre

Se confirmó ESP32-C3 en `/dev/ttyACM0` y se cargó/verificó este programa. El
sensor se identifica e inicializa como VL53L0X. Con blanco fijo, el perfil
normal produjo 207/207 muestras con `RangeStatus=2` (Signal Fail).

La comparación de configuraciones dio:

| Perfil | Presupuesto | Lecturas válidas |
|---|---:|---:|
| Normal (`n`) | ~33 ms | 0/122 |
| Normal lento (`l`) | ~200 ms | 0/32 |
| Alta precisión de Adafruit (`p`) | ~200 ms | 32/32 |

El cambio relevante entre `l` y `p` es la desactivación de
`VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD` que realiza el perfil
`VL53L0X_SENSE_HIGH_ACCURACY` de la propia biblioteca. La señal total fue
aproximadamente 3.3 MCPS para 181 SPAD, unos 0.018 MCPS/SPAD, inferior al umbral
fijo del perfil normal (~0.0345). No se sustituyó `RangeStatus` ni se aceptaron
muestras marcadas como inválidas por el perfil activo. Sus comprobaciones de
señal final y sigma permanecen activas.

En alta precisión: media 129.8 mm, desviación 1.2 mm y rango 126–133 mm durante
8 s. Esto confirma comunicación y medición en esa condición; no calibra la
distancia absoluta ni prueba el alcance de 2 m.

Para que la prueba sea reproducible tras reiniciar, el sketch arranca ahora
en perfil `p`: presupuesto 200 ms y periodo 250 ms (aproximadamente 4 Hz).
Desde el monitor serie se puede enviar `n`, `l`, `p` o `i` (información).
Este perfil lento es para diagnóstico; no se cambió el firmware del dron.

Registros y comparación: `/home/george/DSM/registros/tof_c3/`.

Si funciona en el C3 con cables cortos, queda comprobado que el sensor puede
comunicarse y medir en ese montaje. Después contrastar cableado, alimentación
y configuración del carrito; no se modifica su control para esta prueba.

Para compilar con las herramientas locales:

```bash
/home/george/DSM/herramientas/arduino-cli/arduino-cli \
  --config-file /home/george/DSM/herramientas/arduino-cli.yaml compile \
  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  --build-path /home/george/DSM/tmp/build_prueba_tof_esp32c3 \
  /home/george/DSM/firmware/prueba_tof_esp32c3
```

Referencias: [I2C Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html),
[pines de breakout VL53L0X de Adafruit](https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/pinouts).

### Alcance de las comprobaciones posteriores

La segunda posición dio 34/34 muestras aceptadas pero 42.4 mm, aunque se había
pedido alejar el blanco. No se trató como validación de exactitud. Sin regla,
una colocación aproximada posterior produjo 41–145 mm, con las últimas doce
muestras en 139.9 ± 1.6 mm. No se conoce la distancia física real.

Se cargó y verificó la versión que arranca en alta precisión. Tras reiniciar
produjo 41/41 muestras aceptadas, 64–86 mm. La posición no tuvo referencia
medida, por lo que esto verifica arranque y obtención de datos, no exactitud
ni repetibilidad entre posiciones. No se declara el sensor perfectamente
calibrado ni se descarta una degradación óptica/alimentación.

Resumen completo: `/home/george/DSM/registros/tof_c3/resumen_prueba_c3_20261110.json`.
