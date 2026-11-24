# Láser: perfil de diagnóstico probado en ESP32-C3

Actualización del 10 de noviembre de 2026. Solo cambia la adquisición y el
diagnóstico del láser; las funciones de control angular, mezclador, escritura
PWM y cortes se compararon contra el original y son idénticas. Ver
`registros/tof_nodemcu/control_sin_cambios_20261110.json`.

Se conservan SDA=GPIO23 y SCL=GPIO22, el mapa de ocho ESC, ganancias y límites.
El láser continúa siendo solo telemetría: no regula altura.

Al reconocer el VL53L0X se solicita el perfil `VL53L0X_SENSE_HIGH_ACCURACY` de
Adafruit, con presupuesto de 200 ms y periodo de 250 ms. Este perfil desactiva
el umbral `RANGE_IGNORE_THRESHOLD` de la biblioteca; no se fuerza el estado de
las lecturas a válido. La salida exige RangeStatus=0 y una medida reciente.
La vigencia de telemetría es 750 ms para el sensor que actualiza a unos 4 Hz.
El perfil lento requiere evaluar cadencia y latencias antes de usarlo en un
futuro control vertical; aquí no se habilitó ese control.

Comandos adicionales, únicamente con modo OFF y sin calibración activa:

```text
parar
laser_reiniciar
laser_estado
```

`laser_reiniciar` permite volver a intentar la inicialización después de revisar
la conexión, sin armar motores. `laser_estado` muestra identificación, última
distancia bruta, estado, validez, antigüedad y contadores I2C. `mm=0` con
`valida=0` y `edad_ms=-1` significa ausencia de medida, no una altura de cero.

La compilación y la carga en el ESP32 clásico de `/dev/ttyUSB0` se verificaron.
Se dispone de respaldo de la fuente y de los binarios anteriores; la ubicación
está en `registros/tof_nodemcu/respaldo_20261110.json`.

En la prueba actual, el arranque detectó únicamente I2C 0x68 (IMU). El láser
no respondió en 0x29 y no pudo activarse el perfil. Hubo 21 consultas de estado
sin medidas válidas, incluso tras `laser_reiniciar`. El siguiente diagnóstico
es físico: comprobar tensión en el propio módulo y continuidad de conexiones.

Resultado: `/home/george/DSM/registros/tof_nodemcu/resultado_precision_20261110.json`.
