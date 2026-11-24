# Pruebas del 23/11: métricas, prueba clave del dron y cambios de firmware

Este documento **complementa** el protocolo de Cowork (`laptop2/Protocolo de pruebas finales AeroTray — 23 11.md`), que define el orden y el procedimiento de cada escenario. Aquí se agregan cuatro cosas:

1. el argumento clave del dron y la prueba que lo demuestra (prueba **D**);
2. la definición exacta de cada métrica y su criterio de aceptación;
3. el firmware nuevo para las rotaciones (dron v10) y para el seno del plato (bandeja);
4. cómo calcular todo automáticamente con `metricas_pruebas.py`.

---

## 1. El punto clave: por qué un dron y no solo servos

> **El dron es la única etapa de nivelación cuyo actuador no empuja contra la base, sino contra el aire.**

**Servos fijos a la base.** Un plato movido solo por servos, o un cardán motorizado, genera su par entre la base y el plato. Eso tiene tres consecuencias:

- toda la inclinación de la base (la pendiente y los golpes al entrar a la rampa) la tiene que restar el mismo servo, con su recorrido;
- por acción y reacción, los golpes de la base llegan al plato a través de los engranajes;
- la referencia de nivel es la base, no la gravedad.

Con los S3003, ±220 µs equivalen a unos **±10,8° de plato**. En la rampa de 10°, nivelar consume **~204 µs (93 % del recorrido)** y quedan ~0,8° para equilibrar la bola. En 20° el plato ni siquiera se puede nivelar.

**Dron sobre la rótula.** La rótula no transmite par, solo fuerza. El dron genera su par contra el aire y se referencia a la gravedad con su IMU. Así:

- la pendiente de la base **no entra** al plato;
- el dron nivela los ángulos grandes y lentos (pendiente, transición de rampa);
- los servos quedan libres para lo pequeño y rápido (la bola).

Es una **separación de funciones por escalas**: cada actuador trabaja en su rango óptimo. Además, el empuje del dron es un **lastre ajustable** que ningún cardán tiene, porque descarga peso del módulo según el colectivo.

**Evidencia que ya tenemos (18/11, dron libre a 1300 µs):**

| Recorrido | Uso medio del recorrido del servo | Sesgo mediano |
|---|---|---|
| Rampa (7 recorridos) | 36–57 % | ±60 µs como máximo |
| Llano (4 recorridos) | 43–63 % | — |

En la rampa los servos **no trabajaron más que en el llano**. Si la pendiente hubiera llegado al plato, el sesgo habría sido de ~200 µs.

**Costo honesto:**

- el aire de los rotores y la vibración suben el error de la bola en reposo de 4,4 a 18,9 mm;
- la energía alcanza para 3–7 min de vuelo;
- las aceleraciones lineales sí pasan por la rótula.

Eso se dice en el paper; no se oculta.

---

## 2. Prueba D (la prueba clave): "solo servos" frente a "dron"

Se hace dentro del escenario E4 (rampa de 10°), en el mismo tramo, a la misma velocidad baja y con la bola en el centro.

| Condición | Cómo se arma | Qué representa |
|---|---|---|
| **D-A, solo servos** | Dron **trabado nivelado a la base** (precintos o cinta) y motores apagados | Lo que habría pasado con una bandeja de servos fija a la base |
| **D-B, dron (modo base)** | Rótula libre y dron armado en ralentí (1300 µs) con el firmware **v10** | La solución del proyecto |

- Tres ciclos completos de rampa (subida, meseta y bajada) por condición. Si D-A pierde la bola, **eso también es un resultado**: se anota y se sigue.
- **Celular como inclinómetro** (phyphox → «Inclinación», exportar CSV):
  - en D-A, sobre el marco del dron, para medir la inclinación real del plato;
  - en D-B, la inclinación del dron sale de su propio registro (`vuelo_*.csv`, 10 Hz). Si hay un segundo celular, póngalo sobre el chasis de la base para medir su cabeceo.
- **Qué debe salir:** en D-A, el uso del servo sube hacia el 90–100 %, hay saturación y aumentan el error y las pérdidas de la bola en la subida y en la bajada. En D-B, el uso se mantiene como en el llano.

**Métricas de la prueba D:**

| Métrica | Definición |
|---|---|
| Mejora relativa del error | Δe = (e_RMS,A − e_RMS,B) / e_RMS,A × 100 % |
| Reducción de la saturación del servo | S_A − S_B (puntos porcentuales) |
| Pérdidas de la bola | Cantidad en A y en B |
| Transmisibilidad angular base → plato | T = RMS(θ_plato) / RMS(θ_base) en los quiebres de la rampa. Con el dron trabado, T ≈ 1 por definición; con el dron libre se espera T ≪ 1 |

---

## 3. Métricas y criterios de aceptación

`e(t) = √(x² + y²)` es la distancia de la bola al centro del plato, en mm. Se calcula solo mientras el PID está activo y la bola se ve.

**R_útil = 110 − 13,5 = 96,5 mm** es el radio útil: medio lado corto del plato menos el radio de la bola.

### Bola (escenarios E3, E4, E5 y D)

| Métrica | Fórmula / definición | Unidad | Criterio de aceptación |
|---|---|---|---|
| Error mediano, medio, RMS y P95 | Estadísticos de e(t) | mm | — |
| **Error relativo** | e_RMS / R_útil × 100 | % | E3 apagado ≤ 10 %; E3 ralentí ≤ 30 %; E4/E5 ≤ 45 % |
| Varianza | σ² de e(t) | mm² | La guía la pide en E3 |
| **Efectividad** | % del tiempo con e ≤ 30 mm (también se reportan ≤ 10 y ≤ 50 mm) | % | E3 ≥ 90 %; E4/E5 ≥ 60 % |
| Pérdidas de la bola | Tramos > 1 s con el PID activo y la bola sin ver | n.° | 0 en E3; ≤ 1 por ensayo en E4/E5 |
| **Tasa de éxito** | Ensayos sin pérdida / ensayos totales | % | ≥ 80 % |
| Uso del recorrido del servo | Media de max(\|u_x − n_x\|, \|u_y − n_y\|) / 220 µs | % | — (comparación D-A frente a D-B) |
| Saturación | % del tiempo con un servo a ≥ 95 % del recorrido | % | ≤ 10 % |
| Bola vista | Muestras con bola / muestras con PID | % | ≥ 90 % |

### Recuperación (E3: sueltas desde 20 y 50 mm)

| Métrica | Definición | Criterio de aceptación |
|---|---|---|
| Tiempo de recuperación t_r | Desde la suelta hasta que e ≤ banda y se mantiene 2 s. Banda: 5 mm con el dron apagado; 20 mm en ralentí | 20 mm: ≤ 3 s; 50 mm: ≤ 5 s |
| Sobrepaso | Cuánto cruza al otro lado del centro, en la dirección de la suelta (mm) | ≤ 50 % de la distancia inicial |
| Éxito | Recupera sin caerse | ≥ 80 % (4 de 5) |

### Base (E1)

| Métrica | Definición | Criterio de aceptación |
|---|---|---|
| **Error de traslación** | \|d_medida − d_pedida\| / d_pedida × 100 | ≤ 5 % |
| Desviación lateral | mm al final del recorrido | ≤ 20 mm en 50 cm |
| Error de la odometría | \|d_odometría − d_medida\| / d_medida × 100 | Se reporta |
| **Error de giro** | \|ángulo medido − 360°\| / 360° × 100 | ≤ 5 % |
| Repetibilidad | Desviación estándar de 3 repeticiones | Se reporta |
| Velocidad | Distancia / tiempo (cronómetro o video) | Se reporta (el modelo predice 0,073 m/s al 15 %) |

### Plataforma (E2)

| Métrica | Definición | Criterio de aceptación |
|---|---|---|
| Relación de amplitud | A_medida / A_pedida, medida en el video en cámara lenta o con el celular sobre el plato | — |
| **Frecuencia máxima útil** | Última frecuencia con una relación ≥ 70 % | ≥ 1 Hz con ±100 µs (el modelo predice ~3,5 Hz con ±5°) |
| Error de estimación estático | \|posición medida − marca\| en las 9 marcas | ≤ 5 mm |

### Dron (E4, E5, D, vuelo)

| Métrica | Definición | Criterio de aceptación |
|---|---|---|
| Inclinación RMS y máxima del dron | De `vuelo_*.csv` | Máxima < 12° (sin corte) |
| Cortes del firmware | Cantidad por ensayo | 0 en modo base |
| Esfuerzo | Orden media de los motores (µs) | Se reporta |

### Energía (P7)

| Métrica | Criterio |
|---|---|
| Tensión en reposo antes y después de cada bloque y minutos con los rotores | Cambiar de batería por debajo de 11,4 V |

Los criterios son propuestos por el equipo a partir de los resultados del 17 y 18/11. En el paper se presentan como **requisitos y resultado**: cumple, parcial o no cumple.

---

## 4. Firmware nuevo (grabar **antes** de empezar, con las LiPo desconectadas)

**a) Dron v11** (reemplaza al v10). El dron es independiente: mantiene el rumbo inicial con una ventana de ±20° sobre la base, sin cortar en los giros, y mantiene la altura con el láser cuando este mide. El láser pasa a modo largo alcance y tiene diagnóstico. Detalles en `PLAN_PRUEBAS_Y_VALIDACION.md`, sección 8. Mientras el colectivo está en ralentí (≤ 1400 µs, montado sobre la base):

- el dron **no controla el yaw y no corta por yaw** (20° ni 35°/s), porque la base lo arrastra al girar;
- roll, pitch y el corte de 12° siguen igual.

Por encima de 1400 µs, el rumbo de referencia se reinicia y todo es igual que en v9. Esto permite hacer **E5 con el dron encendido**. Se graba desde la laptop:

```bash
~/DSM/firmware/cargar_vuelo_guia_desde_pi.sh
```

Prueba en banco antes de usarlo: dron en ralentí sobre la base y la base girando 90° despacio. No debe cortar.

**b) Bandeja con seno (E2).** Se graba la aplicación del ESP32-S3 y se sube `seguidor.py` con la ruta `/seno`:

```bash
~/DSM/firmware/cargar_bandeja_seno_desde_pi.sh
```

Para volver al firmware anterior se agrega `--volver`. Antes de grabar, servos sin alimentación o bandeja sin la bola.

Desde el navegador (celular o laptop):

| Acción | URL |
|---|---|
| Barrido completo en x: 0,5–1–1,5–2–3–4 Hz, 10 s cada uno (72 s en total) | `http://<IP_PI>:8080/seno?eje=0&amp=100&barrido=1` |
| Barrido en y | `eje=1` |
| Círculo con los dos ejes | `eje=2` |
| Un seno fijo | `http://<IP_PI>:8080/seno?eje=0&amp=100&f=2` |
| Detener | `http://<IP_PI>:8080/seno?amp=0`, o pasar a PID o neutro |

Los pulsos enviados quedan en `registro_*.csv` (estado 12 = seno). La amplitud real se mide en el video.

---

## 5. Antes de la primera prueba: sincronizar relojes

La Pi tuvo la hora mal el 20/11, y la sincronización es lo que permite cruzar los videos con los datos.

- [ ] En la Pi, `date`. Anoten al mismo tiempo la hora del celular. La diferencia (Pi − celular, en s) es el `--desfase` del script.
- [ ] Al inicio de cada ensayo, la cámara graba 2 s la hora del celular y alguien dice el código.

---

## 6. Hoja de ensayos y cálculo automático

Llenen `hoja_ensayos.csv` durante las pruebas: código, hora de inicio, hora de fin (del celular), tipo y notas. Para las sueltas, la hora de inicio es el momento de la suelta.

| Tipo | Cuándo usarlo |
|---|---|
| `equilibrio` | Ensayos normales |
| `recuperacion20_OFF`, `recuperacion50_OFF` | Sueltas con el dron apagado |
| `recuperacion20_ON`, `recuperacion50_ON` | Sueltas con el dron en ralentí |

Al terminar, se copian los registros de la Pi y se corre el script:

```bash
rsync -av <usuario>@<IP_PI>:pi_control/registros/ ~/DSM/pruebas_20261123/registros/
```

```bash
cd ~/DSM/pruebas_20261123 && python3 metricas_pruebas.py hoja_ensayos.csv registros --desfase <s>
```

Sale `resultados/metricas.csv` (todas las métricas por ensayo) y una gráfica por ensayo: x, y, error con las bandas de 10/30/50 mm y el uso del servo. Esas gráficas son las que pidió el profesor junto al video.

Las métricas de la base (E1) y de la plataforma (E2) se miden a mano o en el video y se anotan en las tablas del protocolo de Cowork.
