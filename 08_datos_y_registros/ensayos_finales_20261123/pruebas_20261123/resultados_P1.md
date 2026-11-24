# P1, resultados (23/11/2026)

## Rango de la rótula (inclinación máxima del dron respecto a la base)

| Dirección | Motor hacia el que se inclina | Ángulo máximo |
|---|---|---|
| Adelante | M8 | 35° |
| Atrás | M6 | 40° |
| Derecha | M5 | 47° |
| Izquierda | M7 | 53° |

Todas las direcciones superan con holgura los 10° de cada quiebre de la rampa. **La rótula no hace tope en la rampa.** Los cortes a 12° del 18/11 no fueron un choque mecánico: el dron se inclinó por la dinámica en los quiebres (golpe + péndulo invertido con poca autoridad en ralentí).

## Vuelco estático del robot completo (dron trabado, rotores apagados)

| Ensayo | Equivale en la rampa a | Ángulo de vuelco |
|---|---|---|
| Levantando adelante: se cae hacia atrás entre 21° y 22° | Subir de frente | **≈ 21,5°** |
| Levantando atrás: se cae hacia adelante entre 30° y 31° | Bajar de frente | **≈ 30,5°** |

Centro de masa calculado con **L = 0,22 m medida** (entre los centros de las ruedas motriz y omni):

- **x_G ≈ 88 mm** por delante del eje motriz. Confirma el valor del CAD (81 mm) y descarta los 180 mm del ensayo del 05/10.
- **h_G ≈ 224 mm** sobre el apoyo.

Márgenes en la rampa, con el dron apagado:

| Rampa | Subiendo | Bajando |
|---|---|---|
| 10° | 11,5° | 20,5° |
| 20° | **solo 1,5°** | 10,5° |

Una rampa de 20° no es segura sin ayuda del dron.



## Otros datos medidos

| Dato | Valor |
|---|---|
| Masa total con todas las baterías | **10,4 kg** (base y todo lo demás: 6,8 kg; módulo aéreo: 3,6 kg) |
| Distancia entre ejes (L) | **0,22 m** |
| Radio de las ruedas | ≈ 5 cm (las dos) |
| Recorrido vertical del dron en la guía | **≈ 14 cm**. El CAD decía 77 mm: hay que corregirlo en el paper (`d_max` en `params.py`) |
| Rampa del ensayo | **Solo 10°** (indicación del profesor). La de 20° queda solo como análisis del modelo |

Del vuelco con el dron trabado se deduce el centro de masa de la base sin el módulo (6,8 kg): **x ≈ 88 mm, h ≈ 81 mm**. Son las baterías y la electrónica, que van abajo.

## Efecto del empuje del dron sobre el vuelco (modelo calibrado con P1)

| Modo del dron | Colectivo | Vuelco subiendo | Vuelco bajando | Margen en 10°, subiendo / bajando | μ para subir 10° |
|---|---|---|---|---|---|
| Apagado | — | 21,5° | 30,5° | 11,5° / 20,5° | 0,25 |
| Ralentí (f ≈ 0,12) | 1300 µs | 22,6° | 31,9° | 12,6° / 21,9° | 0,26 |
| **Modo rampa (f = 0,5)** | ~1540 µs | **27,8°** | **38,3°** | **17,8° / 28,3°** | 0,27 |
| Flotando (f = 1) | ~1750 µs | 47,6° | 58,6° | 37,6° / 48,6° | 0,30 |

**Lectura:**

- En la rampa de 10° el robot es estable en todos los modos.
- El modo rampa aumenta el margen de vuelco subiendo en un **55 %** (de 11,5° a 17,8°) y ayuda a subir un poco más rápido.
- El modulo aereo esta casi encima del centro de masa (88 mm), asi que descargar su peso no mueve el CM hacia adelante ni hacia atras: lo **baja**. Por eso mejora el vuelco en los dos sentidos.
- La tracción sobra: μ necesaria de 0,25–0,30, frente a 0,5–0,8 disponible.

## Ciclo de rampa de 10° (1 m + 0,5 m + 1 m) con la masa real de 10,4 kg (ciclo de trabajo del 15 %)

| Modo | Subida | Meseta | Bajada | Total | Carga de la base |
|---|---|---|---|---|---|
| Ralentí | 2,9 cm/s, 35 s, 2,4 A | 7,2 cm/s, 7 s | 11,6 cm/s, 9 s (el PI la frena) | **≈ 50 s** | 27 mAh |
| Modo rampa | **3,5 cm/s**, 28 s, 2,2 A | 7,3 cm/s, 7 s | 11,1 cm/s, 9 s | **≈ 44 s** | 21 mAh |

Para validar hoy, cronometren la subida en ralentí y en modo rampa: el modelo predice 35 s frente a 28 s (+21 % de velocidad).

## Caracterización del láser VL53L0X (sobre la base, sin hélices)

Firmware v11 con largo alcance, medidas débiles aceptadas y filtro de mediana de 3.

| Dato | Valor |
|---|---|
| Distancia real del sensor al piso (medida a mano) | ≈ 441–460 mm |
| Media de las lecturas (25 diagnósticos) | 441 mm, es decir, −19 mm (−4 %) |
| Desviación | ±52 mm (rango 306–539 mm) |
| Señal | 0,06–0,32 MCPS |
| Luz ambiente | 0,7–0,99 MCPS |
| Medidas aceptadas | ~35 % (6–9 por segundo) |
| Medidas válidas antes del filtro (vuelos del 20/11) | **0 %** |

**Conclusión:** el sensor funciona y mide el piso. La media es correcta, pero cada lectura es ruidosa, porque el piso refleja poco y la luz ambiente es alta.

- **Sirve para** registrar la altura y detectar el despegue.
- **No sirve todavía para** controlar la altura: «Mantener» solo fija el colectivo mientras `estable=0`.
- **Mejora propuesta:** un blanco mate de al menos 20 × 20 cm bajo el sensor y sombra.

## Calibración del láser con regla (blanco: hoja A4 sobre la base)

| Posición del dron en la guía | Regla | Lectura sin corregir | Corregida (firmware) |
|---|---|---|---|
| Abajo | 330 mm | 394 mm | 316–322 mm (Kalman 317–324) |
| Arriba (+120 mm) | 450 mm | 523 mm | ≈ 450 mm (según la fórmula; falta verificarlo en vivo) |

**Fórmula:** distancia real = 0,930 × lectura − 36,5 mm.

**Cuidado:** vale solo para este montaje. Si se mueve la hoja o el sensor, hay que volver a calibrar.

**Antes y después:**

| | Sin hoja, piso del laboratorio | Hoja + filtro + Kalman + calibración |
|---|---|---|
| Medidas válidas | 0 % | ~100 % |
| Dispersión | ±52 mm | ±5–20 mm |
| Error de la media | +64 mm | ≈ −10 mm |
| Incertidumbre del Kalman | — | ~23 mm |
