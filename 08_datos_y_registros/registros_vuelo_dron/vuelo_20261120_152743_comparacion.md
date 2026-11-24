# Vuelo en la guía frente a Isaac Lab

## Predicciones antes de volar

| Fuente | Empuje por motor | Margen | Hover en PWM |
|---|---|---|---|
| Isaac Lab (trim del modelo, 3,6 kg) | 4.41 N/motor = 450 gf | 74 % de su tope ficticio de 6 N | 1898 µs con la curva del firmware |
| Curva provisional del firmware (3.60 kg) | 4.41 N/motor | 82 % de 5,40 N (2000 µs) | 1898 µs |
| Curva del firmware a 1800 µs | 3.53 N/motor | levanta 2.88 kg | — |
| Curva del firmware a 1950 µs (máximo) | 4.90 N/motor | levanta 4.00 kg | — |
| Auditoría de alimentación (A2212 + 9450, 3,10 kg) | 3,80 N/motor | ≈ 77 % de acelerador | ≈ 1770 µs |
| Ensayo histórico (4 motores, solo ball and plate) | — | empezó a subir | ≈ 1850 µs |

Isaac Lab también predijo, con control: inclinación RMS 0.020° (PPO), 0.122° con perturbaciones y 0.040° con el control clásico; altura RMS 0.10 mm (PPO) en 77 mm de guía. En el modelo el reparto es simétrico: los 8 motores necesitan el mismo empuje (diferencia máxima 6 mN).

## Ensayo real

Registro: `vuelo_20261120_152743.csv` · 1239 muestras · método de despegue: orden mantener (sin láser válido).

| Magnitud | Real | Isaac Lab / modelo | Diferencia |
|---|---|---|---|
| PWM medio al despegar | 1775 µs | 1898 µs (trim de Isaac con la curva) | -124 µs |
| Empuje máximo por motor (2000 µs) | 7.18 N | 6.0 N (tope de Isaac) · 5,40 N (curva) | +20 % vs Isaac |
| Relación empuje/peso a 2000 µs | 1.63 | 1.36 | |
| Inclinación RMS en el tramo (5 s) | 1.55° | 0.020° PPO · 0.040° clásico · 0.122° con perturbaciones | ×13 respecto al caso con perturbaciones |
| Reparto de los 8 motores | de 1589 a 1666 µs | todos iguales (simétrico) | centro de empuje en x=+9 mm, y=+2 mm |

Notas: el firmware real no controla la altura (solo el ángulo); Isaac Lab sí lo hacía. El empuje real sale de suponer que al despegar el empuje iguala al peso (sin rozamiento en la guía), así que es un valor mínimo.
