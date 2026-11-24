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

Registro: `vuelo_20261120_093636.csv` · 707 muestras · método de despegue: sin láser válido ni mantener.

No se detectó despegue: el colectivo máximo no alcanzó a levantar el conjunto o el láser no midió.

| Inclinación RMS en el tramo (5 s) | 5.37° | 0.020° PPO · 0.040° clásico · 0.122° con perturbaciones | ×44 respecto al caso con perturbaciones |
| Reparto de los 8 motores | de 1757 a 1839 µs | todos iguales (simétrico) | centro de empuje en x=+12 mm, y=+5 mm |

Notas: el firmware real no controla la altura (solo el ángulo); Isaac Lab sí lo hacía. El empuje real sale de suponer que al despegar el empuje iguala al peso (sin rozamiento en la guía), así que es un valor mínimo.
