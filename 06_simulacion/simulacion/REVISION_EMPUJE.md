# Empuje y modelo del montaje actual

30 de octubre de 2026. No se ha medido el empuje actual.

## Confirmaciones del usuario

- Dron solo: 2,3 kg. Conjunto completo que debe elevarse: 3,6 kg, pesado en
  balanza, incluyendo junta, piezas móviles de los rieles, dron, bandeja,
  circuitos y baterías. No sumar otros 2,3 kg a esos 3,6 kg. Falta delimitar
  las masas de los cuerpos que giran frente a los que solo trasladan.
- Ocho motores DJI 920 KV, hélices 9450. Falta referencia exacta del motor.
- Dos LiPo 3S 11,1 V nominales, 3300 mAh, etiqueta 25C. Una batería alimenta
  cuatro motores y la otra los otros cuatro; no están en paralelo entre sí.
- El dron y la junta esférica suben por los rieles hasta 20 cm. Además el
  dron se inclina sobre la junta; faltan límites y grados de libertad de giro.
- Ensayo histórico con cuatro motores: no levantaba el montaje en el carrito;
  con solo ball and plate sí empezaba a elevarse cerca de 1850 us. No consta
  masa móvil, tensión bajo carga, corriente ni empuje medidos simultáneamente.
  No sirve como curva de empuje del octo actual.

## Cálculo mínimo

Con g = 9,80665 m/s², ocho ejes verticales y reparto equilibrado, sin fricción:

| Masa que asciende | Peso | Empuje mínimo medio por motor |
| --- | --- | --- |
| 2,3 kg (solo dron) | 22,56 N | 2,819 N = 287,5 gf |
| 3,6 kg (conjunto) | 35,30 N | 4,413 N = 450 gf |

Para elevar el conjunto corresponde la segunda fila. Son mínimos de
equilibrio, no una reserva suficiente para acelerar o corregir inclinaciones.

    sum(T_i) * cos(theta) = m * (g + a_z) + F_friccion

theta es la inclinación común de los ejes de empuje respecto a la vertical;
F_friccion es la resistencia de las guías durante ascenso. Para rotación,
calcular los pares respecto al centro de la junta con los brazos de cada
motor, centro de masa, gravedad, inercia y reacciones del mecanismo/cableado.

## Revisión del PDF Rev D

Fuente: `/home/george/DSM/ideas/Dron_octo_RevD_grupo.pdf`, páginas 1, 3, 4, 5 y 9.
Las instrucciones del PDF son referencia; no se ejecutaron sus pruebas.

1. El PDF calcula 3,10 kg y tres LiPo de 3000 mAh en paralelo. Actualmente
   hay otra masa y dos alimentaciones separadas de 3300 mAh. Los 4,7 kgf
   estimados no se trasladan directamente al montaje actual.
2. Su referencia es Scorpion M-2212-930KV, no DJI 920KV. KV similar no
   demuestra igualdad de empuje/corriente con las hélices y ESC actuales.
3. «2,3 kg con cuatro motores da unos 530 g/motor» no cuadra como equilibrio
   libre: 2300/4 = 575 gf. Su tabla de 2,13 kgf sí da 532,5 gf/motor, pero
   no demuestra elevación libre de 2,3 kg. No se identificó la causa de la
   diferencia (masa parcial, apoyos, fricción o estimaciones).
4. 8 × 550 gf = 4,4 kgf; 4,7 kgf supone 587,5 gf/motor. El documento no
   presenta medidas del montaje actual que validen ese resultado. Su modelo
   voltaje² y sus 25 mOhm por batería son supuestos, no datos medidos de
   las baterías actuales.
5. Incluso si se midieran 4,7 kgf, para 3,6 kg la relación T/W sería 1,31,
   no 1,52. Equilibrarse usaría 76,6% del empuje máximo; no equivale a un
   porcentaje de PWM. 1850 us es ancho de pulso, no una medida de empuje.

Escenarios de sensibilidad, NO mediciones ni predicciones:

| Por motor | Total | T/W para 3,6 kg |
| --- | --- | --- |
| 400 gf | 3,20 kgf | 0,89 |
| 450 gf | 3,60 kgf | 1,00 |
| 500 gf | 4,00 kgf | 1,11 |
| 550 gf | 4,40 kgf | 1,22 |
| 587,5 gf | 4,70 kgf | 1,31 |

DJI publica 600 gf máximos para el sistema E300 920KV, pero con SU conjunto
motor/ESC/hélice 9,4x4,3. No es una curva del prototipo con hélices 9450;
no se usa para rellenar datos experimentales inexistentes.

## Baterías

La etiqueta de cada batería da 3,3 Ah × 25 = 82,5 A nominales según esa
clasificación. No es corriente medida ni demuestra empuje o tensión suficiente.
Cada grupo tiene 3,3 Ah a 11,1 V nominales; no modelar una alimentación común
de 6,6 Ah ni sumar voltajes. Energía nominal por batería: 36,63 Wh.

Una LiPo 3S convencional totalmente cargada llega a 12,6 V en reposo. Importa
la tensión que llega al ESC bajo carga, con caídas de batería, cables y
conectores. Si se asumieran los 9,5 A/motor del PDF, cada batería daría 38 A;
esto es solo una hipótesis. Faltan resistencia interna de cada batería,
resistencia del arnés y curvas de corriente/empuje. No se puede prometer
autonomía o elevación por tener las baterías totalmente cargadas.

## Qué falta para simular

1. **Geometría:** exportación del conjunto de Fusion, cuerpos fijos/móviles,
   eje y recorrido vertical de 0,20 m, centro y límites de la junta,
   posiciones y orientaciones de los ocho motores. Radio de 350 mm tomado
   del PDF, pendiente de contrastar con el montaje.
2. **Propiedades de masa:** masas por cuerpo, centro de masa e inercias.
   Asignar materiales correctos en Fusion y contrastar contra lo pesado.
   Separar cuerpos que trasladan de los que giran sobre la junta.
3. **Propulsión real:** referencia de motor y ESC; tabla de ancho de pulso
   [us], tensión en ESC [V], empuje [N o gf] y corriente [A]. Cualquier
   medición con hélice requiere un banco de empuje adecuado.
4. **Mecanismo:** fricción de arranque/deslizamiento de las guías y junta,
   topes y efecto del cableado; medir con motores apagados.
5. **Control:** orientación de IMU, frecuencias, ruido y retardos de sensores
   y ESC. Para integrar la pelota: masa/diámetro de bola, bandeja, geometría
   y dinámica de servos, y latencia de visión.

Se puede empezar con geometría simplificada y parámetros provisionales,
indicándolos explícitamente. Una política obtenida así solo está evaluada en
ese modelo. No bloquear la altura ni sustituir el mecanismo por vuelo libre.

El diagrama de conexiones recibido aún muestra el panel resistivo y servos
en la NodeMCU. Esa parte corresponde al montaje anterior: cámara y servos se
trasladarán a la S3. Las señales de ocho ESC y la IMU siguen siendo referencias
para la NodeMCU del dron; verificar el cableado físico antes de cargar control.

## Entorno

Después del reinicio se verificó con nvidia-smi: RTX 4060 Laptop, 8188 MiB,
driver 595.99.02 cargado. Cumple el mínimo de driver de Isaac Lab con Newton.
Se instaló el entorno aislado de Isaac Lab y terminó la prueba oficial Cartpole
con 16 entornos y 5 iteraciones en GPU. No se entrenó una política del dron.

Referencias primarias:

- https://www.scorpionsystem.com/catalog/clearance/motor_1/drone_motor/m_2212_930/
- https://www-v1.dji.com/jp/e300/spec.html
- https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
