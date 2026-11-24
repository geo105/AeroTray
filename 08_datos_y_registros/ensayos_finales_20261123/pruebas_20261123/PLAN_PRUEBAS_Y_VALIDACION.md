# Plan de pruebas por configuración y validación: modelo, simulación y medición (23/11)

Completa el documento `METRICAS_Y_PRUEBAS.md` (definición de las métricas y prueba D) y el protocolo de Cowork (procedimiento detallado de cada escenario). Todos los números de este documento salen de `calculos_sin_encender.py` y de `metricas_pruebas.py`, que usan el mismo modelo que el paper (`scripts_modelo/params.py`).

---

## 1. Las pruebas, de la configuración más simple a la completa

La idea es agregar **una sola cosa a la vez**. Así, cada aumento de error se puede atribuir a lo que se agregó: primero la bandeja sola, después la base en movimiento y al final el dron. La prueba D compara directamente C2 con C3.

| Config. | Qué está encendido | Qué se agrega respecto a la anterior |
|---|---|---|
| **C0** | Nada (mediciones con el robot apagado) | — |
| **C1** | Solo el *ball and plate*: base quieta, dron apagado y trabado | La bandeja con su PID |
| **C2** | Carrito + *ball and plate*, dron trabado (equivale a «solo servos») | El movimiento de la base |
| **C3** | Todo: dron libre en ralentí, firmware v10 en modo base | El dron nivelando |

### Ensayos y tiempos

| Config. | Prueba | Ensayos | Duración de cada uno | Tiempo con montaje |
|---|---|---|---|---|
| C0 | P1, centro de masa: levantar hacia atrás y hacia adelante, 3 repeticiones de cada uno | 6 | 1 min | 20 min |
| C0 | Rango de la rótula en 4 direcciones; masa total con balanza de baño | 5 | 1 min | 10 min |
| C0 | Tensión en reposo de las 6 baterías; hora de la Pi frente al celular | 7 | — | 5 min |
| **C0 total** | | **18** | | **35 min** |
| C1 | E2, barrido de seno en x, y y círculo (±100 µs, 0,5–4 Hz) | 3 | 72 s | 10 min |
| C1 | E2, estimación de la bola: 9 marcas de 5 s y 2 rodadas con regla | 11 | 5–10 s | 10 min |
| C1 | E3, equilibrio en el centro | 3 | 60 s | 5 min |
| C1 | E3, recuperación desde 20 y 50 mm, 5 sueltas cada una | 10 | ~15 s | 10 min |
| **C1 total** | | **27** | | **35 min** |
| C2 | E1, traslación de 25 y 50 cm en llano, 3 velocidades, 3 repeticiones | 18 | ~20 s | 25 min |
| C2 | E1, traslación de 25 y 50 cm en la rampa de 10°, velocidad baja, 3 repeticiones | 6 | ~20 s | 10 min |
| C2 | E1, giro de 360° sobre el punto medio y sobre una rueda, 3 velocidades, 3 repeticiones | 18 | ~20 s | 25 min |
| C2 | E4-A, llano 1 m, 3 velocidades | 3 | ~30 s | 6 min |
| C2 | **D-A, ciclo completo de rampa con el dron trabado (solo servos)** | 3 | ~60 s | 10 min |
| C2 | E5-A, giros con la bola: 2 tipos, 3 velocidades | 6 | ~30 s | 10 min |
| **C2 total** | | **54** | | **86 min** |
| C3 | Prueba en banco del v10: girar la base 90° con el dron en ralentí; no debe cortar | 1 | 30 s | 5 min |
| C3 | E3-ON, equilibrio con los rotores girando (3) y recuperación (10) | 13 | 15–60 s | 15 min |
| C3 | E4-B, llano 1 m, 3 velocidades | 3 | ~30 s | 6 min |
| C3 | **D-B, ciclo completo de rampa con el dron libre** | 3 | ~60 s | 10 min |
| C3 | E5-B, giros con el dron encendido: 2 tipos, 3 velocidades | 6 | ~30 s | 10 min |
| C3 | P7, subida en ralentí (3) y en modo rampa ~1540 µs (3) | 6 | ~25 s | 12 min |
| C3 | Vuelo en la guía (demostración, opcional) | 1–2 | 60 s | 10 min |
| **C3 total** | | **33–34** | | **68 min** |

| Concepto | Tiempo |
|---|---|
| Preparación y seguridad | 30 min |
| Grabar el firmware y probarlo | 15 min |
| Cambios de batería y pausas | 20 min |
| Respaldo de datos | 15 min |
| **Total del día** | **≈ 132 ensayos, unas 5 h** |

**Versión mínima**, si falta tiempo (unas 3 h 30 min): en E1 hacer una sola repetición a velocidad media y alta, y las tres solo a velocidad baja (30 ensayos menos). **No recorten C1, D-A ni D-B**: son la evidencia central.

**Baterías durante el día:**

- **LiPo:** el dron solo consume en C3. Unos 45 min en ralentí a ~1,2 A por rama son ~0,9 Ah de los 2,6 Ah útiles. Con los 6 ensayos de P7 y los 2 vuelos de 1 min se llega a ~1,7 Ah. **Un par de LiPo alcanza**; el segundo par queda de reserva.
- **NiMH de la base:** C2 y C3 suman unos 40 min de marcha a 1–2 A, o sea ~1,3 Ah de los 2,4 Ah útiles. Lleguen a C3 con la NiMH cargada o cárguenla durante C1.

---

## 2. Lo que se calcula sin encender (`calculos_sin_encender.py`)

### 2.1 Rampa: pendiente máxima según el modo del dron

El resultado depende del centro de masa de la base, que **P1 mide hoy**. Hay dos hipótesis. Después de P1 se corre:

```bash
python3 calculos_sin_encender.py --xg <x medido> --hg <h medido>
```

**Hipótesis CAD (x = 81 mm):**

| Modo del dron | Colectivo (µs) | Vuelco subiendo (°) | Vuelco bajando (°) | μ para subir 10° | μ para subir 20° |
|---|---|---|---|---|---|
| Apagado | — | 14,9 | 22,6 | 0,23 | 0,34 |
| Ralentí (f = 0,12) | 1300 | 15,4 | 23,4 | 0,23 | 0,34 |
| Modo rampa (f = 0,5) | 1537 | 17,6 | 26,8 | 0,24 | 0,36 |
| Flotando (f = 1) | 1752 | **26,3** | **39,5** | 0,26 | 0,43 |

**Hipótesis del ensayo del 05/10 (x = 180 mm):**

| Modo del dron | Vuelco subiendo (°) | Vuelco bajando (°) | μ para subir 10° |
|---|---|---|---|
| Apagado | 23,5 | 13,9 | 0,32 |
| Modo rampa | 30,1 | 13,6 | 0,39 |
| Flotando | 47,7 | 12,4 | 0,65 |

**Cómo leerlo:**

- Con la hipótesis CAD, la rampa de 10° tiene solo ~5° de margen de vuelco subiendo con el dron apagado. Con el dron sosteniendo su peso, el margen sube a 16°. **La rampa de 20° solo es posible con el dron sosteniendo casi todo su peso.** Ese es el argumento cinemático y estático del dron.
- Con la otra hipótesis, el límite está en la bajada (13,9°), y ahí el dron no ayuda.

El caucho sobre madera da μ ≈ 0,5–0,8, así que en todos los casos sobra tracción, salvo flotando en 20° con la hipótesis del 05/10.

### 2.2 Recorrido del servo que se gasta solo en nivelar el plato

Con ±220 µs, el plato se inclina ±10,8°.

| Pendiente | Solo servos: % del recorrido | Sesgo necesario | Con dron (0,55–1,7° residuales) |
|---|---|---|---|
| 5° | 46 % | 102 µs | 5–16 % |
| **10°** | **93 %** | **204 µs** | **5–16 %** |
| 15° | 139 % (imposible) | 306 µs | 5–16 % |
| 20° | 185 % (imposible) | 408 µs | 5–16 % |

**Lo que se midió el 18/11 en la rampa** (dron libre): uso medio del 36–57 % y sesgo de ±60 µs como máximo, igual que en el llano. La prueba D-A de hoy debe mostrar el otro lado: con el dron trabado, el uso sube hacia el 93 % y aparece saturación.

### 2.3 Consumo y autonomía (estimado; P7 lo mide)

| Fuente | Carga | Corriente o potencia | Autonomía (80 % de descarga) |
|---|---|---|---|
| 2 LiPo 3S 3,3 Ah (una por rama) | Dron en ralentí, 1300 µs | 1,2 A por rama | ~2 h |
| | Modo rampa, ~1540 µs | 8,2 A por rama | ~19 min |
| | Flotando, ~1750 µs | 22,6 A por rama | **~7 min** (3–7 min según la carga inicial; ver paper) |
| NiMH TETRIX 12 V 3 Ah | Base en llano | 0,9 A | ~2 h 36 min de marcha continua |
| | Base en rampa de 10° | 2,0 A | ~71 min |
| | Base en rampa de 20° | 3,1 A | ~47 min |
| 3S 18650 1,6 Ah (BT5) | Pi + cámara + ESP32-S3 + NodeMCU, ~6 W | | ~2,5 h |
| 3S 18650 1,6 Ah (BT6) | 2 servos S3003 con el PID, ~2,5 W | | ~6 h |
| Power bank (supuesto de 37 Wh) | myRIO, ~6 W | | ~4,7 h |

**Con todo funcionando y el dron en ralentí sobre la base, el robot dura unas 2 h**, limitado por la NiMH y la lógica.

**En vuelo, lo que manda es el margen de empuje, no la energía:**

- con 12,4 V despega a 1725–1775 µs;
- con 11,1 V en reposo necesitaría 1850–2000 µs y ya no flota (lo observaron el 20/11).

**Regla práctica:** volar solo con más de ~11,6 V en reposo.

**Para medir hoy (P7):** tensión en reposo antes y después de cada bloque, con los minutos de rotores. Si hay pinza amperimétrica, corriente de la base subiendo la rampa y de una rama en ralentí.

---

## 3. Esfuerzo de control (nueva métrica)

Sale automáticamente en `metricas.csv`.

| Subsistema | Métrica | Definición | Para qué sirve |
|---|---|---|---|
| Bola | `esfuerzo_servo_rms_us` | √(media de (u_x − n_x)² + (u_y − n_y)²), en µs | Cuánto inclinan los servos para mantener la bola. Se compara C1, C2 y C3, y D-A con D-B |
| Bola | `actividad_servo_us_por_s` | Σ\|Δu\| / duración, en µs/s | Cuánto se mueven los servos: desgaste y consumo. Sube con la vibración y el ruido |
| Dron | `dron_esfuerzo_dif_us` | Desviación estándar de los 8 motores alrededor de su media, en µs | Par de actitud que pide el control. Sube con perturbaciones y con el desbalance del centro de masa |
| Dron | `dron_motor_medio_us` | Colectivo medio | Energía consumida |
| Base | Ciclo de trabajo PWM medio por rueda (del CSV de LabVIEW) | % | Esfuerzo de la base: llano frente a rampa, ralentí frente a modo rampa |

Referencia del 18/11 (dron libre): esfuerzo RMS de 104 µs en la rampa y 160 µs en el llano. Es decir, **la rampa no exigió más a los servos**, otra evidencia de que el dron absorbe la pendiente.

---

## 4. Validación: calculado, simulado y medido

**Criterio de aceptación:** error ≤ 10 % para valores puntuales; para rangos, que el valor medido caiga dentro del rango calculado o lo solape.

| # | Magnitud | Calculado (modelo) | Simulado | Medido | Error / estado |
|---|---|---|---|---|---|
| 1 | Colectivo de despegue con 12,4 V | 1899 µs (mapa del firmware); 1770 µs (auditoría de potencia) | 1898 µs (Isaac Lab) | 1725–1775 µs | Mapa e Isaac: +8,5 % (empuje real 33–52 % mayor, identificado). **Auditoría: 1,1 % ✓** |
| 2 | Despegue con batería descargada (vuelo 9) | 1787 µs | — | 1796 µs | **0,5 % ✓** |
| 3 | Despegue con 11,1 V en reposo | 1850–2000 µs (no sostenible) | — | No flotó | **✓ cualitativo** |
| 4 | Límite de giro: v6 frente a v8 | Autoridad de 0,37 frente a 0,89 N·m; recuperable 2,8–3,8° frente a 16–19° | Simulación de un eje: v6 corta en ~2,5 s y v8 se mantiene | v6 cortó (vuelos 5–6) y v8 se mantuvo 20 s | **✓ coincide** |
| 5 | Modo lento de actitud | 0,20–0,30 Hz | — | 0,15–0,23 Hz | Rangos que se solapan ✓ |
| 6 | Inclinación RMS en vuelo estacionario | — | 0,02° (PPO), 0,04° (clásico), 0,12° (con perturbaciones) | 0,55–1,7° | ✗ Isaac no modeló la vibración, el desbalance del centro de masa ni la saturación (se discute) |
| 7 | Límite de kd de la bola | 4,5 estable, 6 inestable | Simulación con retardo (`ball_dt.py`) | 4,5 estable, 6 inestable | **0 % ✓** |
| 8 | Velocidad de la base al 15 %, en llano | 0,073 m/s | — | 0,06–0,08 m/s (LabVIEW) | Dentro del rango ✓. **Hoy se mide con cronómetro en E1** |
| 9 | Sesgo de los servos en la rampa de 10° | 204 µs sin dron; ≤ 35 µs con dron | — | ≤ 60 µs (dron libre, 18/11) | ✓ con dron. **D-A mide el caso sin dron** |
| 10 | Error de la bola por la actitud del dron | 1,7–2,1 mm RMS | — | 16–21 mm RMS con los rotores girando | ✗ El modelo no incluye el aire de los rotores ni la vibración. Se reporta como limitación |
| 11 | Frecuencia máxima útil del plato | ~3,5 Hz con ±5° | — | **Hoy (E2)** | Pendiente |
| 12 | Velocidad subiendo la rampa: ralentí frente a modo rampa | 3,9 frente a 4,8 cm/s | — | **Hoy (P7)** | Pendiente |
| 13 | Pendiente de vuelco | Tabla 2.1 | — | **Hoy (P1)** | Pendiente |
| 14 | Autonomía en ralentí y en vuelo | ~2 h / ~7 min | — | **Hoy (P7)**, por caída de tensión | Pendiente |

**Simulaciones disponibles para mostrar desempeños:**

- **Isaac Lab** (dron en la guía, con PPO y con control clásico);
- **simulación de un eje de la autoridad de par** (v6, v7 y v8);
- **lazo discreto de la bola con retardo** (`ball_dt.py`, límite de kd);
- **cadena de transmisibilidad** base → dron → bola (`chain.py`).

Con las mediciones de hoy, las filas 11–14 se cierran.

Si quieren, después de las pruebas armo una **simulación de la subida y bajada de la rampa**: perfil de la rampa → base → rótula → dron → bola, con dron trabado y con dron libre. La validamos contra D-A y D-B, y sería la figura central del argumento.

---

## 5. Costo: ¿es viable el dron?

La comparación es contra alternativas que cumplen la misma función: **nivelar el plato mientras la base sube una rampa**. Los precios de las alternativas son referencias de mercado aproximadas y conviene verificarlos.

| Alternativa | Costo aprox. (USD) | ¿Nivela en 10° y 20°? | ¿Reacciona contra la base? | ¿Lastre ajustable (vuelco)? | Energía | Valor de investigación |
|---|---|---|---|---|---|---|
| Solo servos en la base (2 × S3003, lo actual) | ~20 | 10°: con el 93 % del recorrido; 20°: no | Sí | No | Muy baja | Bajo |
| Cardán de servos grandes + IMU (2 × MG996R, MPU-6050, piezas impresas) | ~30–50 | Sí | **Sí**: golpes y pendiente pasan por la transmisión | No | Baja | Bajo |
| Cardán *brushless* (controlador Storm32 + 2 motores de gimbal) | ~80–150 | Sí, con carga útil limitada (pensado para cámaras) | Sí | No | Baja | Medio |
| **Módulo octocóptero de AeroTray (BOM)** | **~425** | **Sí**, referido a la gravedad | **No**: la rótula no transmite par | **Sí**: f de 0 a 1 | Alta en vuelo; baja en ralentí | **Alto**: dos equilibrios apilados y una plataforma de control y aprendizaje |
| Equipos comerciales de laboratorio (bancos de *ball balancer* o de dron de 3 GDL) | Del orden de miles (pedir cotización) | No son móviles | — | — | — | Un solo problema |

**Conclusión honesta.** Solo para nivelar, un cardán de servos es **8–14 veces más barato**. El dron se justifica por lo que el cardán **no puede** hacer:

- no le pasa a la bola la reacción de la base;
- deja libre el recorrido de los servos;
- su empuje es un lastre ajustable que sube el ángulo de vuelco.

Además, convierte el robot en una plataforma de investigación de equilibrios apilados. Aun así, cuesta mucho menos que el equipo comercial de laboratorio, que resuelve un solo problema y sobre una base fija.

---

## 6. Título más corto (propuesta)

| Opción | Título |
|---|---|
| **1 (recomendada)** | **AeroTray: un octocóptero cautivo sobre un robot móvil para equilibrar una bola en rampas** |
| 2 | AeroTray: robot móvil con octodrón cautivo para nivelar un *ball and plate* en pendientes |
| 3 | Octocóptero cautivo como etapa de nivelación de un *ball and plate* móvil |

«Cautivo» deja claro que el dron va sujeto por la rótula y la guía, no en vuelo libre.

**Título elegido por el equipo (ya está en el paper):** **AeroTray: sistema *ball and plate* sobre un octocóptero acoplado en una base móvil diferencial**

---

## 7. La rampa real: 1 m de subida a 10°, 0,5 m de meseta y 1 m de bajada a 10°

Cálculo con `calculos_sin_encender.py`: modelo del motor TorqueNADO, centro de masa del CAD, ciclo de trabajo del 15 %. Las velocidades son en lazo abierto; el PI de LabVIEW las regula hacia la consigna.

**Dron en ralentí (1300 µs):**

| Tramo | Velocidad | Tiempo | Corriente de la base |
|---|---|---|---|
| Subida, 1 m a 10° | 4,1 cm/s | 24,2 s | 2,0 A |
| Meseta, 0,5 m | 7,4 cm/s | 6,8 s | 0,9 A |
| Bajada, 1 m a −10° | 10,6 cm/s (el PI la frena) | 9,4 s | 0,8 A |
| **Ciclo completo** | | **≈ 40 s** | Base: 17 mAh; dron: 28 mAh |

**Modo rampa (~1540 µs):**

| Tramo | Velocidad | Tiempo | Corriente de la base |
|---|---|---|---|
| Subida | **4,8 cm/s** (+17 %) | 20,8 s | 1,8 A |
| Meseta | 7,4 cm/s | 6,7 s | 0,9 A |
| Bajada | 10,1 cm/s | 9,9 s | 0,8 A |
| **Ciclo completo** | | **≈ 37,5 s** | Base: 14 mAh; dron: 171 mAh |

**Desnivel:** 1 m × sen 10° = **17,4 cm**.

**Ciclos por carga:**

- NiMH: ~140 ciclos (energía de sobra).
- LiPo en ralentí: ~190 ciclos.
- LiPo en modo rampa: ~30 ciclos.

**Quiebres.** Cada ciclo tiene **4 quiebres**: pie, cresta, inicio de la bajada y pie final. En cada uno la base cambia 10° de cabeceo mientras avanza la distancia entre ejes (0,216 m): unos 5 s subiendo y unos 2 s bajando. El dron tiene que absorber esos 10° **en la rótula**. **Si la rótula tiene menos de ~12° de rango libre, el dron choca con el tope y se inclina con la base.** Esa es la explicación más probable de los cortes a 12° al entrar a la rampa del 18/11. Por eso, en P1 midan el rango de la rótula hacia adelante y hacia atrás; es un dato decisivo para el paper.

**Validación de hoy:**

- tiempo de cada tramo con cronómetro o con el video (comparar con la tabla de arriba);
- velocidad de subida en ralentí frente a modo rampa (el modelo predice 4,1 frente a 4,8 cm/s);
- cabeceo del dron en los 4 quiebres, desde `vuelo_*.csv`.

---

## 8. Firmware v11 del dron: independiente, mantiene el rumbo inicial y la altura

El dron funciona **solo**: no recibe nada de la myRIO ni de la base. La Pi solo le manda las órdenes y el *heartbeat* de siempre.

### 8.1 Rumbo (yaw)

| Situación | Comportamiento |
|---|---|
| En ralentí, sobre la base (≤ 1400 µs) | **Mantiene el rumbo con que arrancó.** Si la base lo arrastra más de ±20° (en un giro de la base), la referencia se desliza con él en lugar de cortar. Así no pelea con la base en un giro de 360°, y cuando la base para, se queda quieto con un desfase de 20° como máximo. Solo hay un corte de seguridad: girar a más de 120°/s. |
| Por encima de 1400 µs (modo rampa o volando) | Igual que v9: mantiene el rumbo y corta a 20° o a 35°/s. Al subir para volar no puede girar sobre sí mismo, como pasaba en v3. |

Roll y pitch se controlan siempre a nivel, con la referencia de la calibración. El corte a 12° no cambia.

**Prueba en banco:** dron en ralentí sobre la base, sin la bola. Girar la base 90° despacio y luego 360°. No debe cortar, y al parar la base el dron debe quedarse quieto.

### 8.2 Altura

**Mantener altura** se activa con la orden **Mantener** de siempre:

- Si en ese momento el láser da medidas válidas, el dron guarda la distancia al suelo y corrige el colectivo (±60 µs como máximo) para quedarse a esa altura.
- Si el láser deja de medir, la corrección se congela y queda el *mantener* de siempre (colectivo fijo). A los 2 s el modo se apaga solo.
- Cualquier otra orden de colectivo (Subir, Bajar, Flotar, Aterrizar, Parar) lo apaga.
- El mensaje del dron dice cuál de los dos casos aplicó: «ALTURA: manteniendo … mm» o «ALTURA: sin medidas válidas del láser».

Las ganancias se eligieron por simulación con el láser a 20 Hz y 5 mm de ruido, motores de 70 ms y fricción de la guía de 0 a 1 m/s². Con un escalón de −20 µs (equivalente a la caída de la batería), el dron se queda a ±3–11 mm de la altura pedida. **Mientras el láser no dé medidas válidas, este modo no hace nada**, así que es seguro grabarlo antes de resolver el sensor.

### 8.3 Láser: el diagnóstico y lo que cambió

En los 5 vuelos del 20/11 **no hubo ni una medida válida**:

| Código | Significado | Medidas |
|---|---|---|
| **2**, *signal fail* | La luz que vuelve es más débil que el límite (0,25 MCPS por defecto) | ~80 % |
| **4**, fuera de rango | El objetivo está fuera del alcance | El resto |
| **255** | El sensor no arrancó | Todo el vuelo 8 |

Que domine el código 2 indica que el sensor sí responde por I²C, pero el haz (un cono de 25°) no encuentra una superficie que refleje bien.

**Cambios en v11:**

- el láser arranca en **modo largo alcance** (límite de señal de 0,1 MCPS y pulsos más largos), que acepta retornos más débiles;
- con los motores apagados, cada 2 s el dron imprime `LASER DIAG estado=… mm=… senal=… ambiente=…`. Se ve en la página `/dron`.

**Prueba para encontrar la causa** (motores apagados, dron sobre la base, mirando `/dron`):

1. **Tal como está montado.** Si `estado=2` y `senal < 0,1`, el haz no tiene dónde rebotar.
2. **Una cartulina blanca a 30–40 cm, justo debajo del sensor.** Si pasa a `estado=0` con una distancia correcta, el problema es el blanco: el chasis perforado de TETRIX, los cables o el ángulo. **Solución:** una placa blanca mate fija en la base, debajo del sensor (sobre la base el dron solo sube hasta 77 mm, así que la placa está siempre en rango).
3. **Si con la cartulina sigue en 2:** revisar la ventana del sensor (polvo o el film protector sin quitar) y la orientación (que apunte hacia abajo y no tape el haz ningún tornillo, cable o brazo).
4. **Si `ambiente` es alto (> 1–2 MCPS):** hay luz infrarroja (sol o lámparas) entrando al sensor. Hay que darle sombra.
5. **Si sale `estado=255` o `i2c` sube:** es un problema de cable o de bus I²C (mismo bus que la IMU, SDA 23 y SCL 22). Revisar conectores y soldaduras.

**Grabar** (con las LiPo desconectadas): `firmware/cargar_vuelo_guia_desde_pi.sh`.

La función de seguir el giro de la base con datos de la myRIO (`yb`) quedó en el código, pero **no se usa**: como la myRIO no manda nada, la Pi no le envía nada al dron y el v11 trabaja solo.
