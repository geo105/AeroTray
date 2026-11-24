# Protocolo de pruebas finales AeroTray — 23/11

Nov 23, 2026 · @george

## Plan del día

Hoy cada escenario de la guía tiene que quedar con tres evidencias: un video corto, una tabla con el error en % y una gráfica de la posición y el error de la bola frente al tiempo. Eso es justo lo que el profesor marcó como faltante en el Lab 6.

Orden recomendado (unas 4 h 30 min en total). Va de menor a mayor riesgo y deja el vuelo para el final, cuando la batería ya no importa para el resto:

1. Preparación y seguridad, antes de encender nada: 30 min.
2. Centro de masa, con el robot apagado: 20 min.
3. Escenario 1, base móvil (traslaciones y giros): 60 min.
4. Escenario 2, plataforma con senos y estimación de la bola: 30 min.
5. Escenario 3, equilibrio con base fija y recuperación: 30 min.
6. Escenario 4, equilibrio en traslación, en llano y en la rampa (subida, meseta y bajada): 45 min.
7. Escenario 5, equilibrio durante la rotación: 30 min.
8. Modo rampa y batería, solo si queda tiempo: 20 min.

**Roles.** Cinco tareas, una persona cada una. La asignación es una sugerencia según lo que cada uno hizo en el proyecto.

| Rol | Qué hace | Sugerido |
| --- | --- | --- |
| Operador | Maneja todo desde LabVIEW, la única interfaz que se ve en el video | Diego |
| Seguridad | Tiene el hongo SB1 en la mano y la cuerda del mástil; puede parar cualquier prueba | Johan |
| Cámara | Graba cada ensayo, muestra la hora de la laptop al inicio y usa cámara lenta cuando se pide | Hideky |
| Mediciones | Cinta métrica, transportador, celular como inclinómetro, marcas en el piso | Sebastián |
| Registro | Llena la hoja de datos con la hora de inicio y fin de cada ensayo y revisa que se guarden los CSV | George |

**Lo que evalúa el profesor** (rúbrica del Lab 6 y guía del proyecto):

- Gráficas que demuestren lo que el video muestra como equilibrio, no solo el video.
- Resultados en porcentaje de error o de efectividad.
- Una sola interfaz, y la principal es LabVIEW.
- Esquema del control de trayectoria de la base.
- Un sistema de seguridad visible durante las pruebas.
- Subida y bajada completas de la rampa de 10°.
- Video final de 10 min como máximo, con la participación de todos.

## Antes de encender

Cinco cosas se resuelven antes de la primera prueba; si falta una, las pruebas pierden valor como evidencia.

**1. Seguridad visible** (el profesor la pidió en los Labs 5 y 6):

- [ ] Hongo de emergencia SB1 en la mano de la persona de seguridad durante todas las pruebas con rotores.
- [ ] Cuerda atada al mástil de la cámara, sostenida desde 1,5 m. Nadie pone la mano cerca de las hélices, como pasó en los videos del 20/11.
- [ ] Zona libre de 2 m alrededor del robot y lentes para todos los que estén dentro.
- [ ] Hélices revisadas y apretadas.
- [ ] Primer video del día: pulsar el hongo con los rotores en ralentí y mostrar que todo se detiene.

**2. LabVIEW como única interfaz.** La telemetría ya llega a la myRIO por la UART cada 50 ms (`x,y,roll,pitch,uav,bp`). El panel de LabVIEW debe mostrar:

- [ ] Posición de la bola en un gráfico x–y y el error |e| frente al tiempo.
- [ ] Alabeo y cabeceo del dron.
- [ ] Odometría o trayectoria de la base.
- [ ] Botones para las órdenes del dron y del plato (códigos 1–11).
- [ ] Grabación a CSV con tiempo, x, y, error, alabeo, cabeceo y odometría.

La Pi sigue registrando a 187 Hz como respaldo, pero su página web no aparece en el video.

**3. Corte de guiñada en las rotaciones.** El firmware de banco corta los motores cuando el dron gira 20° con la base, así que con el dron encendido ninguna rotación del Escenario 5 llega a 360°. Dos salidas:

- Hacer las rotaciones con el dron apagado y trabado nivelado. Es lo más seguro y es válido: la guía evalúa el equilibrio de la bola.
- O cambiar en el firmware el límite de 20° de guiñada y probarlo en banco antes de usarlo. Solo si hay tiempo.

**4. Baterías.**

- [ ] Las dos LiPo a 12,6 V en reposo, medidas y anotadas con la hora.
- [ ] Servos alimentados por su pack 18650 (BT6), no por la fuente de banco: la guía exige alimentación autónoma.
- [ ] NiMH de la base cargada y el segundo par de LiPo cargado de reserva.

**5. Sincronización y nombres.** Al empezar cada ensayo, la cámara graba 2 s la hora de la laptop (con segundos) y alguien dice el código del ensayo. El registro anota ese código con la hora de inicio y fin, y el CSV de LabVIEW se guarda con el mismo nombre. Códigos que usa este protocolo: P1 (centro de masa), E1 a E5 (escenarios de la guía), P7 (modo rampa y batería).

## P1. Centro de masa sin balanza

Con dos levantamientos, uno hacia cada lado, salen la posición y la altura del centro de masa sin ninguna balanza. Es el mismo método del celular que ya usaron, pero hecho en los dos sentidos. Este dato decide los números de la rampa en el paper y si la prueba de 20° es segura.

**Preparación**

- [ ] Robot completo como en las pruebas: todas las baterías en su lugar, cámara y bandeja puestas, sin la bola. Sin hélices, por seguridad; pesan poco y son simétricas.
- [ ] Dron trabado nivelado respecto a la base con precintos o cinta, para que no gire en la rótula al inclinar el robot.
- [ ] Medir con cinta métrica la distancia L entre los centros del eje motriz y del eje omnidireccional, y el radio r de cada rueda (diámetro entre 2).
- [ ] Celular con una app de nivel (Medidas → Nivel en iPhone, o phyphox → Inclinación), apoyado plano sobre una parte del chasis paralela al piso. Con el robot en el piso, anotar la lectura de cero.
- [ ] Un tope (libro o tabla) contra las ruedas que quedan en el piso. Las omnidireccionales giran libres y el robot se iría rodando.

**Ensayo A, vuelco hacia atrás.** Levantar despacio el lado de las ruedas omnidireccionales; el robot gira sobre las ruedas motrices. Buscar el ángulo en el que se queda solo en equilibrio: deja de pesar en las manos y, un poco más, se iría hacia atrás. Leer el ángulo θA. Una segunda persona está detrás para atajarlo. Tres repeticiones.

**Ensayo B, vuelco hacia adelante.** Lo mismo levantando el lado de las ruedas motrices, de modo que gira sobre las omnidireccionales. Leer θB. Tres repeticiones.

&#91;embedded content: centro de masa · levantamiento en los dos sentidos\]

CM = centro de masa. El ángulo que se anota es aquel en el que el CM queda justo sobre el eje que sigue en el piso. El dibujo usa el mismo ángulo en los dos lados solo como esquema; en el robot real los dos ángulos salen distintos.

**Cálculo** (con los ángulos ya sin el cero del celular). Al levantar a mano, el robot pivota sobre el eje, así que la altura que sale se mide desde los ejes; se le suma el radio para tenerla desde el piso:

```latex
h_a = \frac{L}{\tan\theta_A + \tan\theta_B}, \qquad x_G = h_a \tan\theta_A, \qquad h_G = h_a + r
```

x\_G se mide desde el eje motriz hacia adelante. **Cómo saber quién tenía razón:** si el centro de masa del CAD es el correcto, θA sale cerca de 14° y θB cerca de 22°. Si el del ensayo de octubre es el correcto, sale al revés: θA cerca de 23° y θB cerca de 13°.

**Masa total sin balanza de precisión.** Una persona se pesa sola en una balanza de baño y luego cargando el robot; la diferencia es la masa M.

**Si consiguen una sola balanza** (verificación opcional): ruedas omnidireccionales sobre la balanza y ruedas motrices sobre libros de la misma altura, comprobando con el celular que el robot quede a 0°. Con la lectura N\_omni, el centro de masa es x\_G = L · N\_omni / M.

**Rango de la rótula.** Destrabar el dron, inclinarlo a mano hacia adelante, atrás, izquierda y derecha hasta el tope, con el celular sobre el marco. Es el límite angular entre la base y el dron que el profesor pidió definir en el Lab 4.

| Medida | Rep. 1 | Rep. 2 | Rep. 3 | Promedio |
| --- | --- | --- | --- | --- |
| L entre ejes (mm) |  |  |  |  |
| r rueda motriz / omni (mm) |  |  |  |  |
| Cero del celular (°) |  |  |  |  |
| θA, gira sobre ruedas motrices (°) |  |  |  |  |
| θB, gira sobre ruedas omni (°) |  |  |  |  |
| M total (kg) |  |  |  |  |
| Rótula: adelante / atrás / izq. / der. (°) |  |  |  |  |

## E1. Base móvil: traslaciones y giros

La guía pide traslaciones de 25 y 50 cm en 0°, 10° y 20°, y giros completos sobre el punto medio del eje y sobre una rueda, a velocidad baja, media y alta. Todo se reporta como error en %.

**Velocidades.** Definan en LabVIEW tres valores fijos para todo el día (por ejemplo, ciclos de trabajo de 15, 25 y 35 %, o las consignas del PI que ya usan) y anótenlos en la hoja.

**Traslación**

1. Pegar una regla al costado del chasis, a la altura del eje motriz, y marcar con cinta en el piso dónde está su punta al inicio.
2. Ordenar 25 cm (y luego 50 cm) desde LabVIEW.
3. Marcar dónde quedó la punta y medir con la cinta métrica la distancia recorrida y la desviación lateral, en mm.
4. Anotar también lo que dice la odometría de LabVIEW, para comparar odometría con realidad.
5. Error % = |d medida − d pedida| / d pedida × 100.

Repeticiones: en llano, las dos distancias a las tres velocidades, tres veces cada una (18 ensayos). En 10°, las dos distancias a velocidad baja, tres veces (6 ensayos).

**Pendiente de 20°.** Solo si P1 da un ángulo de vuelco de 25° o más en el sentido de la subida. Si se hace: base con el dron trabado y los rotores apagados, cuerda atada al chasis sostenida desde arriba de la rampa y velocidad baja. Si P1 da menos de 25°, no se hace y en el paper se explica con el número medido.

**Giros de 360°**

1. Pegar en el piso una línea de cinta alineada con el borde del chasis.
2. Ordenar el giro completo, sobre el punto medio del eje y luego sobre una rueda.
3. Al terminar, medir con un transportador el ángulo entre el borde del chasis y la línea de cinta, o tomar una foto desde arriba y medirlo después.
4. En el giro sobre una rueda, marcar dónde estaba esa rueda y medir cuánto se movió (mm).
5. Error % = |ángulo girado − 360°| / 360° × 100.

Repeticiones: dos tipos de giro a tres velocidades, tres veces cada uno (18 ensayos).

| Código | Superficie | Velocidad | Pedido | Medido | Odometría | Error % | Desv. lateral (mm) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| E1-T25-0-B-1 | 0° | baja | 250 mm |  |  |  |  |
| E1-G360-M-B-1 | 0° | baja | 360° |  |  |  |  |

Código: T = traslación (25 o 50 cm), G = giro (M = sobre el punto medio, R = sobre una rueda), luego superficie, velocidad (B, M, A) y repetición.

## E2. Plataforma y estimación de la bola

Se busca la frecuencia máxima útil de cada eje del plato y se verifica que la cámara mida bien la bola. El modelo predice que el S3003 limita por velocidad cerca de 3,5 Hz con ±5° de plato y cerca de 1,8 Hz con ±10°; si la medición coincide, es una comparación modelo–experimento más para el paper.

**Seno por eje**

1. Hace falta un modo seno en la ESP32-S3. Si el firmware no lo tiene, George lo agrega antes de la prueba.
2. Amplitud de ±100 µs alrededor del neutro (unos ±5° de plato) a 0,5; 1; 1,5; 2; 3 y 4 Hz, 10 s cada una. Primero el eje x, luego el y, y luego los dos juntos (x con seno e y con coseno, de modo que el plato describa un círculo).
3. Grabar de costado, a la altura del plato, en cámara lenta (240 fps), con una tira de cinta de color en el borde del plato y papel cuadriculado o una regla detrás.
4. Frecuencia máxima útil: la última en la que el plato todavía alcanza al menos el 70 % de la amplitud pedida. Del video se mide la amplitud real; de los registros, los pulsos enviados.

| Eje | Frecuencia (Hz) | Amplitud pedida (°) | Amplitud medida (°) | Medida / pedida (%) |
| --- | --- | --- | --- | --- |
| x | 0,5 | 5 |  |  |
| x | 1 | 5 |  |  |

**Estimación de la bola en posiciones conocidas**

1. Imprimir o dibujar una plantilla de papel del tamaño del plato con marcas en el centro, a ±60 mm en x, a ±60 mm en y y en las cuatro diagonales.
2. Con el PID apagado y el plato en neutro, poner la bola en cada marca y registrar 5 s.
3. Error = posición medida − posición real, en mm y en % de la distancia al centro.

**Estimación en movimiento.** Hacer rodar la bola a mano a lo largo de una regla apoyada sobre el plato, en x y en y. La trayectoria registrada debe ser una recta; la desviación máxima respecto de esa recta es el error de estimación en movimiento.

## E3. Equilibrio con base fija

La guía pide mantener la bola en el centro el mayor tiempo posible, cuantificar el error y su varianza, y medir la recuperación desde 2 y 5 cm. Se hace en dos condiciones: dron apagado y trabado, y dron en ralentí (1300 µs) con el lazo de actitud activo.

**Bola en el centro.** Tres registros de 60 s por condición. De cada registro salen el error medio, el RMS, la varianza, el máximo y el porcentaje del tiempo con la bola a menos de 10, 20 y 30 mm del centro. Referencia de la última vez: mediana de 4,4 mm con el dron apagado y 18,9 mm en ralentí.

**Recuperación desde 2 y 5 cm**

1. Poner sobre el plato la plantilla de papel con círculos de 20 y 50 mm de radio.
2. Con el PID activo, sostener la bola con la punta de un lápiz sobre el círculo y soltarla. Hacerlo en cinco direcciones: +x, −x, +y, −y y una diagonal.
3. Tiempo de recuperación: desde que se suelta hasta que la bola entra y se queda 2 s dentro de la banda. Banda de 5 mm con el dron apagado y de 20 mm en ralentí.
4. Anotar también el sobrepaso máximo y si la bola se cayó.

Son cinco sueltas por distancia y por condición (20 en total). Se reporta el tiempo medio con su desviación estándar, el sobrepaso y el porcentaje de éxito.

| Código | Dron | Distancia | Dirección | Tiempo de recuperación (s) | Sobrepaso (mm) | ¿Cayó? |
| --- | --- | --- | --- | --- | --- | --- |
| E3-R2-OFF-1 | apagado | 20 mm | +x |  |  |  |
| E3-R5-ON-1 | ralentí | 50 mm | +x |  |  |  |

## E4. Equilibrio en traslación: llano y rampa de 10°

Es la prueba que define el proyecto: la bola debe seguir en el plato mientras el robot avanza en llano y mientras sube, cruza la meseta y baja la rampa. El profesor pidió expresamente la subida y la bajada completas.

**Llano.** Recorrido de 1 m a las tres velocidades de E1, tres veces cada una (9 ensayos). Primero con el dron apagado y trabado; si sale bien, repetir a velocidad baja con el dron en ralentí.

**Rampa, ciclo completo**

1. Marcar en el piso el arranque a 30 cm del pie de la rampa y la llegada a 30 cm después del final de la bajada.
2. Bola en el centro, PID activo, 5 s quieto antes de arrancar.
3. Velocidad baja y constante en todo el ciclo: subida, meseta y bajada.
4. Grabar de costado con toda la rampa en el cuadro y, si se puede, un segundo celular desde arriba.
5. Al terminar, 5 s quieto antes de parar el registro.
6. Tres ciclos completos con el dron apagado y trabado. Si los tres salen bien, uno o dos con el dron en ralentí y la persona de seguridad con la cuerda.

**Bajada.** Es el tramo de mayor riesgo, porque el eje motriz queda arriba y la tracción baja. Si la base patina o se acelera, parar con el hongo y anotar el tramo. Un patinaje también es un resultado: se reporta como límite de tracción.

**Qué se reporta por ensayo**

- Error de la bola frente al tiempo, con bandas sombreadas para subida, meseta y bajada.
- Error medio y RMS por tramo, y porcentaje del tiempo con la bola a menos de 30 mm del centro.
- Número de caídas de la bola y en qué tramo ocurrieron.
- Cabeceo del dron en cada tramo; con el dron trabado debe seguir a la base.

| Código | Recorrido | Dron | RMS subida (mm) | RMS meseta (mm) | RMS bajada (mm) | % del tiempo < 30 mm | Caídas |
| --- | --- | --- | --- | --- | --- | --- | --- |
| E4-LL-B-1 | llano 1 m | apagado |  |  |  |  |  |
| E4-R10-OFF-1 | rampa 10° | apagado |  |  |  |  |  |
| E4-R10-ON-1 | rampa 10° | ralentí |  |  |  |  |  |

## E5. Equilibrio durante la rotación

La bola debe mantenerse en el plato mientras la base gira sobre el punto medio del eje y sobre una rueda, a las tres velocidades. Por el corte de guiñada (ver "Antes de encender"), esta prueba se hace con el dron apagado y trabado, salvo que el firmware ya esté cambiado y probado.

**Procedimiento**

1. Bola en el centro y PID activo; registrar 5 s quieto.
2. Ordenar un giro de 360° sobre el punto medio a velocidad baja. Registrar hasta 5 s después de que se detenga.
3. Repetir a velocidad media y alta, y luego los tres giros sobre una rueda.
4. Tres repeticiones de cada combinación (18 ensayos).
5. Grabar desde arriba si se puede; la trayectoria de la bola en el plato se ve mejor.

**Qué esperar.** En el giro sobre el punto medio, el plato está casi sobre el eje y la bola apenas siente fuerza centrífuga. En el giro sobre una rueda, el plato describe un círculo y la bola tiende a irse hacia afuera; si el error crece con la velocidad, es el resultado esperado y se explica con el radio de giro.

**Qué se reporta.** Error medio, RMS y máximo durante el giro; porcentaje del tiempo con la bola a menos de 30 mm; caídas. Una gráfica de la trayectoria x–y de la bola durante el giro más rápido de cada tipo.

| Código | Giro | Velocidad | RMS (mm) | Máximo (mm) | % del tiempo < 30 mm | Caída |
| --- | --- | --- | --- | --- | --- | --- |
| E5-M-B-1 | punto medio | baja |  |  |  |  |
| E5-R-A-1 | sobre una rueda | alta |  |  |  |  |

## P7. Modo rampa y batería

Es la prueba que justifica usar el dron y no solo servos: con más colectivo, el empuje descarga peso del eje motriz y sube la aceleración de vuelco admisible. Solo se hace si sobra tiempo y batería, y siempre con la cuerda y el hongo.

**Subida en modo rampa**

1. Rampa de 10°, misma velocidad baja (ciclo de 15 %) y la bola en el plato con PID activo.
2. Tres subidas con el colectivo en ralentí (1300 µs).
3. Tres subidas con el colectivo subido con la orden Subir nueve veces, hasta cerca de 1525–1540 µs. Anotar el valor exacto que muestre LabVIEW.
4. Medir el tiempo de subida con cronómetro, entre las marcas del pie y del final de la rampa.

El modelo predice unos 4,1 cm/s en ralentí y 4,8 cm/s en modo rampa. Si la diferencia aparece, es la validación experimental del efecto de descarga; si no aparece, también se reporta, con el valor medido.

**Batería**

- [ ] Tensión en reposo de cada LiPo antes de la primera prueba con rotores.
- [ ] Tensión en reposo 10 min después de cada bloque con rotores, anotando cuántos minutos estuvieron encendidos.
- [ ] Si la tensión en reposo baja de 11,4 V, cambiar al par de reserva.

Con esos puntos se compara la caída de empuje medida con la que predice el paper (empuje proporcional al cuadrado de la tensión del bus) y se estima la autonomía real.

| Código | Colectivo (µs) | Tiempo de subida (s) | Velocidad (cm/s) | RMS de la bola (mm) | V antes (V) | V después (V) |
| --- | --- | --- | --- | --- | --- | --- |
| P7-1300-1 | 1300 |  |  |  |  |  |
| P7-1530-1 |  |  |  |  |  |  |

## Al terminar

Antes de desarmar, se respalda todo en un solo lugar; con eso preparo las gráficas y tablas del paper y del video.

**Datos**

- [ ] CSV de LabVIEW de cada ensayo, con el código del ensayo en el nombre.
- [ ] Registros de la Pi (carpeta `registros`) copiados completos a la laptop.
- [ ] Hoja de registro con códigos, horas de inicio y fin, mediciones a mano y observaciones (caídas, patinajes, paradas con el hongo).
- [ ] Fotos de las mediciones del transportador y de las marcas en el piso.
- [ ] Todo en `C:\trabajosU\DSM\pruebas_20261123\`, con una subcarpeta por prueba (P1, E1 a E5, P7).

**Videos**

- [ ] Videos renombrados con el código del ensayo.
- [ ] Elegir el mejor ensayo de cada escenario para el video final, que dura 10 min como máximo y debe tener a todos participando.
- [ ] En el video, junto a cada prueba, la gráfica del error frente al tiempo y la tabla con el error en %.

**Lo que hago yo con los datos**

1. Calcular el centro de masa con P1 y actualizar en el paper la tabla y la figura de la rampa.
2. Rehacer la figura de resultados del ball and plate con los registros finales.
3. Generar con los scripts de MATLAB las gráficas de posición y error de la bola frente al tiempo para E3, E4 y E5, con el mismo formato que el resto del paper.
4. Armar las tablas de error en % de E1, E2 y E3.

Cuando tengan la carpeta, díganme la ruta y empiezo.
