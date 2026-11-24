# Prueba conjunta: dron flotando + ball and plate (sin carrito)

En el ensayo con v8, el dron flotó estable entre 1725 y 1775 µs: roll ±1° y pitch ±3° durante 20 s.

## El firmware v9 del dron

- **Flotar** sube hasta **1750 µs**, que es el rango donde flota.
- Hasta 1650 µs sube a 25 µs/s. Desde 1650 µs sube lento, a **8 µs/s**: de 1650 a 1750 µs tarda 12,5 s.
- Las ganancias de vuelo se activan al pasar 1650 µs y solo se desactivan al bajar de 1450 µs, cuando ya está apoyado. Esto corrige el corte del v8 al aterrizar.
- Si hace falta más empuje, **Subir +25** lleva el objetivo a 1775 o a 1800 µs, siempre a 8 µs/s.

## Orden de la prueba

### 1. Solo el dron con v9

Hay que confirmar que el v9 flota y aterriza sin cortar.

1. Calibrar y esperar «CAL OK».
2. Conectar las LiPo y pulsar Armar e iniciar.
3. Pulsar **Flotar** y esperar a que llegue a 1750 µs (unos 30 s en total).
4. Mantenerlo flotando unos 15 s.
5. Pulsar **Aterrizar** y luego **Parar**.

Debe flotar y aterrizar sin corte. Si no lo logra, no pasen al paso 2.

### 2. Dron + ball and plate, sin pelota

1. Con el dron apoyado y los motores apagados, en la página principal pongan la bandeja en **Neutro**.
2. Hagan flotar el dron igual que en el paso 1.
3. Vean si la bandeja vibra y si la cámara sigue detectando bien (fps y tiempo de detección en la página principal).

### 3. Dron + ball and plate con pelota

1. Pongan la pelota en el centro y enciendan el **Inicio PID** con el dron apoyado. Comprueben que la centra.
2. Hagan flotar el dron con **Flotar**.
3. Miren si la pelota se mantiene. La inclinación del dron (±1–3°) la tiene que absorber el PID de la bandeja.
4. Con el dron estable 10–15 s, pulsen **Marca**. Después **Aterrizar**, luego **Parar** y al final **Exportar log**.

Si la pelota se escapa con la inclinación del dron, el siguiente paso es activar **ff**: la bandeja corrige usando el ángulo del dron. Hoy está apagado (`ff_activo` no está configurado en `ajustes.json`).

## Seguridad

- Una persona en `/dron`, con la mano en **Espacio**, y otra mirando el dron.
- Si el dron se inclina más de 6°, **Bajar** o **Aterrizar**. El firmware corta solo a 12°.
- La bandeja va en neutro o en PID, nunca en «Nivelar placa» mientras el dron vuela.
- Hagan intentos cortos: las LiPo a 1750 µs duran pocos minutos.

## Después

Pásenme `registros/vuelo_*.csv` y el `registro_*.csv` del ball and plate del mismo momento, y comparo la inclinación del dron con la posición de la pelota.

Luego viene el carrito con todo montado.
