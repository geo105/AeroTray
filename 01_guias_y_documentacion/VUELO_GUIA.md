# Primer vuelo en la guía (dron flotando, hasta máxima potencia)

## Qué se preparó

| Pieza | Dónde | Qué hace |
|---|---|---|
| Firmware `captura_octo_m8_vuelo_guia` | `firmware/captura_octo_m8_vuelo_guia/` (bins en `firmware/bin/…`) | El mismo control que ya funciona (ángulo, ganancias, mezclador, cortes de 12° y 20° de yaw). Además, el colectivo deja de estar fijo en 1300 µs y sube con rampa hasta 1950 µs (máxima potencia). |
| Órdenes nuevas | firmware + Pi + página `/dron` | `flotar` (objetivo 1800 µs), **`maximo` (objetivo 1950 µs)**, `subir` / `bajar` (±25 µs), `mantener`, `aterrizar` (vuelve a 1300) |
| Seguridad nueva | firmware | Sin el hb de la Pi no pasa de 1400 µs. Si el hb se corta 1,5 s, baja solo a 1300 µs. Pasados 90 s sobre 1500 µs, baja solo. **A máxima potencia (sobre 1850 µs) aguanta 20 s y baja sola a 1800.** El techo es de 1950 µs de colectivo y 2000 µs por motor (tope del ESC). |
| Registro | Pi | `registros/vuelo_*.csv` guarda todo el encendido a 10 Hz con el láser y el objetivo. `log` exporta los últimos 16 s a 100 Hz. |
| Comparación con Isaac Lab | `herramientas/comparar_vuelo_isaac.py` | Saca el PWM de despegue, el empuje real, la inclinación RMS y el reparto de los motores, y los compara con lo que predijo Isaac Lab. |
| Grabar desde la Pi | `firmware/cargar_vuelo_guia_desde_pi.sh` | Graba el firmware nuevo y actualiza la Pi. `--volver` regresa al firmware de siempre. |

## Lo que predicen los modelos (antes de volar)

| Fuente | Hover |
|---|---|
| Isaac Lab: 4,41 N por motor para 3,6 kg, con la curva del firmware | ≈ 1900 µs |
| Curva del firmware a 1800 µs | levanta solo ≈ 2,9 kg (80 % del peso de 3,6 kg) |
| Curva del firmware a 1950 µs (máximo) | levanta ≈ 4,0 kg (111 % del peso): debería despegar |
| Auditoría de alimentación (tabla A2212 + 9450, 3,10 kg) | ≈ 1770 µs |
| Ensayo histórico (4 motores, solo ball and plate) | empezó a subir en ≈ 1850 µs |

Lo más probable es que despegue entre 1770 y 1900 µs. Al máximo (1950 µs) la curva da un 11 % más que el peso, así que debería subir, pero con poco margen: el rozamiento de la guía y la caída de voltaje de las LiPo (≈ 5 %) se lo comen. Si a 1950 µs no despega, el problema es de empuje (hélices, motores o masa), no de software.

**Control de ángulo al máximo.** Lo comprobé en la PC con el mezclador del firmware. Con el colectivo en 1950 µs, los motores trabajan entre 1787 y 2000 µs y todavía dan hasta 1,18 N·m de corrección de roll. Es más de lo que tiene a 1300 µs (0,78 N·m), donde ya se mantuvo a nivel. Si hace falta, baja el lado contrario: prioriza el ángulo sobre el empuje total.

**Riesgos a máxima potencia:**
- **Corriente:** cerca de 31 A por rama al 100 % (la auditoría también estima hasta 40 A). Los fusibles son de 30 A: aguantan un rato al 104 %, pero al 133 % pueden abrir entre 0,75 s y 10 min. Por eso el firmware solo deja 20 s seguidos sobre 1850 µs.
- **Interruptor:** S1 no debe estar en el camino de potencia (según la auditoría ya pasó a la línea de bobinas).
- **Batería:** a tope, las LiPo duran unos 5 min. Hagan intentos cortos y carguen entre intentos.
- **Tope de la guía:** con empuje/peso de 1,1 sube despacio, pero llega al tope superior: pulsen **Mantener** o **Bajar** apenas despegue.

## Lista antes de encender

- [ ] Guía libre: sin cables que tiren y con el tope superior puesto. El dron puede caer sobre el tope inferior si se corta.
- [ ] Hélices revisadas, apretadas y con el sentido correcto. Nadie dentro del radio de las hélices; gafas.
- [ ] Las 2 LiPo cargadas (12,6 V) y al mismo voltaje. Fusibles de 30 A puestos. El corte físico (relés o XT60) a mano de alguien.
- [ ] La Pi con su fuente; servos y relés en la fuente de laboratorio.
- [ ] Bandeja en neutro y **sin pelota** en el primer vuelo.
- [ ] Una persona en la página `/dron` con la barra espaciadora (parada inmediata) y otra mirando el dron.

## Pasos

1. Con las LiPo **desconectadas**: `firmware/cargar_vuelo_guia_desde_pi.sh`. En `/dron` el panel «Vuelo en la guía» queda activo.
2. Dron nivelado y quieto: **Calibrar** (5 s, «CAL OK»). Revisa la **Vista** si quieres.
3. Conecta las LiPo. **Armar** y luego **Iniciar**: sube a 1300 µs como siempre. Espera 5 s con el apoyo quitado y comprueba que se mantiene nivelado.
4. **Subir +25** varias veces hasta 1500 µs y observa: no debe vibrar raro ni inclinarse.
5. **Máximo**: el objetivo pasa a 1950 µs y el colectivo sube a 25 µs/s (de 1500 a 1950 tarda 18 s). Si prefieren ir por partes: **Flotar** (1800) y después **Subir +25** hasta 1950.
6. Apenas despegue de la guía: **Mantener** y **Marca**. Se queda en ese colectivo. Si estaba sobre 1850, a los 20 s baja solo a 1800.
7. Mantén 5–10 s flotando y luego **Aterrizar** (baja a 60 µs/s). Ya abajo, **Parar** y **Exportar log**.
8. Ante cualquier cosa rara: **Bajar** si hay tiempo, **Espacio** si no. Espacio corta en el aire y el dron cae sobre el tope.

## Después: comparar con Isaac Lab

```bash
scp <usuario>@<IP_PI>:pi_control/registros/vuelo_*.csv ~/DSM/registros_dron/
python3 ~/DSM/herramientas/comparar_vuelo_isaac.py ~/DSM/registros_dron/vuelo_AAAAMMDD_HHMMSS.csv --masa 3.6
```

Compara:
- el PWM real de despegue con los ≈ 1900 µs de Isaac;
- el empuje máximo real por motor con el tope de 6 N de Isaac;
- la inclinación RMS flotando con la de Isaac (0,02° PPO, 0,04° clásico, 0,12° con perturbaciones);
- el reparto de los motores (en Isaac era simétrico).

**Ojo con el láser:** en los ensayos en el carrito dio pocas medidas válidas. Si no mide, el despegue se toma del momento en que pulsaste **Mantener**, así que pulsarlo a tiempo importa.
