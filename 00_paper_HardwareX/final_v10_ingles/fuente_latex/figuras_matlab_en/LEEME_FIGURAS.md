# Figuras del paper en MATLAB

Todas las figuras de datos y del modelo se dibujan con estos scripts, con un solo estilo:
- tamaño final exacto (13,7 cm de ancho, el ancho de texto del paper): en LaTeX se incluyen **sin escalar**, así que la letra sale en 8–9 pt;
- fuente Latin Modern (la del paper) si está instalada; si no, Times New Roman;
- coma decimal en los ejes;
- paleta apta para daltonismo y para impresión en gris (azul, rojo, verde, ocre, morado), siempre en el mismo orden;
- nombres de ejes y leyendas en español.

## Cómo regenerarlas
1. Calcular los datos (Python, una vez o cuando cambie un parámetro, por ejemplo el centro de masa medido):
   `cd scripts_modelo && python exportar_datos_matlab.py` → escribe `figuras_matlab/datos/*.csv`.
2. En MATLAB, con la carpeta `figuras_matlab` como carpeta actual: `generar_todas` → escribe `figures/*.pdf`.
   También funciona en GNU Octave 8 (así se generaron los PDF de esta versión).

## Archivos
| Script | Figura del paper |
|---|---|
| `fig_base.m` | `fig_base_limits.pdf` (velocidad y vuelco de la base) |
| `fig_identificacion.m` | `fig_ident.pdf` (centro de masa y mapa de empuje) |
| `fig_rampa.m` | `fig_rampa.pdf` (descarga de peso: vuelco, aceleración, tracción) |
| `fig_bateria.m` | `fig_bateria.pdf` (colectivo de despegue y empuje/peso frente a la tensión) |
| `fig_estabilidad_dron.m` | `fig_drone_stability.pdf` |
| `fig_lazo_bola.m` | `fig_ball_stability.pdf` |
| `fig_transmisibilidad.m` | `fig_transmissibility.pdf` |
| `fig_banco.m` | `fig_drone_bench.pdf` |
| `fig_vuelos.m` | `fig_flight_campaign.pdf` |
| `fig_bp_resultados.m` | `fig_bp_results.pdf` (error de la bola 17–18/11) |
| `fig_ensayos_23nov.m` | `fig_ensayos_23nov.pdf` (13 ensayos del 23/11) |
| `fig_bp23_despegue.m` | `fig_bp23_despegue.pdf` (despegue con la bola encima) |

Para cambiar algo en **todas** las figuras (fuente, tamaños, colores, grosor de línea), editar `estilo_paper.m`.
Las funciones comunes son `nueva_figura`, `crear_ejes`, `rotular` (nombres de ejes, letra del panel y coma decimal), `leyenda`, `coma_decimal`, `guardar_figura` y `leer_csv`.
Para las figuras nuevas de los ensayos del 23/11 conviene copiar `fig_banco.m` como plantilla.

Los datos de las figuras experimentales salen de `scripts_modelo/exportar_ensayos_matlab.py` (necesita la carpeta `AeroTray_Repositorio_Completo`). Los CSV ya están en `datos/` con los parámetros del 23/11 (masa 10,4 kg, centro de masa medido), así que basta con correr `generar_todas` en MATLAB.

`caja.m` dibuja las cajas de bigotes a partir de percentiles (no necesita el Statistics Toolbox). Los scripts `fig_bp_resultados`, `fig_ensayos_23nov` y `fig_bp23_despegue` se escribieron sin MATLAB a mano: si alguno da error, el mensaje dice la línea.
