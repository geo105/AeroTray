# Conjunto CAD local completo

Se recibieron los STEP de `nuevo 8 camara`, `x8_UAV_R350_imprimible_bandeja_v5`
y `ball_and_plate_camara`. Los originales y respaldos nativos se conservan en
`/home/george/DSM/cad/recibidos`; hashes e integridad en
`recepcion_verificada.json`. Ya no falta el STEP del carrito.

## Resultado actual

Carpeta: `/home/george/DSM/cad/junta_nuevo_de_8_motores_con_camara/conjunto_completo`.

- `junta nuevo de 8 motores con camara.step`: conjunto neutral, 383 componentes.
- `junta nuevo de 8 motores con camara.glb`: visual ligero, unos 12 MB.
- `vista_conjunto_completo.png`: conjunto y detalle de la interfaz heredada.
- `ensamblaje_completo.json`: procedencia, cuerpos, colocación y pendientes.
- `verificacion_step_completo.json`: relectura y comprobación de componentes.

Se conserva el carrito, mástil y electrónica de referencia; se reemplazan
la estructura UAV y la bandeja anteriores. Los ocho conjuntos de brazo/motor
provienen del UAV nuevo; las hélices 9450 de referencia se recolocaron sobre
los ejes reales (radio 350 mm). Se añadieron las cuatro torres nuevas.
La base nueva se colocó en la cota inferior antigua, centrando su patrón
20 x 55 mm sobre los agujeros de la interfaz original. No se fabricó ni
inventó un adaptador. La batería de referencia del CAD sigue rotulada 2200
mAh; el usuario declara dos baterías reales de 3300 mAh.

El STEP nuevo volvió a abrir con los 383 componentes y nombres. Las seis
piezas de biela tienen cajas analíticas conservadoras después de exportar;
sus recortes se verificaron por triangulación en la copia anterior. Las
mallas son visuales y no sirven como colisiones aprobadas.

## Uniones y revisión pendiente

Los originales quedaron intactos. No se modificó un archivo nativo de Fusion
ni se guardaron uniones nativas: STEP no conserva esas restricciones.
Los cuerpos fijo, lineal y rotatorio se reconstruyeron por geometría.
La rótula de siete sólidos se dividió en carcasa/tornillos (suben) y
esfera/vástago/soporte superior (giran); esta asignación es interpretada.

El usuario confirmó que Base.ipt se conserva en el montaje nuevo.
El CAD recibido ya presenta solapes entre la rótula y Base.ipt,
joint_base_esphericaljoint y low_base. La interfaz se conserva para revisión;
se marca en magenta en el detalle. `procesados/union_interferencias.json`
contiene los volúmenes: también la base nueva intersecta esa rótula en la
colocación heredada. Esto no es un montaje mecánico validado.

El usuario indicó priorizar CAD sobre su estimación anterior de 20 cm.
Los topes inferiores contactan con los rodamientos tras subir 78.629459 mm
DESDE LA POSE GUARDADA, comprobado por pares de sólidos. La simulación usa
76.629459 mm con margen numérico de 2 mm. No se identificó un tope inferior
ni el recorrido total. Ver `procesados/recorrido_verificado.json`.
El cálculo exploratorio de bajada no está validado y no se utiliza.

## Reproducción

Entorno: `/home/george/DSM/herramientas/cad-env/bin/python`.

1. `leer_ensamblaje.py` lee el STEP conservando árbol y posiciones.
2. `preparar_uav_camara.py` crea la copia de la parte superior.
3. `completar_conjunto.py` incorpora carrito, guía, rótula y referencias.
4. `ver_conjunto_completo.py` genera la vista de revisión.

Los STEP grandes pueden tardar varios minutos. Las carpetas `download`,
`download (1)` y sus inventarios son auxiliares históricos. La copia del UAV
sin carrito permanece en la carpeta superior como revisión anterior.

Simulación actual: `/home/george/DSM/simulacion/aerotray_conjunto_cad.xml`.
Instrucciones y límites en `/home/george/DSM/simulacion/EMPEZAR.md`.
