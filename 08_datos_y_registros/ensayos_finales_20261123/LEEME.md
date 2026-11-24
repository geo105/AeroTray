# Ensayos finales del 23/11/2026 (robot completo)

- `registro_20261123_182316.csv`, `vuelo_20261123_183652.csv`: registros de la Raspberry Pi (18:23–18:39).
- `recuperado_video/`: telemetría recuperada de las grabaciones de pantalla de la interfaz web (17:29–18:31 y 18:36–19:15),
  midiendo cuadro a cuadro los puntos que dibuja la página (`extraer_de_grabacion.py`). Verificada contra el CSV de la Pi:
  0,6–1,4 mm de error mediano. `analizar_23nov.py` separa los 13 ensayos BP23-01…13 y calcula las métricas;
  `figuras_paper_23nov.py` genera las figuras del paper.
- `resultados/base_adelante_labview.json`: intervalos con la orden ADELANTE en LabVIEW (grabaciones 18:48–18:58 y 19:00–19:10;
  el nombre de esas grabaciones de Windows es la hora de FIN). `ensayos_con_base_en_movimiento.json`: métricas de BP23-10…13 durante el cruce.
- Videos del celular: `VID_20261123_HHMMSS.mp4`, el nombre es la hora de inicio (confirmado con la info del teléfono).
