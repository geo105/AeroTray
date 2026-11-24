# Carrito completo en RViz

Abrir en una terminal:

```bash
/home/george/DSM/ver_carrito_completo_rviz.sh
```

Abre RViz y robot_state_publisher con ROS 2 Jazzy. Cerrar con Ctrl+C en
esa terminal. No hace falta conectar ninguna placa. Si ya está abierto,
no es necesario ejecutar otra copia.

En RViz: arrastrar con botón izquierdo para orbitar, rueda para acercar,
botón central para desplazar. En Displays se puede desactivar el piso o
los ejes; el modelo está en «Conjunto CAD completo - 383 piezas».

El URDF usa las 383 mallas del ensamblaje local recompuesto:
`/home/george/DSM/cad/junta_nuevo_de_8_motores_con_camara/conjunto_completo`.
Incluye 116 piezas fijas del carrito, 11 de la guía móvil y 256 del conjunto
rotatorio, incluidos UAV y bandeja con soporte de cámara. Los STL están
en metros y ya tienen las transformaciones del ensamblaje aplicadas.
No se sustituyen piezas por cilindros o cajas. Los colores son los de
identificación de la exportación local, no materiales nativos de Fusion.

Esta vista muestra la pose CAD fija (sin la elevación inicial de la simulación).
No ejecuta física ni aprendizaje y no recibe telemetría. No abre el puerto
serie, no transmite comandos ni cambia los programas existentes del dron.
Los solapes heredados y componentes de referencia del CAD se conservan.

Se usa por defecto el dominio ROS 77 y los marcos `cad_mundo`,
`cad_carrito_fijo`, `cad_carro_vertical` y `cad_conjunto_rotatorio`.
Descripción: `/cad_completo/robot_description`, QoS reliable/transient_local.
Este dominio está separado del monitor habitual del dron.

Archivos:

- `conjunto_completo.urdf`: geometría y materiales para RobotModel.
- `conjunto_completo.rviz`: cámara y configuración de RViz.
- `conjunto_cad.launch.py`: inicia los dos procesos y los cierra juntos.
- `generar_conjunto_cad.py`: regenera el URDF desde el manifiesto CAD.
- `conjunto_completo_verificacion.json`: dimensiones y verificación de mallas.
- `/home/george/DSM/registros/rviz_carrito_completo.log`: registro de apertura.

Comprobado: 383 mallas finitas, 956780 triángulos, límites coincidentes con
el manifiesto con diferencia máxima de 7.4e-9 m. RViz se suscribe al modelo
y a TF estático; OpenGL 4.6 inicia sin errores de carga registrados.
