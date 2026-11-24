# Primera prueba de cámara: ESP32-S3-CAM N16R8 + OV2640

## Diagnóstico 0.3: enlace USB rápido

La versión 0.3 conserva JPEG QVGA y servos desactivados. Arranca a 115200 y
acepta `baud 460800` o `baud 921600`. Confirma `# BAUD <valor>` a la velocidad
anterior antes de cambiar. El visor negocia automáticamente la nueva velocidad
y comprueba `/ver`; al cerrar normalmente vuelve a 115200. Reiniciar la placa
también recupera 115200. El firmware 0.2 conserva el modo lento y no admite
ese comando; el visor detecta la diferencia mediante `baud_max` en su estado.

El comando `ver` añade `baudrate` y `baud_max`. No cambia la red Wi-Fi.
La compilación de 0.3 ocupa 976074 bytes de programa y 57872 de variables
globales. Antes de la carga se leyó 1 MiB del firmware real, se comprobó su
coincidencia con la aplicación 0.2 conservada y se verificó que el respaldo
cubre los sectores de 0.3. Los resultados de la prueba real de velocidad se
documentan en `/home/george/DSM/estado_camara/velocidad/`.

Para el visor con pelota metálica y escala confirmada de 18,7 x 14 cm:

```bash
bash /home/george/DSM/ver_camara_ball_plate.sh
```

La información de 0.2 que sigue es el historial de la primera prueba.

Este programa comprueba la cámara. Todavía no detecta la pelota ni acciona los servos.
No modifica el fuzzy anterior. Su compilación y su prueba física son comprobaciones distintas.

## Conexión inicial

Usar el conector USB-C rotulado **USB-TTL/COM**. Usar un cable que transporte
datos. La placa se alimenta por ese USB durante esta prueba. Los servos quedan
desconectados y la microSD retirada. La tarjeta de 32 GB podrá usarse después
para registros; este programa no la monta ni escribe en ella.

La placa fotografiada es una ESP32-S3-CAM N16R8 RE1.3. `pines_camara.h` contiene
el perfil de referencia ESP32S3_EYE (Espressif/Freenove). La compatibilidad de
esta revisión concreta se debe confirmar en la placa: la etiqueta N16R8 no
demuestra cómo está cableado su conector de cámara.

## Configuración de Arduino

- Paquete: ESP32 de Espressif, versión 3.3.12.
- Placa: ESP32S3 Dev Module.
- Flash Size: 16 MB.
- PSRAM: OPI PSRAM.
- USB CDC On Boot: Disabled al usar el puerto USB-TTL/COM.
- Monitor serie: 115200 baudios.

Se puede compilar desde Ubuntu con `firmware/compilar_camara.sh`.
La instalación local de Arduino CLI está en `herramientas/arduino-cli/` y su
configuración en `herramientas/arduino-cli.yaml`.

La compilación de la versión 0.2 pasó con Arduino-ESP32 3.3.12: 975714 bytes de
programa y 57864 bytes de variables globales. No es una prueba en hardware.
El resultado se registra en `firmware/bin/camara_s3_diagnostico/verificacion_compilacion.json`.

## Puerto y permiso de Ubuntu

En la primera conexión se detectó `/dev/ttyUSB0` (adaptador USB serie
VID 1A86, PID 7523). El puerto pertenece a `root:dialout` y el usuario
`george` todavía no tiene acceso. Para conceder acceso temporal al puerto:

```bash
sudo setfacl -m u:george:rw /dev/ttyUSB0
```

Introducir la contraseña únicamente en la terminal. Este permiso se limita
al dispositivo conectado y puede perderse al desconectarlo. Si cambia el
nombre del puerto, volver a identificarlo antes de usar el comando.

## Qué comprobar en la placa

El puerto serie debe mostrar el banner `camara-diag-0.2`, la memoria detectada
y el sensor. Para el OV2640 se espera PID `0x0026`. Ante un error, conservar
el mensaje exacto; no elegir otros mapas de pines al azar.

Por defecto la captura funciona con Wi-Fi apagado. El comando `ver` devuelve
los indicadores y `foto` envia una foto JPEG con cabecera binaria por el puerto
serie. Para recibirla desde Ubuntu sin cambiar de red:

```bash
python3 /home/george/DSM/herramientas/foto_camara_usb.py
```

La imagen se guarda en `estado_camara/primera_foto.jpg`, junto con sus indicadores
en JSON. El script requiere pyserial, NumPy y OpenCV, disponibles en este Ubuntu.

Opcionalmente, el comando serie `wifi` crea una red local:

- Nombre: `BolaCam-prueba`
- Clave temporal: `ballplate1`
- Página: la dirección que imprime el puerto serie (normalmente `http://192.168.4.1`).

Se puede abrir desde un teléfono para mantener la computadora conectada a su
red habitual. La página muestra imágenes y estado. `/foto.jpg` devuelve una
foto y `/estado` devuelve los indicadores. El comando serie `ver` también
muestra esos indicadores.

Las fotos web se actualizan una vez por segundo. `fps_captura` mide las capturas
internas en ventanas de dos segundos; no equivale a la frecuencia de detección
ni valida un lazo de control. Conservar al menos una foto de la bandeja y un
registro de estado cuando esté funcionando.

## Resultado de la primera prueba real

Diagnostico 0.2 cargado y verificado por esptool. La placa identifica 16 MB de
Flash, 8 MB de PSRAM y OV2640. El perfil de pines utilizado produjo imagenes
JPEG QVGA 320x240 validas, por lo que se comprobo su funcionamiento en esta
placa concreta. Se recibieron fotos por USB sin cambiar de red en la laptop.

La primera recepcion no paso la comprobacion de dimensiones. Las dos capturas
siguientes se decodificaron correctamente; falta una prueba larga para evaluar
la estabilidad. La ultima lectura, con el AP activado, dio 50.02 FPS de captura,
19.85 ms en `fb_get`, 21 ms de edad de la ultima foto, sin fallos ni descartes
reportados en esa ejecucion. Son medidas puntuales de captura, no de deteccion,
latencia total de control ni robustez del equilibrio.

Evidencia: `estado_camara/primera_foto.jpg`, `foto_con_wifi.jpg` y sus JSON.
El encuadre actual muestra una lampara y el techo. El siguiente paso es apuntar
la camara a la bandeja y la pelota para validar contraste, cobertura y calibracion.

La red `BolaCam-prueba` se activo mediante el comando `wifi`. Tras reiniciar,
la version 0.2 vuelve al modo USB con Wi-Fi apagado; se puede activarla y recibir
una foto con `python3 /home/george/DSM/herramientas/foto_camara_usb.py --wifi`.
Los servos permanecen desactivados.

## Respaldo antes de la primera carga

La lectura por el stub a 921600 y 460800 baud produjo errores de integridad.
La lectura por ROM a 115200 funciono para los primeros 64 KiB y para el
respaldo del intervalo `[0, 0x100000)`, que cubre todos los sectores modificados
por esta version (bootloader, particiones, boot_app0 y aplicacion hasta 0xfe3f0).
Los bytes de Flash fuera de ese intervalo no se borran ni se escriben.

`herramientas/cargar_camara_diagnostico.py` exige ese respaldo, comprueba su
coincidencia con la primera lectura y valida que cada sector a escribir este
cubierto. La escritura se hace por ROM a 115200 y esptool verifica su integridad.
Los respaldos son privados y se guardan en `firmware/backups/`.
La cabecera del respaldo de 1 MiB coincidio con la lectura independiente de
64 KiB. Su SHA256 y la cobertura de los sectores se registraron antes de grabar.
`herramientas/restaurar_camara_anterior.py` permite restaurar la region original
si se necesita; no se ejecuto la restauracion.
