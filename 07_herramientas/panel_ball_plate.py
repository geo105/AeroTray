#!/usr/bin/env python3
"""Panel local: camara ArUco y pruebas limitadas de los servos 41/42."""
import argparse
from datetime import datetime
import json
import multiprocessing as mp
from pathlib import Path
import queue
import signal
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image,ImageTk,ImageGrab
from vision_ball_plate import FuenteUSB,crear_detector,procesar
from control_ball_plate import ControlPD
from calibracion_ball_plate import cargar,guardar,crear_args,validar

RAIZ=Path(__file__).resolve().parent.parent


def trabajo_panel(puerto,imagen,ordenes,salida,parar,apagar,generacion,ui_viva,guardado,registro,perfil=None,video_activo=None,guardar_imagenes=None):
    """El receptor USB tiene su propio proceso: Tk no puede bloquear su lectura."""
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    salida.cancel_join_thread()
    src=None
    datos={'conexion':'Conectando cámara…','foto':None,'error':'','servo':{},'ultima':0,'r':None,'fps':0,'evento':'PWM apagado al arrancar'}
    def publicar():
        try:salida.put_nowait(datos.copy())
        except queue.Full:pass
    def mostrar_video():
        return video_activo is None or bool(video_activo.value)
    def guardar_fotos():
        return guardar_imagenes is None or bool(guardar_imagenes.value)
    with registro.open('w') as log:
        def registrar(tipo,**campos):
            log.write(json.dumps({'fecha':datetime.now().astimezone().isoformat(),'tipo':tipo,**campos},ensure_ascii=False)+'\n');log.flush()
        try:
            config=cargar(perfil) if perfil is not None else cargar()
            args=crear_args(config)
            datos['calibracion']=config
            det=crear_detector();previa=None;consulta=0;archivo=0
            def configurar(orden):
                nonlocal args
                config=validar(json.loads(orden.partition(' ')[2]))
                if perfil is None:guardar(config)
                else:guardar(config,perfil)
                args=crear_args(config)
                datos.update(calibracion=config,evento='Calibración guardada',error='')
                registrar('calibracion',**config)
            if imagen:
                im=cv2.imread(str(imagen))
                if im is None:raise RuntimeError('No se puede leer la foto de prueba')
                while not parar.is_set():
                    try:
                        _,orden=ordenes.get(timeout=.15)
                        if orden.startswith('vision_cfg '):configurar(orden)
                    except queue.Empty:pass
                    except (ValueError,RuntimeError) as e:datos['error']=str(e)
                    ver=mostrar_video()
                    foto,mask,r=procesar(im,det,args,dibujar=ver)
                    datos.update(foto=foto if ver else None,mascara=mask if ver else None,r=r,ultima=time.monotonic(),conexion='Vista de prueba: sin dispositivo',demo=True)
                    publicar()
                return
            src=FuenteUSB(puerto)
            firmware=src.leer_estado(2)
            if firmware.get('version') not in ('camara-diag-0.4','camara-diag-0.5'):raise RuntimeError('Este panel requiere firmware de cámara 0.4 o 0.5')
            disponible=firmware['version']=='camara-diag-0.5'
            soporta_cfg='max_delta_deg' in firmware.get('servos',{})
            datos.update(conexion='Conectada · USB rápido',servo=firmware['servos'],lista=True,control_disponible=disponible,control_activo=False)
            def cmd(s):src.port.write((s+'\n').encode())
            def ejecutar(orden):
                esperado={'servo_arm':'# SERVO_ARM','servo_test':'# SERVO_TEST','servo_centro':'# SERVO_CENTRO','servo_off':'# SERVO_OFF','servo_set':'# SERVO_SET','servo_cfg':'# SERVO_CFG'}[orden.split()[0]]
                cmd('hb');cmd(orden)
                fin=time.monotonic()+.8
                while time.monotonic()<fin:
                    l=src.port.readline().decode(errors='replace').strip()
                    if l.startswith('# ERROR_SERVO'):raise RuntimeError(l)
                    if l.startswith(esperado):
                        if not orden.startswith('servo_set '):
                            registrar('respuesta',orden=orden,texto=l)
                            datos.update(evento=l[2:],error='')
                        return
                raise RuntimeError('No llegó confirmación del comando de servo')
            jpeg_malos=0;ui_cortada=False;inicio_imagen=time.monotonic();control=None;buenas=0;fallos_control=0;ultimos_angulos=[90.0,90.0]
            def cortar_control(razon):
                nonlocal control,fallos_control
                control=None;fallos_control=0;datos.update(control_activo=False,error=razon,evento=razon)
                ejecutar('servo_off')
                datos.update(error=razon,evento=razon)
                registrar('control_detenido',motivo=razon)
            while not parar.is_set():
                if time.monotonic()-ui_viva.value>2:
                    if not ui_cortada:
                        control=None;datos['control_activo']=False
                        ejecutar('servo_off');registrar('interfaz_inactiva');ui_cortada=True
                        with generacion.get_lock():generacion.value+=1
                    datos.update(error='Interfaz sin respuesta: PWM apagado')
                else:ui_cortada=False
                if apagar.is_set():
                    control=None;datos['control_activo']=False
                    ejecutar('servo_off');apagar.clear()
                while True:
                    try:gen,orden=ordenes.get_nowait()
                    except queue.Empty:break
                    if gen!=generacion.value or ui_cortada:continue
                    try:
                        if orden.startswith('vision_cfg '):
                            if control is not None:cortar_control('Cambio de calibración: control detenido')
                            configurar(orden);buenas=0
                            continue
                        elif orden.startswith('control_on '):
                            if not disponible:raise RuntimeError('Se requiere firmware 0.5 para control')
                            if buenas<2 or not datos['r'].get('control_valido') or time.monotonic()-datos['ultima']>.45:raise RuntimeError('Se requieren cuatro ArUco observados, pelota y escala entre centros')
                            partes=orden.split()
                            e0,s0,e1,s1=int(partes[1]),int(partes[2]),int(partes[3]),int(partes[4])
                            kp=float(partes[5]) if len(partes)>5 else 0.55
                            kd=float(partes[6]) if len(partes)>6 else 0.80
                            max_d=float(partes[7]) if len(partes)>7 else 10.0
                            c0=float(partes[8]) if len(partes)>8 else 90.0
                            c1=float(partes[9]) if len(partes)>9 else 90.0
                            ki=float(partes[10]) if len(partes)>10 else 0.50
                            if soporta_cfg:
                                ejecutar(f'servo_cfg {max_d:.1f} {c0:.1f} {c1:.1f}')
                            else:
                                max_d,c0,c1=2.0,90.0,90.0
                            candidato=ControlPD([(e0,s0),(e1,s1)],kp=kp,kd=kd,max_deg=max_d,centros=(c0,c1),ki=ki)
                            ultimos_angulos=candidato.paso(datos['r']['bola']['xy'],datos['ultima'])
                            ejecutar(f'servo_arm {c0:.1f} {c1:.1f}' if soporta_cfg else 'servo_arm 90 90')
                            control=candidato;fallos_control=0
                            datos.update(control_activo=True,evento=f'CONTROL ACTIVO · ±{max_d:g}° (Kp={kp:g}, Kd={kd:g})',error='')
                            registrar('control_iniciado',mapa=candidato.mapa,kp=candidato.kp,kd=candidato.kd,max_deg=max_d,centros=[c0,c1])
                        else:
                            if control is not None:
                                cortar_control('Cambio a prueba manual')
                            ejecutar(orden)
                    except RuntimeError as e:datos.update(error=str(e),evento='Comando rechazado')
                    except ValueError as e:datos.update(error=str(e),evento='Configuración no válida')
                cmd('hb')
                t=time.monotonic()
                try:im=src.read()
                except (OSError,RuntimeError,ValueError) as e:
                    jpeg_malos+=1;registrar('imagen_descartada',mensaje=str(e))
                    if control is not None:
                        fallos_control+=1
                        if fallos_control<=15:
                            try:ejecutar(f'servo_set {ultimos_angulos[0]:.2f} {ultimos_angulos[1]:.2f}')
                            except RuntimeError:pass
                            continue
                        cortar_control('Control detenido: fallaron imágenes durante más de 0.8 s')
                    buenas=0
                    datos.update(error=str(e));publicar()
                    if previa is None:
                        time.sleep(.05)
                    elif jpeg_malos>=6:
                        try:src.port.reset_input_buffer()
                        except OSError:pass
                        jpeg_malos=0
                    continue
                if jpeg_malos:datos['error']=''
                jpeg_malos=0
                ver=mostrar_video();guardar_img=guardar_fotos()
                t_vision=time.monotonic()
                foto,mask,r=procesar(im,det,args,dibujar=ver or guardar_img);ahora=time.monotonic()
                vision_ms=(ahora-t_vision)*1000
                fps=1/(ahora-previa) if previa else 0;previa=ahora
                valida=r['control_valido'] and ahora-t<.30
                buenas=buenas+1 if valida else 0
                if control is not None:
                    try:
                        if not valida:
                            fallos_control+=1
                            if fallos_control<=15:
                                ejecutar(f'servo_set {ultimos_angulos[0]:.2f} {ultimos_angulos[1]:.2f}')
                            else:
                                raise ValueError('Se perdió la pelota o ArUco durante más de 0.8 s')
                        else:
                            fallos_control=0
                            # Acotar posición a los límites seguros para no cortar el control
                            # cuando la pelota toca el borde, sino aplicar el máximo ángulo corrector.
                            bx,by=r['bola']['xy']
                            xy_acotada=[max(-8.5,min(8.5,bx)),max(-6.1,min(6.1,by))]
                            angulos=control.paso(xy_acotada,ahora)
                            ultimos_angulos=angulos
                            ejecutar(f'servo_set {angulos[0]:.2f} {angulos[1]:.2f}')
                            datos['evento']=f'CONTROL AL CENTRO · ±{control.max_deg:g}°'
                            registrar('control',xy_cm=r['bola']['xy'],mando_deg=angulos)
                    except (ValueError,RuntimeError) as e:cortar_control('Control detenido: '+str(e))
                if ahora-consulta>.45:
                    consulta=ahora
                    try:
                        firmware=src.leer_estado(1)
                        if firmware['servos']['error_hardware']:raise RuntimeError('Error al activar canales PWM')
                        if control is not None and not firmware['servos']['habilitado']:
                            cortar_control('Control detenido por vigilancia del firmware')
                        registrar('servos',fps=fps,**firmware['servos']);datos['servo']=firmware['servos']
                    except RuntimeError:
                        try:src.port.reset_input_buffer()
                        except OSError:pass
                datos.update(foto=foto if ver else None,mascara=mask if ver else None,r=r,ultima=ahora,fps=fps,ms=(ahora-t)*1000,vision_ms=vision_ms)
                publicar()
                if ahora-archivo>.5:
                    if guardar_img:
                        cv2.imwrite(str(guardado/'camara.jpg'),foto)
                        cv2.imwrite(str(guardado/'original.jpg'),im)
                        cv2.imwrite(str(guardado/'mascara.png'),mask)
                    registrar('vision',**r)
                    estado={'vision':r,'servos':firmware['servos'],'fecha':datetime.now().astimezone().isoformat(),
                            'fps':fps,'vision_ms':vision_ms,'ciclo_ms':datos['ms'],
                            'video_visible':ver,'guardando_imagenes':guardar_img,'control_activo':control is not None}
                    temporal=guardado/'estado.tmp';temporal.write_text(json.dumps(estado,indent=2,ensure_ascii=False)+'\n');temporal.replace(guardado/'estado.json');archivo=ahora
        except Exception as e:
            registrar('error',mensaje=str(e))
            datos.update(error=str(e),conexion='Conexión interrumpida',lista=False)
            try:salida.put(datos.copy(),timeout=.5)
            except queue.Full:pass
            time.sleep(.2)
        finally:
            if src:
                try:src.port.write(b'servo_centro\n');time.sleep(.15);src.port.write(b'servo_off\n');time.sleep(.08)
                except OSError:pass
                src.close()


class Panel(tk.Tk):
    def __init__(self,puerto,imagen=None,sin_video=True,guardar_imagenes=False):
        super().__init__()
        self.title('DSM — Ball & Plate · Cámara y servos')
        self.geometry('1160x860');self.configure(bg='#eef2f7')
        ctx=mp.get_context('spawn')
        self.puerto=puerto;self.imagen=imagen;self.ordenes=ctx.Queue();self.salida=ctx.Queue(maxsize=1)
        self.parar=ctx.Event();self.apagar=ctx.Event();self.generacion=ctx.Value('i',0);self.ui_viva=ctx.Value('d',time.monotonic())
        self.prueba_activa=False
        self.video_activo=ctx.Value('b',not sin_video)
        self.guardar_imagenes=ctx.Value('b',guardar_imagenes)
        self.ver_video=tk.BooleanVar(value=not sin_video)
        self.guardar_fotos_var=tk.BooleanVar(value=guardar_imagenes)
        config=cargar()
        self.ver_mascara=tk.BooleanVar(value=False)
        self.vision_vars={k:tk.StringVar(value='' if config[k] is None else str(config[k]))
                          for k in ('contraste','ventana_local','area_min','area_max','circularidad','ancho_cm','alto_cm')}
        self.auto_contraste=tk.BooleanVar(value=config['auto_contraste'])
        self.datos={'conexion':'Conectando cámara…','foto':None,'error':'','servo':{},'ultima':0,'r':None,'fps':0,'evento':'PWM apagado al arrancar'}
        self.guardado=RAIZ/'estado_camara'/'panel';self.guardado.mkdir(parents=True,exist_ok=True)
        registro=self.guardado/datetime.now().strftime('eventos_%Y%m%d_%H%M%S.jsonl')
        ttk.Style().configure('TButton',font=('Sans',11),padding=6)
        cab=tk.Frame(self,bg='#13243a');cab.pack(fill='x')
        tk.Label(cab,text='Ball & Plate',font=('Sans',23,'bold'),fg='white',bg='#13243a').pack(anchor='w',padx=20,pady=(12,0))
        tk.Label(cab,text='Control por cámara · Servos GPIO41 / GPIO42',font=('Sans',11),fg='#b9d9f8',bg='#13243a',wraplength=390).pack(anchor='w',padx=20,pady=(0,4))
        tk.Checkbutton(cab,text='Mostrar vídeo',variable=self.ver_video,command=self.cambiar_video,bg='#13243a',fg='white',selectcolor='#13243a',activebackground='#13243a',activeforeground='white').pack(anchor='w',padx=16,pady=(0,8))
        self.medida=tk.StringVar(value='Esperando referencias de la bandeja')
        tk.Label(self,textvariable=self.medida,font=('Sans',13,'bold'),bg='white',fg='#123852',anchor='w',wraplength=390,padx=12,pady=10).pack(fill='x')
        cuerpo=tk.Frame(self,bg='#eef2f7');cuerpo.pack(fill='both',expand=True,padx=14,pady=12)
        izquierda=tk.Frame(cuerpo,bg='white',width=760);izquierda.pack(side='left',fill='both',expand=True);izquierda.pack_propagate(False);self.izquierda=izquierda
        self.video=tk.Label(izquierda,text='Esperando imagen…',bg='#12202d',fg='white',font=('Sans',16));self.video.pack(fill='both',expand=True)
        lateral=tk.Frame(cuerpo,bg='#eef2f7',width=330);lateral.pack(side='right',fill='y',padx=(14,0));lateral.pack_propagate(False);self.lateral=lateral
        tk.Button(lateral,text='PARAR · APAGAR PWM',command=self.detener,bg='#c53232',fg='white',activebackground='#a72222',font=('Sans',12,'bold'),pady=12,relief='flat').pack(fill='x',side='bottom',pady=(10,0))
        self.pestanas=ttk.Notebook(lateral);self.pestanas.pack(fill='both',expand=True)
        derecha=tk.Frame(self.pestanas,bg='#eef2f7');self.pestanas.add(derecha,text='Pruebas manuales')
        automatico=tk.Frame(self.pestanas,bg='#eef2f7');self.pestanas.add(automatico,text='Control al centro')
        camara=tk.Frame(self.pestanas,bg='#eef2f7');self.pestanas.add(camara,text='Cámara')
        tk.Label(camara,text='Calibración de visión',font=('Sans',14,'bold'),bg='#eef2f7').pack(anchor='w',pady=10)
        tk.Label(camara,text='Pelota metálica. Ajusta con la pelota en varias posiciones y comprueba también la bandeja vacía.',wraplength=265,justify='left',bg='#eef2f7').pack(anchor='w',pady=6)
        campos=tk.Frame(camara,bg='#eef2f7');campos.pack(fill='x')
        for fila,(key,label) in enumerate([
            ('contraste','Contraste local (1–254)'),('ventana_local','Ventana impar (7–151)'),
            ('area_min','Área mínima (px²)'),('area_max','Área máxima (px²)'),
            ('circularidad','Circularidad (0–1)'),('ancho_cm','Centros 0→1 (cm)'),('alto_cm','Centros 0→3 (cm)')]):
            tk.Label(campos,text=label,bg='#eef2f7').grid(row=fila,column=0,sticky='w',pady=4)
            ttk.Entry(campos,textvariable=self.vision_vars[key],width=9).grid(row=fila,column=1,sticky='e',padx=4)
        tk.Checkbutton(camara,text='Recuperar contraste automáticamente',variable=self.auto_contraste,bg='#eef2f7',wraplength=265).pack(anchor='w',pady=8)
        tk.Checkbutton(camara,text='Ver máscara de detección',variable=self.ver_mascara,command=self.mostrar_mascara,bg='#eef2f7').pack(anchor='w')
        tk.Checkbutton(camara,text='Guardar fotos para diagnóstico',variable=self.guardar_fotos_var,command=lambda:setattr(self.guardar_imagenes,'value',self.guardar_fotos_var.get()),bg='#eef2f7').pack(anchor='w')
        ttk.Button(camara,text='Aplicar y guardar calibración',command=self.aplicar_vision).pack(fill='x',pady=10)
        self.diagnostico=tk.StringVar(value='Esperando imagen')
        tk.Label(camara,textvariable=self.diagnostico,bg='#eef2f7',wraplength=265,justify='left').pack(anchor='w',pady=8)
        tk.Label(camara,text='La escala exige medidas entre CENTROS de ArUco. Deja ambas vacías si todavía no las has medido.',bg='#eef2f7',wraplength=265,justify='left').pack(anchor='w',pady=6)
        ttk.Button(derecha,text='Guardar vista',command=self.capturar).pack(fill='x',side='bottom')
        tk.Label(derecha,text='Prueba de servos',font=('Sans',16,'bold'),bg='#eef2f7').pack(anchor='w',pady=(0,8))
        tk.Label(derecha,text='Centros: 90° / 90°\nPruebas: ±2° durante 0,5 s\nRegresan a 90° automáticamente',justify='left',font=('Sans',11),bg='#eef2f7',fg='#42546a').pack(anchor='w',pady=(0,12))
        self.botones=[]
        def boton(texto,orden):
            b=ttk.Button(derecha,text=texto,command=lambda:self.encolar(orden));b.pack(fill='x',pady=4);b.state(['disabled']);self.botones.append(b)
        boton('Activar y centrar a 90°','servo_arm 90 90')
        boton('GPIO41 · +2°','servo_test 41 2 500')
        boton('GPIO41 · −2°','servo_test 41 -2 500')
        boton('GPIO42 · +2°','servo_test 42 2 500')
        boton('GPIO42 · −2°','servo_test 42 -2 500')
        boton('Volver al centro','servo_centro')
        self.servos=tk.StringVar(value='PWM apagado')
        self.resumen_servos=tk.StringVar(value='PWM apagado')
        tk.Label(derecha,textvariable=self.servos,justify='left',wraplength=265,bg='#eef2f7',fg='#163450',font=('Sans',11)).pack(anchor='w',pady=8)
        self.evento=tk.StringVar(value='Esperando conexión')
        tk.Label(derecha,textvariable=self.evento,justify='left',wraplength=260,bg='#eef2f7',fg='#43546c',font=('Sans',10)).pack(anchor='w',pady=8)
        tk.Label(automatico,text='Equilibrio al centro (0, 0)',font=('Sans',14,'bold'),bg='#eef2f7',wraplength=270).pack(anchor='w',pady=(10,8))
        tk.Label(automatico,text='Pon la pelota cerca del centro. Confirma ejes y sentido con las pruebas manuales de ±2°.',justify='left',wraplength=340,font=('Sans',11),bg='#eef2f7').pack(anchor='w',pady=8)
        self.opciones_mapa={'Sin confirmar':None,'Mueve hacia +X':(0,1),'Mueve hacia −X':(0,-1),'Mueve hacia +Y':(1,1),'Mueve hacia −Y':(1,-1)}
        self.mapa_vars=[]
        for pin,defecto in ((41,'Mueve hacia +Y'),(42,'Mueve hacia +X')):
            tk.Label(automatico,text=f'GPIO{pin} +2° mueve la pelota:',bg='#eef2f7',font=('Sans',10,'bold')).pack(anchor='w',pady=(6,2))
            v=tk.StringVar(value=defecto);self.mapa_vars.append(v)
            ttk.Combobox(automatico,textvariable=v,values=list(self.opciones_mapa),state='readonly').pack(fill='x',pady=2)
        params=tk.Frame(automatico,bg='#eef2f7');params.pack(fill='x',pady=(8,4))
        self.kp_var=tk.DoubleVar(value=0.55);self.kd_var=tk.DoubleVar(value=0.80);self.ki_var=tk.DoubleVar(value=0.50)
        self.max_deg_var=tk.DoubleVar(value=10.0);self.c41_var=tk.DoubleVar(value=90.0);self.c42_var=tk.DoubleVar(value=90.0)
        for fila,(txt,var,mn,mx,inc) in enumerate([
            ('Límite servo (±°):',self.max_deg_var,2.0,15.0,1.0),
            ('Kp (°/cm):',self.kp_var,0.10,3.00,0.05),
            ('Kd (°/(cm/s)):',self.kd_var,0.00,2.00,0.05),
            ('Ki (°/(cm·s)):',self.ki_var,0.00,1.50,0.05),
            ('Centro GPIO41 (°):',self.c41_var,80.0,100.0,0.5),
            ('Centro GPIO42 (°):',self.c42_var,80.0,100.0,0.5),
        ]):
            tk.Label(params,text=txt,bg='#eef2f7',font=('Sans',9)).grid(row=fila,column=0,sticky='w',pady=1)
            tk.Spinbox(params,textvariable=var,from_=mn,to=mx,increment=inc,width=6,font=('Sans',9)).grid(row=fila,column=1,sticky='e',padx=(8,0),pady=1)
        params.columnconfigure(0,weight=1)
        self.iniciar_boton=ttk.Button(automatico,text='INICIAR CONTROL AL CENTRO',command=self.iniciar_control);self.iniciar_boton.pack(fill='x',pady=(10,6));self.iniciar_boton.state(['disabled'])
        self.estado_control=tk.StringVar(value='Confirma el movimiento de ambos servos')
        tk.Label(automatico,textvariable=self.estado_control,wraplength=265,justify='left',font=('Sans',11,'bold'),bg='#eef2f7',fg='#163450').pack(anchor='w',pady=6)
        tk.Label(automatico,text='PARAR o Escape apaga el PWM.',wraplength=360,justify='left',font=('Sans',9),bg='#eef2f7').pack(anchor='w',pady=4)
        tk.Label(automatico,textvariable=self.resumen_servos,wraplength=360,justify='left',font=('Sans',10),bg='#eef2f7').pack(anchor='w',pady=6)
        tk.Label(automatico,textvariable=self.evento,wraplength=360,justify='left',font=('Sans',10),bg='#eef2f7').pack(anchor='w',pady=4)
        self.estado=tk.StringVar(value='Conectando…')
        tk.Label(self,textvariable=self.estado,bg='#dbe6f1',fg='#1f344a',anchor='w',padx=16,pady=9,font=('Sans',10)).pack(fill='x',side='bottom',before=cuerpo)
        self.protocol('WM_DELETE_WINDOW',self.cerrar)
        self.bind('<Escape>',lambda e:self.detener())
        self.cambiar_video()
        self.receptor=ctx.Process(target=trabajo_panel,args=(puerto,imagen,self.ordenes,self.salida,self.parar,self.apagar,self.generacion,self.ui_viva,self.guardado,registro,None,self.video_activo,self.guardar_imagenes),daemon=True)
        self.receptor.start();self.pestanas.select(1);self.after(50,self.actualizar)

    def cambiar_video(self):
        ver=self.ver_video.get()
        self.video_activo.value=ver
        if ver:
            self.geometry('1160x930')
            self.izquierda.pack(side='left',fill='both',expand=True,before=self.lateral)
            self.lateral.pack_configure(fill='y',expand=False,padx=(14,0))
        else:
            self.izquierda.pack_forget()
            self.lateral.pack_configure(fill='both',expand=True,padx=0)
            self.video.configure(image='',text='Vídeo desactivado')
            self.imagen_tk=None
            self.geometry('430x930')

    def mostrar_mascara(self):
        if self.ver_mascara.get():
            self.ver_video.set(True);self.cambiar_video()

    def encolar(self,orden,automatico=False):
        if not automatico:self.prueba_activa=False
        self.ordenes.put((self.generacion.value,orden))
        self.datos['evento']='Solicitado: '+orden

    def aplicar_vision(self):
        try:
            config=cargar()
            for key,var in self.vision_vars.items():
                valor=var.get().strip().replace(',','.')
                if key in ('ancho_cm','alto_cm') and not valor:config[key]=None
                else:config[key]=int(valor) if key in ('contraste','ventana_local') else float(valor)
            config['auto_contraste']=self.auto_contraste.get()
            config=validar(config)
        except (ValueError,tk.TclError) as e:
            self.diagnostico.set(str(e));return
        self.encolar('vision_cfg '+json.dumps(config))
        self.diagnostico.set('Calibración enviada; se detendrá el control si estaba activo')

    def detener(self):
        self.prueba_activa=False
        with self.generacion.get_lock():self.generacion.value+=1
        self.apagar.set()
        self.datos['evento']='Parada solicitada'

    def iniciar_control(self):
        mapa=[self.opciones_mapa[v.get()] for v in self.mapa_vars]
        if any(m is None for m in mapa) or mapa[0][0]==mapa[1][0]:
            self.estado_control.set('Cada servo debe mover un eje distinto. Revisa las pruebas +2°.')
            return
        try:
            kp=float(self.kp_var.get());kd=float(self.kd_var.get());ki=float(self.ki_var.get())
            mx=float(self.max_deg_var.get());c0=float(self.c41_var.get());c1=float(self.c42_var.get())
        except (ValueError,tk.TclError):
            self.estado_control.set('Revisa los valores numéricos de Kp, Kd, Ki, límite y centros.')
            return
        self.encolar(f'control_on {mapa[0][0]} {mapa[0][1]} {mapa[1][0]} {mapa[1][1]} {kp:.3f} {kd:.3f} {mx:.1f} {c0:.1f} {c1:.1f} {ki:.3f}')

    def actualizar(self):
        self.ui_viva.value=time.monotonic()
        nueva=False
        while True:
            try:self.datos=self.salida.get_nowait();nueva=True
            except queue.Empty:break
        d=self.datos.copy()
        if self.ver_video.get() and d['foto'] is not None and nueva:
            fuente=d.get('mascara') if self.ver_mascara.get() else d['foto']
            if fuente is None:fuente=d['foto']
            frame=cv2.cvtColor(fuente,cv2.COLOR_GRAY2RGB if fuente.ndim==2 else cv2.COLOR_BGR2RGB)
            ancho=max(320,self.video.winfo_width());alto=max(240,self.video.winfo_height())
            ratio=min(ancho/frame.shape[1],alto/frame.shape[0])
            img=Image.fromarray(frame).resize((int(frame.shape[1]*ratio),int(frame.shape[0]*ratio)))
            self.imagen_tk=ImageTk.PhotoImage(img);self.video.configure(image=self.imagen_tk,text='')
        if nueva and d.get('r') is not None:
            r=d['r'];bola=r['bola'];edad=time.monotonic()-d['ultima']
            if d.get('demo'):self.medida.set('Imagen guardada · controles desactivados')
            elif edad>2:self.medida.set('IMAGEN ANTIGUA · no hay medición actual')
            elif bola:self.medida.set(f"Pelota: X {bola['xy'][0]:+.2f}    Y {bola['xy'][1]:+.2f} ({r['unidad']})" + (' · referencia en memoria' if not r.get('geometria_observada') else ''))
            else:self.medida.set(r['estado'])
            diag=r.get('diagnostico_bola',{})
            self.diagnostico.set(f"{r['estado']}\nCandidatos: {diag.get('candidatos',0)} · contraste usado: {diag.get('contraste_usado','—')}\nÁreas (px²): {diag.get('areas_px',[])}")
        serv=d['servo'];activo=serv.get('pwm_activo',False)
        pos=serv.get('mando_deg',[90,90])
        self.servos.set(f"PWM: {'ACTIVO' if activo else 'APAGADO'}\nMando GPIO41: {pos[0]:.1f}°\nMando GPIO42: {pos[1]:.1f}°\nÁngulos ordenados, sin encoder")
        self.resumen_servos.set(f"PWM: {'ACTIVO' if activo else 'APAGADO'}\nGPIO41: {pos[0]:.1f}° · GPIO42: {pos[1]:.1f}°")
        self.evento.set(d['error'] or d['evento'])
        centros=(d.get('r') or {}).get('centros_cm')
        escala=f'Centros: {centros[0]:g} × {centros[1]:g} cm' if centros else 'Escala entre centros pendiente'
        modo='Vídeo visible' if self.ver_video.get() else 'Control sin vídeo'
        self.estado.set(f"{modo} · {d.get('fps',0):.1f} FPS\n{d['conexion']} · {escala}")
        reciente=time.monotonic()-d['ultima']<2
        for i,b in enumerate(self.botones):
            disponible=d.get('lista') and reciente and (i in (0,5) or activo)
            b.state(['!disabled'] if disponible else ['disabled'])
        mapa=[self.opciones_mapa[v.get()] for v in self.mapa_vars]
        confirmado=all(m is not None for m in mapa) and mapa[0][0]!=mapa[1][0]
        bola=d.get('r') and d['r'].get('control_valido',False)
        puede=d.get('lista') and reciente and d.get('control_disponible') and confirmado and bola
        self.iniciar_boton.state(['!disabled'] if puede and not d.get('control_activo') else ['disabled'])
        if d.get('demo'):self.estado_control.set('Vista de prueba · controles desactivados')
        elif d.get('control_activo'):self.estado_control.set('CONTROL ACTIVO · objetivo X=0, Y=0')
        elif not confirmado:self.estado_control.set('Confirma el movimiento de ambos servos')
        elif not d.get('control_disponible'):self.estado_control.set('Se requiere firmware de cámara 0.5')
        elif not centros:self.estado_control.set('Mide la escala entre centros en la pestaña Cámara')
        elif not bola:self.estado_control.set('Coloca la pelota dentro de los cuatro ArUco')
        else:self.estado_control.set('Listo para una primera prueba al centro')
        if d.get('r') is not None and not reciente and not d.get('demo'):
            self.medida.set('IMAGEN ANTIGUA · no hay medición actual')
        if not self.parar.is_set():self.after(60,self.actualizar)

    def capturar(self):
        self.update_idletasks()
        caja=(self.winfo_rootx(),self.winfo_rooty(),self.winfo_rootx()+self.winfo_width(),self.winfo_rooty()+self.winfo_height())
        ruta=self.guardado/datetime.now().strftime('panel_%Y%m%d_%H%M%S.png')
        ImageGrab.grab(bbox=caja).save(ruta)
        self.datos['evento']='Vista guardada en '+str(ruta)

    def prueba_inicial(self,intento=0):
        if self.parar.is_set():return
        d=self.datos.copy()
        if not d.get('lista') or d.get('r') is None:
            if intento<100:self.after(200,lambda:self.prueba_inicial(intento+1))
            return
        self.prueba_activa=True
        self.encolar('servo_arm 90 90',automatico=True)
        def paso(orden):
            if self.prueba_activa and not self.parar.is_set():self.encolar(orden,automatico=True)
        self.after(1500,lambda:paso('servo_test 41 2 500'))
        self.after(3000,lambda:paso('servo_test 42 2 500'))
        self.after(4500,lambda:self.detener() if self.prueba_activa else None)

    def cerrar(self):
        self.prueba_activa=False
        self.parar.set();self.apagar.set();self.destroy()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    p.add_argument('--imagen',type=Path,help='Vista sin hardware para comprobar la interfaz')
    p.add_argument('--capturar-despues',type=int,help='Guardar captura del panel tras N ms')
    p.add_argument('--prueba-inicial',action='store_true',help='90/90, +2 en GPIO41, +2 en GPIO42, retorno y apagado')
    p.add_argument('--con-video',action='store_true',help='Mostrar vídeo; por defecto se prioriza el control sin vídeo')
    p.add_argument('--guardar-imagenes',action='store_true',help='Guardar fotos de diagnóstico a ~2 Hz')
    a=p.parse_args();app=Panel(a.port,a.imagen,sin_video=not a.con_video,guardar_imagenes=a.guardar_imagenes)
    signal.signal(signal.SIGINT,lambda *_:app.after(0,app.cerrar))
    if a.capturar_despues:app.after(a.capturar_despues,app.capturar)
    if a.prueba_inicial:app.after(200,app.prueba_inicial)
    try:app.mainloop()
    finally:
        app.parar.set();app.receptor.join(timeout=25)
