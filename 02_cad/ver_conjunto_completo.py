"""Dos vistas del conjunto neutral recompuesto y de su interfaz heredada."""
from pathlib import Path
import json
import os
import vtk

ROOT=Path(__file__).resolve().parent/'junta_nuevo_de_8_motores_con_camara/conjunto_completo'
os.environ.setdefault('MESA_SHADER_CACHE_DIR',str(ROOT/'cache'))


def text(renderer,value,position,size=18):
    actor=vtk.vtkTextActor();actor.SetInput(value);actor.SetPosition(*position)
    actor.GetTextProperty().SetFontSize(size);actor.GetTextProperty().SetColor(.08,.12,.18)
    actor.GetTextProperty().SetBackgroundColor(.94,.96,.98);actor.GetTextProperty().SetBackgroundOpacity(.9)
    renderer.AddViewProp(actor)


def main():
    data=json.loads((ROOT/'ensamblaje_completo.json').read_text())
    window=vtk.vtkRenderWindow();window.SetOffScreenRendering(True);window.SetSize(1500,950)
    geometry=[]
    for part in data['piezas']:
        reader=vtk.vtkSTLReader();reader.SetFileName(str(ROOT/part['malla']));reader.Update()
        geometry.append((part,reader.GetOutput()))
    for i in range(2):
        renderer=vtk.vtkRenderer();renderer.SetViewport(i*.5,0,(i+1)*.5,1)
        renderer.SetBackground(.94,.96,.98);window.AddRenderer(renderer)
        for part,poly in geometry:
            if i==1:
                source=part['ruta_origen']
                from_new_uav=source[0].startswith(('ball_and_plate','x8_UAV'))
                if from_new_uav and part['nombre'] not in ('IMPRIMIBLE tapa_central:1','IMPRIMIBLE base_Q1:1','IMPRIMIBLE base_Q2:1','IMPRIMIBLE base_Q3:1','IMPRIMIBLE base_Q4:1'):
                    continue
                if 'HELICE' in part['nombre']:continue
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly)
            actor=vtk.vtkActor();actor.SetMapper(mapper);actor.GetProperty().SetColor(*part['color'][:3])
            if i==1 and part['nombre'] in ('Base.ipt:1','joint_base_esphericaljoint:1'):
                actor.GetProperty().SetColor(.8,.15,.5)
                actor.GetProperty().SetOpacity(.25)
            renderer.AddActor(actor)
        camera=renderer.GetActiveCamera();camera.SetViewUp(0,0,1);camera.SetParallelProjection(True)
        camera.SetPosition(1,-1.7,1.15);renderer.ResetCamera()
        if i==0:
            camera.Zoom(1.30)
            text(renderer,'Carrito + UAV de 8 motores + bandeja nueva',(20,905),22)
            text(renderer,'Copia local con geometria CAD\nOriginales conservados; archivo STEP neutral',(20,22),16)
        else:
            pivot=data['pivot_modelo_m']
            camera.SetFocalPoint(pivot[0],pivot[1],pivot[2]-.08)
            camera.SetPosition(pivot[0]+.65,pivot[1]-.95,pivot[2]+.45)
            camera.SetParallelScale(.24)
            text(renderer,'Detalle de guias y rotula',(20,905),22)
            text(renderer,'Azul: conjunto que sube | Gris: estructura fija\nMagenta: interfaz original con solapes por revisar',(20,22),16)
        renderer.ResetCameraClippingRange()
    window.Render();capture=vtk.vtkWindowToImageFilter();capture.SetInput(window)
    capture.ReadFrontBufferOff();capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(ROOT/'vista_conjunto_completo.png'))
    writer.SetInputConnection(capture.GetOutputPort());writer.Write();window.Finalize()
    print(ROOT/'vista_conjunto_completo.png')


if __name__=='__main__':main()
