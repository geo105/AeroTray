"""Vista local del STEP recompuesto, a partir de sus mallas en metros."""
from pathlib import Path
import json
import os
import vtk

ROOT = Path(__file__).resolve().parent / 'junta_nuevo_de_8_motores_con_camara'
os.environ.setdefault('MESA_SHADER_CACHE_DIR', str(ROOT / 'cache'))


def color(part):
    n = part['ruta_origen'][1]
    if n.startswith('CAM '):
        return (0.84, 0.42, 0.12)
    if 'servo' in n.lower() or 'brazo 25T' in n:
        return (0.12, 0.35, 0.65)
    if 'bola 25' in n:
        return (0.83, 0.12, 0.13)
    if 'mesa mitad' in n or 'pantalla' in n:
        return (0.25, 0.58, 0.39)
    if part['grupo'] == 'bandeja_camara':
        return (0.46, 0.49, 0.52)
    return (0.23, 0.27, 0.32)


def main():
    report = json.loads((ROOT / 'ensamblaje.json').read_text())
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(True)
    window.SetSize(1500, 850)
    data = []
    for part in report['piezas']:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(ROOT / part['malla']))
        reader.Update()
        data.append((part, reader.GetOutput()))
    for i, (position, title) in enumerate([
        ((1,-1.5,1.0), 'UAV de 8 motores + bandeja con camara'),
        ((1,-.15,.32), 'Vista lateral del montaje')]):
        ren = vtk.vtkRenderer()
        ren.SetViewport(i*.5, 0, (i+1)*.5, 1)
        ren.SetBackground(.94,.96,.98)
        window.AddRenderer(ren)
        for part, poly in data:
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(poly)
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color(part))
            actor.GetProperty().SetSpecular(.18)
            ren.AddActor(actor)
        camera = ren.GetActiveCamera()
        camera.SetPosition(*position)
        camera.SetViewUp(0,0,1)
        camera.SetParallelProjection(True)
        ren.ResetCamera()
        camera.Zoom(1.15)
        label = vtk.vtkTextActor()
        label.SetInput(title)
        label.SetPosition(20,805)
        label.GetTextProperty().SetFontSize(21)
        label.GetTextProperty().SetColor(.08,.12,.18)
        ren.AddViewProp(label)
        foot = vtk.vtkTextActor()
        foot.SetInput('Copia neutral local | 58 componentes CAD\nCarrito, rotula y uniones pendientes de integrar')
        foot.SetPosition(20,20)
        foot.GetTextProperty().SetFontSize(15)
        foot.GetTextProperty().SetColor(.16,.20,.25)
        ren.AddViewProp(foot)
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.ReadFrontBufferOff()
    capture.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(ROOT / 'vista_previa.png'))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()
    print(ROOT / 'vista_previa.png')


if __name__ == '__main__':
    main()
