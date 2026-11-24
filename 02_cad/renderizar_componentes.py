"""Vista estática de los componentes recibidos; escalas de cámara independientes."""
from pathlib import Path
import json
import vtk

ROOT = Path(__file__).resolve().parent


def main():
    data = json.loads((ROOT / 'procesados/inventario_descargas.json').read_text())
    parts = [x for x in data['archivos'] if x.get('estado') == 'STEP_leido_y_malla_generada']
    if not parts:
        raise RuntimeError('No hay mallas STEP verificadas para mostrar')
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1200, 480)
    renderers = []
    for i, item in enumerate(parts):
        renderer = vtk.vtkRenderer()
        renderer.SetViewport(i / len(parts), 0, (i + 1) / len(parts), 1)
        renderer.SetBackground(0.94, 0.96, 0.98)
        window.AddRenderer(renderer)
        reader = vtk.vtkSTLReader()
        reader.SetFileName(item['malla_metros'])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.27, 0.43, 0.63)
        renderer.AddActor(actor)
        renderer.GetActiveCamera().SetPosition(1, -1.5, 1.1)
        renderer.GetActiveCamera().SetViewUp(0, 0, 1)
        renderer.ResetCamera()
        renderer.GetActiveCamera().Zoom(0.75)
        title = vtk.vtkTextActor()
        title.SetInput(Path(item['archivo']).stem)
        title.SetPosition(15, 445)
        title.GetTextProperty().SetFontSize(15)
        title.GetTextProperty().SetColor(0.08, 0.12, 0.18)
        renderer.AddActor2D(title)
        footer = vtk.vtkTextActor()
        dimensions = ' x '.join(f'{v:.1f}' for v in item['dimensiones_mm'])
        footer.SetInput(f'{dimensions} mm\nComponente auxiliar; escala de vista independiente')
        footer.SetPosition(15, 18)
        footer.GetTextProperty().SetFontSize(13)
        footer.GetTextProperty().SetColor(0.08, 0.12, 0.18)
        renderer.AddActor2D(footer)
        renderers.append(renderer)
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.ReadFrontBufferOff()
    capture.Update()
    writer = vtk.vtkPNGWriter()
    destination = ROOT / 'procesados/componentes_recibidos.png'
    writer.SetFileName(str(destination))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()
    print(destination)


if __name__ == '__main__':
    main()
