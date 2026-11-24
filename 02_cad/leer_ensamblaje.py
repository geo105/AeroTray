"""Leer un STEP con árbol XCAF y posiciones, conservando los archivos originales."""
import argparse
import json
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_Tool
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.IFSelect import IFSelect_RetDone


def label_id(label):
    value = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, value)
    return value.ToCString()


def label_name(label):
    value = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), value):
        return value.Get().ToExtString()
    return label_id(label)


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    if box.IsVoid():
        return None
    return list(box.CornerMin().Coord()) + list(box.CornerMax().Coord())


def read_document(path):
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError('STEP no legible: ' + str(path))
    reader.ChangeReader().SetSystemLengthUnit(1.0)
    doc = TDocStd_Document(TCollection_ExtendedString('CadXCAF'))
    if not reader.Transfer(doc):
        raise RuntimeError('No se pudo transferir el STEP al documento XCAF')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    return doc, tool, roots


def walk_document(tool, roots):
    rows = []
    shapes = {}

    def walk(label, parent_world, parent_path, depth):
        if depth > 60:
            raise RuntimeError('Árbol de ensamblaje demasiado profundo')
        definition = TDF_Label()
        is_reference = tool.GetReferredShape_s(label, definition)
        if not is_reference:
            definition = label
        shape = tool.GetShape_s(definition)
        if shape.IsNull():
            return
        own_location = shape.Location()
        world = parent_world * (tool.GetLocation_s(label) if is_reference else TopLoc_Location()) * own_location
        name = label_name(label)
        definition_name = label_name(definition)
        path = parent_path + [name]
        assembly = tool.IsAssembly_s(definition)
        matrix = world.Transformation()
        row = {
            'id': label_id(label), 'definicion_id': label_id(definition),
            'nombre': name, 'definicion': definition_name, 'ruta': path,
            'nivel': depth, 'ensamblaje': assembly,
            'matriz_mundo_mm': [[matrix.Value(i,j) for j in range(1,5)] for i in range(1,4)]
        }
        if assembly:
            children = TDF_LabelSequence()
            tool.GetComponents_s(definition, children, False)
            row['hijos'] = children.Length()
            rows.append(row)
            for i in range(1, children.Length()+1):
                walk(children.Value(i), world, path, depth+1)
        else:
            local_shape = shape.Located(TopLoc_Location())
            placed = local_shape.Moved(world)
            row['limites_mundo_mm'] = bounds(placed)
            rows.append(row)
            shapes[len(rows)-1] = placed

    root_bounds = []
    for i in range(1, roots.Length()+1):
        root = roots.Value(i)
        root_bounds.append(bounds(tool.GetShape_s(root)))
        walk(root, TopLoc_Location(), [], 0)
    return rows, shapes, root_bounds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archivo', type=Path)
    parser.add_argument('--salida', required=True, type=Path)
    parser.add_argument('--cache-brep', type=Path)
    args = parser.parse_args()
    print('Leyendo ' + args.archivo.name, flush=True)
    doc, tool, roots = read_document(args.archivo)
    print('STEP transferido. Recorriendo componentes...', flush=True)
    rows, shapes, root_bounds = walk_document(tool, roots)
    result = {'origen':str(args.archivo.resolve()), 'unidad':'mm', 'limites_raices_mm':root_bounds,
              'nodos':len(rows), 'hojas':len(shapes), 'componentes':rows}
    args.salida.parent.mkdir(exist_ok=True, parents=True)
    args.salida.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if args.cache_brep:
        from OCP.BRepTools import BRepTools
        args.cache_brep.mkdir(exist_ok=True, parents=True)
        for index, shape in shapes.items():
            if not BRepTools.Write_s(shape, str(args.cache_brep / f'{index:04d}.brep')):
                raise RuntimeError('No se pudo guardar BREP ' + str(index))
        print('Cache BREP lista: ' + str(args.cache_brep), flush=True)
    print(json.dumps({'informe':str(args.salida),'nodos':len(rows),'hojas':len(shapes),'limites':root_bounds},ensure_ascii=False),flush=True)
    for row in rows:
        if row['nivel'] <= 1:
            print('  '*row['nivel'] + row['nombre'] + ' | ' + row['definicion'], flush=True)


if __name__ == '__main__':
    main()
