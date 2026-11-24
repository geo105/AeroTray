"""Inventariar las descargas y convertir STEP a mallas métricas sin tocar originales."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "procesados"


def inspect_step(source, mesh_path):
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.gp import gp_Pnt, gp_Trsf
    import numpy as np
    import trimesh

    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("OpenCascade no pudo leer el STEP")
    # El factor expresa la unidad de trabajo en milímetros (1 = mm).
    reader.SetSystemLengthUnit(1.0)
    transferred = reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise ValueError("STEP sin geometría transferida")
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    bounds = list(box.CornerMin().Coord()) + list(box.CornerMax().Coord())
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    count = 0
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        count += 1
        explorer.Next()
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0, 0, 0), 0.001)
    metric = BRepBuilderAPI_Transform(shape, scale, True).Shape()
    BRepMesh_IncrementalMesh(metric, 0.00025, False, 0.35, True)
    face_count = 0
    missing_faces = 0
    face_explorer = TopExp_Explorer(metric, TopAbs_FACE)
    while face_explorer.More():
        face_count += 1
        triangulation = BRep_Tool.Triangulation_s(TopoDS.Face(face_explorer.Current()), TopLoc_Location())
        if triangulation is None or triangulation.NbTriangles() == 0:
            missing_faces += 1
        face_explorer.Next()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(metric, str(mesh_path)):
        raise ValueError("No se pudo escribir la malla STL")
    mesh = trimesh.load_mesh(mesh_path, process=True)
    assert len(mesh.faces) > 0 and np.isfinite(mesh.vertices).all()
    geometric_span = np.array(bounds[3:]) - np.array(bounds[:3])
    assert np.allclose(mesh.extents, geometric_span / 1000, atol=0.001)
    return {
        "estado": "STEP_leido_y_malla_generada",
        "raices_transferidas": transferred,
        "solidos": count,
        "limites_mm": bounds,
        "dimensiones_mm": geometric_span.tolist(),
        "volumen_geometrico_mm3": props.Mass(),
        "malla_metros": str(mesh_path),
        "triangulos": len(mesh.faces),
        "caras_BRep": face_count,
        "caras_sin_triangulacion": missing_faces,
        "malla_cerrada": bool(mesh.is_watertight),
        "uso_malla": "visualizacion; no aprobada como colision fisica",
        "advertencia": "No asigna masa, materiales, articulaciones ni posición en el robot. STL no conserva el árbol del ensamblaje."
    }


def main():
    OUT.mkdir(exist_ok=True)
    inputs = []
    for folder in (ROOT / "download", ROOT / "download (1)", ROOT / "recibidos"):
        if folder.exists():
            inputs.extend(sorted(p for p in folder.rglob("*") if p.is_file()))
    results = []
    for source in inputs:
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        record = {"archivo": str(source), "bytes": source.stat().st_size,
                  "sha256": digest, "formato": source.suffix.lower()}
        if source.suffix.lower() in (".step", ".stp"):
            stem = source.stem.replace(" ", "_")
            try:
                record.update(inspect_step(source, OUT / f"{stem}_{digest[:8]}_m.stl"))
            except Exception as exc:
                record.update(estado="error", error=str(exc))
        elif source.suffix.lower() == ".f3d":
            record["estado"] = "respaldo_nativo_Fusion_no_editado"
            record["zip_valido"] = zipfile.is_zipfile(source)
        else:
            record["estado"] = "original_conservado"
        results.append(record)
    archives = []
    for source in (Path('/home/george/Descargas/download.zip'), Path('/home/george/Descargas/download (1).zip')):
        if source.exists() and zipfile.is_zipfile(source):
            with zipfile.ZipFile(source) as archive:
                archives.append({"archivo": str(source), "miembros": archive.namelist()})
    expected = ["nuevo 8 camara", "x8_UAV_R350_imprimible_bandeja_v5", "ball_and_plate_camara"]
    received_stems = [p.stem.lower() for p in inputs if p.suffix.lower() in ('.step', '.stp')]
    missing = [name for name in expected if not any(stem.startswith(name.lower()) for stem in received_stems)]
    result = {
        "fecha": "2026-10-30", "archivos": results, "archivos_zip": archives,
        "ensamblajes_necesarios_no_recibidos": missing,
        "nota": "Las mallas de componentes auxiliares no sustituyen los tres ensamblajes. No se modificó el CAD nativo ni el modelo de simulación del dron."
    }
    (OUT / "inventario_descargas.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    for record in results:
        if record["formato"] in (".step", ".stp"):
            print(json.dumps({"archivo":Path(record['archivo']).name, "estado":record['estado'],
                              "dimensiones_mm":record.get('dimensiones_mm'), "error":record.get('error')}, ensure_ascii=False))
    print(f"Archivos recibidos: {len(results)}. Informe: {OUT / 'inventario_descargas.json'}")


if __name__ == "__main__":
    main()
