import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
import numpy as np, pandas as pd, glob, os, time
R=_os.path.join(ROOT,"08_datos_y_registros","copia_raspberry_pi_20261120","pi","registros")
V=_os.path.join(ROOT,"08_datos_y_registros","registros_vuelo_dron")
def reg(name):
    d=pd.read_csv(os.path.join(R,name),on_bad_lines='skip')
    for c in d.columns: d[c]=pd.to_numeric(d[c],errors='coerce')
    d=d.dropna(subset=['tiempo_unix'])
    return d
