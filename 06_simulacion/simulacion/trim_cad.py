"""Reparto estático de empuje ideal; no es un controlador de vuelo."""
import numpy as np


def trim(model):
    # El gear aplica fuerza local Z y par local Z en cada site.
    matrix=[]
    for actuator in range(model.nu):
        site=model.actuator_trnid[actuator,0]
        pos=model.site_pos[site]
        matrix.append([1,pos[1],-pos[0],model.actuator_gear[actuator,5]])
    allocation=np.array(matrix).T
    target=np.array([model.body_mass.sum()*9.80665,0,0,0])
    forces=np.linalg.lstsq(allocation,target,rcond=None)[0]
    assert np.all(forces>=model.actuator_ctrlrange[:,0])
    assert np.all(forces<=model.actuator_ctrlrange[:,1])
    assert np.allclose(allocation@forces,target,atol=1e-8)
    return forces
