"""Octocopter on a spherical joint that is carried by an accelerating carriage.
Newton-Euler about the moving pivot A: J w' + w x J w = sum M_A - m r_c x R^T a_A  (body components)."""
import sympy as sp
ph,th,ps=sp.symbols('phi theta psi')
m,g,hc,dx,dy=sp.symbols('m g h_c delta_x delta_y')
ax,ay,az=sp.symbols('a_x a_y a_z')       # pivot acceleration, world comps
def Rx(a): return sp.Matrix([[1,0,0],[0,sp.cos(a),-sp.sin(a)],[0,sp.sin(a),sp.cos(a)]])
def Ry(a): return sp.Matrix([[sp.cos(a),0,sp.sin(a)],[0,1,0],[-sp.sin(a),0,sp.cos(a)]])
def Rz(a): return sp.Matrix([[sp.cos(a),-sp.sin(a),0],[sp.sin(a),sp.cos(a),0],[0,0,1]])
R=Rz(ps)*Ry(th)*Rx(ph)
rc=sp.Matrix([dx,dy,hc])
gv=sp.Matrix([0,0,-g]); aA=sp.Matrix([ax,ay,az])
tau=m*rc.cross(R.T*(gv-aA))
lin={}
for k,name in enumerate(['roll','pitch','yaw']):
    e=tau[k].subs(ps,0)
    # first-order Taylor in (phi,theta,ax,ay,az,dx,dy) about zero
    vars_=[ph,th,ax,ay,az,dx,dy]
    e0=e.subs({v:0 for v in vars_})
    l=e0+sum(sp.diff(e,v).subs({vv:0 for vv in vars_})*v for v in vars_)
    print(name,': ',sp.simplify(l))
# rotor moments: r_i x F_i e_z, r_i=(x_i,y_i,h_r)
xi,yi,hr,Fi=sp.symbols('x_i y_i h_r F_i')
print('rotor moment:',sp.Matrix([xi,yi,hr]).cross(sp.Matrix([0,0,Fi])).T)
# Euler-rate kinematics used in firmware (ZYX): check  [phi',theta',psi'] = W^-1 [p,q,r]
p,q,r=sp.symbols('p q r')
# body rates from Euler rates: omega_body = R^T * d/dt... build via standard formula
phd,thd,psd=sp.symbols('phidot thetadot psidot')
wb=Rx(ph).T*Ry(th).T*sp.Matrix([0,0,psd])+Rx(ph).T*sp.Matrix([0,thd,0])+sp.Matrix([phd,0,0])
sol=sp.solve(list(wb-sp.Matrix([p,q,r])),[phd,thd,psd],dict=True)[0]
for k in (phd,thd,psd): print(k,'=',sp.simplify(sol[k]))
