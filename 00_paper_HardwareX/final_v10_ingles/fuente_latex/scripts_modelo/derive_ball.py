"""Symbolic verification of the ball-on-a-moving-plate equations (AeroTray, Sec. 3).
Newton-Euler + rolling constraint, everything in plate-frame components."""
import sympy as sp
t=sp.symbols('t')
m,rb,eps,g=sp.symbols('m r_b epsilon g',positive=True)
I=sp.Rational(2,5)*m*rb**2
x=sp.Function('x')(t); y=sp.Function('y')(t)
Ox,Oy,Oz=[sp.Function(n)(t) for n in ('Omega_x','Omega_y','Omega_z')]   # plate ang. vel. (P comps)
aOx,aOy,aOz=[sp.Function(n)(t) for n in ('a_Ox','a_Oy','a_Oz')]          # gimbal-centre accel (P comps)
gx,gy,gz=sp.symbols('g_x g_y g_z')                                           # gravity in P comps
wz=sp.Function('w_z')(t)                                                     # relative spin about plate normal
Om=sp.Matrix([Ox,Oy,Oz]); rho=sp.Matrix([x,y,eps]); rhod=sp.Matrix([x.diff(t),y.diff(t),0]); rhodd=rhod.diff(t)
a=sp.Matrix([aOx,aOy,aOz])+Om.diff(t).cross(rho)+Om.cross(Om.cross(rho))+2*Om.cross(rhod)+rhodd
w=sp.Matrix([-y.diff(t)/rb, x.diff(t)/rb, wz])        # rolling: rho_dot = r_b (w x e_z)
wb_dot=Om.diff(t)+w.diff(t)+Om.cross(w)               # absolute ang. accel of ball, P comps
Fx,Fy,N=sp.symbols('F_x F_y N')
F=sp.Matrix([Fx,Fy,N])
eqT=m*a-(m*sp.Matrix([gx,gy,gz])+F)
eqR=I*wb_dot-sp.Matrix([0,0,-rb]).cross(F)
sol=sp.solve([eqT[0],eqT[1],eqR[0],eqR[1]],[Fx,Fy,x.diff(t,2),y.diff(t,2)],dict=True)[0]
xdd=sp.simplify(sp.expand(sol[x.diff(t,2)])); ydd=sp.simplify(sp.expand(sol[y.diff(t,2)]))
print("xdd =",sp.collect(sp.expand(xdd*sp.Rational(7,5)),[gx,aOx]))
print("ydd =",sp.collect(sp.expand(ydd*sp.Rational(7,5)),[gy,aOy]))
# Claimed closed form (paper Eq.):
kap=eps+sp.Rational(2,5)*rb
Od=Om.diff(t)
claim_x=sp.Rational(5,7)*(gx-aOx-kap*Od[1]+Od[2]*y+(Oy**2+Oz**2)*x-Ox*Oy*y-Ox*Oz*eps
          +sp.Rational(12,5)*Oz*y.diff(t)+sp.Rational(2,5)*rb*Ox*wz)
res_x=sp.simplify(sp.expand(xdd-claim_x))
print("residual x (should be 0):",res_x)
claim_y=sp.Rational(5,7)*(gy-aOy+kap*Od[0]-Od[2]*x+(Ox**2+Oz**2)*y-Ox*Oy*x-Oy*Oz*eps
          -sp.Rational(12,5)*Oz*x.diff(t)+sp.Rational(2,5)*rb*Oy*wz)
res_y=sp.simplify(sp.expand(ydd-claim_y))
print("residual y (should be 0):",res_y)
assert res_x == 0 and res_y == 0, f"Non-zero residual: {res_x}, {res_y}"
# spin equation
spin=sp.solve(eqR[2],wz.diff(t))[0]; print("w_z dot =",sp.simplify(spin))
