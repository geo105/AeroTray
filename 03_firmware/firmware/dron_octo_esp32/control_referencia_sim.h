#pragma once
#include <math.h>
#include "parametros_sim.h"

// Puerto de ControlReferencia en simulacion/control_simulacion.py.
// Unidades SI: m, m/s, vector de rotacion [rad], rad/s, N, Nm.
// NO convierte N en PWM; el mapa y la curva del hardware siguen pendientes.
namespace simref {
inline float clip(float x,float a,float b){return x<a?a:(x>b?b:x);}
struct Estado {float z,v,rot[3],omega[3],coseno;};
struct ControlReferencia {
  float integralZ=0, integralT[3]={};
  float solicitado[4]={},alcanzado[4]={};
  bool saturado=false;
  int iteraciones=0;
  void reiniciar(){integralZ=0;for(float &x:integralT)x=0;}
  bool paso(const Estado& s,float dt,float referenciaM,float fuerzas[8]){
    bool valida=isfinite(s.z)&&isfinite(s.v)&&isfinite(s.coseno)&&isfinite(dt)&&dt>0&&dt<=0.05f&&isfinite(referenciaM);
    for(int k=0;k<3;k++)valida=valida&&isfinite(s.rot[k])&&isfinite(s.omega[k]);
    if(!valida){reiniciar();for(int i=0;i<8;i++)fuerzas[i]=0;return false;}
    float c=fmaxf(.5f,s.coseno),error=referenciaM-s.z;
    float vertical=MASA*G+KP_Z*error-KD_Z*s.v+integralZ;
    solicitado[0]=vertical/c;
    for(int k=0;k<3;k++)solicitado[k+1]=-KP[k]*s.rot[k]-KD[k]*s.omega[k]+integralT[k];
    asignar(solicitado,fuerzas);
    integralZ=clip(integralZ+dt*(KI_Z*error+5*(alcanzado[0]*c-vertical)),-12,12);
    const float limites[3]={1,1,.08f};
    for(int k=0;k<3;k++)integralT[k]=clip(integralT[k]+dt*(-KI[k]*s.rot[k]+5*(alcanzado[k+1]-solicitado[k+1])),-limites[k],limites[k]);
    return true;
  }
  void asignar(const float pedido[4],float f[8]){
    saturado=false;iteraciones=0;
    for(int i=0;i<8;i++){
      f[i]=0;for(int k=0;k<4;k++)f[i]+=INV[i][k]*pedido[k];
      if(f[i]<0||f[i]>MAX_N)saturado=true;
      f[i]=clip(f[i],0,MAX_N);
    }
    if(saturado){
      // Misma funcion objetivo LS acotada que SciPy/BVLS en simulacion.
      // Descenso coordenado determinista, sin asignacion dinamica ni SciPy.
      const float w[4]={1,6/.35f,6/.35f,.2f/.015f};
      float residual[4];
      for(int k=0;k<4;k++){float v=-pedido[k];for(int j=0;j<8;j++)v+=A[k][j]*f[j];residual[k]=w[k]*v;}
      for(int it=0;it<800;it++){
        float mayor=0;
        for(int i=0;i<8;i++){
          float grad=0,diagonal=0;
          for(int k=0;k<4;k++){float a=w[k]*A[k][i];grad+=a*residual[k];diagonal+=a*a;}
          float nuevo=clip(f[i]-grad/diagonal,0,MAX_N),delta=nuevo-f[i];f[i]=nuevo;
          mayor=fmaxf(mayor,fabsf(delta));
          for(int k=0;k<4;k++)residual[k]+=w[k]*A[k][i]*delta;
        }
        iteraciones=it+1;if(mayor<1e-6f)break;
      }
    }
    for(int k=0;k<4;k++){alcanzado[k]=0;for(int i=0;i<8;i++)alcanzado[k]+=A[k][i]*f[i];}
  }
};
}
