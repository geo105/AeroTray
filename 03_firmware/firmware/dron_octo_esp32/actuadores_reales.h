#pragma once
#include <math.h>
// Completar solo con empuje medido por motor y mapeo verificado fisicamente.
// Los 6 N de la simulacion NO se convierten en una recta de PWM inventada.
namespace hardware_real {
constexpr bool VERIFICADO=false;
struct Punto {float pwmUs, fuerzaN;};
constexpr unsigned MAX_PUNTOS=16;
static const unsigned cantidad[8]={};
static const Punto tabla[8][MAX_PUNTOS]={};
// Indice fisico (0..7) correspondiente a cada actuador del modelo CAD.
static const unsigned fisicoDesdeSim[8]={0,1,2,3,4,5,6,7}; // PROVISIONAL, bloqueado.
inline bool convertir(const float fuerzasSim[8],float pwmFisico[8]){
  for(int i=0;i<8;i++)pwmFisico[i]=1000;
  if(!VERIFICADO)return false;
  bool usados[8]={};
  for(int sim=0;sim<8;sim++){
    unsigned m=fisicoDesdeSim[sim],n=m<8?cantidad[m]:0;
    if(m>=8 || usados[m] || n<2 || n>MAX_PUNTOS || !isfinite(fuerzasSim[sim]))return false;
    usados[m]=true;
    for(unsigned j=0;j<n;j++){
      if(!isfinite(tabla[m][j].pwmUs)||!isfinite(tabla[m][j].fuerzaN)||tabla[m][j].pwmUs<1000||tabla[m][j].pwmUs>1950||tabla[m][j].fuerzaN<0)return false;
      if(j && (tabla[m][j].pwmUs<=tabla[m][j-1].pwmUs || tabla[m][j].fuerzaN<=tabla[m][j-1].fuerzaN))return false;
    }
    float f=fuerzasSim[sim];
    if(f<tabla[m][0].fuerzaN || f>tabla[m][n-1].fuerzaN)return false; // Sin extrapolar.
    for(unsigned j=1;j<n;j++)if(f<=tabla[m][j].fuerzaN){
      float t=(f-tabla[m][j-1].fuerzaN)/(tabla[m][j].fuerzaN-tabla[m][j-1].fuerzaN);
      pwmFisico[m]=tabla[m][j-1].pwmUs+t*(tabla[m][j].pwmUs-tabla[m][j-1].pwmUs);break;
    }
  }
  return true;
}
}
