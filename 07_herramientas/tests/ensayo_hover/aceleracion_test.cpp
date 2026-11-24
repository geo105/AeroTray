#include "../../firmware/ensayo_hover_1600/PoliticaEnsayo.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>

using namespace ensayo;
int main() {
  AccelGuard g;
  g.reset(1.055f,0);
  assert(g.update(1.6442f,6000,true)==AccelTrip::NONE);
  assert(g.update(1.2328f,12000,true)==AccelTrip::NONE);
  assert(g.filtered<1.55f);
  // Oscilacion grande alrededor de 1 g: no equivale a una aceleracion sostenida.
  for(uint32_t t=17000;t<2000000;t+=5000)
    assert(g.update((t/5000)%2 ? .72f : 1.54f,t,true)==AccelTrip::NONE);
  // Los umbrales duros funcionan aun con dt cero / filtro en reposo.
  g.reset(1,0); assert(g.update(1.9f,0,true)==AccelTrip::HARD_HIGH);
  assert(g.update(1,5000,true)==AccelTrip::HARD_HIGH); // enclavado hasta reset
  g.reset(1,0); assert(g.update(.2f,0,true)==AccelTrip::HARD_LOW);
  g.reset(1,0); assert(g.update(std::numeric_limits<float>::quiet_NaN(),5000,true)==AccelTrip::INVALID);
  g.reset(1,0); assert(g.update(INFINITY,5000,true)==AccelTrip::INVALID);
  for(float value : {1.8f,1.56f,.4f,.54f}) {
    g.reset(1,0);
    uint32_t trip=0;
    for(uint32_t t=5000;t<=250000;t+=5000) {
      if(g.update(value,t,true)!=AccelTrip::NONE){trip=t;break;}
    }
    assert(trip>0 && trip<=250000);
    assert(g.lastTrip==AccelTrip::SUSTAINED && g.outsideUs>=40000);
    if(value==1.8f || value==.4f)assert(trip<=100000);
    printf("sostenida %.2f g: corte en %u us\n",value,trip);
  }
  g.reset(1,0);
  for(uint32_t t=5000;t<=30000;t+=5000)g.update(1.8f,t,true);
  assert(g.outside && g.outsideUs<40000);
  for(uint32_t t=35000;t<=80000;t+=5000)assert(g.update(1,t,true)==AccelTrip::NONE);
  assert(!g.outside && g.outsideUs==0);
  // Observar sin armar nunca dispara; el rearme borra el estado anterior.
  g.reset(1,0); assert(g.update(2,5000,false)==AccelTrip::NONE);
  g.reset(1,10000); assert(g.lastTrip==AccelTrip::NONE && g.outsideUs==0);
  // Rollover de micros durante una excursion sostenida.
  constexpr uint32_t start=0xffffff00u;
  g.reset(1,start);
  uint32_t trip=0;
  for(uint32_t dt=5000;dt<=100000;dt+=5000)
    if(g.update(1.8f,start+dt,true)!=AccelTrip::NONE){trip=dt;break;}
  assert(trip>0 && trip<=100000);
  puts("ACCEL rev2: picos, limites duros, persistencia, recuperacion, enclavamiento, invalidos y rollover OK");
}
