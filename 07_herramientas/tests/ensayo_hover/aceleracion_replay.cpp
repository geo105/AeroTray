#include "../../firmware/ensayo_hover_1600/PoliticaEnsayo.h"
#include <iostream>
#include <cstdint>

int main() {
  ensayo::AccelGuard guard;
  uint32_t us;
  float g;
  unsigned count=0,trips=0,firstTrip=0;
  while(std::cin>>us>>g) {
    if(!count)guard.reset(g,us);
    const auto trip=guard.update(g,us,true);
    if(trip!=ensayo::AccelTrip::NONE) { if(!trips)firstTrip=us; ++trips; }
    ++count;
  }
  std::cout<<count<<" "<<trips<<" "<<firstTrip<<" "<<guard.filtered<<"\n";
  return count ? 0 : 1;
}
