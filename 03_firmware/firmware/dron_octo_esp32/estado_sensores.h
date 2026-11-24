#pragma once
#include <math.h>
#include <stdint.h>

namespace octo {
struct ReferenciaAltura {
  bool tieneBase = false, tieneMedida = false;
  float baseVerticalMm = 0, zCm = 0;
  uint32_t ultimaMs = 0;

  void invalidar() { tieneBase = tieneMedida = false; zCm = 0; }
  bool actualizar(float mm, float cosVertical, uint32_t ahora) {
    if (!isfinite(mm) || !isfinite(cosVertical) || mm <= 0 || mm > 1200 || cosVertical < 0.7f || cosVertical > 1.001f) return false;
    float vertical = mm*cosVertical;
    if (!tieneBase) { baseVerticalMm = vertical; tieneBase = true; }
    zCm = (vertical-baseVerticalMm)*0.1f;
    tieneMedida = true;
    ultimaMs = ahora;
    return true;
  }
  bool cero(float mm, float cosVertical, uint32_t ahora) {
    invalidar();
    return actualizar(mm, cosVertical, ahora);
  }
  bool valida(uint32_t ahora, uint32_t maxEdad = 200) const {
    return tieneBase && tieneMedida && uint32_t(ahora-ultimaMs) <= maxEdad;
  }
};

inline bool imuVigente(bool ultimaLecturaOk, uint32_t ahora, uint32_t ultimaMs) {
  return ultimaLecturaOk && uint32_t(ahora-ultimaMs) <= 20;
}
}  // namespace octo
