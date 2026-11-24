#include <cmath>
#include <algorithm>
template<class T> T constrain(T x,T a,T b){return std::max(a,std::min(x,b));}
float kp=1.875f;
float ki=0.5f;
float kd=1.5625f;
float filtro_v=0.75f;
float vel_integral=25.0f;
float autoridad_i=0.9f;
float ab_alfa=0.0f;
float ab_beta=0.0f;
float friccion_us=0.0f, limite_us=220.0f;
bool reinicio_integral=false;
float ZONA_MUERTA_PX=1.5f, VELOCIDAD_QUIETA=8.f;
struct EjePID {
  float integral = 0, velocidad = 0, pos_anterior = 0, error_anterior = 0, pos_est = 0;
  bool inicializado = false;

  void reiniciar() { integral = 0; velocidad = 0; error_anterior = 0; inicializado = false; }

  // devuelve la correccion en us de servo. Con usar_vel_ext la velocidad viene de fuera (la Pi)
  float paso(float pos, float consigna, float dt, bool usar_vel_ext = false, float vel_ext = 0) {
    if (!inicializado) { pos_anterior = pos; pos_est = pos; velocidad = 0; inicializado = true; }
    if (usar_vel_ext) {
      velocidad = vel_ext;
    } else if (ab_alfa > 0) {
      // filtro alfa-beta: predice con la velocidad y corrige con la medida; da posicion y
      // velocidad suaves con mucho menos retraso que un filtro paso bajo
      float pred = pos_est + velocidad * dt;
      float r = pos - pred;
      pos_est = pred + ab_alfa * r;
      velocidad += ab_beta / dt * r;
      pos = pos_est;
    } else {
      // velocidad con filtro paso bajo: el centroide tiene ruido de ~1 pixel
      float v = (pos - pos_anterior) / dt;
      velocidad = filtro_v * velocidad + (1 - filtro_v) * v;
    }
    pos_anterior = pos;
    float error = consigna - pos;
    // opcional: al cruzar la consigna se borra la integral para que no la pase de largo
    if (reinicio_integral && error * error_anterior < 0) integral = 0;
    error_anterior = error;
    // integral condicional: solo acumula con la bola lenta. Una base inclinada deja la bola apartada
    // y casi quieta (ahi hay que acumular); el vaiven normal la mueve rapido (ahi acumular la pasa de largo)
    if (vel_integral <= 0 || fabsf(velocidad) < vel_integral) integral += error * dt;
    // anti-windup: el termino integral puede aportar hasta esta fraccion del recorrido
    // (con la base inclinada es lo que mantiene la placa compensada)
    float max_i = ki > 0 ? autoridad_i * limite_us / ki : 0;
    integral = constrain(integral, -max_i, max_i);
    float u = kp * error + ki * integral - kd * velocidad;
    // compensacion de rozamiento: la bola quieta fuera del centro no se despega con correcciones
    // pequenas; se suma un empujon fijo hacia la consigna mientras este quieta y lejos del centro
    if (friccion_us > 0 && fabsf(error) > ZONA_MUERTA_PX && fabsf(velocidad) < VELOCIDAD_QUIETA)
      u += error > 0 ? friccion_us : -friccion_us;
    return u;
  }
};
float holgura_rampa_us=30.0f;
static float compensarHolgura(float d, float hueco_pos, float hueco_neg) {
  if (d > 0) return d + hueco_pos * fminf(1.0f, d / holgura_rampa_us);
  if (d < 0) return d - hueco_neg * fminf(1.0f, -d / holgura_rampa_us);
  return 0;
}

EjePID e;
extern "C" void reset(){e.reiniciar();}
extern "C" float step(float p,float dt,float v,int ext){return e.paso(p,0,dt,ext,v);}
extern "C" int pulse(float u,float gp,float gn){return (int)compensarHolgura(constrain(u,-limite_us,limite_us),gp,gn);}
