// Ejecuta el .ino real con entradas/salidas simuladas. No accede al hardware.
#include "../../firmware/ensayo_hover_1600/ensayo_hover_1600.ino"
#include <cassert>
#include <cstdio>
void stopped() {
 assert(mode==OFF);
 for(int p:appliedUs)assert(p==1000);
 for(auto p:pwmComparator)assert(p->value==1000);
}
void level() {
 calibrated=true; calibrating=false; imuOK=pwmOK=true;
 roll=pitch=yaw=0; accelNorm=1;
 memset(rates,0,sizeof(rates));
 for(int i=0;i<3;++i)for(int j=0;j<3;++j)correction[i][j]=(i==j);
}
void runStart() {
 level(); command("hb"); command("armar"); assert(mode==ARMED);
 command("arrancar"); assert(mode==RAMP);
 lastSample=micros(); // cada escenario empieza con reloj de muestreo sincronizado
}
void tick(unsigned ms) {
 for(unsigned i=0;i<ms;++i){fakeUs+=1000;loop();}
}
int main() {
 setup(); stopped(); assert(!calibrated);
 command("arrancar"); stopped();
 level(); command("armar"); stopped(); // sin heartbeat
 command("hb"); command("arrancar"); stopped(); // sin armar
 runStart();
 command("apagar"); stopped();
 command("arrancar"); stopped(); // nunca reinicia automaticamente
 runStart();
 Serial.inject("!"); serialInput(); stopped(); assert(cutReason==1); // sin Enter
 Serial.inject("\n"); serialInput();
 runStart();
 // Mantener heartbeat durante arranque y comprobar cada PWM real ordenado.
 for(int i=0;i<1500;++i){if(i%50==0)command("hb");tick(1);for(int p:appliedUs)assert(p>=1000&&p<=1600);}
 assert(mode==CONTROL);
 command("rampa1400"); assert(targetUs==1400);
 command("rampa"); assert(targetUs==1600);
 for(int i=0;i<3000;++i){if(i%50==0)command("hb");tick(1);}
 assert(collectiveUs>1300 && collectiveUs<1350);
 command("mantener"); assert(targetUs==lroundf(collectiveUs));
 command("hover"); assert(marked);
 // No HB durante >=500 ms: apagado aun si el controlador sigue corriendo.
 tick(400); stopped(); assert(cutReason==2);
 command("hb"); command("arrancar"); stopped();
 runStart();
 roll=14.9f*rad; checkAttitudeCutoff(); assert(driving());
 roll=15.0f*rad; checkAttitudeCutoff(); stopped(); assert(cutReason&4);
 runStart(); roll=pitch=11*rad; checkAttitudeCutoff(); stopped();
 // El pico real no dispara ACCEL, y 15 grados siguen cortando en esa misma muestra.
 runStart(); accelNorm=1.6442f; fakeUs+=6000; checkAttitudeCutoff(); assert(driving());
 roll=15*rad; checkAttitudeCutoff(); stopped(); assert(cutReason&4);
 runStart(); accelNorm=1.90f; checkAttitudeCutoff(); stopped(); assert(cutReason==32);
 assert(accelGuard.lastTrip==ensayo::AccelTrip::HARD_HIGH);
 runStart(); accelNorm=.20f; checkAttitudeCutoff(); stopped(); assert(cutReason==32);
 runStart(); accelNorm=1.8f;
 for(int i=0;i<25 && driving();++i){fakeUs+=5000;checkAttitudeCutoff();}
 stopped(); assert(cutReason==32 && accelGuard.lastTrip==ensayo::AccelTrip::SUSTAINED);
 runStart(); accelNorm=1.8f;
 for(int i=0;i<6;++i){fakeUs+=5000;checkAttitudeCutoff();}
 assert(driving() && accelGuard.outside && accelGuard.outsideUs<40000);
 roll=15*rad; checkAttitudeCutoff(); stopped(); assert(cutReason&4);
 runStart(); accelNorm=1.8f;
 for(int i=0;i<6;++i){fakeUs+=5000;checkAttitudeCutoff();}
 Serial.inject("!");serialInput();stopped();assert(cutReason==1);
 Serial.inject("\n");serialInput();
 runStart(); command("cualquier_cosa"); stopped(); assert(cutReason==512);
 runStart(); runMs=millis()-90000; loop(); stopped(); assert(cutReason==256);
 runStart(); calibrated=false; imuFault("fail",5000,16000,true); stopped();
 runStart(); fakeUs+=10000; hbMs=millis(); lastSample=micros()-31000; loop(); stopped(); assert(cutReason==64);
 // Al techo conservar correccion, sin exceder ningun canal.
 level(); mode=CONTROL; collectiveUs=1600;
 mix(.01f,0,0);
 assert(scaleRP>.999f);
 int mn=1600,mx=0;for(int p:commandUs){assert(p>=1180&&p<=1600);mn=min(mn,p);mx=max(mx,p);}
 assert(mx==1600&&mn<1600);
 command("parar"); stopped();
 puts("firmware real: apagado, doble arranque, !, timeout, tilt independiente, ACCEL rev2, rampa y limites OK");
}
