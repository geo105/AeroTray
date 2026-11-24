/* Text of a LabVIEW Formula Node, extracted from the VI for reading without LabVIEW. */
/* =========================================
   DSM - ROBOT DIFERENCIAL myRIO

   CONTROL MANUAL
   CONTROL PI POR RUEDA
   ODOMETRIA X, Y, THETA
   CONTROL DE DISTANCIA
   NAVEGACION AUTONOMA XY

   MOVIMIENTO_ID:
   0 STOP
   1 ADELANTE
   2 ATRAS
   3 GIRO_IZQ
   4 GIRO_DER
   5 IR_A_XY
   ========================================= */


/* ===== VARIABLES ===== */

float dt_s;
float dsIzq, dsDer, dsCentro;
float dTheta, thetaMed;

float ref;
float refIzq, refDer;
float sentidoIzq, sentidoDer;

float errorIzq, errorDer;
float velSentidoIzq, velSentidoDer;

float intPruebaIzq, intPruebaDer;
float uPruebaIzq, uPruebaDer;
float uIzq, uDer;

float dutyManual;

float errorX, errorY;
float anguloObjetivo, diferenciaAngular;

float vMaxXY;
float vComando, wComando;
float vRuedaIzq, vRuedaDer;

float toleranciaXY;
float umbralGiro;
float limiteAngular;
float limiteRueda;


/* =========================================
   1. TIEMPO
   ========================================= */

dt_s = 0.0;

if (Tick_ant_ms > 0.0 &&
    Tick_actual_ms > Tick_ant_ms) {

    dt_s = (Tick_actual_ms - Tick_ant_ms)
           / 1000.0;
}


/* =========================================
   2. VELOCIDADES MEDIDAS
   ========================================= */

Vel_Izq = 0.0;
Vel_Der = 0.0;

if (dt_s > 0.0) {

    Vel_Izq = (N_Izq - N_Izq_Ant)
              * 0.0002151 / dt_s;

    Vel_Der = (N_Der - N_Der_Ant)
              * (-0.0002151) / dt_s;
}


/* =========================================
   3. ODOMETRIA
   ========================================= */

X_Nuevo = X_Ant;
Y_Nuevo = Y_Ant;
Theta_Nuevo = Theta_Ant;
S_Nueva = S_Ant;

if (dt_s > 0.0) {

    dsIzq = (N_Izq - N_Izq_Ant)
             * 0.0002151;

    dsDer = (N_Der - N_Der_Ant)
             * (-0.0002151);

    dsCentro = (dsIzq + dsDer) / 2.0;

    dTheta = (dsDer - dsIzq) / 0.3794;

    thetaMed = Theta_Ant + dTheta / 2.0;

    X_Nuevo = X_Ant
            + dsCentro * cos(thetaMed);

    Y_Nuevo = Y_Ant
            + dsCentro * sin(thetaMed);

    Theta_Nuevo = Theta_Ant + dTheta;

    if (dsCentro >= 0.0) {
        S_Nueva = S_Ant + dsCentro;
    } else {
        S_Nueva = S_Ant - dsCentro;
    }
}


/* =========================================
   4. RESET DE ODOMETRIA
   ========================================= */

if (Reset_01 == 1) {

    X_Nuevo = 0.0;
    Y_Nuevo = 0.0;

    Theta_Nuevo = 0.0;
    S_Nueva = 0.0;
}


/* =========================================
   5. ERRORES DE TRAYECTORIA XY
   ========================================= */

errorX = X_Obj - X_Nuevo;
errorY = Y_Obj - Y_Nuevo;

Error_XY_m = sqrt(errorX * errorX
                + errorY * errorY);

Error_Angulo_rad = 0.0;

if (Error_XY_m > 0.000001) {

    anguloObjetivo = atan2(errorY, errorX);

    diferenciaAngular =
        anguloObjetivo - Theta_Nuevo;

    Error_Angulo_rad =
        atan2(sin(diferenciaAngular),
              cos(diferenciaAngular));
}


/* =========================================
   6. DISTANCIA Y MEMORIA DE LLEGADA
   ========================================= */

Error_Dist = Dist_Objetivo - S_Nueva;

Llegada_Nueva = Llegada_Ant;

toleranciaXY = 0.02;

/* RESET borra la memoria de llegada */

if (Reset_01 == 1) {

    Llegada_Nueva = 0.0;

} else {

    /* Llegada por distancia */

    if ((Movimiento_ID == 1 ||
         Movimiento_ID == 2) &&
        Dist_Objetivo > 0.0 &&
        S_Nueva >= Dist_Objetivo) {

        Llegada_Nueva = 1.0;
    }

    /* Llegada por posicion XY */

    if (Movimiento_ID == 5 &&
        Error_XY_m <= toleranciaXY) {

        Llegada_Nueva = 1.0;
    }
}

Llegada_01 = Llegada_Nueva;


/* =========================================
   7. REFERENCIA DE VELOCIDAD
   ========================================= */

ref = Vel_Ref;

if (ref < 0.0) {
    ref = -ref;
}

/* Referencias iniciales */

refIzq = ref;
refDer = ref;

sentidoIzq = 0.0;
sentidoDer = 0.0;


/* =========================================
   8. MOVIMIENTOS CONVENCIONALES
   ========================================= */

if (Movimiento_ID == 1) {

    sentidoIzq = 1.0;
    sentidoDer = 1.0;
}

if (Movimiento_ID == 2) {

    sentidoIzq = -1.0;
    sentidoDer = -1.0;
}

if (Movimiento_ID == 3) {

    sentidoIzq = -1.0;
    sentidoDer = 1.0;
}

if (Movimiento_ID == 4) {

    sentidoIzq = 1.0;
    sentidoDer = -1.0;
}


/* =========================================
   9. CONTROL DE TRAYECTORIA XY
   ========================================= */

/*
   ESTE MODO NECESITA MODO_PI = 1.

   VELOCIDAD MAXIMA XY:
   0.05 m/s

   VELOCIDAD ANGULAR MAXIMA:
   0.20 rad/s

   TOLERANCIA DE LLEGADA:
   0.02 m
*/

if (Movimiento_ID == 5 &&
    Modo_PI == 1 &&
    Llegada_Nueva == 0.0 &&
    ref > 0.0) {

    /* Limitar velocidad por seguridad */

    vMaxXY = ref;

    if (vMaxXY > 0.05) {
        vMaxXY = 0.05;
    }

    umbralGiro = 0.20;
    limiteAngular = 0.20;
    limiteRueda = 0.05;

    vComando = 0.0;
    wComando = 0.0;


    /* ===== ORIENTAR EL ROBOT ===== */

    if (Error_Angulo_rad > umbralGiro ||
        Error_Angulo_rad < -umbralGiro) {

        /* Primero girar sobre el eje */

        vComando = 0.0;

        wComando = 1.2 * Error_Angulo_rad;

    } else {

        /* ===== AVANZAR HACIA OBJETIVO ===== */

        vComando = vMaxXY;

        /* Reducir velocidad cerca del destino */

        if (Error_XY_m < 0.08) {

            vComando = 0.8 * Error_XY_m;

            if (vComando < 0.025 &&
                vMaxXY >= 0.025) {

                vComando = 0.025;
            }

            if (vComando > vMaxXY) {
                vComando = vMaxXY;
            }
        }

        /* Corregir orientacion en movimiento */

        wComando = 1.2 * Error_Angulo_rad;
    }


    /* Limitar velocidad angular */

    if (wComando > limiteAngular) {
        wComando = limiteAngular;
    }

    if (wComando < -limiteAngular) {
        wComando = -limiteAngular;
    }


    /* =====================================
       10. CINEMATICA DIFERENCIAL
       ===================================== */

    vRuedaIzq =
        vComando - 0.1897 * wComando;

    vRuedaDer =
        vComando + 0.1897 * wComando;


    /* Limitar referencia de rueda izquierda */

    if (vRuedaIzq > limiteRueda) {
        vRuedaIzq = limiteRueda;
    }

    if (vRuedaIzq < -limiteRueda) {
        vRuedaIzq = -limiteRueda;
    }


    /* Limitar referencia de rueda derecha */

    if (vRuedaDer > limiteRueda) {
        vRuedaDer = limiteRueda;
    }

    if (vRuedaDer < -limiteRueda) {
        vRuedaDer = -limiteRueda;
    }


    /* ===== SENTIDO RUEDA IZQUIERDA ===== */

    sentidoIzq = 0.0;
    refIzq = vRuedaIzq;

    if (refIzq > 0.0) {
        sentidoIzq = 1.0;
    }

    if (refIzq < 0.0) {
        sentidoIzq = -1.0;
        refIzq = -refIzq;
    }


    /* ===== SENTIDO RUEDA DERECHA ===== */

    sentidoDer = 0.0;
    refDer = vRuedaDer;

    if (refDer > 0.0) {
        sentidoDer = 1.0;
    }

    if (refDer < 0.0) {
        sentidoDer = -1.0;
        refDer = -refDer;
    }
}


/* =========================================
   11. INICIALIZAR SALIDAS
   ========================================= */

I_Izq_Nueva = 0.0;
I_Der_Nueva = 0.0;

PWM_PI_Izq_pct = 0.0;
PWM_PI_Der_pct = 0.0;

PWM_Final_Izq_pct = 0.0;
PWM_Final_Der_pct = 0.0;

Duty_Izq_01 = 0.0;
Duty_Der_01 = 0.0;

Dir_Izq_01 = 0.0;
Dir_Der_01 = 0.0;


/* =========================================
   12. HABILITACION DEL MOVIMIENTO
   ========================================= */

if (Stop_01 == 0 &&
    Reset_01 == 0 &&
    Llegada_Nueva == 0.0 &&
    (
      (
        Movimiento_ID >= 1 &&
        Movimiento_ID <= 4 &&
        (
          Movimiento_ID >= 3 ||
          Dist_Objetivo > 0.0
        )
      )
      ||
      (
        Movimiento_ID == 5 &&
        Modo_PI == 1 &&
        ref > 0.0
      )
    )) {


    /* =====================================
       13. CONTROL PI POR RUEDA
       ===================================== */

    if (Modo_PI == 1 &&
        dt_s > 0.0) {


        /* ===== PI IZQUIERDO ===== */

        if (sentidoIzq != 0.0 &&
            refIzq > 0.0) {

            velSentidoIzq =
                Vel_Izq * sentidoIzq;

            errorIzq =
                refIzq - velSentidoIzq;

            I_Izq_Nueva = I_Izq_Ant;

            if (Ki_Izq > 0.0) {

                intPruebaIzq =
                    I_Izq_Ant
                    + errorIzq * dt_s;

                uPruebaIzq =
                    Kp_Izq * errorIzq
                    + Ki_Izq * intPruebaIzq;

                if ((uPruebaIzq > 100.0 &&
                     errorIzq > 0.0) ||
                    (uPruebaIzq < 0.0 &&
                     errorIzq < 0.0)) {

                    I_Izq_Nueva = I_Izq_Ant;

                } else {

                    I_Izq_Nueva = intPruebaIzq;
                }
            }

            uIzq =
                Kp_Izq * errorIzq
                + Ki_Izq * I_Izq_Nueva;

            if (uIzq > 100.0) {
                uIzq = 100.0;
            }

            if (uIzq < 0.0) {
                uIzq = 0.0;
            }

            PWM_PI_Izq_pct =
                sentidoIzq * uIzq;
        }


        /* ===== PI DERECHO ===== */

        if (sentidoDer != 0.0 &&
            refDer > 0.0) {

            velSentidoDer =
                Vel_Der * sentidoDer;

            errorDer =
                refDer - velSentidoDer;

            I_Der_Nueva = I_Der_Ant;

            if (Ki_Der > 0.0) {

                intPruebaDer =
                    I_Der_Ant
                    + errorDer * dt_s;

                uPruebaDer =
                    Kp_Der * errorDer
                    + Ki_Der * intPruebaDer;

                if ((uPruebaDer > 100.0 &&
                     errorDer > 0.0) ||
                    (uPruebaDer < 0.0 &&
                     errorDer < 0.0)) {

                    I_Der_Nueva = I_Der_Ant;

                } else {

                    I_Der_Nueva = intPruebaDer;
                }
            }

            uDer =
                Kp_Der * errorDer
                + Ki_Der * I_Der_Nueva;

            if (uDer > 100.0) {
                uDer = 100.0;
            }

            if (uDer < 0.0) {
                uDer = 0.0;
            }

            PWM_PI_Der_pct =
                sentidoDer * uDer;
        }


        /* PWM final controlado por PI */

        PWM_Final_Izq_pct =
            PWM_PI_Izq_pct;

        PWM_Final_Der_pct =
            PWM_PI_Der_pct;
    }


    /* =====================================
       14. CONTROL MANUAL
       ===================================== */

    if (Modo_PI == 0) {

        dutyManual = Duty_Manual;

        if (dutyManual > 100.0) {
            dutyManual = 100.0;
        }

        if (dutyManual < 0.0) {
            dutyManual = 0.0;
        }

        PWM_Final_Izq_pct =
            sentidoIzq * dutyManual;

        PWM_Final_Der_pct =
            sentidoDer * dutyManual;

        /* Compensacion: solo ADELANTE manual, -5% rueda derecha */
        if (Movimiento_ID == 1) {
            PWM_Final_Der_pct = PWM_Final_Der_pct * 0.99;
        }
    }


    /* =====================================
       15. DIRECCIONES FISICAS CORREGIDAS
       ===================================== */

    Dir_Izq_01 = 0.0;

    if (PWM_Final_Izq_pct < 0.0) {
        Dir_Izq_01 = 1.0;
    }

    Dir_Der_01 = 0.0;

    if (PWM_Final_Der_pct > 0.0) {
        Dir_Der_01 = 1.0;
    }


    /* =====================================
       16. NORMALIZAR PWM A 0..1
       ===================================== */

    Duty_Izq_01 = PWM_Final_Izq_pct;
    Duty_Der_01 = PWM_Final_Der_pct;

    if (Duty_Izq_01 < 0.0) {
        Duty_Izq_01 = -Duty_Izq_01;
    }

    if (Duty_Der_01 < 0.0) {
        Duty_Der_01 = -Duty_Der_01;
    }

    Duty_Izq_01 =
        Duty_Izq_01 / 100.0;

    Duty_Der_01 =
        Duty_Der_01 / 100.0;
}


/* =========================================
   17. RESET DE CONTROL Y SALIDAS
   ========================================= */

if (Reset_01 == 1) {

    I_Izq_Nueva = 0.0;
    I_Der_Nueva = 0.0;

    PWM_PI_Izq_pct = 0.0;
    PWM_PI_Der_pct = 0.0;

    PWM_Final_Izq_pct = 0.0;
    PWM_Final_Der_pct = 0.0;

    Duty_Izq_01 = 0.0;
    Duty_Der_01 = 0.0;

    Dir_Izq_01 = 0.0;
    Dir_Der_01 = 0.0;
}