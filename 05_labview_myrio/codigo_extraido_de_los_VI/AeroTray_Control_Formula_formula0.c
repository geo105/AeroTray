/* Text of a LabVIEW Formula Node, extracted from the VI for reading without LabVIEW. */
/* AEROTRAY - CONTROL DE MOVIMIENTO */

/* Normalizar y limitar duty */
float64 u;

u = duty_pct / 100.0;

if (u < 0.0) {
    u = 0.0;
}

if (u > 1.0) {
    u = 1.0;
}

/* STOP por defecto */
pwm_izq = 0.0;
pwm_der = 0.0;
dir_izq = 0;
dir_der = 0;

/* Permitir movimiento solo sin emergencia */
if ((stop_emergencia == 0) && (finalizar == 0)) {

    /* 0 - STOP */

    /* 1 - ADELANTE */
    if (comando == 1) {
        pwm_izq = u;
        pwm_der = u;
        dir_izq = 1;
        dir_der = 1;
    }

    /* 2 - ATRAS */
    else if (comando == 2) {
        pwm_izq = u;
        pwm_der = u;
        dir_izq = 0;
        dir_der = 0;
    }

    /* 3 - GIRO IZQUIERDA */
    else if (comando == 3) {
        pwm_izq = u;
        pwm_der = u;
        dir_izq = 0;
        dir_der = 1;
    }

    /* 4 - GIRO DERECHA */
    else if (comando == 4) {
        pwm_izq = u;
        pwm_der = u;
        dir_izq = 1;
        dir_der = 0;
    }
}