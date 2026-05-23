#include <PS4Controller.h>
#include <Wire.h>
#include <VL53L0X.h>

/*
========================================================
                MINI SUMO ESP32  v4.6
      DRV8833 + PS4 + 3x VL53L0X + TCA9548A

  NUEVO v4.6 (control fútbol separado por sticks):
  - LStickY  →  adelante / atrás (drive)
  - RStickX  →  giro en su propio eje (tank turn)
  - Prioridad: si LStickY está fuera de zona muerta,
    RStickX se IGNORA (no se suman).
  - D-Pad sigue funcionando igual (overrides directos).

  - KICK 100 ms a 255 al arrancar desde parado
  - RAMPA lineal de 300 ms hasta el PWM destino
  - NO BLOQUEANTE (millis())
  
  - 4 canales ledc fijos desde setup()
  - Sentido de giro corregido
  - Freno activo: ambos canales a 255
========================================================
*/

/*================= DRV8833 PINES =================*/
#define AIN1 27
#define AIN2 26
#define BIN1 12
#define BIN2 14
#define STBY 25

/*================= CANALES ledc (fijos para siempre) =================*/
#define CH_AIN1 0
#define CH_AIN2 1
#define CH_BIN1 2
#define CH_BIN2 3

const int freq       = 5000;
const int resolution = 8;  // 0-255

/*================= ARRANQUE SUAVE: KICK + RAMPA =================*/
#define KICK_MS        100
#define RAMPA_MS       300
#define PWM_RAMPA_INI   90

int8_t        estadoIzq    = 0;
int8_t        estadoDer    = 0;
uint8_t       faseIzq      = 0;
uint8_t       faseDer      = 0;
unsigned long arranqueIzq  = 0;
unsigned long arranqueDer  = 0;
int           pwmDestinoIzq = 0;
int           pwmDestinoDer = 0;
int8_t        dirIzq        = 0;
int8_t        dirDer        = 0;

/*================= TCA9548A =================*/
#define TCA_ADDR 0x70
#define CH_DER   0
#define CH_CEN   1
#define CH_IZQ   2

/*================= VL53L0X =================*/
VL53L0X tof_izq, tof_cen, tof_der;

#define UMBRAL         200
#define TIMEOUT_SENSOR 8000

/*================= MODOS =================*/
#define MODO_TELE 1
#define MODO_SUMO 2

int           modo_actual = MODO_TELE;
bool          sumo_activo = false;
unsigned long sumo_start  = 0;

/*================= AUTO-INICIO =================*/
#define TIEMPO_ESPERA_CONTROL 3000

bool          control_conectado = false;
bool          auto_sumo_lanzado = false;
unsigned long boot_time         = 0;

/*================= VELOCIDADES =================*/
int speed_rc    = 255;
int minspeed_rc = 40;

/*================= ZONAS MUERTAS DE STICKS =================*/
#define DEADZONE_LY  15   // zona muerta stick izquierdo (drive)
#define DEADZONE_RX  15   // zona muerta stick derecho   (giro)

/*================= BOTONES =================*/
bool options_prev = false;
bool cruz_prev    = false;
bool circulo_prev = false;

/*========================================================
                    TCA9548A
========================================================*/
void tcaSelect(uint8_t canal) {
  if (canal > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << canal);
  Wire.endTransmission();
  delay(5);
}

/*========================================================
                ESCANEO DE CANALES
========================================================*/
void verificarCanales() {
  Serial.println("\n=== VERIFICANDO VL53L0X ===");
  for (int i = 0; i < 3; i++) {
    tcaSelect(i);
    Serial.print("Canal "); Serial.print(i);
    Wire.beginTransmission(0x29);
    Serial.println(Wire.endTransmission() == 0 ? " -> OK" : " -> NO DETECTADO");
    delay(100);
  }
  Serial.println();
}

/*========================================================
                  INICIAR SENSORES
========================================================*/
void iniciarSensores() {
  tcaSelect(CH_DER); delay(100);
  if (tof_der.init()) { tof_der.setTimeout(500); tof_der.startContinuous(); Serial.println("VL53 DER OK"); }
  else Serial.println("VL53 DER FAIL");
  delay(100);

  tcaSelect(CH_CEN); delay(100);
  if (tof_cen.init()) { tof_cen.setTimeout(500); tof_cen.startContinuous(); Serial.println("VL53 CEN OK"); }
  else Serial.println("VL53 CEN FAIL");
  delay(100);

  tcaSelect(CH_IZQ); delay(100);
  if (tof_izq.init()) { tof_izq.setTimeout(500); tof_izq.startContinuous(); Serial.println("VL53 IZQ OK"); }
  else Serial.println("VL53 IZQ FAIL");
  delay(100);
}

/*========================================================
          LEER SENSOR CON FILTRO DE TIMEOUT
========================================================*/
uint16_t leerSensor(VL53L0X &sensor) {
  uint16_t d = sensor.readRangeContinuousMillimeters();
  return (sensor.timeoutOccurred() || d >= TIMEOUT_SENSOR) ? TIMEOUT_SENSOR : d;
}

/*========================================================
                LEER SENSORES SERIAL
========================================================*/
void leerSensoresSerial() {
  tcaSelect(CH_IZQ); uint16_t d_izq = leerSensor(tof_izq);
  tcaSelect(CH_CEN); uint16_t d_cen = leerSensor(tof_cen);
  tcaSelect(CH_DER); uint16_t d_der = leerSensor(tof_der);

  Serial.print("IZQ: "); Serial.print(d_izq == TIMEOUT_SENSOR ? "---" : String(d_izq));
  Serial.print(" | CEN: "); Serial.print(d_cen == TIMEOUT_SENSOR ? "---" : String(d_cen));
  Serial.print(" | DER: "); Serial.println(d_der == TIMEOUT_SENSOR ? "---" : String(d_der));
}

/*========================================================
    ESCRITURA DIRECTA DE CANALES ledc
========================================================*/
inline void rawIzqAdelante(int pwm) { ledcWrite(CH_AIN1, pwm); ledcWrite(CH_AIN2, 0);   }
inline void rawIzqAtras(int pwm)    { ledcWrite(CH_AIN1, 0);   ledcWrite(CH_AIN2, pwm); }
inline void rawIzqFreno()           { ledcWrite(CH_AIN1, 255); ledcWrite(CH_AIN2, 255); }
inline void rawIzqLibre()           { ledcWrite(CH_AIN1, 0);   ledcWrite(CH_AIN2, 0);   }

inline void rawDerAdelante(int pwm) { ledcWrite(CH_BIN1, pwm); ledcWrite(CH_BIN2, 0);   }
inline void rawDerAtras(int pwm)    { ledcWrite(CH_BIN1, 0);   ledcWrite(CH_BIN2, pwm); }
inline void rawDerFreno()           { ledcWrite(CH_BIN1, 255); ledcWrite(CH_BIN2, 255); }
inline void rawDerLibre()           { ledcWrite(CH_BIN1, 0);   ledcWrite(CH_BIN2, 0);   }

/*========================================================
    PROCESAR ARRANQUE (KICK + RAMPA) — NO BLOQUEANTE
========================================================*/
inline void aplicarIzq(int pwm) {
  if      (dirIzq ==  1) rawIzqAdelante(pwm);
  else if (dirIzq == -1) rawIzqAtras(pwm);
}
inline void aplicarDer(int pwm) {
  if      (dirDer ==  1) rawDerAdelante(pwm);
  else if (dirDer == -1) rawDerAtras(pwm);
}

void procesarArranque() {
  unsigned long now = millis();

  if (faseIzq == 1) {
    if (now - arranqueIzq >= KICK_MS) {
      faseIzq = 2;
      aplicarIzq(PWM_RAMPA_INI);
    }
  }
  else if (faseIzq == 2) {
    unsigned long t = now - arranqueIzq - KICK_MS;
    if (t >= RAMPA_MS) {
      faseIzq = 3;
      aplicarIzq(pwmDestinoIzq);
    } else {
      int pwm = PWM_RAMPA_INI +
                (int)((long)(pwmDestinoIzq - PWM_RAMPA_INI) * (long)t / RAMPA_MS);
      if (pwm < 0)   pwm = 0;
      if (pwm > 255) pwm = 255;
      aplicarIzq(pwm);
    }
  }

  if (faseDer == 1) {
    if (now - arranqueDer >= KICK_MS) {
      faseDer = 2;
      aplicarDer(PWM_RAMPA_INI);
    }
  }
  else if (faseDer == 2) {
    unsigned long t = now - arranqueDer - KICK_MS;
    if (t >= RAMPA_MS) {
      faseDer = 3;
      aplicarDer(pwmDestinoDer);
    } else {
      int pwm = PWM_RAMPA_INI +
                (int)((long)(pwmDestinoDer - PWM_RAMPA_INI) * (long)t / RAMPA_MS);
      if (pwm < 0)   pwm = 0;
      if (pwm > 255) pwm = 255;
      aplicarDer(pwm);
    }
  }
}

/*========================================================
    FUNCIONES DE MOTOR DE ALTO NIVEL
========================================================*/
void motorIzqAdelante(int pwm) {
  pwm = constrain(pwm, 0, 255);
  pwmDestinoIzq = pwm;
  dirIzq        = 1;

  if (estadoIzq == 0) {
    rawIzqAdelante(255);
    arranqueIzq = millis();
    faseIzq     = 1;
  } else if (estadoIzq == -1) {
    rawIzqAdelante(255);
    arranqueIzq = millis();
    faseIzq     = 1;
  } else if (faseIzq == 3) {
    rawIzqAdelante(pwm);
  }
  estadoIzq = 1;
}

void motorIzqAtras(int pwm) {
  pwm = constrain(pwm, 0, 255);
  pwmDestinoIzq = pwm;
  dirIzq        = -1;

  if (estadoIzq == 0) {
    rawIzqAtras(255);
    arranqueIzq = millis();
    faseIzq     = 1;
  } else if (estadoIzq == 1) {
    rawIzqAtras(255);
    arranqueIzq = millis();
    faseIzq     = 1;
  } else if (faseIzq == 3) {
    rawIzqAtras(pwm);
  }
  estadoIzq = -1;
}

void motorIzqFreno() {
  faseIzq    = 0;
  dirIzq     = 0;
  estadoIzq  = 0;
  rawIzqFreno();
}

void motorDerAdelante(int pwm) {
  pwm = constrain(pwm, 0, 255);
  pwmDestinoDer = pwm;
  dirDer        = 1;

  if (estadoDer == 0) {
    rawDerAdelante(255);
    arranqueDer = millis();
    faseDer     = 1;
  } else if (estadoDer == -1) {
    rawDerAdelante(255);
    arranqueDer = millis();
    faseDer     = 1;
  } else if (faseDer == 3) {
    rawDerAdelante(pwm);
  }
  estadoDer = 1;
}

void motorDerAtras(int pwm) {
  pwm = constrain(pwm, 0, 255);
  pwmDestinoDer = pwm;
  dirDer        = -1;

  if (estadoDer == 0) {
    rawDerAtras(255);
    arranqueDer = millis();
    faseDer     = 1;
  } else if (estadoDer == 1) {
    rawDerAtras(255);
    arranqueDer = millis();
    faseDer     = 1;
  } else if (faseDer == 3) {
    rawDerAtras(pwm);
  }
  estadoDer = -1;
}

void motorDerFreno() {
  faseDer    = 0;
  dirDer     = 0;
  estadoDer  = 0;
  rawDerFreno();
}

void forward(int pwm)  { motorIzqAdelante(pwm); motorDerAdelante(pwm); }
void backward(int pwm) { motorIzqAtras(pwm);    motorDerAtras(pwm);    }
void giroD(int pwm)    { motorIzqAdelante(pwm); motorDerAtras(pwm);    }
void giroI(int pwm)    { motorIzqAtras(pwm);    motorDerAdelante(pwm); }

void mstop() {
  motorIzqFreno();
  motorDerFreno();
}

void setMotorIzq(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0)  motorIzqAdelante(pwm);
  else if (pwm < 0)  motorIzqAtras(-pwm);
  else               motorIzqFreno();
}

void setMotorDer(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm > 0)  motorDerAdelante(pwm);
  else if (pwm < 0)  motorDerAtras(-pwm);
  else               motorDerFreno();
}

/*========================================================
                    MODO SUMO
========================================================*/
void modoSumo() {
  if (!sumo_activo) return;

  if (millis() - sumo_start < 5000) { mstop(); return; }

  tcaSelect(CH_IZQ); uint16_t d_izq = leerSensor(tof_izq);
  tcaSelect(CH_CEN); uint16_t d_cen = leerSensor(tof_cen);
  tcaSelect(CH_DER); uint16_t d_der = leerSensor(tof_der);

  bool ei = (d_izq < UMBRAL);
  bool ec = (d_cen < UMBRAL);
  bool ed = (d_der < UMBRAL);

  if      (ec)       { forward(255); Serial.println("ATAQUE CENTRO");      }
  else if (ei && ed) { forward(255); Serial.println("ATAQUE AMBOS");       }
  else if (ei)       { giroI(220);   Serial.println("ATAQUE IZQUIERDA");   }
  else if (ed)       { giroD(220);   Serial.println("ATAQUE DERECHA");     }
  else               { giroD(120);   Serial.println("BUSCANDO");           }
}

/*========================================================
                        SETUP
========================================================*/
void setup() {
  Serial.begin(115200);

  ledcSetup(CH_AIN1, freq, resolution); ledcAttachPin(AIN1, CH_AIN1);
  ledcSetup(CH_AIN2, freq, resolution); ledcAttachPin(AIN2, CH_AIN2);
  ledcSetup(CH_BIN1, freq, resolution); ledcAttachPin(BIN1, CH_BIN1);
  ledcSetup(CH_BIN2, freq, resolution); ledcAttachPin(BIN2, CH_BIN2);

  rawIzqLibre();
  rawDerLibre();

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  Wire.begin(21, 22);
  Wire.setClock(50000);

  Serial.println("=== MINI SUMO v4.6 ===");
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("TCA9548A OK");
  } else {
    Serial.println("TCA9548A NO DETECTADO");
    while (1);
  }

  verificarCanales();
  iniciarSensores();
  mstop();

  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin("c0:cd:d6:8f:22:ec");

  Serial.println("Esperando control PS4...");
  Serial.println("Sin control en 3s -> AUTO MODO SUMO");

  boot_time = millis();
}

/*========================================================
                        LOOP
========================================================*/
void loop() {

  procesarArranque();
  leerSensoresSerial();

  /*============= AUTO INICIO SIN CONTROL =============*/
  if (!control_conectado && !auto_sumo_lanzado) {
    if (millis() - boot_time >= TIEMPO_ESPERA_CONTROL) {
      auto_sumo_lanzado = true;
      modo_actual       = MODO_SUMO;
      sumo_activo       = true;
      sumo_start        = millis();
      Serial.println("AUTO MODO SUMO");
      Serial.println("ESPERA 5 SEGUNDOS...");
    }
  }

  /*============= CON CONTROL =============*/
  if (control_conectado) {
    bool options_now = PS4.Options();
    if (options_now && !options_prev) {
      sumo_activo = false; mstop();
      if (modo_actual == MODO_TELE) {
        modo_actual = MODO_SUMO;
        PS4.setLed(255, 0, 0);
        Serial.println("MODO SUMO");
      } else {
        modo_actual = MODO_TELE;
        PS4.setLed(0, 0, 255);
        Serial.println("MODO FUTBOL");
      }
      PS4.sendToController();
    }
    options_prev = options_now;

    bool cruz_now = PS4.Cross();
    if (cruz_now && !cruz_prev && modo_actual == MODO_SUMO) {
      sumo_activo = true; sumo_start = millis();
      Serial.println("ESPERA 5 SEGUNDOS...");
    }
    cruz_prev = cruz_now;

    bool circulo_now = PS4.Circle();
    if (circulo_now && !circulo_prev && modo_actual == MODO_SUMO) {
      sumo_activo = false; mstop();
      Serial.println("SUMO DETENIDO");
    }
    circulo_prev = circulo_now;
  }

  /*============= MODO FUTBOL =============*/
  //
  //  Control separado por sticks:
  //    - LStickY  → drive (adelante / atrás)
  //    - RStickX  → tank turn (giro en su propio eje)
  //    - Prioridad: LStickY manda. Si está fuera de la zona muerta,
  //      el RStickX se ignora completamente.
  //    - D-Pad: overrides directos (igual que antes).
  //
  if (modo_actual == MODO_TELE && control_conectado) {

    // --- D-Pad: máxima prioridad, override directo ---
    if      (PS4.Up())    forward(255);
    else if (PS4.Down())  backward(255);
    else if (PS4.Right()) giroD(255);
    else if (PS4.Left())  giroI(255);
    else {
      int ly = PS4.LStickY();    // -128..127, positivo = arriba
      int rx = PS4.RStickX();    // -128..127, positivo = derecha

      bool ly_activo = (ly >  DEADZONE_LY) || (ly < -DEADZONE_LY);
      bool rx_activo = (rx >  DEADZONE_RX) || (rx < -DEADZONE_RX);

      if (ly_activo) {
        // --- DRIVE: stick izquierdo manda, derecho se ignora ---
        int vel = 0;
        if (ly >  DEADZONE_LY) vel = map(ly,  DEADZONE_LY,  127,  minspeed_rc,  speed_rc);
        else                   vel = map(ly, -DEADZONE_LY, -128, -minspeed_rc, -speed_rc);

        setMotorIzq(vel);
        setMotorDer(vel);
      }
      else if (rx_activo) {
        // --- TANK TURN: solo derecho, gira en su propio eje ---
        int giro = 0;
        if (rx >  DEADZONE_RX) giro = map(rx,  DEADZONE_RX,  127,  minspeed_rc,  speed_rc);
        else                   giro = map(rx, -DEADZONE_RX, -128, -minspeed_rc, -speed_rc);

        // giro > 0  → derecha: izq adelante, der atrás
        // giro < 0  → izquierda: izq atrás, der adelante
        setMotorIzq( giro);
        setMotorDer(-giro);
      }
      else {
        // --- ambos sticks en reposo → frenar ---
        mstop();
      }
    }
  }

  /*============= MODO SUMO =============*/
  else if (modo_actual == MODO_SUMO) {
    modoSumo();
  }

  delay(20);
}

/*========================================================
                    PS4 CONNECT / DISCONNECT
========================================================*/
void onConnect() {
  control_conectado = true;
  if (auto_sumo_lanzado) {
    auto_sumo_lanzado = false; sumo_activo = false;
    modo_actual = MODO_TELE; mstop();
    Serial.println("Control conectado -> MODO FUTBOL");
  }
  modo_actual = MODO_TELE;
  PS4.setLed(0, 0, 255);
  PS4.sendToController();
  Serial.println("PS4 CONECTADO");
}

void onDisConnect() {
  control_conectado = false;
  sumo_activo       = false;
  mstop();
  Serial.println("PS4 DESCONECTADO");
}
