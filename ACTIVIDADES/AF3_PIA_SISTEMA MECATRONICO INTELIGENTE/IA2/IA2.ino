//#INTELIGENCIA ARTIFICIAL 013 N4 - N6
//#AF3 PRODUCTO INTEGRADOR DE APRENDIZAJE- BANDA CLASIFICADORA
//#NOMBRES                    MATRICULA         CARRERA
//#ISRAEL ALEXANDER PÉREZ REMIGIO  2007050       IMC
//#CARMEN BELÉN CONTRERAS VÁZQUEZ 2173848        IMC
//#FÁTIMA QUETZALI RODRÍGUEZ REYNA 1958057       IMC

// === Pines ultrasónico + motor ===
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const int MOTOR_PIN = 11;   

// === Pines TCS3200 ===
const int TCS_S0  = 5;
const int TCS_S1  = 6;
const int TCS_S2  = 7;
const int TCS_S3  = 8;
const int TCS_OUT = 9;      

// === Servo rampa ===
const int SERVO_PIN = 10;
Servo ramp;
// Ángulos 
const int ANGLE_NEUTRAL = 90;   // Centro
const int ANGLE_FORWARD = 60;   // ROJO → “adelante”
const int ANGLE_BACK    = 120;  // VERDE → “atrás”
const unsigned long SERVO_HOLD_MS = 1500; // tiempo desviando antes de volver al centro

// === Parámetros ultrasónico/motor ===
const float STOP_DIST_CM   = 5.0;   // pausar si < 12 cm
const float CLEAR_DIST_CM  = 16.0;   // rearmar si > 16 cm
const int   MOTOR_SPEED    = 200;    // 0..255
const unsigned long PAUSE_MS      = 3000; // tiempo de pausa fija
const unsigned long CLEAR_HOLD_MS = 400;  // despeje mínimo para rearmar

// Ajustes de medición del ultrasónico
const int   N_SAMPLES_DIST = 3;          
const unsigned long US_TIMEOUT = 15000UL; 

// === Config TCS ===
const bool TCS_S0_STATE = HIGH;  // S0=HIGH, S1=LOW → 20% escala
const bool TCS_S1_STATE = LOW;
const int  TCS_SAMPLES_PER_CH = 8;        
const unsigned long TCS_PULSE_TIMEOUT_US = 30000UL;

// === IA (ROJO vs VERDE, entrenada en sklearn) ===
// z = W0*r + W1*g + BIAS; z>0 → VERDE, z<=0 → ROJO
float W0   = -14.2284f;
float W1   =  9.0652f;
float BIAS =  2.5978f;

// --- Pines y parámetros del sensor IR + LEDs ---
const int IR_PIN = A0;          // salida analógica del TCRT5000
const int IR_THRESHOLD = 500;   // UMBRAL: ajusta luego viendo analogRead()

const int LED_VERDE_PIN = 12;
const int LED_ROJO_PIN  = 13;

// Si pasan 7 s sin ver ninguna caja → LED rojo avisa
const unsigned long NO_DETECT_TIMEOUT_MS = 10000;

// Variables de estado para el IR y los LEDs
bool prevIrDetected = false;          // para detectar flanco de subida

// Parpadeo verde (2 destellos)
bool greenBlinkActive = false;
bool greenLedState    = false;
int  greenBlinkStep   = 0;
unsigned long nextGreenBlinkTime = 0;
const unsigned long GREEN_BLINK_PERIOD_MS = 200;  // destellos rápidos

// Parpadeo rojo (continuo mientras no hay cajas)
bool redLedState      = false;
unsigned long nextRedBlinkTime   = 0;
const unsigned long RED_BLINK_PERIOD_MS   = 300;

// Última vez que se detectó una caja en la rampa
unsigned long lastBoxTime = 0;

// --- Máquina de estados (banda + color) ---
enum State { RUNNING, PAUSED, SORTING_WAIT_CLEAR };
State state = RUNNING;

unsigned long t_pause_end   = 0;
unsigned long t_clear_start = 0;
unsigned long t_servo_reset = 0;  // cuándo regresar el servo a neutro

// Acumuladores de color durante la PAUSA
float accR=0, accG=0, accB=0; 
int   accN=0;

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Ultrasónico / motor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // TCS3200
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, TCS_S0_STATE);
  digitalWrite(TCS_S1, TCS_S1_STATE);

  // Servo
  ramp.attach(SERVO_PIN);
  ramp.write(ANGLE_NEUTRAL);

  // IR + LEDs
  pinMode(IR_PIN, INPUT);
  pinMode(LED_VERDE_PIN, OUTPUT);
  pinMode(LED_ROJO_PIN, OUTPUT);
  digitalWrite(LED_VERDE_PIN, LOW);
  digitalWrite(LED_ROJO_PIN, LOW);

  // banda y temporizador de "sin cajas"
  setMotor(true);
  lastBoxTime = millis();   // empieza a contar desde que se enciende

  Serial.println(F("Sistema listo (HC-SR04 + Motor + TCS + IA + Servo + IR + LEDs)"));
}

void loop() {
  float dist = distanciaPromediada();  // ultrasónico

  // --- Máquina de estados principal (banda + color) ---
  switch (state) {
    case RUNNING: {
      if (dist > 0 && dist < STOP_DIST_CM) {
        setMotor(false);
        t_pause_end = millis() + PAUSE_MS;

        accR = accG = accB = 0;
        accN = 0;

        state = PAUSED;
        Serial.print(F("PAUSE a ")); Serial.print(dist); Serial.println(F(" cm"));
      }
    } break;

    case PAUSED: {
      // Mientras dura la pausa, acumulamos R,G,B del TCS
      if (millis() < t_pause_end) {
        float R,G,B; 
        tcsReadRGB(R,G,B);
        if (R>0 || G>0 || B>0) {
          accR += R; accG += G; accB += B; accN++;
        }
      } else {
        // Promedio y clasificación
        float Rm = (accN>0)? accR/accN : 0;
        float Gm = (accN>0)? accG/accN : 0;
        float Bm = (accN>0)? accB/accN : 0;
        int cls = classifyRG(Rm, Gm, Bm); // 0=ROJO, 1=VERDE

        // Log al Serial
        Serial.print(F("[TCS] Rm=")); Serial.print(Rm,1);
        Serial.print(F(" Gm=")); Serial.print(Gm,1);
        Serial.print(F(" Bm=")); Serial.print(Bm,1);
        Serial.print(F(" -> CLASE = "));
        Serial.println(cls==0 ? F("ROJO") : F("VERDE"));

        // Mueve el servo según clase
        if (cls == 0) {
          ramp.write(ANGLE_FORWARD); // ROJO
          Serial.println(F("Servo → ADELANTE (ROJO)"));
        } else {
          ramp.write(ANGLE_BACK);    // VERDE
          Serial.println(F("Servo → ATRÁS (VERDE)"));
        }
        t_servo_reset = millis() + SERVO_HOLD_MS;

        // Reanuda la banda y pasa a esperar despeje
        setMotor(true);
        t_clear_start = 0;
        state = SORTING_WAIT_CLEAR;
        Serial.println(F("RUN → desviando y esperando despeje"));
      }
    } break;

    case SORTING_WAIT_CLEAR: {
      // Regreso automático del servo a neutro después de SERVO_HOLD_MS
      if (millis() >= t_servo_reset && ramp.read() != ANGLE_NEUTRAL) {
        ramp.write(ANGLE_NEUTRAL);
        Serial.println(F("Servo → NEUTRO"));
      }

      // Rearme de detección: espera a que se despeje la zona
      if (dist > 0 && dist > CLEAR_DIST_CM) {
        if (t_clear_start == 0) t_clear_start = millis();
        if (millis() - t_clear_start >= CLEAR_HOLD_MS) {
          state = RUNNING;
          Serial.println(F("Rearmado (zona despejada)"));
        }
      } else {
        t_clear_start = 0;
      }
    } break;
  }

  // --- Lógica del sensor IR + LEDs ---
  handleIRandLeds();

  delay(20); // pequeño respiro
}

// ========== IR + LEDs ==========

bool irDetectaCaja() {
  int valor = analogRead(IR_PIN);
  // NOTA: si ves que cuando hay caja el valor BAJA en vez de subir,
  // cambia '>' por '<' y ajusta IR_THRESHOLD.
  return (valor > IR_THRESHOLD);  // true = se detecta caja
}

void handleIRandLeds() {
  unsigned long now = millis();
  bool irDetected = irDetectaCaja();

  // --- 1) Flanco de subida: cuando aparece una caja ---
  if (irDetected && !prevIrDetected) {
    // Actualizamos "última vez que vi caja"
    lastBoxTime = now;

    // Apagar cualquier parpadeo rojo
    redLedState = false;
    digitalWrite(LED_ROJO_PIN, LOW);

    // Iniciar parpadeo verde de 2 destellos
    greenBlinkActive     = true;
    greenLedState        = false;
    greenBlinkStep       = 0;
    nextGreenBlinkTime   = now;

    Serial.println(F("[IR] Caja detectada en la rampa → LED VERDE 2 destellos"));
  }
  prevIrDetected = irDetected;

  // --- 2) Gestionar parpadeo VERDE (2 destellos y se apaga) ---
  if (greenBlinkActive && now >= nextGreenBlinkTime) {
    greenLedState = !greenLedState;
    digitalWrite(LED_VERDE_PIN, greenLedState);

    greenBlinkStep++;
    nextGreenBlinkTime = now + GREEN_BLINK_PERIOD_MS;

    // 4 cambios: ON,OFF,ON,OFF → 2 destellos
    if (greenBlinkStep >= 4) {
      greenBlinkActive = false;
      greenLedState    = false;
      digitalWrite(LED_VERDE_PIN, LOW);
    }
  }

  // --- 3) Gestionar parpadeo ROJO por falta de cajas ---
  // Si NO estamos en parpadeo verde y ya pasaron 7 s sin cajas:
  if (!greenBlinkActive && (now - lastBoxTime >= NO_DETECT_TIMEOUT_MS)) {
    // LED rojo parpadea continuamente
    if (now >= nextRedBlinkTime) {
      redLedState = !redLedState;
      digitalWrite(LED_ROJO_PIN, redLedState);
      nextRedBlinkTime = now + RED_BLINK_PERIOD_MS;
    }
  } else {
    // Si aún no pasan 7 s o se detectan cajas, el rojo permanece apagado
    redLedState = false;
    digitalWrite(LED_ROJO_PIN, LOW);
  }
}

// ========== Helpers ==========

void setMotor(bool on) {
  analogWrite(MOTOR_PIN, on ? MOTOR_SPEED : 0);
}

// --- Distancia con promedio rápido ---
float distanciaPromediada() {
  float acc = 0; 
  int ok = 0;
  for (int i = 0; i < N_SAMPLES_DIST; i++) {
    float d = medirDistanciaCM();
    if (d > 0) { 
      acc += d; 
      ok++; 
    }
    delay(5);
  }
  return (ok > 0) ? acc / ok : -1.0;
}

float medirDistanciaCM() {
  digitalWrite(TRIG_PIN, LOW);  
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return -1.0;

  float dist = (dur * 0.0343f) * 0.5f;
  if (dist < 1.5f || dist > 300.0f) return -1.0;
  return dist;
}

// --- TCS3200: selección de filtro ---
inline void tcsRed()   { digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, LOW);  }
inline void tcsBlue()  { digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, HIGH); }
inline void tcsGreen() { digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH); }

float tcsReadChannelHz() {
  unsigned long sumPeriod = 0; 
  int ok = 0;
  for (int i=0; i<TCS_SAMPLES_PER_CH; i++){
    unsigned long th = pulseIn(TCS_OUT, HIGH, TCS_PULSE_TIMEOUT_US);
    unsigned long tl = pulseIn(TCS_OUT, LOW,  TCS_PULSE_TIMEOUT_US);
    if (th>0 && tl>0){ 
      sumPeriod += (th+tl); 
      ok++; 
    }
  }
  if (ok==0) return 0.0f;
  float period_us = (float)sumPeriod / (float)ok;
  if (period_us <= 0) return 0.0f;
  return 1000000.0f / period_us; // Hz aprox.: más luz → más Hz
}

void tcsReadRGB(float &R, float &G, float &B){
  tcsRed();   delay(10); R = tcsReadChannelHz();
  tcsGreen(); delay(10); G = tcsReadChannelHz();
  tcsBlue();  delay(10); B = tcsReadChannelHz();
}

// --- IA lineal con r,g (0=ROJO, 1=VERDE) ---
int classifyRG(float R, float G, float B){
  float S = R + G + B;
  if (S < 1e-6f) return 0; // baja luz → asumimos ROJO por seguridad
  float r = R / S;
  float g = G / S;
  float z = W0*r + W1*g + BIAS;
  return (z > 0.0f) ? 1 : 0; // 1=VERDE, 0=ROJO
}
