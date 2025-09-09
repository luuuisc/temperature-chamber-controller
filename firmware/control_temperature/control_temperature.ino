/*
  Control de Temperatura – LM35 + Arduino UNO (MOSFET low-side)
  - HEAT: foco 12 V (resistivo) en D3 (active-HIGH, PWM)
  - FAN : ventilador 12 V (inductivo, con diodo flyback) en D5 (active-HIGH, PWM)
  - Lectura LM35 con referencia interna (~1.1 V) y promediado
  - Control PI (solo calefacción) con banda muerta ±2 °C
  - Circulación mínima de aire a partir de sp-2 °C
  - UI Web por Web Serial (JSON periódico + comandos: S=xx.x, KPF=x, KPV=x, KIF=x)
*/

#include <Arduino.h>

// --------- Pines ----------
const int PIN_LM35   = A0;
const int PIN_HEATER = 3;   // PWM (Timer2)
const int PIN_FAN    = 5;   // PWM (Timer0)

// --------- Parámetros de control --------
float setpointC = 37.0;           // °C inicial
const float DEADBAND = 2.0;       // ±2 °C banda muerta

// Ganancias (ajustables desde UI)
float kpF = 10.0;                 // Kp foco (HEAT)
float kiF = 0.08;                 // Ki foco (HEAT) [PWM/°C·s]
float kpV = 5.0;                  // Kp ventilador (FAN)

// Extras para mejorar respuesta
const float BOOST_DELTA   = 5.0;  // si faltan >5°C -> calor a tope
const int   HEAT_MIN_PWM  = 60;   // piso mínimo de PWM al aproximarse

// Circulación mínima de aire cerca del setpoint
const float FAN_PREHEAT_DELTA = 2.0f; // enciende FAN desde sp-2°C
const int   CIRC_MIN_PWM      = 60;   // 0–255

// Escala PWM
const int PWM_MAX = 255;

// Calibración LM35
const float CAL_OFFSET = 0.0;     // °C
const float CAL_GAIN   = 1.000;   // 1.0 = sin ajuste

// Muestreo / UI
const int   NSAMPLES       = 32;
const unsigned long UI_MS  = 500;
unsigned long tUI          = 0;

// Estado
int pwmHeat = 0; // 0–255
int pwmFan  = 0; // 0–255

// Integrador (calefacción) y tiempo
float iHeat = 0.0f;
unsigned long tPrev = 0;

// Serial buffer
String inbuf;

// ---------- Util ----------
static inline int clampPWM(float x){
  if (x < 0) x = 0;
  if (x > PWM_MAX) x = PWM_MAX;
  return (int)(x + 0.5f);
}

// Anti-windup: integra solo si la salida no está saturada o si la integral "ayuda" a salir de saturación
static inline bool allowIntegrate(int u, float err){
  if (u > 0 && u < PWM_MAX) return true;
  if (u <= 0 && err > 0)    return true;   // si está en 0 pero el error pide subir
  if (u >= PWM_MAX && err < 0) return true; // si está saturado arriba pero el error pide bajar
  return false;
}

// ---------- Setup ----------
void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  analogWrite(PIN_HEATER, 0);
  analogWrite(PIN_FAN, 0);

  Serial.begin(115200);
  analogReference(INTERNAL); // ~1.1 V

  tPrev = millis();

  // Mensaje inicial con configuración
  Serial.print(F("{\"info\":\"TempCtrl ready (PI)\",\"sp\":"));
  Serial.print(setpointC, 2);
  Serial.print(F(",\"kpF\":"));
  Serial.print(kpF, 2);
  Serial.print(F(",\"kiF\":"));
  Serial.print(kiF, 3);
  Serial.print(F(",\"kpV\":"));
  Serial.print(kpV, 2);
  Serial.print(F(",\"db\":"));
  Serial.print(DEADBAND, 2);
  Serial.println(F("}"));
}

// ---------- Loop ----------
void loop() {
  // 1) Comandos entrantes
  readSerial();

  // 2) Medición temperatura + calibración
  float tC = readTempC();
  tC = CAL_GAIN * tC + CAL_OFFSET;

  // 2.1) dt en segundos
  unsigned long now = millis();
  float dt = (now - tPrev) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f; // mínimo
  tPrev = now;

  // 3) Error (positivo -> calentar; negativo -> enfriar)
  float e = setpointC - tC;

  // 3.1) Lógica de control
  if (fabs(e) <= DEADBAND) {
    // Banda muerta: apaga acciones proporcionales
    pwmHeat = 0;
    pwmFan  = 0;
    // NO integramos en deadband (evita “creep”)
  }
  else if (e > DEADBAND) {
    // ---- CALEFACCION (PI) ----
    // a) Boost si estás muy por debajo
    if (e > BOOST_DELTA) {
      pwmFan  = 0;
      pwmHeat = PWM_MAX;  // empuje máximo
      // No integramos en boost para evitar exceso de acumulación
    } else {
      // b) PI con anti-windup
      float p = kpF * e;
      // Integra solo si conviene (anti-windup)
      if (allowIntegrate(pwmHeat, e)) {
        iHeat += (kiF * e * dt);
        // Limita la contribución integral a un rango razonable en términos de PWM
        if (iHeat > PWM_MAX) iHeat = PWM_MAX;
        if (iHeat < 0)       iHeat = 0;   // no “empujes” negativo en calefacción
      }
      float u = p + iHeat;
      pwmHeat = clampPWM(u);

      // Piso mínimo al aproximarse (evita que caiga demasiado pronto)
      if (pwmHeat > 0) {
        if (pwmHeat < HEAT_MIN_PWM) pwmHeat = HEAT_MIN_PWM;
      }

      pwmFan = 0;
    }
  }
  else { // e < -DEADBAND
    // ---- ENFRIAMIENTO (P) ----
    pwmHeat = 0;
    iHeat   = 0; // resetea integral cuando estás enfriando (opcional, simplifica)
    pwmFan  = clampPWM(kpV * (-e));
  }

  // --- Circulación mínima al estar a ≤2°C del setpoint ---
  bool nearSetpoint = (tC >= (setpointC - FAN_PREHEAT_DELTA));
  if (nearSetpoint) {
    pwmFan = max(pwmFan, CIRC_MIN_PWM);
  }

  // 4) Salida PWM
  analogWrite(PIN_HEATER, pwmHeat);
  analogWrite(PIN_FAN,    pwmFan);

  // 5) Telemetría JSON periódica
  if (millis() - tUI > UI_MS) {
    tUI = millis();
    Serial.print(F("{\"t\":"));
    Serial.print(tC, 2);
    Serial.print(F(",\"sp\":"));
    Serial.print(setpointC, 2);

    // Modo textual
    Serial.print(F(",\"mode\":\""));
    if (pwmHeat > 0 && pwmFan == 0) Serial.print(F("HEAT"));
    else if (pwmFan > 0 && pwmHeat == 0) Serial.print(F("FAN"));
    else if (pwmFan > 0 && pwmHeat > 0) Serial.print(F("HEAT+FAN"));
    else Serial.print(F("IDLE"));
    Serial.print(F("\""));

    // PWM en % para UI
    float pwmH_pct = 100.0f * ((float)pwmHeat / PWM_MAX);
    float pwmF_pct = 100.0f * ((float)pwmFan  / PWM_MAX);
    Serial.print(F(",\"pwmH\":")); Serial.print(pwmH_pct, 1);
    Serial.print(F(",\"pwmF\":")); Serial.print(pwmF_pct, 1);

    // Ganancias actuales
    Serial.print(F(",\"kpF\":")); Serial.print(kpF, 2);
    Serial.print(F(",\"kiF\":")); Serial.print(kiF, 3);
    Serial.print(F(",\"kpV\":")); Serial.print(kpV, 2);
    Serial.println(F("}"));
  }
}

// ---------- Lectura LM35 ----------
float readTempC() {
  unsigned long acc = 0;
  for (int i = 0; i < NSAMPLES; i++) {
    acc += analogRead(PIN_LM35);
    delayMicroseconds(300);
  }
  float counts = acc / (float)NSAMPLES;
  const float mV_per_count = 1100.0 / 1024.0; // Vref ~1.1 V
  float mV = counts * mV_per_count;
  return mV / 10.0; // 10 mV/°C del LM35
}

// ---------- Parser Serial ----------
void processLine(String s) {
  s.trim();
  s.replace(",", ".");

  // Setpoint: S=xx.x
  if (s.startsWith("S=") || s.startsWith("s=")) {
    float v = s.substring(2).toFloat();
    if (v >= 30.0 && v <= 50.0) {
      setpointC = v;
      Serial.print(F("{\"ack\":\"SP\",\"sp\":"));
      Serial.print(setpointC, 2);
      Serial.println(F("}"));
    } else {
      Serial.println(F("{\"error\":\"SP out of range (30-50)\"}"));
    }
    return;
  }

  // KPF=valor (o PF=valor)   1–30
  if (s.startsWith("KPF=") || s.startsWith("kpf=") || s.startsWith("PF=") || s.startsWith("pf=")) {
    int eq = s.indexOf('=');
    float v = s.substring(eq+1).toFloat();
    if (v >= 1.0 && v <= 30.0) {
      kpF = v;
      Serial.print(F("{\"ack\":\"KPF\",\"kpF\":"));
      Serial.print(kpF, 2);
      Serial.println(F("}"));
    } else {
      Serial.println(F("{\"error\":\"KPF out of range (1-30)\"}"));
    }
    return;
  }

  // KIF=valor   0.00–1.00
  if (s.startsWith("KIF=") || s.startsWith("kif=")) {
    int eq = s.indexOf('=');
    float v = s.substring(eq+1).toFloat();
    if (v >= 0.00 && v <= 1.00) {
      kiF = v;
      // Reset opcional del integrador si cambias Ki mucho:
      // iHeat = 0;
      Serial.print(F("{\"ack\":\"KIF\",\"kiF\":"));
      Serial.print(kiF, 3);
      Serial.println(F("}"));
    } else {
      Serial.println(F("{\"error\":\"KIF out of range (0.00-1.00)\"}"));
    }
    return;
  }

  // KPV=valor (o PV=valor)   1–30
  if (s.startsWith("KPV=") || s.startsWith("kpv=") || s.startsWith("PV=") || s.startsWith("pv=")) {
    int eq = s.indexOf('=');
    float v = s.substring(eq+1).toFloat();
    if (v >= 1.0 && v <= 30.0) {
      kpV = v;
      Serial.print(F("{\"ack\":\"KPV\",\"kpV\":"));
      Serial.print(kpV, 2);
      Serial.println(F("}"));
    } else {
      Serial.println(F("{\"error\":\"KPV out of range (1-30)\"}"));
    }
    return;
  }

  Serial.println(F("{\"error\":\"Unknown cmd\"}"));
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) { processLine(inbuf); inbuf = ""; }
    } else {
      inbuf += c;
      if (inbuf.length() > 60) inbuf = ""; // sanity
    }
  }
}
