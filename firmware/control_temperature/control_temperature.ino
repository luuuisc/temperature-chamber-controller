/*
  Control de Temperatura con Arduino UNO + LabVIEW
  -------------------------------------------------
  - Sensor: LM35 en A0 (referencia interna ~1.1 V, promediado).
  - Actuadores:
      * HEAT: foco 12 V en D3 (MOSFET, PWM activo HIGH).
      * FAN : ventilador 12 V en D5 (MOSFET + diodo flyback, PWM activo HIGH).
  - Control: PI en calefacción, P en ventilación, banda muerta ±2 °C.
  - Circulación mínima de aire desde (sp - 2 °C).
  - Telemetría hacia LabVIEW: línea JSON por ciclo, terminada en '\n'.
    Ejemplo: {"t":36.42,"sp":37.00,"mode":"HEAT","pwmH":42.3,"pwmF":0.0,"kpF":10.0,"kiF":0.08,"kpV":5.0}
  - Comandos desde LabVIEW (terminados en '\n'):
      S=xx.x     (30.0–50.0 °C)
      KPF=x      (1–30)
      KIF=x.xx   (0.00–1.00)
      KPV=x      (1–30)

  Ajustes recomendados en LabVIEW:
  - 115200 baudios, 8N1, sin flow control.
  - Habilitar termination char = 0x0A ('\n') tanto para lectura como para escritura.
*/

/*
  Arduino + LabVIEW (formato JSON estable)
  - SIEMPRE imprimimos la misma estructura JSON con campos: t, sp, mode, pwmH, pwmF, kpF, kiF, kpV
  - Si llega un comando (S=, KPF=, KIF=, KPV=), NO imprimimos un JSON distinto;
    solo marcamos un "ack" DENTRO de la siguiente telemetría (LabVIEW lo ignora).
*/

#include <Arduino.h>

// --------- Pines ----------
const int PIN_LM35   = A0;
const int PIN_HEATER = 3;   // PWM
const int PIN_FAN    = 5;   // PWM

// --------- Parámetros de control --------
float setpointC = 37.0;
const float DEADBAND = 2.0;

float kpF = 10.0;    // foco
float kiF = 0.08;    // foco
float kpV = 5.0;     // ventilador

const float BOOST_DELTA   = 5.0;
const int   HEAT_MIN_PWM  = 60;
const float FAN_PREHEAT_DELTA = 2.0f;
const int   CIRC_MIN_PWM      = 60;

const int PWM_MAX = 255;

// Calibración LM35
const float CAL_OFFSET = 0.0;
const float CAL_GAIN   = 1.000;

// Muestreo / UI
const int   NSAMPLES       = 32;
const unsigned long UI_MS  = 500;
unsigned long tUI          = 0;

// Estado
int pwmHeat = 0;
int pwmFan  = 0;
float iHeat = 0.0f;
unsigned long tPrev = 0;

// Serial buffer
String inbuf;

// ACK embebido
bool ackPending = false;
String ackTag = "";

// ---------- Utilidades ----------
static inline int clampPWM(float x){
  if (x < 0) x = 0;
  if (x > PWM_MAX) x = PWM_MAX;
  return (int)(x + 0.5f);
}

static inline bool allowIntegrate(int u, float err){
  if (u > 0 && u < PWM_MAX) return true;
  if (u <= 0 && err > 0)    return true;
  if (u >= PWM_MAX && err < 0) return true;
  return false;
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
  return mV / 10.0; // 10 mV/°C
}

// ---------- Telemetría ----------
void printTelemetry(float tC) {
  Serial.print(F("{\"t\":"));
  Serial.print(tC, 2);
  Serial.print(F(",\"sp\":"));
  Serial.print(setpointC, 2);

  Serial.print(F(",\"mode\":\""));
  if (pwmHeat > 0 && pwmFan == 0) Serial.print(F("HEAT"));
  else if (pwmFan > 0 && pwmHeat == 0) Serial.print(F("FAN"));
  else if (pwmFan > 0 && pwmHeat > 0)  Serial.print(F("HEAT+FAN"));
  else                                  Serial.print(F("IDLE"));
  Serial.print(F("\""));

  float pwmH_pct = 100.0f * ((float)pwmHeat / PWM_MAX);
  float pwmF_pct = 100.0f * ((float)pwmFan  / PWM_MAX);
  Serial.print(F(",\"pwmH\":")); Serial.print(pwmH_pct, 1);
  Serial.print(F(",\"pwmF\":")); Serial.print(pwmF_pct, 1);

  Serial.print(F(",\"kpF\":")); Serial.print(kpF, 2);
  Serial.print(F(",\"kiF\":")); Serial.print(kiF, 3);
  Serial.print(F(",\"kpV\":")); Serial.print(kpV, 2);

  // ACK embebido (opcional; LabVIEW lo ignora)
  if (ackPending) {
    Serial.print(F(",\"ack\":\""));
    Serial.print(ackTag);
    Serial.print(F("\""));
    ackPending = false;
    ackTag = "";
  }

  Serial.println(F("}"));
}

// ---------- Parser de comandos ----------
void processLine(String s) {
  s.trim();
  s.replace(",", ".");

  if (s.startsWith("S=")) {
    float v = s.substring(2).toFloat();
    if (v >= 30.0 && v <= 50.0) {
      setpointC = v;
      ackPending = true; ackTag = "SP";
    }
    return;
  }

  if (s.startsWith("KPF=")) {
    float v = s.substring(4).toFloat();
    if (v >= 1.0 && v <= 30.0) { kpF = v; ackPending = true; ackTag = "KPF"; }
    return;
  }

  if (s.startsWith("KIF=")) {
    float v = s.substring(4).toFloat();
    if (v >= 0.00 && v <= 1.00) { kiF = v; ackPending = true; ackTag = "KIF"; }
    return;
  }

  if (s.startsWith("KPV=")) {
    float v = s.substring(4).toFloat();
    if (v >= 1.0 && v <= 30.0) { kpV = v; ackPending = true; ackTag = "KPV"; }
    return;
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) { processLine(inbuf); inbuf = ""; }
    } else {
      inbuf += c;
      if (inbuf.length() > 60) inbuf = "";
    }
  }
}

// ---------- Setup ----------
void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  analogWrite(PIN_HEATER, 0);
  analogWrite(PIN_FAN, 0);

  Serial.begin(115200);
  analogReference(INTERNAL); // ~1.1 V

  // Pequeño retardo para evitar basura inicial
  delay(700);

  tPrev = millis();
}

// ---------- Loop ----------
void loop() {
  // 1) Comandos
  readSerial();

  // 2) Medición y dt
  float tC = CAL_GAIN * readTempC() + CAL_OFFSET;

  unsigned long now = millis();
  float dt = (now - tPrev) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  tPrev = now;

  // 3) Control
  float e = setpointC - tC;

  if (fabs(e) <= DEADBAND) {
    pwmHeat = 0;
    pwmFan  = 0;
  } else if (e > DEADBAND) {
    if (e > BOOST_DELTA) {
      pwmFan  = 0;
      pwmHeat = PWM_MAX;
    } else {
      float p = kpF * e;
      if (allowIntegrate(pwmHeat, e)) {
        iHeat += (kiF * e * dt);
        if (iHeat > PWM_MAX) iHeat = PWM_MAX;
        if (iHeat < 0)       iHeat = 0;
      }
      float u = p + iHeat;
      pwmHeat = clampPWM(u);
      if (pwmHeat > 0 && pwmHeat < HEAT_MIN_PWM) pwmHeat = HEAT_MIN_PWM;
      pwmFan = 0;
    }
  } else { // e < -DEADBAND
    pwmHeat = 0;
    iHeat   = 0;
    pwmFan  = clampPWM(kpV * (-e));
  }

  // Circulación mínima al acercarse al setpoint
  if (tC >= (setpointC - FAN_PREHEAT_DELTA)) {
    pwmFan = max(pwmFan, CIRC_MIN_PWM);
  }

  // 4) PWM
  analogWrite(PIN_HEATER, pwmHeat);
  analogWrite(PIN_FAN,    pwmFan);

  // 5) Telemetría periódica (formato SIEMPRE igual)
  if (millis() - tUI > UI_MS) {
    tUI = millis();
    printTelemetry(tC);
  }
}
