/*
  Control de Temperatura – LM35 + Arduino UNO (MOSFET low-side)
  - HEAT: foco 12 V (resistivo) en D3 (active-HIGH)
  - FAN : ventilador 12 V (inductivo, con diodo flyback) en D5 (active-HIGH)
  - Lectura LM35 con referencia interna (~1.1 V) y promediado
  - Control ON/OFF con histéresis y exclusión mutua
  - UI Web por Web Serial (JSON periódico + comando S=xx.x)
*/

#include <Arduino.h>

// --------- Pines ----------
const int PIN_LM35   = A0;
const int PIN_HEATER = 3;   // Gate MOSFET HEAT (active-HIGH)
const int PIN_FAN    = 5;   // Gate MOSFET FAN  (active-HIGH)

// --------- Control --------
float setpointC = 37.0;     // °C inicial
float hyst      = 0.5;      // histéresis +/- °C (MOSFET: 0.5–1.5 recomendable)

// Calibración LM35
const float CAL_OFFSET = 0.0;   // °C (ajusta tras comparar con termómetro)
const float CAL_GAIN   = 1.000; // 1.0 = sin ajuste

// Muestreo / UI
const int   NSAMPLES       = 32;
const unsigned long UI_MS  = 500;
unsigned long tUI          = 0;

// Estado
bool heatOn = false;
bool fanOn  = false;

// Serial buffer
String inbuf;

// ---------- Setup ----------
void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  digitalWrite(PIN_FAN, LOW);

  Serial.begin(115200);

  // Mejor resolución con referencia interna (~1.1 V) en UNO
  analogReference(INTERNAL); // en algunos clones: INTERNAL1V1

  // Mensaje inicial
  Serial.println(F("{\"info\":\"TempCtrl ready (MOSFET)\",\"sp\":37.0}"));
}

// ---------- Loop ----------
void loop() {
  // 1) Comandos entrantes
  readSerialSetpoint();

  // 2) Medición temperatura + calibración
  float tC = readTempC();
  tC = CAL_GAIN * tC + CAL_OFFSET;

  // 3) Control ON/OFF con histéresis y exclusión mutua
  bool wantHeat = false, wantFan = false;

  if (tC < (setpointC - hyst)) {
    wantHeat = true;  wantFan = false;
  } else if (tC > (setpointC + hyst)) {
    wantFan  = true;  wantHeat = false;
  } else {
    wantHeat = false; wantFan  = false;
  }

  // Exclusión mutua
  if (wantHeat) { setFan(false); setHeater(true); }
  else if (wantFan) { setHeater(false); setFan(true); }
  else { setHeater(false); setFan(false); }

  // 4) Telemetría JSON periódica
  if (millis() - tUI > UI_MS) {
    tUI = millis();
    Serial.print(F("{\"t\":"));
    Serial.print(tC, 2);
    Serial.print(F(",\"sp\":"));
    Serial.print(setpointC, 2);
    Serial.print(F(",\"mode\":\""));
    if (heatOn) Serial.print(F("HEAT"));
    else if (fanOn) Serial.print(F("FAN"));
    else Serial.print(F("IDLE"));
    Serial.println(F("\"}"));
  }
}

// ---------- Salidas ----------
void setHeater(bool on) {
  heatOn = on;
  digitalWrite(PIN_HEATER, heatOn ? HIGH : LOW); // MOSFET active-HIGH
}
void setFan(bool on) {
  fanOn = on;
  digitalWrite(PIN_FAN, fanOn ? HIGH : LOW); // MOSFET active-HIGH
}

// ---------- Lectura LM35 ----------
float readTempC() {
  unsigned long acc = 0;
  for (int i = 0; i < NSAMPLES; i++) {
    acc += analogRead(PIN_LM35);
    delayMicroseconds(300);
  }
  float counts = acc / (float)NSAMPLES;

  // Vref interna ~1.1 V -> 1100 mV / 1024 counts
  const float mV_per_count = 1100.0 / 1024.0; // ≈1.074 mV
  float mV = counts * mV_per_count;
  return mV / 10.0; // 10 mV/°C del LM35
}

// ---------- Parser Serial ----------
void readSerialSetpoint() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) { processLine(inbuf); inbuf = ""; }
    } else {
      inbuf += c;
      if (inbuf.length() > 50) inbuf = ""; // sanity
    }
  }
}
void processLine(String s) {
  s.trim();
  s.replace(",", ".");
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
  } else if (s.startsWith("H=") || s.startsWith("h=")) {
    // Permite ajustar histéresis desde la web (opcional): H=1.5
    float v = s.substring(2).toFloat();
    if (v >= 0.5 && v <= 3.0) {
      hyst = v;
      Serial.print(F("{\"ack\":\"H\",\"h\":"));
      Serial.print(hyst, 2);
      Serial.println(F("}"));
    } else {
      Serial.println(F("{\"error\":\"H out of range (0.5-3.0)\"}"));
    }
  } else {
    Serial.println(F("{\"error\":\"Unknown cmd\"}"));
  }
}