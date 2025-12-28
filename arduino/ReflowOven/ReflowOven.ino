// Reflow oven controller for Arduino Uno R3
// Hardware:
// 5V
//  |
// [100k fixed]
//  |
//  +---- A0
//  |
// [NTC 100k]
//  |
// GND
// SSR-25DA connected to SSR_PIN (controls toaster oven heating element)

#include <Arduino.h>

const int THERMISTOR_PIN = A0;
const int SSR_PIN = 9;

// NTC thermistor configuration
const float SERIES_RESISTOR = 100000.0f;
const float NOMINAL_RESISTANCE = 100000.0f;
const float NOMINAL_TEMPERATURE = 25.0f; // Celsius
const float BETA_COEFFICIENT = 3950.0f;


// PID configuration
const float KP = 8.0f;
const float KI = 0.06f;
const float KD = 2.0f;

const float PID_INTEGRAL_MIN = -50.0f;
const float PID_INTEGRAL_MAX = 50.0f;

const unsigned long CONTROL_WINDOW_MS = 1000;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

struct ProfileStep {
  unsigned long durationMs;
  float startTempC;
  float endTempC;
  const char *label;
};

ProfileStep profile[] = {
    {90000, 25.0f, 150.0f, "PREHEAT"},
    {90000, 150.0f, 180.0f, "SOAK"},
    {60000, 180.0f, 235.0f, "REFLOW"},
    {120000, 235.0f, 50.0f, "COOL"}};

const int PROFILE_COUNT = sizeof(profile) / sizeof(profile[0]);

bool running = false;
bool completed = false;
unsigned long profileStartMs = 0;
unsigned long windowStartMs = 0;
unsigned long lastTelemetryMs = 0;

float pidIntegral = 0.0f;
float previousTemperature = 0.0f;
unsigned long lastPidMs = 0;

float convertAdcToNtcC(int adc) {
  if (adc <= 0 || adc >= 1023) {
    return -273.15f;
  }

  float resistance = SERIES_RESISTOR * ((float)adc / (1023.0f - adc));
  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0f / (NOMINAL_TEMPERATURE + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;
  return steinhart;
}

float readTemperatureC() {
  int adc = analogRead(THERMISTOR_PIN);
  return convertAdcToNtcC(adc);
}

float getSetpointC(unsigned long elapsedMs, const char **phaseLabel) {
  unsigned long accumulated = 0;
  for (int i = 0; i < PROFILE_COUNT; ++i) {
    unsigned long stepEnd = accumulated + profile[i].durationMs;
    if (elapsedMs <= stepEnd) {
      float t = (float)(elapsedMs - accumulated) / profile[i].durationMs;
      if (phaseLabel != nullptr) {
        *phaseLabel = profile[i].label;
      }
      return profile[i].startTempC +
             (profile[i].endTempC - profile[i].startTempC) * t;
    }
    accumulated = stepEnd;
  }
  if (phaseLabel != nullptr) {
    *phaseLabel = "DONE";
  }
  return profile[PROFILE_COUNT - 1].endTempC;
}

float computePid(float setpoint, float temperature) {
  unsigned long now = millis();
  if (lastPidMs == 0) {
    lastPidMs = now;
    previousTemperature = temperature;
  }
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) {
    dt = 0.001f;
  }

  float error = setpoint - temperature;
  float derivative = -(temperature - previousTemperature) / dt;

  float output = KP * error + KI * pidIntegral + KD * derivative;
  bool saturatedHigh = output >= 1.0f;
  bool saturatedLow = output <= 0.0f;
  if ((!saturatedHigh || error < 0.0f) && (!saturatedLow || error > 0.0f)) {
    pidIntegral += error * dt;
  }
  pidIntegral = constrain(pidIntegral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
  output = KP * error + KI * pidIntegral + KD * derivative;

  previousTemperature = temperature;
  lastPidMs = now;

  if (output > 1.0f) {
    output = 1.0f;
  } else if (output < 0.0f) {
    output = 0.0f;
  }
  return output;
}

void setHeater(bool enabled) { digitalWrite(SSR_PIN, enabled ? HIGH : LOW); }

void startProfile() {
  running = true;
  completed = false;
  profileStartMs = millis();
  windowStartMs = millis();
  pidIntegral = 0.0f;
  previousTemperature = readTemperatureC();
  lastPidMs = millis();
}

void stopProfile() {
  running = false;
  setHeater(false);
}

void setup() {
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("START")) {
      startProfile();
    } else if (command.equalsIgnoreCase("ABORT")) {
      stopProfile();
      completed = false;
    }
  }

  unsigned long now = millis();
  float temperature = readTemperatureC();
  float setpoint = 0.0f;
  const char *phaseLabel = "IDLE";

  if (running) {
    unsigned long elapsed = now - profileStartMs;
    setpoint = getSetpointC(elapsed, &phaseLabel);
    unsigned long totalMs = 0;
    for (int i = 0; i < PROFILE_COUNT; ++i) {
      totalMs += profile[i].durationMs;
    }
    if (elapsed >= totalMs) {
      running = false;
      completed = true;
      setHeater(false);
    } else {
      float duty = computePid(setpoint, temperature);
      if (now - windowStartMs >= CONTROL_WINDOW_MS) {
        windowStartMs = now;
      }
      if (duty * CONTROL_WINDOW_MS > (now - windowStartMs)) {
        setHeater(true);
      } else {
        setHeater(false);
      }
    }
  } else {
    setHeater(false);
  }

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    Serial.print("T:");
    Serial.print(temperature, 2);
    Serial.print(";S:");
    Serial.print(setpoint, 2);
    Serial.print(";STATE:");
    Serial.print(running ? "RUNNING" : (completed ? "DONE" : "IDLE"));
    Serial.print(";PHASE:");
    Serial.println(phaseLabel);
  }
}
