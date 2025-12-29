#include <Arduino.h>
#include <math.h>

// -------------------- Pins --------------------
const int THERMISTOR_PIN = A0;
const int SSR_PIN = 9;

// If your SSR input is active-low, set true:
const bool SSR_ACTIVE_LOW = false;

// -------------------- NTC parameters --------------------
const float SERIES_RESISTOR      = 100000.0f;  // fixed resistor
const float NOMINAL_RESISTANCE   = 100000.0f;  // NTC @25C
const float NOMINAL_TEMPERATURE  = 25.0f;      // C
const float BETA_COEFFICIENT     = 3950.0f;

// -------------------- Filters --------------------
const float TEMP_FILTER_ALPHA = 0.15f;
const float RATE_FILTER_ALPHA = 0.20f;

// -------------------- PID (0..1 duty) --------------------
const float KP = 8.0f;
const float KI = 0.06f;
const float KD = 2.0f;

const float PID_INTEGRAL_MIN = -50.0f;
const float PID_INTEGRAL_MAX =  50.0f;

// -------------------- SSR windowing --------------------
const unsigned long CONTROL_WINDOW_MS = 1000;

// -------------------- Telemetry --------------------
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// -------------------- Adaptive profile follower (“time-warp”) --------------------
const float TIME_WARP_GAIN = 0.035f;   // 1/°C
const float MIN_TIME_RATE  = 0.15f;
const float MAX_TIME_RATE  = 1.50f;

// Ramp-limit setpoint (reduces command jumps)
const float MAX_RAMP_UP_C_PER_S   = 1.0f;
const float MAX_RAMP_DOWN_C_PER_S = 2.0f;

// -------------------- Warmup gating --------------------
const float PREHEAT_START_RATE_C_PER_S = 0.30f;

// -------------------- Your requested T1/T2 behaviors --------------------
// T1: coast at start of SOAK (heaters OFF) to reduce overshoot
const float SOAK_COAST_SECONDS = 20.0f;

// T2: boost during REFLOW if behind slope/temperature
const float REFLOW_BOOST_ERROR_C = 8.0f;     // if setpoint - actual > this => boost
const float REFLOW_BOOST_MIN_RATE = 0.7f;    // if temp rise rate is below this and behind => boost

// Overshoot guard: if actual exceeds setpoint by this, force OFF until back under (hysteresis)
const float OVERSHOOT_OFF_C = 3.0f;
const float OVERSHOOT_ON_C  = 1.0f;

// -------------------- Profile --------------------
struct ProfileStep {
  unsigned long durationMs;
  float startTempC;
  float endTempC;
  const char *label;
};

ProfileStep profile[] = {
  {150000, 25.0f, 150.0f, "PREHEAT"},
  {120000, 150.0f, 180.0f, "SOAK"},
  {45000,  180.0f, 225.0f, "REFLOW"},
  {120000, 225.0f, 50.0f,  "COOL"}
};

const int PROFILE_COUNT = sizeof(profile) / sizeof(profile[0]);

static unsigned long totalProfileMs()
{
  unsigned long total = 0;
  for (int i = 0; i < PROFILE_COUNT; i++) total += profile[i].durationMs;
  return total;
}

// -------------------- State --------------------
enum RunState { IDLE, WARMUP, RUNNING, DONE };
RunState state = IDLE;

bool running = false;
bool completed = false;

unsigned long lastLoopMs = 0;

// “Profile time” PT (warped) in seconds
float profileTime_s = 0.0f;

// SSR window
unsigned long windowStartMs = 0;

// Telemetry
unsigned long lastTelemetryMs = 0;

// PID
float pidIntegral = 0.0f;
float prevTempForD = 0.0f;
unsigned long lastPidMs = 0;

// Temperature
float filteredTempC = 25.0f;
float filteredRateCps = 0.0f;
float lastRateTemp = 25.0f;
unsigned long lastRateMs = 0;

// Step timing helpers
static bool getStepInfo(unsigned long ptMs, int &stepIndex, unsigned long &stepStartMs, unsigned long &stepEndMs)
{
  unsigned long acc = 0;
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    unsigned long end = acc + profile[i].durationMs;
    if (ptMs <= end)
    {
      stepIndex = i;
      stepStartMs = acc;
      stepEndMs = end;
      return true;
    }
    acc = end;
  }
  stepIndex = PROFILE_COUNT - 1;
  stepStartMs = totalProfileMs() - profile[PROFILE_COUNT - 1].durationMs;
  stepEndMs = totalProfileMs();
  return false;
}

static void ssrWrite(bool enabled)
{
  bool level = enabled ? HIGH : LOW;
  if (SSR_ACTIVE_LOW) level = !level;
  digitalWrite(SSR_PIN, level);
}

static float convertAdcToTempC(int adc)
{
  if (adc <= 0) adc = 1;
  if (adc >= 1023) adc = 1022;

  // Rntc = Rfixed * adc / (1023 - adc)
  float r = SERIES_RESISTOR * ((float)adc / (1023.0f - adc));

  float steinhart = r / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0f / (NOMINAL_TEMPERATURE + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;

  if (steinhart < -40.0f) steinhart = -40.0f;
  if (steinhart > 350.0f) steinhart = 350.0f;
  return steinhart;
}

static float readTempC()
{
  return convertAdcToTempC(analogRead(THERMISTOR_PIN));
}

static float computePidDuty(float setpoint, float tempC)
{
  unsigned long now = millis();
  if (lastPidMs == 0)
  {
    lastPidMs = now;
    prevTempForD = tempC;
  }

  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  float error = setpoint - tempC;
  float dTemp = (tempC - prevTempForD) / dt;
  float derivative = -dTemp;

  float output = KP * error + KI * pidIntegral + KD * derivative;

  bool satHigh = output >= 1.0f;
  bool satLow  = output <= 0.0f;

  if ((!satHigh || error < 0.0f) && (!satLow || error > 0.0f))
    pidIntegral += error * dt;

  pidIntegral = constrain(pidIntegral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);

  output = KP * error + KI * pidIntegral + KD * derivative;

  prevTempForD = tempC;
  lastPidMs = now;

  if (output > 1.0f) output = 1.0f;
  if (output < 0.0f) output = 0.0f;
  return output;
}

static float nominalSetpointFromPT(unsigned long ptMs, const char *&phaseLabel)
{
  unsigned long acc = 0;
  for (int i = 0; i < PROFILE_COUNT; i++)
  {
    unsigned long end = acc + profile[i].durationMs;
    if (ptMs <= end)
    {
      float u = (float)(ptMs - acc) / (float)profile[i].durationMs;
      phaseLabel = profile[i].label;
      return profile[i].startTempC + (profile[i].endTempC - profile[i].startTempC) * u;
    }
    acc = end;
  }
  phaseLabel = "DONE";
  return profile[PROFILE_COUNT - 1].endTempC;
}

static void startProfile()
{
  running = true;
  completed = false;
  state = WARMUP;

  profileTime_s = 0.0f;
  lastLoopMs = millis();

  windowStartMs = millis();
  lastTelemetryMs = 0;

  pidIntegral = 0.0f;
  lastPidMs = 0;

  float t = readTempC();
  filteredTempC = t;
  lastRateTemp = t;
  lastRateMs = 0;
  filteredRateCps = 0.0f;
}

static void stopProfile()
{
  running = false;
  completed = false;
  state = IDLE;
  ssrWrite(false);
}

void setup()
{
  pinMode(SSR_PIN, OUTPUT);
  ssrWrite(false);
  Serial.begin(115200);
}

void loop()
{
  // Commands
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("START")) startProfile();
    if (cmd.equalsIgnoreCase("ABORT")) stopProfile();
  }

  unsigned long now = millis();

  // Read and filter temp
  float t = readTempC();
  filteredTempC = filteredTempC + TEMP_FILTER_ALPHA * (t - filteredTempC);

  // Filtered rate
  if (lastRateMs == 0)
  {
    lastRateMs = now;
    lastRateTemp = filteredTempC;
  }
  else
  {
    float dt = (now - lastRateMs) / 1000.0f;
    if (dt > 0.0f)
    {
      float rate = (filteredTempC - lastRateTemp) / dt;
      filteredRateCps = filteredRateCps + RATE_FILTER_ALPHA * (rate - filteredRateCps);
      lastRateTemp = filteredTempC;
      lastRateMs = now;
    }
  }

  float setpoint = 0.0f;
  const char *phaseLabel = "IDLE";

  // Default heater off
  bool heaterOn = false;

  if (running)
  {
    // dt for PT integration
    float dt_s = (now - lastLoopMs) / 1000.0f;
    if (dt_s < 0.0f) dt_s = 0.0f;
    if (dt_s > 1.5f) dt_s = 1.5f;
    lastLoopMs = now;

    if (state == WARMUP)
    {
      phaseLabel = "PREHEAT";
      // Force ON to overcome initial dead-time
      heaterOn = true;
      setpoint = profile[0].startTempC;

      // Only start profile time once we see heating rate
      if (filteredRateCps >= PREHEAT_START_RATE_C_PER_S)
      {
        state = RUNNING;
        profileTime_s = 0.0f;
        windowStartMs = now;
        pidIntegral = 0.0f;
        lastPidMs = 0;
      }
    }
    else if (state == RUNNING)
    {
      // ---- PT always advances (IMPORTANT: fixes “time stops” on your plot) ----
      unsigned long ptMs = (unsigned long)(profileTime_s * 1000.0f);

      // nominal setpoint and phase
      float nominal = nominalSetpointFromPT(ptMs, phaseLabel);

      // done?
      if (ptMs >= totalProfileMs())
      {
        running = false;
        completed = true;
        state = DONE;
        ssrWrite(false);
      }
      else
      {
        // time-warp rate based on error to nominal
        float errorNominal = nominal - filteredTempC;
        float timeRate = 1.0f - TIME_WARP_GAIN * errorNominal;
        if (timeRate < MIN_TIME_RATE) timeRate = MIN_TIME_RATE;
        if (timeRate > MAX_TIME_RATE) timeRate = MAX_TIME_RATE;

        // Advance PT regardless of phase (including COOL)
        profileTime_s += dt_s * timeRate;

        // recompute with updated PT
        ptMs = (unsigned long)(profileTime_s * 1000.0f);
        nominal = nominalSetpointFromPT(ptMs, phaseLabel);

        // ramp-limit setpoint to avoid big jumps
        static float rampedSetpoint = 25.0f;
        float maxUp = MAX_RAMP_UP_C_PER_S * dt_s;
        float maxDown = MAX_RAMP_DOWN_C_PER_S * dt_s;

        if (nominal > rampedSetpoint) rampedSetpoint = min(nominal, rampedSetpoint + maxUp);
        else                          rampedSetpoint = max(nominal, rampedSetpoint - maxDown);

        setpoint = rampedSetpoint;

        // COOL: heater off
        if (strcmp(phaseLabel, "COOL") == 0)
        {
          heaterOn = false;
          pidIntegral = 0.0f;
        }
        else
        {
          // Determine step timing for T1/T2 decisions
          int stepIndex;
          unsigned long stepStart, stepEnd;
          getStepInfo(ptMs, stepIndex, stepStart, stepEnd);
          float inStep_s = (ptMs - stepStart) / 1000.0f;

          // -------------------- T1: SOAK coast (heaters OFF at start of SOAK) --------------------
          bool soakCoastActive = (strcmp(phaseLabel, "SOAK") == 0) && (inStep_s < SOAK_COAST_SECONDS);

          // -------------------- Overshoot guard (force OFF if above setpoint) --------------------
          static bool overshootHoldOff = false;
          float overshoot = filteredTempC - setpoint;
          if (!overshootHoldOff && overshoot >= OVERSHOOT_OFF_C) overshootHoldOff = true;
          if (overshootHoldOff && overshoot <= OVERSHOOT_ON_C)   overshootHoldOff = false;

          // -------------------- T2: REFLOW boost (force FULL ON if behind) --------------------
          bool reflowBoost = false;
          if (strcmp(phaseLabel, "REFLOW") == 0)
          {
            float err = setpoint - filteredTempC; // positive means we're behind
            if (err >= REFLOW_BOOST_ERROR_C) reflowBoost = true;
            if (err > 4.0f && filteredRateCps < REFLOW_BOOST_MIN_RATE) reflowBoost = true;
          }

          // Apply control decisions
          if (soakCoastActive || overshootHoldOff)
          {
            heaterOn = false;
            pidIntegral = 0.0f;
          }
          else if (reflowBoost)
          {
            heaterOn = true; // full on
          }
          else
          {
            // Normal PID -> SSR window
            float duty = computePidDuty(setpoint, filteredTempC);

            if (now - windowStartMs >= CONTROL_WINDOW_MS)
              windowStartMs = now;

            unsigned long onTime = (unsigned long)(duty * CONTROL_WINDOW_MS);
            heaterOn = ((now - windowStartMs) < onTime);
          }
        }
      }
    }
  }
  else
  {
    ssrWrite(false);
  }

  // Apply heater output
  if (running && state == RUNNING)
  {
    // heaterOn already computed
    ssrWrite(heaterOn);
  }
  else if (running && state == WARMUP)
  {
    ssrWrite(true);
  }
  else
  {
    ssrWrite(false);
  }

  // Telemetry
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMs = now;

    Serial.print("T:");
    Serial.print(filteredTempC, 2);

    Serial.print(";S:");
    Serial.print(setpoint, 2);

    Serial.print(";STATE:");
    if (running) Serial.print("RUNNING");
    else Serial.print(completed ? "DONE" : "IDLE");

    Serial.print(";PHASE:");
    Serial.print(phaseLabel);

    Serial.print(";PT:");
    Serial.print(profileTime_s, 1);

    Serial.println();
  }
}
