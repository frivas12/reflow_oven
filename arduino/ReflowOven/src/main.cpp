#include <Arduino.h>
#include <math.h>

// ============================================================
// Reflow Oven Controller (T0–T4 Heater State Machine)
// - T0: Full power boost (open loop) until near setpoint
// - T1: PID tracking during preheat ramp
// - T2: Heater OFF “coast” to prevent overshoot into soak
// - T3: PID tracking (late soak + early reflow tracking)
// - T4: Full power “pre-charge” before final reflow push
//
// Serial commands (115200):
//   START
//   ABORT
//
// Telemetry (every 500ms):
//   T:<temp>;S:<setpoint>;STATE:<RUNNING/IDLE/DONE>;PHASE:<PREHEAT/SOAK/REFLOW/COOL>;
//   MODE:<T0/T1/T2/T3/T4>;PT:<sec>;R:<C/s>
// ============================================================

// -------------------- Pins --------------------
const int THERMISTOR_PIN = A0;
const int SSR_PIN        = 9;

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

// -------------------- Profile --------------------
struct ProfileStep {
  unsigned long durationMs;
  float startTempC;
  float endTempC;
  const char *label;
};

// Your profile (matches UI):
ProfileStep profile[] = {
  {150000, 25.0f, 150.0f, "PREHEAT"},
  {120000, 150.0f, 180.0f, "SOAK"},
  {45000,  180.0f, 225.0f, "REFLOW"},
  {120000, 225.0f, 50.0f,  "COOL"}
};

const int PROFILE_COUNT = sizeof(profile) / sizeof(profile[0]);

// -------------------- Setpoint ramp limiting --------------------
const float MAX_RAMP_UP_C_PER_S   = 1.0f;
const float MAX_RAMP_DOWN_C_PER_S = 2.0f;

// -------------------- Warmup gating --------------------
const float PREHEAT_START_RATE_C_PER_S = 0.30f;

// -------------------- Profile-time slow-down (never speed up) --------------------
// This prevents entering COOL early if oven lags.
// Profile time runs at <= 1.0x, and slows if behind setpoint.
const float TIME_SLOW_GAIN = 0.060f;   // bigger => slows more when behind
const float MIN_TIME_RATE  = 0.10f;    // allow strong slowing if needed
const float MAX_TIME_RATE  = 1.00f;    // never faster than real time

// -------------------- Global overshoot guard --------------------
const float OVERSHOOT_OFF_C = 3.0f;   // force off if temp > setpoint + this
const float OVERSHOOT_ON_C  = 1.0f;   // release hold when temp <= setpoint + this

// -------------------- T0–T4 tuning --------------------
// T0: full power boost until close to setpoint
const float T0_ENTER_ERR_C = 10.0f;    // if behind setpoint by >= this, go T0
const float T0_EXIT_ERR_C  = 2.0f;     // exit T0 when within this of setpoint

// T1: PID in PREHEAT. T2: hard off near end of PREHEAT / early SOAK to prevent overshoot
const float T2_PREHEAT_CUTOFF_TEMP_C   = 143.0f; // if temp reaches this in PREHEAT, force coast
const float T2_PREHEAT_LAST_SEC        = 80.0f;  // or if last N sec of PREHEAT, force coast
const float T2_COAST_EXIT_RATE_CPS     = 0.08f;  // exit coast when slope flattens (C/s)
const float T2_COAST_RESUME_ERR_C      = 3.0f;   // exit coast if we fall behind by this

// T4: full power pre-charge before final reflow push
const float T4_SOAK_TRIGGER_FRACTION   = 0.60f;  // in SOAK, after 60% of SOAK time => T4
const float T4_REFLOW_LAG_ERR_C        = 6.0f;   // in REFLOW, if behind by >= this => T4
const float T4_EXIT_ERR_C              = 1.5f;   // exit T4 when within this of setpoint

// Reflow completion qualification (prevents COOL early)
const float REFLOW_PEAK_TEMP_C         = 220.0f; // “we truly hit reflow”
const float REFLOW_FALL_RATE_CPS       = -0.05f; // cooling started
const unsigned long REFLOW_MIN_HOLD_MS = 3000;   // optional: stay above peak for a moment
const unsigned long HARD_EXTRA_MS       = 60000; // absolute max extra run time beyond profile

// ============================================================
// State
// ============================================================
enum RunState { IDLE, WARMUP, RUNNING, DONE };
RunState runState = IDLE;

enum HeaterMode { HEATER_FULL_ON, HEATER_PID, HEATER_OFF };
enum ControlPhase { CP_T0, CP_T1, CP_T2, CP_T3, CP_T4, CP_COOL };

bool running = false;
bool completed = false;

unsigned long lastLoopMs = 0;

// “Profile time” PT (warped/slow only) in seconds
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

// Setpoint ramping / overshoot state
float rampedSetpoint = 25.0f;
bool overshootHoldOff = false;

// Control phase & reflow qualification
ControlPhase controlPhase = CP_T0;
unsigned long startRunMs = 0;

bool reflowPeakReached = false;
unsigned long reflowPeakHoldStartMs = 0;

// ============================================================
// Helpers
// ============================================================
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

static unsigned long totalProfileMs()
{
  unsigned long total = 0;
  for (int i = 0; i < PROFILE_COUNT; i++) total += profile[i].durationMs;
  return total;
}

static unsigned long endOfStepMs(int stepIndexInclusive)
{
  unsigned long acc = 0;
  for (int i = 0; i <= stepIndexInclusive && i < PROFILE_COUNT; i++)
    acc += profile[i].durationMs;
  return acc;
}

// Returns current step index and step start/end in ms based on ptMs
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

// Nominal setpoint from profile time
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

// PID returns duty 0..1
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

static const char* controlPhaseName(ControlPhase p)
{
  switch (p)
  {
    case CP_T0:   return "T0";
    case CP_T1:   return "T1";
    case CP_T2:   return "T2";
    case CP_T3:   return "T3";
    case CP_T4:   return "T4";
    case CP_COOL: return "COOL";
    default:      return "?";
  }
}

// ============================================================
// Run control
// ============================================================
static void resetPid()
{
  pidIntegral = 0.0f;
  lastPidMs = 0;
  prevTempForD = filteredTempC;
}

static void startProfile()
{
  running = true;
  completed = false;
  runState = WARMUP;

  profileTime_s = 0.0f;
  lastLoopMs = millis();
  startRunMs = millis();

  windowStartMs = millis();
  lastTelemetryMs = 0;

  float t = readTempC();
  filteredTempC = t;
  lastRateTemp = t;
  lastRateMs = 0;
  filteredRateCps = 0.0f;

  rampedSetpoint = profile[0].startTempC;

  overshootHoldOff = false;

  controlPhase = CP_T0;
  resetPid();

  reflowPeakReached = false;
  reflowPeakHoldStartMs = 0;
}

static void stopProfile()
{
  running = false;
  completed = false;
  runState = IDLE;
  ssrWrite(false);
}

// ============================================================
// Arduino entry points
// ============================================================
void setup()
{
  pinMode(SSR_PIN, OUTPUT);
  ssrWrite(false);
  Serial.begin(115200);
}

void loop()
{
  // -------------------- Serial commands --------------------
  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("START")) startProfile();
    if (cmd.equalsIgnoreCase("ABORT")) stopProfile();
  }

  unsigned long now = millis();

  // -------------------- Read and filter temperature --------------------
  float t = readTempC();
  filteredTempC = filteredTempC + TEMP_FILTER_ALPHA * (t - filteredTempC);

  // Rate estimate
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

  // -------------------- Main control --------------------
  float setpoint = filteredTempC;
  const char *phaseLabel = "IDLE";
  bool heaterOn = false;

  if (running)
  {
    float dt_s = (now - lastLoopMs) / 1000.0f;
    if (dt_s < 0.0f) dt_s = 0.0f;
    if (dt_s > 1.5f) dt_s = 1.5f;
    lastLoopMs = now;

    if (runState == WARMUP)
    {
      // Warmup: just full on until we see temperature rising at a reasonable rate
      phaseLabel = "PREHEAT";
      setpoint = profile[0].startTempC;
      heaterOn = true;

      if (filteredRateCps >= PREHEAT_START_RATE_C_PER_S)
      {
        runState = RUNNING;
        profileTime_s = 0.0f;
        rampedSetpoint = profile[0].startTempC;
        overshootHoldOff = false;
        controlPhase = CP_T0;
        resetPid();

        reflowPeakReached = false;
        reflowPeakHoldStartMs = 0;

        startRunMs = now;
      }
    }
    else if (runState == RUNNING)
    {
      // Compute PT (slow only, never faster than real time)
      unsigned long ptMs = (unsigned long)(profileTime_s * 1000.0f);

      // Reflow boundary (end of REFLOW step = start of COOL step)
      const unsigned long endReflowMs = endOfStepMs(2); // step 0..2 inclusive

      // Clamp profile time at end of REFLOW until "reflow complete" is true
      // This prevents dropping into COOL early.
      bool allowEnterCool = false;

      // Determine nominal setpoint & label for current ptMs (or clamped)
      float nominal = nominalSetpointFromPT(ptMs, phaseLabel);

      // Slowdown factor based on being behind nominal
      float errorNominal = nominal - filteredTempC; // positive if behind
      float timeRate = 1.0f - TIME_SLOW_GAIN * errorNominal;
      if (timeRate < MIN_TIME_RATE) timeRate = MIN_TIME_RATE;
      if (timeRate > MAX_TIME_RATE) timeRate = MAX_TIME_RATE;

      // Tentative advance
      float nextPT_s = profileTime_s + dt_s * timeRate;
      unsigned long nextPT_ms = (unsigned long)(nextPT_s * 1000.0f);

      // Reflow completion qualification
      if (!reflowPeakReached && filteredTempC >= REFLOW_PEAK_TEMP_C)
      {
        reflowPeakReached = true;
        reflowPeakHoldStartMs = now;
      }

      bool coolingStarted = (filteredRateCps <= REFLOW_FALL_RATE_CPS);
      bool peakHeldLongEnough = reflowPeakReached && (now - reflowPeakHoldStartMs >= REFLOW_MIN_HOLD_MS);

      // Consider reflow complete when peak was reached AND (cooling started OR held for minimum time)
      bool reflowComplete = reflowPeakReached && (coolingStarted || peakHeldLongEnough);

      // Hard timeout to avoid getting stuck forever
      bool hardTimeout = (now - startRunMs) > (totalProfileMs() + HARD_EXTRA_MS);

      // Only allow PT to cross into COOL once reflow is complete (or timeout)
      if ((reflowComplete || hardTimeout) && nextPT_ms > endReflowMs)
        allowEnterCool = true;

      if (!allowEnterCool && nextPT_ms > endReflowMs)
      {
        // Hold at end of REFLOW
        nextPT_ms = endReflowMs;
        nextPT_s  = endReflowMs / 1000.0f;
      }

      profileTime_s = nextPT_s;
      ptMs = nextPT_ms;

      // If profile fully done (including COOL), finish.
      if (ptMs >= totalProfileMs())
      {
        running = false;
        completed = true;
        runState = DONE;
        ssrWrite(false);
      }
      else
      {
        // Recompute nominal setpoint after PT update
        nominal = nominalSetpointFromPT(ptMs, phaseLabel);

        // Ramp-limit setpoint (helps avoid command jumps)
        float maxUp = MAX_RAMP_UP_C_PER_S * dt_s;
        float maxDown = MAX_RAMP_DOWN_C_PER_S * dt_s;

        if (nominal > rampedSetpoint) rampedSetpoint = min(nominal, rampedSetpoint + maxUp);
        else                          rampedSetpoint = max(nominal, rampedSetpoint - maxDown);

        setpoint = rampedSetpoint;

        // Step timing info for fraction-in-step
        int stepIndex;
        unsigned long stepStartMs, stepEndMs;
        getStepInfo(ptMs, stepIndex, stepStartMs, stepEndMs);

        float inStep_s = (ptMs - stepStartMs) / 1000.0f;
        float stepDuration_s = max(1.0f, (float)profile[stepIndex].durationMs / 1000.0f);
        float frac = constrain(inStep_s / stepDuration_s, 0.0f, 1.0f);

        // Global overshoot guard
        float overshoot = filteredTempC - setpoint;
        if (!overshootHoldOff && overshoot >= OVERSHOOT_OFF_C) overshootHoldOff = true;
        if (overshootHoldOff && overshoot <= OVERSHOOT_ON_C)   overshootHoldOff = false;

        // If in COOL phase, force heater off and control phase = COOL
        if (strcmp(phaseLabel, "COOL") == 0)
        {
          controlPhase = CP_COOL;
          heaterOn = false;
          resetPid();
        }
        else
        {
          // -------------------- T0–T4 state machine transitions --------------------
          float err = setpoint - filteredTempC; // positive if behind

          // T0 enter: if we are far behind (boost) and not coasting
          if (controlPhase != CP_T2 && controlPhase != CP_COOL)
          {
            if (err >= T0_ENTER_ERR_C && strcmp(phaseLabel, "PREHEAT") == 0)
              controlPhase = CP_T0;
          }

          // Phase-specific logic
          if (strcmp(phaseLabel, "PREHEAT") == 0)
          {
            // T0 -> T1: once close to setpoint
            if (controlPhase == CP_T0 && err <= T0_EXIT_ERR_C)
            {
              controlPhase = CP_T1;
              resetPid();
            }

            // Enter T2 coast near end of PREHEAT to prevent overshoot into SOAK
            float stepRemaining_s = (stepEndMs - ptMs) / 1000.0f;
            if (controlPhase != CP_T2)
            {
              if (filteredTempC >= T2_PREHEAT_CUTOFF_TEMP_C || stepRemaining_s <= T2_PREHEAT_LAST_SEC)
              {
                controlPhase = CP_T2;
                resetPid();
              }
            }

            // If still in PREHEAT and not coasting, ensure we are at least T1 after T0
            if (controlPhase != CP_T2 && controlPhase != CP_T0)
            {
              // if we haven't explicitly set T1, keep tracking PID in preheat
              controlPhase = CP_T1;
            }
          }
          else if (strcmp(phaseLabel, "SOAK") == 0)
          {
            // If we arrive in SOAK while still T1, keep coasting briefly (T2) to avoid overshoot
            if (controlPhase == CP_T1)
            {
              controlPhase = CP_T2;
              resetPid();
            }

            // T2 -> T3: exit coast when slope flattens OR we fall behind setpoint
            if (controlPhase == CP_T2)
            {
              if (filteredRateCps <= T2_COAST_EXIT_RATE_CPS || err >= T2_COAST_RESUME_ERR_C)
              {
                controlPhase = CP_T3;
                resetPid();
              }
            }

            // T3 -> T4: pre-charge late in SOAK (fraction-based)
            if (controlPhase == CP_T3)
            {
              if (frac >= T4_SOAK_TRIGGER_FRACTION)
              {
                controlPhase = CP_T4;
                resetPid();
              }
            }

            // T4 -> T3: once close to setpoint, return to PID to avoid overshoot
            if (controlPhase == CP_T4)
            {
              if (err <= T4_EXIT_ERR_C)
              {
                controlPhase = CP_T3;
                resetPid();
              }
            }
          }
          else if (strcmp(phaseLabel, "REFLOW") == 0)
          {
            // In REFLOW, use PID (T3) unless we are lagging, then T4 boost.
            if (controlPhase == CP_T2)
            {
              // Coast doesn't really make sense here; go to PID
              controlPhase = CP_T3;
              resetPid();
            }

            if (controlPhase == CP_T3)
            {
              if (err >= T4_REFLOW_LAG_ERR_C)
              {
                controlPhase = CP_T4;
                resetPid();
              }
            }

            if (controlPhase == CP_T4)
            {
              // Return to PID when close to setpoint to avoid overshoot
              if (err <= T4_EXIT_ERR_C)
              {
                controlPhase = CP_T3;
                resetPid();
              }
            }

            // If we never used T3 yet, default to PID
            if (controlPhase != CP_T4)
              controlPhase = CP_T3;
          }
          else
          {
            // Unknown label, safest: PID
            controlPhase = CP_T3;
          }

          // -------------------- Heater mode selection --------------------
          HeaterMode heaterMode = HEATER_PID;

          if (overshootHoldOff)
          {
            heaterMode = HEATER_OFF;
          }
          else
          {
            switch (controlPhase)
            {
              case CP_T0: heaterMode = HEATER_FULL_ON; break;
              case CP_T1: heaterMode = HEATER_PID;     break;
              case CP_T2: heaterMode = HEATER_OFF;     break;
              case CP_T3: heaterMode = HEATER_PID;     break;
              case CP_T4: heaterMode = HEATER_FULL_ON; break;
              case CP_COOL: heaterMode = HEATER_OFF;   break;
            }
          }

          // -------------------- Apply heater output --------------------
          if (heaterMode == HEATER_FULL_ON)
          {
            heaterOn = true;
          }
          else if (heaterMode == HEATER_OFF)
          {
            heaterOn = false;
          }
          else // PID windowed
          {
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

  // -------------------- Output to SSR --------------------
  if (running && runState == RUNNING) ssrWrite(heaterOn);
  else if (running && runState == WARMUP) ssrWrite(true);
  else ssrWrite(false);

  // -------------------- Telemetry --------------------
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMs = now;

    // Find current phase label from current PT (or IDLE)
    const char *pl = "IDLE";
    if (running && runState == RUNNING)
    {
      unsigned long ptMs = (unsigned long)(profileTime_s * 1000.0f);
      (void)nominalSetpointFromPT(ptMs, pl);
    }
    else if (running && runState == WARMUP)
    {
      pl = "PREHEAT";
    }
    else if (completed)
    {
      pl = "DONE";
    }

    Serial.print("T:");
    Serial.print(filteredTempC, 2);

    Serial.print(";S:");
    Serial.print(setpoint, 2);

    Serial.print(";STATE:");
    if (running) Serial.print("RUNNING");
    else Serial.print(completed ? "DONE" : "IDLE");

    Serial.print(";PHASE:");
    Serial.print(pl);

    Serial.print(";MODE:");
    if (running) Serial.print(controlPhaseName(controlPhase));
    else Serial.print("-");

    Serial.print(";PT:");
    Serial.print(profileTime_s, 1);

    Serial.print(";R:");
    Serial.print(filteredRateCps, 2);

    Serial.println();
  }
}
