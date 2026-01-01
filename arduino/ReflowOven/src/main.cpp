#include <Arduino.h>
#include <math.h>

// ============================================================
// Reflow Oven Controller (T0–T4 Heater State Machine + TAL gating)
//
// Fixes included (per your latest screenshots/behavior):
//  1) PREHEAT T2 “time-trigger coast” bug fixed:
//     - T2 no longer traps you with heater OFF for the remainder of PREHEAT.
//     - T2 in PREHEAT has an explicit exit condition when you fall behind.
//     - T2 time trigger made later (default 35s).
//
//  2) SOAK entry coast improved:
//     - No forced T2 at start of SOAK unless overshoot risk is real.
//
//  3) T4 start in SOAK is time-based:
//     - Starts ~20s before SOAK ends (SOAK->REFLOW boundary).
//
//  4) TAL (Time Above Liquidus) tracking and gating:
//     - "Reflow complete" requires Peak + TAL + (cooling started or hold time).
//     - Profile time cannot advance into COOL until reflowComplete (or timeout).
//
// Serial commands (115200):
//   START
//   ABORT
//
// Telemetry (every 500ms):
//   T:<temp>;S:<setpoint>;STATE:<RUNNING/IDLE/DONE>;PHASE:<PREHEAT/SOAK/REFLOW/COOL>;
//   MODE:<T0/T1/T2/T3/T4/COOL>;PT:<sec>;R:<C/s>;LOCK:<0/1>;TAL:<sec>;PEAK:<C>
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
const float TEMP_CAL_OFFSET_C    = 10.0f;      // calibration offset (add to measured temp)

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

// Matches your UI:
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
const float TIME_SLOW_GAIN = 0.060f;
const float MIN_TIME_RATE  = 0.10f;
const float MAX_TIME_RATE  = 1.00f;

// -------------------- Global overshoot guard --------------------
const float OVERSHOOT_OFF_C = 3.0f;
const float OVERSHOOT_ON_C  = 1.0f;

// -------------------- T0–T4 tuning --------------------
// T0: full power boost until close to setpoint
const float T0_ENTER_ERR_C = 10.0f;
const float T0_EXIT_ERR_C  = 2.0f;

// T2: hard off near end of PREHEAT / early SOAK to prevent overshoot
// NOTE: Time trigger made later, and PREHEAT now has explicit exit from T2.
const float T2_PREHEAT_CUTOFF_TEMP_C   = 147.0f; // enter coast if we hit this temp in PREHEAT
const float T2_PREHEAT_LAST_SEC        = 35.0f;  // enter coast only in last N sec of PREHEAT
const float T2_COAST_EXIT_RATE_CPS     = 0.08f;  // exit coast when slope flattens (SOAK use)
const float T2_COAST_RESUME_ERR_C      = 3.0f;   // exit coast if we fall behind by this

// T4: full power pre-charge BEFORE final reflow push (timed burst)
const float T4_REFLOW_LAG_ERR_C        = 6.0f;     // in REFLOW, if behind by >= this => start T4
const unsigned long T4_PRECHARGE_MS    = 12000;    // committed full-blast duration
const float T4_SOAK_PRECHARGE_BEFORE_REFLOW_SEC = 20.0f; // start T4 this many sec before SOAK ends

// Post-T4 lock: keep a minimum PID duty until we see enough upward momentum
const float POST_T4_MIN_RISE_CPS       = 0.25f;
const float POST_T4_MIN_DUTY           = 0.35f;

// -------------------- Reflow completion (TAL gating) --------------------
const float TAL_LIQUIDUS_C             = 217.0f;    // SAC liquidus
const unsigned long TAL_REQUIRED_MS    = 40000;     // require 40s above liquidus (tune 30–90s)
const float PEAK_REQUIRED_C            = 225.0f;    // require at least this peak

const float REFLOW_FALL_RATE_CPS       = -0.05f;    // cooling started
const unsigned long REFLOW_MIN_HOLD_MS = 3000;      // optional peak hold alternative
const unsigned long HARD_EXTRA_MS      = 60000;     // absolute max extra run time beyond profile

// ============================================================
// State
// ============================================================
enum RunState { IDLE, WARMUP, RUNNING, DONE };
RunState runState = IDLE;

enum HeaterMode { HEATER_FULL_ON, HEATER_PID, HEATER_OFF };
enum ControlPhase { CP_T0, CP_T1, CP_T2, CP_T3, CP_T4, CP_COOL };

bool running   = false;
bool completed = false;

// Timing
unsigned long lastLoopMs = 0;
unsigned long windowStartMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long startRunMs = 0;

// “Profile time” PT (warped/slow only) in seconds
float profileTime_s = 0.0f;

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

// Control phase
ControlPhase controlPhase = CP_T0;

// T4 committed burst timer
unsigned long t4StartMs = 0;

// Post-T4 lock flag
bool postT4Lock = false;

// Reflow qualification metrics
float peakTempC = 0.0f;
unsigned long talMs = 0;
bool peakReached = false;
unsigned long peakHoldStartMs = 0;

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
  float tempC = convertAdcToTempC(analogRead(THERMISTOR_PIN)) + TEMP_CAL_OFFSET_C;
  if (tempC < -40.0f) tempC = -40.0f;
  if (tempC > 350.0f) tempC = 350.0f;
  return tempC;
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
static void getStepInfo(unsigned long ptMs, int &stepIndex, unsigned long &stepStartMs, unsigned long &stepEndMs)
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
      return;
    }
    acc = end;
  }

  stepIndex = PROFILE_COUNT - 1;
  stepStartMs = totalProfileMs() - profile[PROFILE_COUNT - 1].durationMs;
  stepEndMs = totalProfileMs();
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

  // anti-windup
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

static void resetPid()
{
  pidIntegral = 0.0f;
  lastPidMs = 0;
  prevTempForD = filteredTempC;
}

static void finishProfile()
{
  running = false;
  completed = true;
  runState = DONE;
  ssrWrite(false);

  t4StartMs = 0;
  postT4Lock = false;
  controlPhase = CP_COOL;
  resetPid();
}

static void startProfile()
{
  running = true;
  completed = false;
  runState = WARMUP;

  unsigned long now = millis();
  lastLoopMs = now;
  startRunMs = now;

  windowStartMs = now;
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

  // Reflow metrics
  peakTempC = filteredTempC;
  talMs = 0;
  peakReached = false;
  peakHoldStartMs = 0;

  // T4
  t4StartMs = 0;
  postT4Lock = false;

  profileTime_s = 0.0f;
}

static void stopProfile()
{
  running = false;
  completed = false;
  runState = IDLE;
  ssrWrite(false);

  t4StartMs = 0;
  postT4Lock = false;
  controlPhase = CP_T0;
  resetPid();
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
    unsigned long dt_ms = now - lastLoopMs;
    if (dt_ms > 1500) dt_ms = 1500;
    float dt_s = dt_ms / 1000.0f;
    lastLoopMs = now;

    if (runState == WARMUP)
    {
      // Warmup: full on until temperature is clearly rising
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

        // Reset reflow metrics at start of controlled run
        peakTempC = filteredTempC;
        talMs = 0;
        peakReached = false;
        peakHoldStartMs = 0;

        startRunMs = now;
        t4StartMs = 0;
        postT4Lock = false;
      }
    }
    else if (runState == RUNNING)
    {
      unsigned long ptMs = (unsigned long)(profileTime_s * 1000.0f);

      // Boundary: end of REFLOW step = start of COOL step
      const unsigned long endReflowMs = endOfStepMs(2);

      // Nominal setpoint and label at current PT
      float nominal = nominalSetpointFromPT(ptMs, phaseLabel);

      // Slow profile time if behind
      float errorNominal = nominal - filteredTempC;
      float timeRate = 1.0f - TIME_SLOW_GAIN * errorNominal;
      if (timeRate < MIN_TIME_RATE) timeRate = MIN_TIME_RATE;
      if (timeRate > MAX_TIME_RATE) timeRate = MAX_TIME_RATE;

      float nextPT_s = profileTime_s + dt_s * timeRate;
      unsigned long nextPT_ms = (unsigned long)(nextPT_s * 1000.0f);

      // Step info at current PT (for TAL accumulation, transitions)
      int stepIndex;
      unsigned long stepStartMs, stepEndMs;
      getStepInfo(ptMs, stepIndex, stepStartMs, stepEndMs);

      // Track peak continuously
      if (filteredTempC > peakTempC) peakTempC = filteredTempC;

      // TAL accumulation only during REFLOW step (index 2)
      if (stepIndex == 2 && filteredTempC >= TAL_LIQUIDUS_C)
        talMs += dt_ms;

      // Peak reached for hold timing
      if (!peakReached && peakTempC >= PEAK_REQUIRED_C)
      {
        peakReached = true;
        peakHoldStartMs = now;
      }

      bool talMet  = (talMs >= TAL_REQUIRED_MS);
      bool peakMet = (peakTempC >= PEAK_REQUIRED_C);

      bool coolingStarted = (filteredRateCps <= REFLOW_FALL_RATE_CPS);
      bool peakHeldLongEnough = peakReached && (now - peakHoldStartMs >= REFLOW_MIN_HOLD_MS);

      bool reflowComplete = peakMet && talMet && (coolingStarted || peakHeldLongEnough);

      bool hardTimeout = (now - startRunMs) > (totalProfileMs() + HARD_EXTRA_MS);

      // Prevent crossing into COOL until reflowComplete (or timeout)
      if (!reflowComplete && !hardTimeout && nextPT_ms > endReflowMs)
      {
        nextPT_ms = endReflowMs;
        nextPT_s  = endReflowMs / 1000.0f;
      }

      profileTime_s = nextPT_s;
      ptMs = nextPT_ms;

      if (ptMs >= totalProfileMs())
      {
        finishProfile();
      }
      else
      {
        // Recompute nominal after PT update
        nominal = nominalSetpointFromPT(ptMs, phaseLabel);

        // Ramp-limit setpoint
        float maxUp = MAX_RAMP_UP_C_PER_S * dt_s;
        float maxDown = MAX_RAMP_DOWN_C_PER_S * dt_s;

        if (nominal > rampedSetpoint) rampedSetpoint = min(nominal, rampedSetpoint + maxUp);
        else                          rampedSetpoint = max(nominal, rampedSetpoint - maxDown);

        setpoint = rampedSetpoint;

        // Updated step info after PT update
        getStepInfo(ptMs, stepIndex, stepStartMs, stepEndMs);

        float inStep_s = (ptMs - stepStartMs) / 1000.0f;
        float stepDuration_s = max(1.0f, (float)profile[stepIndex].durationMs / 1000.0f);
        float frac = constrain(inStep_s / stepDuration_s, 0.0f, 1.0f);

        float stepRemaining_s = (stepEndMs - ptMs) / 1000.0f;

        // Overshoot guard
        float overshoot = filteredTempC - setpoint;
        if (!overshootHoldOff && overshoot >= OVERSHOOT_OFF_C) overshootHoldOff = true;
        if (overshootHoldOff && overshoot <= OVERSHOOT_ON_C)   overshootHoldOff = false;

        // ============================================================
        // COOL: heater forced OFF, PID reset, clear special modes
        // ============================================================
        if (strcmp(phaseLabel, "COOL") == 0)
        {
          controlPhase = CP_COOL;
          heaterOn = false;

          overshootHoldOff = false;
          t4StartMs = 0;
          postT4Lock = false;
          resetPid();
        }
        else
        {
          float err = setpoint - filteredTempC;

          // Clear post-T4 lock once upward momentum is confirmed
          if (postT4Lock && filteredRateCps >= POST_T4_MIN_RISE_CPS)
            postT4Lock = false;

          bool t4CommittedActive =
            (controlPhase == CP_T4 && t4StartMs != 0 && (now - t4StartMs) < T4_PRECHARGE_MS);

          if (!t4CommittedActive)
          {
            // -------------------- PREHEAT --------------------
            if (strcmp(phaseLabel, "PREHEAT") == 0)
            {
              // Enter T0 if far behind and not coasting
              if (controlPhase != CP_T2 && err >= T0_ENTER_ERR_C)
                controlPhase = CP_T0;

              // T0 -> T1 when close
              if (controlPhase == CP_T0 && err <= T0_EXIT_ERR_C)
              {
                controlPhase = CP_T1;
                resetPid();
              }

              // Enter T2 coast only near the end of PREHEAT or near cutoff temp
              if (controlPhase != CP_T2)
              {
                if (filteredTempC >= T2_PREHEAT_CUTOFF_TEMP_C || stepRemaining_s <= T2_PREHEAT_LAST_SEC)
                {
                  controlPhase = CP_T2;
                  resetPid();
                }
              }

              // *** BUG FIX: Exit PREHEAT T2 if we fall behind ***
              if (controlPhase == CP_T2)
              {
                if (err >= T2_COAST_RESUME_ERR_C)
                {
                  controlPhase = CP_T1;   // resume PID tracking in PREHEAT
                  resetPid();
                }
              }

              // If not coasting or boosting, use PID
              if (controlPhase != CP_T2 && controlPhase != CP_T0)
                controlPhase = CP_T1;
            }

            // -------------------- SOAK --------------------
            else if (strcmp(phaseLabel, "SOAK") == 0)
            {
              // If entering SOAK while in T1, only coast if overshoot risk is real
              if (controlPhase == CP_T1)
              {
                bool overshootRisk = (filteredTempC > setpoint + 0.5f) ||
                                     (filteredRateCps > 0.35f && (setpoint - filteredTempC) < 1.5f);

                if (overshootRisk)
                {
                  controlPhase = CP_T2;
                  resetPid();
                }
                else
                {
                  controlPhase = CP_T3;
                  resetPid();
                }
              }

              // Exit coast when slope flattens OR we fall behind
              if (controlPhase == CP_T2)
              {
                if (filteredRateCps <= T2_COAST_EXIT_RATE_CPS || err >= T2_COAST_RESUME_ERR_C)
                {
                  controlPhase = CP_T3;
                  resetPid();
                }
              }

              // Start T4 near end of SOAK (time-based, ~20s before REFLOW)
              if (controlPhase == CP_T3 && stepRemaining_s <= T4_SOAK_PRECHARGE_BEFORE_REFLOW_SEC)
              {
                controlPhase = CP_T4;
                t4StartMs = now;
                resetPid();
              }

              if (controlPhase != CP_T2 && controlPhase != CP_T4)
                controlPhase = CP_T3;
            }

            // -------------------- REFLOW --------------------
            else if (strcmp(phaseLabel, "REFLOW") == 0)
            {
              // Ensure we are not stuck in preheat/soak-only modes
              if (controlPhase == CP_T2 || controlPhase == CP_T1 || controlPhase == CP_T0)
              {
                controlPhase = CP_T3;
                resetPid();
              }

              // Start T4 if lagging
              if (controlPhase == CP_T3 && err >= T4_REFLOW_LAG_ERR_C)
              {
                controlPhase = CP_T4;
                t4StartMs = now;
                resetPid();
              }

              if (controlPhase != CP_T4)
                controlPhase = CP_T3;
            }

            // -------------------- Unknown phase safety --------------------
            else
            {
              if (controlPhase != CP_T4)
              {
                controlPhase = CP_T3;
                resetPid();
              }
            }
          }

          // T4: exit only when committed timer finishes, then enable post-T4 lock
          if (controlPhase == CP_T4 && t4StartMs != 0)
          {
            if ((now - t4StartMs) >= T4_PRECHARGE_MS)
            {
              controlPhase = CP_T3;
              t4StartMs = 0;
              postT4Lock = true;
              resetPid();
            }
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
              case CP_T0:   heaterMode = HEATER_FULL_ON; break;
              case CP_T1:   heaterMode = HEATER_PID;     break;
              case CP_T2:   heaterMode = HEATER_OFF;     break;
              case CP_T3:   heaterMode = HEATER_PID;     break;
              case CP_T4:   heaterMode = HEATER_FULL_ON; break;
              case CP_COOL: heaterMode = HEATER_OFF;     break;
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
          else
          {
            float duty = computePidDuty(setpoint, filteredTempC);

            if (postT4Lock && duty < POST_T4_MIN_DUTY)
              duty = POST_T4_MIN_DUTY;

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

    Serial.print(";LOCK:");
    Serial.print(postT4Lock ? 1 : 0);

    Serial.print(";TAL:");
    Serial.print((float)talMs / 1000.0f, 1);

    Serial.print(";PEAK:");
    Serial.print(peakTempC, 1);

    Serial.println();
  }
}
