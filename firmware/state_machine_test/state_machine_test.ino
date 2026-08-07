// =============================================================================
// HULC Motion Shirt — STATE MACHINE TEST BUILD
// State transitions ONLY. No Bluetooth. No flash storage.
//
// PURPOSE:
//   A stripped-down copy of firmware.ino for verifying the state machine
//   behaviour in isolation. The IDLE / STATIC_POSTURE / ACTIVE_RECORDING
//   transition logic and all its timers are kept byte-for-byte identical to
//   the production firmware, so what you observe here is what the real device
//   does. Everything NOT related to state changes has been removed:
//     • BLE / ArduinoBLE           — no advertising, no GATT, no control cmds
//     • QSPI flash / nrfx_qspi      — samples are PRINTED, not stored
//     • Time sync, offload          — gone with BLE
//     • Watermark throttle          — flash-fill behaviour, N/A without flash
//     • "flash full → IDLE"         — flash behaviour, N/A without flash
//
// Hardware : Seeed XIAO nRF52840 + SparkFun BNO086 (I2C)
// Libraries: SparkFun BNO08x Arduino Library  (ArduinoBLE NOT needed)
//
// ── Two deliberate differences from firmware.ino, both for testability ──
//   1. devSleep is DISABLED. The production IDLE state deep-sleeps the BNO hub
//      (~7mA) but the hub then self-reboots every ~6.6s, which clutters the
//      IDLE logs with reset/re-arm handling. This test keeps the hub AWAKE in
//      IDLE so IDLE→motion→ACTIVE is deterministic and easy to read. Motion
//      detection is unchanged — the Stability Detector (0x1C) still arms and
//      still emits EXITED on motion; only the power-saving sleep is removed.
//   2. IDLE detector interval is 200ms (production: 10s). At 10s, motion onset
//      can feel laggy on the bench; 200ms makes it snappy. Purely a test knob.
//
// ── How to test each transition (IMU-driven) ──
//   IDLE → ACTIVE_RECORDING     : pick the sensor up / move it.
//   ACTIVE → STATIC_POSTURE     : hold it still for 10s (NOT_MOTION_TO_STATIC),
//                                 OR lay it flat ("on table") for 3s.
//   STATIC → ACTIVE_RECORDING   : move it again.
//   STATIC → IDLE               : leave it still for 60s (NOT_MOTION_TO_IDLE),
//                                 OR lay it flat for 5s (ON_TABLE fast path).
//
// LED indicator (XIAO onboard, active-low):
//   IDLE = off    STATIC = blue    ACTIVE = red
// =============================================================================


// =============================================================================
// SECTION 1 — Includes
// =============================================================================

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>


// =============================================================================
// SECTION 2 — Pin Assignments
// =============================================================================

#define BNO_INT_PIN     2
#define BNO_RST_PIN     3
#define BNO08X_I2C_ADDR 0x4B

#define PIN_LED_BLUE    LED_BLUE
#define PIN_LED_RED     LED_RED


// =============================================================================
// SECTION 3 — Stability Detector (0x1C)
// =============================================================================
// IDLE arms the Stability Detector as its motion wake source. The detector
// reports ENTERED (stability gained) / EXITED (stability broken = motion).

#ifndef SH2_STABILITY_DETECTOR
#define SH2_STABILITY_DETECTOR 0x1C
#endif
#ifndef SH2_SIGNIFICANT_MOTION
#define SH2_SIGNIFICANT_MOTION 0x12
#endif
#define DETECTOR_EXITED   2
#define DETECTOR_ENTERED  1

// ── IDLE wake-source experiment selector ───────────────────────────────────
// The bench log showed the BNO self-reboots every ~6.4s in IDLE even with the
// hub AWAKE (devSleep off) — so the reboot is NOT a devSleep artifact. Use this
// switch to localize the cause. Recompile with each value and watch the
// "held Nms since arm" figure printed on every IDLE reset:
//   DETECTOR  — arm Stability Detector 0x1C (baseline; the ~6.4s case)
//   NONE      — arm NOTHING. If the reboot STOPS, arming 0x1C is the cause.
//               If it PERSISTS, the reboot is systemic (I2C/wiring/hub), not
//               the detector. This is the decisive experiment.
//   SIGMOTION — arm Significant Motion 0x12. Production notes claim ~1.17s
//               under devSleep; if it's also ~1.17s here, the reboot period is
//               set by the armed sensor type and devSleep is irrelevant.
//   ACCEL     — arm a normal STREAMING sensor (accelerometer) and detect motion
//               host-side by |a| deviation from gravity. Bench results so far:
//               NONE/SIGMOTION reset ~0.68s, DETECTOR ~6.4s, all reason=2
//               "Internal System Reset" — a fixed internal timer, independent of
//               devSleep AND report interval. Open question this tests: does a
//               continuously-streaming sensor avoid the internal reset entirely?
//               If it runs reset-free, it's a viable IDLE wake with host-side
//               motion thresholding — no reboot to design around.
#define IDLE_WAKE_DETECTOR   0
#define IDLE_WAKE_NONE       1
#define IDLE_WAKE_SIGMOTION  2
#define IDLE_WAKE_ACCEL      3

#define IDLE_WAKE_SOURCE     IDLE_WAKE_DETECTOR   // poll experiment needs detector regime

// ── IDLE servicing mode (poll vs sleep) experiment ──────────────────────────
// Tests whether the ~6.5s detector reboot is a MISSED-READ artifact or a real
// independent reset. In WFE mode the host sleeps between INT pulses (production
// behaviour); an edge-triggered ISR can miss a level-asserted INT and sleep
// through a pending report. In POLL mode the host never sleeps — loop() calls
// getSensorEvent() continuously, so a pending report can NEVER be slept through.
//   ~6.5s reboot PERSISTS under POLL -> real independent hub reset (not our fault)
//   ~6.5s reboot DISAPPEARS/stretches -> it was a missed-read artifact (fixable!)
// Keep IDLE_WAKE_SOURCE = DETECTOR and a short interval (<=500ms) so we're in
// the ~6.5s regime where the ceiling is visible.
#define IDLE_SERVICE_WFE     0
#define IDLE_SERVICE_POLL    1

#define IDLE_SERVICE_MODE    IDLE_SERVICE_POLL


// =============================================================================
// SECTION 4 — Sampling Rates
// =============================================================================

#define DEFAULT_ACTIVE_HZ           10
#define STATIC_SAMPLE_INTERVAL_MS   5000
#define BNO_RV_INTERVAL_MS          65
#define ACTIVE_STABILITY_MS         500

// TEST-ONLY: 200ms so motion is caught quickly on the bench.
// Production firmware.ino uses 10s here for idle power (see header note 2).
#define IDLE_DETECTOR_INTERVAL_US   200000UL     // 200ms

// IDLE_WAKE_ACCEL tuning: stream the accelerometer and call it "motion" when
// |a| deviates from 1g by more than the threshold. Interval is in MILLISECONDS
// (SparkFun enableAccelerometer takes ms, unlike enableReport which takes us).
#define IDLE_ACCEL_INTERVAL_MS      200          // 5Hz — enough for motion onset
#define IDLE_ACCEL_MOTION_THRESH    2.0f         // m/s^2 deviation from gravity

// ── Detector interval sweep (automatic) ─────────────────────────────────────
// When IDLE_DETECTOR_SWEEP is 1 AND IDLE_WAKE_SOURCE is DETECTOR, IDLE ignores
// IDLE_DETECTOR_INTERVAL_US and instead auto-cycles through idleSweepIntervals_us
// (defined in Section 10), re-arming with the next interval after
// IDLE_SWEEP_RESETS_PER_STEP genuine reboots. Each reboot prints its interval +
// held time, so ONE flash yields a full table showing whether the reboot period
// tracks the report interval. Resets shorter than IDLE_SWEEP_SETTLE_MS are
// treated as post-arm settle noise and not counted.
#define IDLE_DETECTOR_SWEEP         0   // sweep complete — results recorded below
#define IDLE_SWEEP_RESETS_PER_STEP  3
#define IDLE_SWEEP_SETTLE_MS        300

// ── SWEEP RESULT (bench, devSleep OFF, hub AWAKE) ───────────────────────────
//   interval   held       reports-before-reboot
//   50 ms      ~6480 ms    91
//   200 ms     ~6420 ms    30
//   500 ms     ~6484 ms    13
//   800 ms     ~930 ms      0
//   1 s        ~930 ms      0
//   5 s        ~930 ms      0
// CONCLUSION: the detector reboot (reason=2) is a RACE. The bare hub reboots at
// a ~930 ms baseline; if a detector report is serviced before then, the hold
// extends to a ~6.5 s CEILING — and that's a hard cap (50 ms feeds 91 reports
// and still tops out at ~6.5 s). The first report lands ~one interval after
// arming, so intervals <=500 ms beat the baseline (long hold) while >=800 ms
// miss it entirely (0 reports, ~930 ms reboot). Net: ~6.5 s is the best hold
// the detector allows; NO interval tuning eliminates the reboot. Removing it
// outright needs a non-detector (streaming) wake source — see IDLE_WAKE_ACCEL.
// CAVEAT: this is the AWAKE regime. devSleep runs a different governing timer
// (production saw ~6.6 s at a 10 s interval), so do NOT port these interval
// numbers to the devSleep path.


// =============================================================================
// SECTION 5 — Timeouts (ms)  — identical to firmware.ino
// =============================================================================

#define NOT_MOTION_TO_STATIC_MS     10000
#define ON_TABLE_DEBOUNCE_ACTIVE_MS 3000
#define NOT_MOTION_TO_IDLE_MS       60000
#define ON_TABLE_FAST_PATH_MS       5000


// =============================================================================
// SECTION 6 — BNO086 Stability Classifier Values
// =============================================================================

#define STABILITY_ON_TABLE    0
#define STABILITY_STATIONARY  1
#define STABILITY_STABLE      2
#define STABILITY_MOTION      3


// =============================================================================
// SECTION 7 — Watchdog  — identical to firmware.ino
// =============================================================================

#define WATCHDOG_MULTIPLIER          5
#define WATCHDOG_MAX_RESETS          3
#define RUNNING_WATCHDOG_MS          (ACTIVE_STABILITY_MS * WATCHDOG_MULTIPLIER)


// =============================================================================
// SECTION 8 — Serial Debug Logging
// =============================================================================

#define DEBUG_SERIAL 1

#if DEBUG_SERIAL
  #define LOG(msg)       Serial.println(msg)
  #define LOGF(fmt, ...) { char _buf[128]; snprintf(_buf, sizeof(_buf), \
                           "[%lums] " fmt, millis(), ##__VA_ARGS__); \
                           Serial.println(_buf); }
#else
  #define LOG(msg)
  #define LOGF(fmt, ...)
#endif


// =============================================================================
// SECTION 9 — State Machine Types
// =============================================================================

typedef enum {
  STATE_IDLE,
  STATE_STATIC_POSTURE,
  STATE_ACTIVE_RECORDING
} SystemState;

const char* stateName(SystemState s) {
  switch (s) {
    case STATE_IDLE:             return "IDLE";
    case STATE_STATIC_POSTURE:   return "STATIC_POSTURE";
    case STATE_ACTIVE_RECORDING: return "ACTIVE_RECORDING";
    default:                     return "UNKNOWN";
  }
}

const char* stabilityName(uint8_t s) {
  switch (s) {
    case STABILITY_ON_TABLE:   return "ON_TABLE";
    case STABILITY_STATIONARY: return "STATIONARY";
    case STABILITY_STABLE:     return "STABLE";
    case STABILITY_MOTION:     return "MOTION";
    default:                   return "UNKNOWN(motion assumed)";
  }
}


// =============================================================================
// SECTION 10 — Globals
// =============================================================================

// ── IMU ──
BNO08x       imu;

// ── State Machine ──
SystemState  currentState        = STATE_IDLE;
SystemState  pendingState        = STATE_IDLE;

uint8_t      activeHz            = DEFAULT_ACTIVE_HZ;

// ── Sample counter (replaces flash write pointer for visibility) ──
uint32_t     sampleCount         = 0;

// ── Timers ──
uint32_t     lastMotionTime      = 0;
uint32_t     onTableStartTime    = 0;
uint32_t     lastActiveSample    = 0;
uint32_t     lastStaticSample    = 0;
uint8_t      lastLoggedStability = 255;

// ── Watchdog ──
uint32_t     lastStabilityEvent  = 0;
uint8_t      consecutiveResets   = 0;

// ── BNO config guard ──
bool         bnoInRunningMode    = false;

// ── IDLE reset instrumentation ──
// millis() captured right after each IDLE arm, so handleIdle() can report how
// long the hub ran before self-rebooting (the precise per-config reboot period).
uint32_t     idleArmedMs         = 0;

// ── Detector interval sweep state (see Section 4) ──
// Extra points (500ms, 800ms) bracket the knee seen between 200ms and 1s.
const uint32_t idleSweepIntervals_us[] = {
  50000UL, 200000UL, 500000UL, 800000UL, 1000000UL, 5000000UL
};
const uint8_t  idleSweepCount =
  sizeof(idleSweepIntervals_us) / sizeof(idleSweepIntervals_us[0]);
uint8_t        sweepIdx                = 0;   // which interval is armed now
uint8_t        sweepResetCount         = 0;   // genuine reboots at this interval

// Detector reports seen since the last arm. Tests the "first report must beat
// the baseline reboot" model: a reboot with reports==0 means no heartbeat
// arrived before the reset (the long-hold regime was never entered).
uint16_t       reportsSinceArm         = 0;

volatile bool imuDataReady = false;


// =============================================================================
// SECTION 11 — ISR
// =============================================================================

void onBNOInterrupt() {
  imuDataReady = true;
}


// =============================================================================
// SECTION 12 — Utility Helpers
// =============================================================================

bool isMotion(uint8_t s) {
  return s >= STABILITY_MOTION;
}

// LED state indicator — off=IDLE, blue=STATIC, red=ACTIVE (active-low).
void setStateLED(SystemState s) {
  digitalWrite(PIN_LED_RED,  HIGH);   // both off
  digitalWrite(PIN_LED_BLUE, HIGH);
  switch (s) {
    case STATE_IDLE:                                              break;
    case STATE_STATIC_POSTURE:   digitalWrite(PIN_LED_BLUE, LOW); break;
    case STATE_ACTIVE_RECORDING: digitalWrite(PIN_LED_RED,  LOW); break;
  }
}

// Wait for the next BNO report. Behaviour depends on IDLE_SERVICE_MODE:
//
// POLL: never sleep — return immediately. The caller's getSensorEvent() loop
//   then reads whatever is pending every loop() pass, so a level-asserted INT
//   can never be slept through. Guarantees no missed reads (the experiment).
//
// WFE (production behaviour): sleep between INT pulses with the same missed-edge
//   guard as firmware.ino — the INT line is LEVEL-meaningful (LOW = data
//   waiting) but our ISR is edge-triggered, so never sleep while INT is already
//   LOW, and treat LOW as a wake. (No BLE branch here — no central to keep alive.)
void waitForIMUData() {
#if IDLE_SERVICE_MODE == IDLE_SERVICE_POLL
  imuDataReady = false;
  return;                           // tight poll — caller drains getSensorEvent()
#else
  if (digitalRead(BNO_INT_PIN) == LOW) {
    imuDataReady = false;
    return;                         // data already pending — go read it now
  }
  __SEV();
  __WFE();
  while (!imuDataReady && digitalRead(BNO_INT_PIN) == HIGH) {
    __WFE();
  }
  imuDataReady = false;
#endif
}


// =============================================================================
// SECTION 13 — BNO086 Mode Switching
// =============================================================================

// Arms the IDLE motion wake source (selected by IDLE_WAKE_SOURCE).
// NOTE: no devSleep in this test build — hub stays awake (see header note 1),
// so this is just a plain enableReport(), no sh2_setSensorConfig() dance.
void enableIdleReports() {
#if   IDLE_WAKE_SOURCE == IDLE_WAKE_NONE
  // Arm nothing — baseline to test whether the ~6.4s reboot is systemic.
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_SIGMOTION
  imu.enableReport(SH2_SIGNIFICANT_MOTION, IDLE_DETECTOR_INTERVAL_US);
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_ACCEL
  imu.enableAccelerometer(IDLE_ACCEL_INTERVAL_MS);   // streaming (ms interval)
#else
  uint32_t interval_us = IDLE_DETECTOR_INTERVAL_US;
  #if IDLE_DETECTOR_SWEEP
    interval_us = idleSweepIntervals_us[sweepIdx];   // auto-sweep overrides
  #endif
  imu.enableReport(SH2_STABILITY_DETECTOR, interval_us);
#endif
  idleArmedMs     = millis();
  reportsSinceArm = 0;
}

void configureBNO_Idle() {
  LOGF("BNO: soft reset -> IDLE mode (Stability Detector 0x1C wake, hub AWAKE)");
  imu.softReset();
  delay(150);

  enableIdleReports();
  bnoInRunningMode = false;

  imu.wasReset();

  LOGF("BNO: IDLE Stability Detector (0x1C) armed as wake source");
}

void configureBNO_Running() {
  if (bnoInRunningMode) {
    LOGF("BNO: already in running mode — skipping reconfiguration");
    return;
  }

  LOGF("BNO: configuring RUNNING mode (RV @ %dms, Classifier @ %dms)",
       BNO_RV_INTERVAL_MS, ACTIVE_STABILITY_MS);

  imu.enableRotationVector(BNO_RV_INTERVAL_MS);
  imu.enableStabilityClassifier(ACTIVE_STABILITY_MS);
  bnoInRunningMode = true;
}


// =============================================================================
// SECTION 14 — State Transitions
// =============================================================================

void requestTransition(SystemState newState) {
  if (newState != currentState) {
    pendingState = newState;
  }
}

void applyPendingTransition() {
  if (pendingState == currentState) return;

  LOGF("STATE: %s -> %s", stateName(currentState), stateName(pendingState));

  currentState = pendingState;
  setStateLED(currentState);

  switch (currentState) {
    case STATE_IDLE:
      configureBNO_Idle();
      break;
    case STATE_STATIC_POSTURE:
    case STATE_ACTIVE_RECORDING:
      configureBNO_Running();
      break;
  }

  lastStabilityEvent = millis();
  consecutiveResets  = 0;
}


// =============================================================================
// SECTION 15 — Watchdog  — identical logic to firmware.ino
// =============================================================================

void checkWatchdog() {
  if (currentState == STATE_IDLE) return;

  uint32_t elapsed = millis() - lastStabilityEvent;

  if (elapsed <= RUNNING_WATCHDOG_MS) return;

  consecutiveResets++;
  LOGF("WATCHDOG: no stability event in %lums — resetting BNO (attempt %d/%d)",
       elapsed, consecutiveResets, WATCHDOG_MAX_RESETS);

  if (consecutiveResets >= WATCHDOG_MAX_RESETS) {
    LOGF("WATCHDOG: %d consecutive resets with no recovery -> forcing IDLE",
         WATCHDOG_MAX_RESETS);
    pendingState = STATE_IDLE;
    return;
  }

  imu.softReset();
  delay(150);
  bnoInRunningMode = false;
  configureBNO_Running();

  lastStabilityEvent = millis();
}


// =============================================================================
// SECTION 16 — Sample "writer" (PRINT-ONLY — no flash)
//
// In firmware.ino this packs a 20-byte record and appends it to QSPI flash,
// plus watermark throttling and flash-full handling. None of that is state
// behaviour, so here it just prints the quaternion and counts samples. The
// call sites and their rate-gating are unchanged, so sample TIMING still
// matches the real firmware.
// =============================================================================

void writeQuaternionSample(float qi, float qj, float qk, float qr) {
  sampleCount++;
  LOGF("SAMPLE #%lu [%s]: w=%.4f x=%.4f y=%.4f z=%.4f",
       sampleCount, stateName(currentState), qr, qi, qj, qk);
}


// =============================================================================
// SECTION 17 — Stability Change Logger
// =============================================================================

void logStabilityIfChanged(uint8_t stability) {
  if (stability != lastLoggedStability) {
    LOGF("STABILITY: %s (%d)", stabilityName(stability), stability);
    lastLoggedStability = stability;
  }
}


// =============================================================================
// SECTION 18 — State: IDLE
//
// Simplified vs firmware.ino: no devSleep, so no reboot/re-arm handling. Just
// wait for the detector, and on EXITED (motion) go to ACTIVE_RECORDING.
// =============================================================================

void handleIdle() {
  if (imu.wasReset()) {
    // How long the hub ran before rebooting — the precise per-config reboot
    // period. Compare this figure across IDLE_WAKE_SOURCE values / intervals.
    uint32_t heldFor = millis() - idleArmedMs;

#if IDLE_WAKE_SOURCE == IDLE_WAKE_DETECTOR && IDLE_DETECTOR_SWEEP
    uint32_t curIv = idleSweepIntervals_us[sweepIdx];
    if (heldFor < IDLE_SWEEP_SETTLE_MS) {
      LOGF("SWEEP: interval=%luus  held=%lums  reports=%u  — SETTLE, not counted",
           curIv, heldFor, reportsSinceArm);
    } else {
      sweepResetCount++;
      LOGF("SWEEP: interval=%luus  held=%lums  reports=%u  (%u/%u)",
           curIv, heldFor, reportsSinceArm, sweepResetCount, IDLE_SWEEP_RESETS_PER_STEP);
      if (sweepResetCount >= IDLE_SWEEP_RESETS_PER_STEP) {
        sweepResetCount = 0;
        sweepIdx = (sweepIdx + 1) % idleSweepCount;
        LOGF("SWEEP: --> advancing to interval=%luus", idleSweepIntervals_us[sweepIdx]);
      }
    }
#else
    LOGF("IDLE: BNO reset — held %lums  reports=%u  (reason=%u) — re-arming",
         heldFor, reportsSinceArm, (unsigned)imu.getResetReason());
#endif

    bnoInRunningMode = false;
    enableIdleReports();   // re-arms with the (possibly advanced) sweep interval
    imu.wasReset();
    imuDataReady = false;
    return;   // don't sleep this pass — loop again to service post-reset events
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id = imu.getSensorEventID();

    // IDLE is near-silent, so log every raw report id for on-bench visibility.
    // If nothing prints here between resets, the wake sensor emits nothing at
    // rest and the reboot is the only IDLE activity.
    LOGF("IDLE: BNO event id=0x%02X", id);

#if IDLE_WAKE_SOURCE == IDLE_WAKE_SIGMOTION
    if (id == SH2_SIGNIFICANT_MOTION) {
      LOGF("SIGMOTION: fired — moving -> ACTIVE_RECORDING");
      activeHz         = DEFAULT_ACTIVE_HZ;
      lastMotionTime   = millis();
      onTableStartTime = 0;
      lastActiveSample = 0;
      requestTransition(STATE_ACTIVE_RECORDING);
      return;
    }
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_ACCEL
    if (id == SENSOR_REPORTID_ACCELEROMETER) {
      float ax = imu.getAccelX(), ay = imu.getAccelY(), az = imu.getAccelZ();
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      if (fabsf(mag - 9.81f) > IDLE_ACCEL_MOTION_THRESH) {
        LOGF("ACCEL: |a|=%d.%02d m/s^2 (dev>%d) — moving -> ACTIVE_RECORDING",
             (int)mag, (int)(mag * 100) % 100, (int)IDLE_ACCEL_MOTION_THRESH);
        activeHz         = DEFAULT_ACTIVE_HZ;
        lastMotionTime   = millis();
        onTableStartTime = 0;
        lastActiveSample = 0;
        requestTransition(STATE_ACTIVE_RECORDING);
        return;
      }
    }
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_DETECTOR
    if (id == SH2_STABILITY_DETECTOR) {
      reportsSinceArm++;
      lastStabilityEvent = millis();
      consecutiveResets  = 0;

      // EXITED = stability broken = motion started → wake into ACTIVE.
      // (ENTERED is ignored.)
      uint8_t val = imu.getStabilityClassifier();
      if (val == DETECTOR_EXITED) {
        LOGF("DETECTOR: EXITED — moving -> ACTIVE_RECORDING");
        activeHz         = DEFAULT_ACTIVE_HZ;
        lastMotionTime   = millis();
        onTableStartTime = 0;
        lastActiveSample = 0;
        requestTransition(STATE_ACTIVE_RECORDING);
        return;
      }
    }
#endif
    // IDLE_WAKE_NONE: nothing armed — no motion wake; reset the board to exit
    // IDLE. This mode exists only to observe the reboot cadence in isolation.
  }
}


// =============================================================================
// SECTION 19 — State: STATIC_POSTURE  — identical logic to firmware.ino
// =============================================================================

void handleStaticPosture() {
  if (imu.wasReset()) {
    LOGF("IMU: reset in STATIC_POSTURE — reason: %u — reconfiguring", (unsigned)imu.getResetReason());
    bnoInRunningMode = false;
    configureBNO_Running();
    lastStabilityEvent = millis();
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id  = imu.getSensorEventID();
    uint32_t now = millis();

    if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      bool timeToSample = (now - lastStaticSample >= STATIC_SAMPLE_INTERVAL_MS);
      if (timeToSample) {
        writeQuaternionSample(
          imu.getQuatI(), imu.getQuatJ(),
          imu.getQuatK(), imu.getQuatReal()
        );
        lastStaticSample = now;
      }
    }

    if (id == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
      uint8_t s = imu.getStabilityClassifier();
      logStabilityIfChanged(s);
      lastStabilityEvent = now;
      consecutiveResets  = 0;

      if (isMotion(s)) {
        lastMotionTime   = now;
        onTableStartTime = 0;
        activeHz         = DEFAULT_ACTIVE_HZ;
        lastActiveSample = 0;
        requestTransition(STATE_ACTIVE_RECORDING);
        return;
      }

      if (lastMotionTime > 0 && (now - lastMotionTime >= NOT_MOTION_TO_IDLE_MS)) {
        LOGF("TIMER: no motion for %lums -> IDLE", now - lastMotionTime);
        requestTransition(STATE_IDLE);
        return;
      }

      if (s == STABILITY_ON_TABLE) {
        if (onTableStartTime == 0) {
          onTableStartTime = now;
          LOGF("TIMER: ON_TABLE fast-path started (timeout: %ds)",
               ON_TABLE_FAST_PATH_MS / 1000);
        }
        if (now - onTableStartTime >= ON_TABLE_FAST_PATH_MS) {
          LOGF("TIMER: ON_TABLE sustained %lums -> IDLE", now - onTableStartTime);
          requestTransition(STATE_IDLE);
          return;
        }
      } else {
        if (onTableStartTime != 0) {
          LOG("TIMER: ON_TABLE fast-path reset");
          onTableStartTime = 0;
        }
      }
    }
  }
}


// =============================================================================
// SECTION 20 — State: ACTIVE_RECORDING  — identical logic to firmware.ino
// =============================================================================

void handleActiveRecording() {
  if (imu.wasReset()) {
    LOGF("IMU: reset in ACTIVE_RECORDING — reason: %u — reconfiguring", (unsigned)imu.getResetReason());
    bnoInRunningMode = false;
    configureBNO_Running();
    lastStabilityEvent = millis();
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id   = imu.getSensorEventID();
    uint32_t now = millis();

    if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      bool timeToSample = (now - lastActiveSample >= (1000u / activeHz));
      if (timeToSample) {
        writeQuaternionSample(
          imu.getQuatI(), imu.getQuatJ(),
          imu.getQuatK(), imu.getQuatReal()
        );
        lastActiveSample = now;
      }
    }

    if (id == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
      uint8_t s = imu.getStabilityClassifier();
      logStabilityIfChanged(s);
      lastStabilityEvent = now;
      consecutiveResets  = 0;

      if (isMotion(s)) {
        lastMotionTime   = now;
        onTableStartTime = 0;

      } else {
        if (lastMotionTime > 0 &&
            (now - lastMotionTime >= NOT_MOTION_TO_STATIC_MS)) {
          LOGF("TIMER: no motion for %lums -> STATIC_POSTURE",
               now - lastMotionTime);
          onTableStartTime = 0;
          lastStaticSample = now;
          requestTransition(STATE_STATIC_POSTURE);
          return;
        }

        if (s == STABILITY_ON_TABLE) {
          if (onTableStartTime == 0) {
            onTableStartTime = now;
            LOGF("TIMER: ON_TABLE debounce started (timeout: %ds)",
                 ON_TABLE_DEBOUNCE_ACTIVE_MS / 1000);
          }
          if (now - onTableStartTime >= ON_TABLE_DEBOUNCE_ACTIVE_MS) {
            LOGF("TIMER: ON_TABLE sustained %lums -> STATIC_POSTURE",
                 now - onTableStartTime);
            onTableStartTime = 0;
            lastStaticSample = now;
            requestTransition(STATE_STATIC_POSTURE);
            return;
          }
        } else {
          if (onTableStartTime != 0) {
            LOG("TIMER: ON_TABLE debounce reset");
            onTableStartTime = 0;
          }
        }
      }
    }
  }
}


// =============================================================================
// SECTION 21 — Periodic status heartbeat (bench visibility)
// =============================================================================

void printStatusHeartbeat() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 3000) return;
  last = now;

  if (currentState == STATE_IDLE) {
    LOGF("STATUS: IDLE — waiting for motion (move the sensor to enter ACTIVE)");
  } else {
    uint32_t sinceMotion = (lastMotionTime > 0) ? (now - lastMotionTime) : 0;
    LOGF("STATUS: %s — %lums since last motion, samples=%lu",
         stateName(currentState), sinceMotion, sampleCount);
  }
}


// =============================================================================
// SECTION 22 — setup()
// =============================================================================

void setup() {
  Serial.begin(115200);

  unsigned long serialTimeout = millis();
  while (!Serial && (millis() - serialTimeout < 3000)) {
    delay(10);
  }

  LOG("=== HULC Motion Shirt — STATE MACHINE TEST (no BLE, no flash) ===");
  LOG("Transition rules:");
  LOG("  IDLE   -> ACTIVE : move the sensor");
  LOG("  ACTIVE -> STATIC : still 10s, or lay flat 3s");
  LOG("  STATIC -> ACTIVE : move the sensor");
  LOG("  STATIC -> IDLE   : still 60s, or lay flat 5s");
  LOG("LED: off=IDLE  blue=STATIC  red=ACTIVE");
  LOG("");

  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED,  OUTPUT);
  digitalWrite(PIN_LED_BLUE, HIGH);   // LEDs are active-low on XIAO
  digitalWrite(PIN_LED_RED,  HIGH);

  // ── IMU ──
  pinMode(BNO_INT_PIN, INPUT_PULLUP);
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, HIGH);

  Wire.begin();
  LOGF("I2C initialised. Connecting to BNO086 at 0x%02X...", BNO08X_I2C_ADDR);

  if (!imu.begin(BNO08X_I2C_ADDR, Wire, BNO_INT_PIN, BNO_RST_PIN)) {
    LOG("ERROR: BNO086 not detected — check wiring. Halting.");
    digitalWrite(PIN_LED_RED, LOW);   // Red LED on = error
    while (1) {
      digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
      delay(200);
    }
  }

  LOG("BNO086 connected successfully.");

  imuDataReady = false;
  attachInterrupt(digitalPinToInterrupt(BNO_INT_PIN), onBNOInterrupt, FALLING);
  LOG("ISR attached to BNO INT pin.");

  lastMotionTime     = 0;
  lastStabilityEvent = millis();

  configureBNO_Idle();
  currentState = STATE_IDLE;
  pendingState = STATE_IDLE;
  setStateLED(STATE_IDLE);

  LOG("Setup complete. State machine running — waiting for movement...\n");
}


// =============================================================================
// SECTION 23 — loop()
// =============================================================================

void loop() {
  applyPendingTransition();

  checkWatchdog();

  // Re-check in case watchdog forced a transition
  applyPendingTransition();

  switch (currentState) {
    case STATE_IDLE:             handleIdle();            break;
    case STATE_STATIC_POSTURE:   handleStaticPosture();   break;
    case STATE_ACTIVE_RECORDING: handleActiveRecording(); break;
  }

  printStatusHeartbeat();
}
