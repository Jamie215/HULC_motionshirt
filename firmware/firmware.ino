// =============================================================================
// HULC Motion Shirt — Phase 3e Firmware
// Adaptive Power State Machine + QSPI Flash + BLE Sync Protocol
//
// Hardware : Seeed XIAO nRF52840 + SparkFun BNO086 (I2C)
// Libraries: ArduinoBLE, SparkFun BNO08x Arduino Library
//            nrfx_qspi (built into Seeed mbed core — no install needed)
//
// BUILD LINEAGE:
//   Phase 3a (state machine, adaptive power)
//   + Phase 3b (QSPI flash layer from Phase 2b)
//   + Phase 3c (BLE GATT service, advertising, connection-aware sleep)
//   + Phase 3d (control commands: stream, erase, offload)
//   + Phase 3e (time sync, fast offload, IDLE-only sync enforcement)
//
// MULTI-NODE:
//   A motion shirt uses several IMU nodes (e.g. one per body segment). Each
//   node advertises a UNIQUE name — DEVICE_NAME_PREFIX + "-" + the last 4 hex
//   chars of its BLE MAC (stable per board, no per-board reflash).
//   A central (laptop harness / phone) connects to N nodes at once. For the
//   nodes' quaternion streams to be fusable they must share a common time base
//   to within a few ms, so this build adds a millisecond-resolution sync
//   (command 0x05) and a read-only Time Info characteristic (A005) the central
//   reads from each node to measure cross-node clock offset. See
//   firmware/MULTINODE_TESTING.md and tools/multinode_test.py.
//
// BLE GATT layout:
//   Service  : IMU Motion Service         (UUID A0010000-...)
//   Char A001: Quaternion stream          NOTIFY  20 bytes   (debug only)
//   Char A002: Control                    WRITE    9 bytes
//   Char A003: Status                     READ     4 bytes
//   Char A004: Log offload                NOTIFY  200 bytes
//   Char A005: Time Info                  READ    16 bytes
//
// Status characteristic (A003) layout:
//   Byte 0  : Device state (0=IDLE, 1=STATIC_POSTURE, 2=ACTIVE_RECORDING)
//   Byte 1  : Flags (bit 0 = streaming, bit 1 = time synced)
//   Byte 2-3: Log size in KB (little-endian uint16)
//
// Time Info characteristic (A005) layout (little-endian) — for skew measurement:
//   Byte 0-3   : node millis() at the moment of the read (live)
//   Byte 4-11  : sync epoch in ms (uint64, 0 if never synced)
//   Byte 12-15 : node millis() captured at the last sync
//   The central computes each node's epoch-now as
//     sync_epoch_ms + (node_millis_now - sync_millis)
//   and compares two nodes to get their clock offset.
//
// Control commands:
//   0x00  stop BLE streaming (debug)
//   0x01  start BLE streaming (debug)
//   0x02  time sync (seconds) — payload: [0x02, epoch_b0..b3]  (legacy)
//         Firmware stores mapping between millis() and Unix epoch.
//         Records continue using raw millis(); phone applies correction.
//   0x05  time sync (ms) — payload: [0x05, epoch_ms_b0..b7]  (uint64 LE)
//         Millisecond-resolution variant used for multi-node alignment.
//   0x03  erase flash log
//   0x04  begin log offload over A004 (IDLE only — rejected in other states)
//   0x06  re-send a byte range over A004 for gap recovery (IDLE only) —
//         payload: [0x06, offset_b0..b3, length_b0..b3]  (uint32 LE each)
//
// Offload protocol:
//   Phone connects → reads A003 → if state == IDLE, sends a time sync (0x05,
//   or legacy 0x02) then 0x04 (offload). Firmware streams flash in 200-byte
//   chunks at 3ms pacing. If the phone detects dropped notifications it can
//   request just the missing bytes with 0x06. Phone confirms receipt, then
//   sends 0x03 (erase).
//
// Connection behavior:
//   When no BLE central is connected, state handlers sleep via __WFE()
//   between BNO086 INT pulses (low power). When a central connects,
//   waitForIMUData() switches to BLE.poll() loop to keep the BLE stack
//   alive. State machine runs identically in both modes. The firmware also
//   requests a fast connection interval (15–30 ms, iOS-compliant, see
//   setup()) so a connected central gets low-latency GATT reads/notifies and
//   usable offload throughput — the central still has final say.
//
// FLASH APPROACH: nrfx_qspi direct driver
//   Targets EXTERNAL 2MB QSPI chip (P25Q16H). Bootloader lives on
//   INTERNAL flash — no risk of bricking.
//   No filesystem — simple sequential append-only log.
//
// LOG STRUCTURE (raw binary):
//   Header sector (4KB):
//     Byte 0–3   : Magic number 0xC0DE0001 — confirms flash is initialized
//     Byte 4–7   : Write pointer (byte offset of next write)
//     Byte 8–19  : Reserved header padding
//   Data region (byte 4096 onward):
//     20-byte quaternion records, sequentially appended
//
// Each 20-byte record (little-endian):
//   [0–3]   uint32  timestamp_ms
//   [4–7]   float   quat_w  (real)
//   [8–11]  float   quat_x  (i)
//   [12–15] float   quat_y  (j)
//   [16–19] float   quat_z  (k)
//
// States:
//   IDLE             — Selectable wake source (see Section 3, IDLE_WAKE_SOURCE).
//                      All sources run the hub AWAKE (the BNO's own devSleep was
//                      dropped — it suppressed the motion wake; see Section 3):
//                      • DETECTOR (default) — Stability Detector (0x1C), accel
//                        only, reported on-change. ~8.8mA idle; self-resets ~6.7s
//                        (inherent, benign re-arm). Catches slow motion including
//                        held-limb stretches. Motion = EXITED.
//                      • CLASSIFIER — Stability Classifier (0x13), accel+gyro /
//                        MotionEngine. Reset-FREE idle (fusion keeps the hub
//                        active) at higher idle current. Motion = classifier ==
//                        MOTION.
//                      • SIGMOTION — Significant Motion (0x12), accel-only
//                        one-shot. Lowest idle (~7.4mA, no self-reset) but only
//                        fires on energetic motion (shake/pickup), NOT slow
//                        held-limb stretches. Motion = event fires.
//   STATIC_POSTURE   — RV @ ~1Hz (slow) + Classifier @ 500ms, writes gated to 0.2Hz
//   ACTIVE_RECORDING — RV @ ~10Hz + Classifier @ 500ms, writes gated to activeHz
//
// Phase 3 complete. Next: Phase 4 (mobile app), Phase 5 (data pipeline).
// =============================================================================


// =============================================================================
// SECTION 1 — Includes
// =============================================================================

#include <ArduinoBLE.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "nrfx_qspi.h"


// =============================================================================
// SECTION 2 — Pin Assignments
// =============================================================================

#define BNO_INT_PIN     2
#define BNO_RST_PIN     3
#define BNO08X_I2C_ADDR 0x4B

// The Seeed XIAO nRF52840 mbed core does not define the SDA/SCL convenience
// macros (unlike the AVR/ESP cores), so i2cBusRecover() won't compile against
// them bare. Alias to the canonical Wire pin defines, which this core does
// provide (it's what Wire.begin() itself uses). Guarded so cores that DO define
// SDA/SCL keep their own values.
#ifndef SDA
  #define SDA PIN_WIRE_SDA
#endif
#ifndef SCL
  #define SCL PIN_WIRE_SCL
#endif

#define PIN_LED_BLUE    LED_BLUE
#define PIN_LED_RED     LED_RED


// =============================================================================
// SECTION 2b — BLE Configuration (from Phase 2b)
// =============================================================================

// Base advertised name. Each node appends a unique "-XXXX" suffix (from the
// last 4 hex chars of its BLE MAC) at boot so multiple nodes never collide on
// the air. See makeDeviceName(). The full name is held in g_deviceName.
#define DEVICE_NAME_PREFIX "HULC-IMU"
#define UUID_SERVICE   "A0010000-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_QUAT      "A0010001-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_CONTROL   "A0010002-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_STATUS    "A0010003-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_OFFLOAD   "A0010004-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_SYNCINFO  "A0010005-B0CE-4A4A-8F0B-0011223344FF"

// Advertising interval in ArduinoBLE 0.625ms units (1600 = 1000ms). Slowed and
// gated to IDLE to save radio power; see firmware/POWER_OPTIMIZATION.md.
#define ADV_INTERVAL_UNITS    1600

// Offload transfer tuning
#define OFFLOAD_CHUNK_SIZE    200    // max bytes per BLE notification (up from 20)
#define OFFLOAD_PACING_MS     3      // ms delay between chunks (down from 10)
#define OFFLOAD_PROGRESS_KB   10     // print progress every N KB

// A004 notifications are unacknowledged, so a notification the central drops
// (common on some BLE stacks) vanishes silently — a run of records simply
// disappears from the middle of the reconstructed file. To make loss
// detectable and recoverable, every notification is framed:
//     [0..3]  uint32 LE  offset of this payload within the log data region
//     [4..]              up to OFFLOAD_DATA_SIZE bytes of record data
// and a header notification is sent first with a sentinel offset carrying the
// exact total length. The host places each payload by its offset, so a dropped
// notification leaves a locatable hole it can fill by re-offloading (the log is
// not erased until the host sends 0x03). Framing keeps the notification within
// the existing 200-byte size, so no MTU change is needed.
#define OFFLOAD_HEADER_SIZE   4                                       // LE offset prefix
#define OFFLOAD_DATA_SIZE     (OFFLOAD_CHUNK_SIZE - OFFLOAD_HEADER_SIZE)  // 196 record bytes/notification
#define OFFLOAD_OFFSET_HDR    0xFFFFFFFFUL   // sentinel offset: payload is the 4-byte total length
#define OFFLOAD_SEND_TIMEOUT_MS 3000         // give up on one notification after this long backpressured


// =============================================================================
// SECTION 3 — Stability Detector (0x1C)
// =============================================================================

#ifndef SH2_STABILITY_DETECTOR
#define SH2_STABILITY_DETECTOR 0x1C
#endif
#ifndef SH2_SIGNIFICANT_MOTION
#define SH2_SIGNIFICANT_MOTION 0x12  // mirrors the sh2 driver enum (sh2.h, sh2_SensorId_e) — one-shot wake-on-motion
#endif
#define DETECTOR_EXITED   2

// ── BNO hub power note ─────────────────────────────────────────────────────
// The hub's own devSleep was explored as a low-power idle path and DROPPED: with
// the hub asleep the detector's EXITED wake event is coalesced/rarely delivered,
// so shaking failed to wake IDLE→ACTIVE (see IDLE_WAKE_SOURCE.md, "devSleep was
// explored and dropped"). All wake sources now run the hub AWAKE — the proven
// config. Getting below hub-awake idle would need a hardware wake (a separate
// low-power accel/motion interrupt waking the nRF), not the BNO's own devSleep.

// ── IDLE wake source (compile-time A/B switch) ─────────────────────────────
// DETECTOR   — Stability Detector (0x1C), accelerometer only, reported on-change.
//              ~8.8mA idle; self-resets every ~6.7s (inherent — a benign re-arm;
//              IDLE logs nothing). Catches slow motion, including held-limb
//              stretches. Default.
// CLASSIFIER — Stability Classifier (0x13), accel+gyro through the MotionEngine.
//              No idle reset (fusion keeps the hub active), at higher idle
//              current. Wakes IDLE on the same MOTION classification
//              STATIC_POSTURE/ACTIVE act on.
// SIGMOTION  — Significant Motion (0x12), accelerometer-only one-shot. Lowest
//              idle (~7.4mA) and does not self-reset, but fires only on energetic
//              motion (shake/pickup), NOT slow held-limb stretches — it is an
//              energy-over-a-window detector (Android-style significant motion).
//              Its accel threshold + window ARE tunable via the FRS record
//              SIG_MOTION_DETECT_CONFIG (0xC274), but the SparkFun library exposes
//              no FRS-write path, so we cannot set it today (see IDLE_WAKE_SOURCE.md
//              "Tuning SIGMOTION" for the investigation). Even tuned, its energy/
//              window nature is a poor fit for slow motion. Suitable only where
//              slow-motion wake latency is acceptable (e.g. off-body standby), not
//              for on-body capture.
//
// See firmware/IDLE_WAKE_SOURCE.md and state_machine_test/FINDINGS.md for the
// full investigation. Flip IDLE_WAKE_SOURCE and reflash to A/B compare.
#define IDLE_WAKE_DETECTOR    0
#define IDLE_WAKE_CLASSIFIER  1
#define IDLE_WAKE_SIGMOTION   2
// DETECTOR is the default: it catches slow, deliberate motion (held-limb
// stretches) at ~8.8mA. SIGMOTION idles ~1.4mA lower because it does not run the
// detector's ~6.7s self-reset, but misses those slow motions, so it is not a
// drop-in replacement. CLASSIFIER is the reset-free option at higher current.
#define IDLE_WAKE_SOURCE      IDLE_WAKE_DETECTOR

#if   IDLE_WAKE_SOURCE == IDLE_WAKE_DETECTOR
  #define IDLE_WAKE_NAME "Stability Detector (0x1C)"
  #define IDLE_WAKE_SENSOR_ID SH2_STABILITY_DETECTOR
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER
  #define IDLE_WAKE_NAME "Stability Classifier (0x13)"
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_SIGMOTION
  #define IDLE_WAKE_NAME "Significant Motion (0x12)"
  #define IDLE_WAKE_SENSOR_ID SH2_SIGNIFICANT_MOTION
#else
  #error "IDLE_WAKE_SOURCE must be DETECTOR, CLASSIFIER, or SIGMOTION"
#endif


// =============================================================================
// SECTION 3b — Active Fusion Source (compile-time A/B switch)
// =============================================================================
// The RUNNING states log an orientation quaternion from one of two BNO086 fusion
// reports. Flip this switch and reflash to A/B compare (same pattern as
// IDLE_WAKE_SOURCE). The 20-byte record format is identical either way, so
// nothing downstream changes — only the fusion source. The tradeoff (power
// measured negligible; absolute heading vs. slow yaw drift; magnetometer
// interference) is written up in firmware/POWER_OPTIMIZATION.md.
//   RV  — Rotation Vector (0x05): 9-axis, absolute heading, uses magnetometer.
//   GRV — Game Rotation Vector (0x08): 6-axis, no mag; yaw drifts slowly.
#define ACTIVE_FUSION_RV      0
#define ACTIVE_FUSION_GRV     1
#define ACTIVE_FUSION_SOURCE  ACTIVE_FUSION_RV

#if   ACTIVE_FUSION_SOURCE == ACTIVE_FUSION_GRV
  #define FUSION_NAME            "Game Rotation Vector (0x08, 6-axis, no mag)"
  #define FUSION_REPORT_ID       SENSOR_REPORTID_GAME_ROTATION_VECTOR
  #define enableFusionVector(ms) imu.enableGameRotationVector(ms)
  #define fusionQuatI()          imu.getGameQuatI()
  #define fusionQuatJ()          imu.getGameQuatJ()
  #define fusionQuatK()          imu.getGameQuatK()
  #define fusionQuatReal()       imu.getGameQuatReal()
#elif ACTIVE_FUSION_SOURCE == ACTIVE_FUSION_RV
  #define FUSION_NAME            "Rotation Vector (0x05, 9-axis, abs heading)"
  #define FUSION_REPORT_ID       SENSOR_REPORTID_ROTATION_VECTOR
  #define enableFusionVector(ms) imu.enableRotationVector(ms)
  #define fusionQuatI()          imu.getQuatI()
  #define fusionQuatJ()          imu.getQuatJ()
  #define fusionQuatK()          imu.getQuatK()
  #define fusionQuatReal()       imu.getQuatReal()
#else
  #error "ACTIVE_FUSION_SOURCE must be ACTIVE_FUSION_RV or ACTIVE_FUSION_GRV"
#endif


// =============================================================================
// SECTION 4 — Sampling Rates
// =============================================================================

#define DEFAULT_ACTIVE_HZ           10
#define STATIC_SAMPLE_INTERVAL_MS   5000
// RV report interval, matched to the ACTIVE log period (1000/DEFAULT_ACTIVE_HZ
// = 100ms) so the BNO doesn't fuse and ship samples we'd only discard. Tier A;
// rationale and the STATIC-specific follow-up are in firmware/POWER_OPTIMIZATION.md.
#define BNO_RV_INTERVAL_MS          100
// STATIC-specific (slow) RV interval — Tier B. STATIC only snapshots posture at
// 0.2Hz, so running the fusion vector slow saves the wasted reads the 10Hz rate
// would discard. The Classifier stays at ACTIVE_STABILITY_MS, so motion
// detection is unaffected. See firmware/POWER_OPTIMIZATION.md.
#define BNO_RV_STATIC_INTERVAL_MS   1000
#define ACTIVE_STABILITY_MS         500
// Classifier report interval when IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER.
// Also the worst-case IDLE->ACTIVE motion-onset latency for that build (the
// classifier reaches MOTION within one interval). Unused by the detector build.
#define IDLE_STABILITY_MS           1000

// IDLE arms the Stability DETECTOR (0x1C) as the wake source (see Section 3).
// enableReport() takes MILLISECONDS. The detector is a change/event sensor: it
// fires EXITED the instant motion breaks stability regardless of this interval,
// so the periodic report is only a keepalive. It is set long (on-change) so the
// detector stays silent while still instead of waking the nRF ~1×/s — a small
// idle-current trim with no loss of motion sensitivity and no effect on the
// detector's inherent ~6.7s self-reset. See firmware/POWER_OPTIMIZATION.md.
#define IDLE_DETECTOR_INTERVAL_MS   60000UL      // on-change: 60s keepalive, EXITED fires immediately


// =============================================================================
// SECTION 5 — Timeouts (ms)
// =============================================================================

#define NOT_MOTION_TO_STATIC_MS     10000
#define ON_TABLE_DEBOUNCE_ACTIVE_MS 3000
#define NOT_MOTION_TO_IDLE_MS       60000
#define ON_TABLE_FAST_PATH_MS       5000


// =============================================================================
// SECTION 6 — QSPI Flash Layout (from Phase 2b)
// =============================================================================
// External P25Q16H 2MB flash on XIAO nRF52840.
// Sector = 4KB, Block = 64KB, Total = 2MB = 2,097,152 bytes.

#define QSPI_FLASH_SIZE      (2UL * 1024UL * 1024UL)
#define QSPI_SECTOR_SIZE     (4UL * 1024UL)
#define QSPI_PAGE_SIZE       256UL

// Header occupies first sector (4KB) — its own erase unit.
// Data records start at byte LOG_DATA_START.
#define LOG_HEADER_ADDR      0x00000000
#define LOG_DATA_START       QSPI_SECTOR_SIZE           // 0x00001000 = 4096
#define LOG_MAGIC            0xC0DE0001UL
#define LOG_FLASH_MAX_BYTES  (QSPI_FLASH_SIZE - QSPI_SECTOR_SIZE)

// Stop logging 200KB before end to leave headroom
#define LOG_STOP_BYTES       (LOG_FLASH_MAX_BYTES - (200UL * 1024UL))

// Watermark: when 80% of usable space is consumed, start reducing activeHz
#define LOG_WATERMARK_BYTES  ((LOG_FLASH_MAX_BYTES * 80UL) / 100UL)

// Fsync header every 10 writes (~1s at 10Hz). Limits data loss on
// power cut to at most 1 second of samples.
#define FSYNC_EVERY          10

#define BYTES_PER_SAMPLE     20


// =============================================================================
// SECTION 7 — BNO086 Stability Classifier Values
// =============================================================================
// These MUST match the sh2 library's raw classification values, returned by
// getStabilityClassifier() (un.stabilityClassifier.classification): the BNO086
// reports 0=Unknown, 1=OnTable, 2=Stationary, 3=Stable, 4=Motion. An earlier
// version of these defines was shifted down by one (ON_TABLE=0…MOTION=3), which
// bench-confirmed as the raw value 4 printing as "UNKNOWN(motion assumed)":
// real Motion(4) still tripped isMotion (>=3), but real Stable(3) was wrongly
// counted as motion and the ON_TABLE checks matched Unknown(0) instead of a real
// on-table. Aligned to the datasheet so all three states classify correctly.
#define STABILITY_UNKNOWN     0
#define STABILITY_ON_TABLE    1
#define STABILITY_STATIONARY  2
#define STABILITY_STABLE      3
#define STABILITY_MOTION      4


// =============================================================================
// SECTION 8 — Watchdog
// =============================================================================

#define WATCHDOG_MULTIPLIER          5
#define WATCHDOG_MAX_RESETS          3
#define RUNNING_WATCHDOG_MS          (ACTIVE_STABILITY_MS * WATCHDOG_MULTIPLIER)


// =============================================================================
// SECTION 9 — Serial Debug Logging
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
// SECTION 10 — State Machine Types
// =============================================================================

typedef enum {
  STATE_IDLE,
  STATE_STATIC_POSTURE,
  STATE_ACTIVE_RECORDING
} SystemState;

// Which fusion config the BNO currently holds. The two RUNNING states use
// different fusion rates, so configureBNO_Running() needs to tell an ACTIVE↔STATIC
// switch from a same-state re-entry. Kept here (not in Section 11) so it precedes
// the auto-generated prototype of the function that takes it — as SystemState is.
typedef enum {
  BNO_CFG_NONE,     // no running report armed (IDLE / post-reset)
  BNO_CFG_ACTIVE,   // fusion @ BNO_RV_INTERVAL_MS (fast)
  BNO_CFG_STATIC    // fusion @ BNO_RV_STATIC_INTERVAL_MS (slow)
} BnoRunningCfg;

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
    case STABILITY_UNKNOWN:    return "UNKNOWN";
    case STABILITY_ON_TABLE:   return "ON_TABLE";
    case STABILITY_STATIONARY: return "STATIONARY";
    case STABILITY_STABLE:     return "STABLE";
    case STABILITY_MOTION:     return "MOTION";
    default:                   return "UNKNOWN(motion assumed)";
  }
}


// =============================================================================
// SECTION 11 — Globals
// =============================================================================

// ── IMU ──
BNO08x       imu;

// ── BLE (from Phase 2b) ──
BLEService        imuService(UUID_SERVICE);
BLECharacteristic quatChar(UUID_QUAT,       BLENotify,  20);
BLECharacteristic ctrlChar(UUID_CONTROL,    BLEWrite,    9);  // 1 cmd + up to 8-byte payload
BLECharacteristic statChar(UUID_STATUS,     BLERead,     4);
BLECharacteristic offloadChar(UUID_OFFLOAD, BLENotify,  OFFLOAD_CHUNK_SIZE);
BLECharacteristic syncInfoChar(UUID_SYNCINFO, BLERead,  16);  // clock info for skew measurement
bool              bleConnected     = false;
bool              bleReady         = false;   // BLE.begin() succeeded — guards advertise()/stopAdvertise()
bool              streaming        = false;

// ── Node identity ──
// Full advertised name: DEVICE_NAME_PREFIX + "-" + last 4 hex of BLE MAC.
// Filled once in makeDeviceName(); unique per physical board.
char              g_deviceName[24] = DEVICE_NAME_PREFIX;

// ── Time Sync ──
uint32_t          syncEpoch        = 0;     // Unix epoch (seconds) from central
uint64_t          syncEpochMs      = 0;     // Unix epoch (ms) — multi-node alignment
uint32_t          syncMillis       = 0;     // millis() at time of sync
bool              timeSynced       = false;

// ── State Machine ──
SystemState  currentState        = STATE_IDLE;
SystemState  pendingState        = STATE_IDLE;

uint8_t      activeHz            = DEFAULT_ACTIVE_HZ;

// ── QSPI Flash (from Phase 2b) ──
bool         qspiReady           = false;
bool         qspiAsleep          = false;   // external flash parked in deep power-down
bool         logging             = true;
uint32_t     writeAddr           = LOG_DATA_START;
uint32_t     writeCount          = 0;
// MUST be 4-byte aligned: nRF52 QSPI EasyDMA requires word-aligned buffers.
// An unaligned pktBuf caused every flash record to be written shifted by one
// byte (a stray leading byte + the last byte truncated) — see git history.
alignas(4) uint8_t pktBuf[20];     // Shared record packing buffer

// ── Timers ──
uint32_t     lastMotionTime      = 0;
uint32_t     onTableStartTime    = 0;
uint32_t     lastActiveSample    = 0;
uint32_t     lastStaticSample    = 0;
uint8_t      lastLoggedStability = 255;

// ── Watchdog ──
uint32_t     lastStabilityEvent  = 0;
uint8_t      consecutiveResets   = 0;

// ── BNO config guard (type in Section 10) ──
BnoRunningCfg bnoRunningCfg      = BNO_CFG_NONE;

volatile bool imuDataReady = false;


// =============================================================================
// SECTION 12 — QSPI Flash Helpers (from Phase 2b, verbatim)
// =============================================================================

// ---------------------------------------------------------------------------
// QSPI pin config for XIAO nRF52840 (P25Q16H flash chip)
// ---------------------------------------------------------------------------
static void qspiConfig(nrfx_qspi_config_t* cfg) {
  cfg->xip_offset                  = NRFX_QSPI_CONFIG_XIP_OFFSET;
  cfg->pins.sck_pin                = NRF_GPIO_PIN_MAP(0, 21);
  cfg->pins.csn_pin                = NRF_GPIO_PIN_MAP(0, 25);
  cfg->pins.io0_pin                = NRF_GPIO_PIN_MAP(0, 20);
  cfg->pins.io1_pin                = NRF_GPIO_PIN_MAP(0, 24);
  cfg->pins.io2_pin                = NRF_GPIO_PIN_MAP(0, 22);
  cfg->pins.io3_pin                = NRF_GPIO_PIN_MAP(0, 23);
  cfg->prot_if.readoc              = NRF_QSPI_READOC_FASTREAD;
  cfg->prot_if.writeoc             = NRF_QSPI_WRITEOC_PP;
  cfg->prot_if.addrmode            = NRF_QSPI_ADDRMODE_24BIT;
  cfg->prot_if.dpmconfig           = false;
  cfg->phy_if.sck_delay            = 0x05;
  cfg->phy_if.dpmen                = false;
  cfg->phy_if.sck_freq             = NRF_QSPI_FREQ_32MDIV4;     // 8MHz
  cfg->phy_if.spi_mode             = NRF_QSPI_MODE_0;
  cfg->irq_priority                = NRFX_QSPI_CONFIG_IRQ_PRIORITY;
}

static bool qspiWait() {
  uint32_t timeout = 10000;
  while (nrfx_qspi_mem_busy_check() != NRFX_SUCCESS) {
    if (--timeout == 0) {
      Serial.println("[QSPI] Timeout waiting for ready");
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

bool initQSPI() {
  nrfx_qspi_config_t cfg;
  qspiConfig(&cfg);

  nrfx_err_t err = nrfx_qspi_init(&cfg, NULL, NULL);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] Init failed: 0x");
    Serial.println(err, HEX);
    return false;
  }

  nrf_qspi_cinstr_conf_t cinstr = {
    .opcode    = 0x06,   // WREN
    .length    = NRF_QSPI_CINSTR_LEN_1B,
    .io2_level = true,
    .io3_level = true,
    .wipwait   = true,
    .wren      = false
  };
  nrfx_qspi_cinstr_xfer(&cinstr, NULL, NULL);

  Serial.println("[QSPI] Init OK");
  return true;
}

// ---------------------------------------------------------------------------
// External flash deep power-down (P25Q16H: 0xB9 enters, 0xAB releases). IDLE
// never touches flash, so park the chip there and wake it before any access —
// on BLE connect (offload/erase arrive as control commands) and at the top of
// applyPendingTransition() for the header write. Saving is small (~tens of uA
// next to the ~9mA idle) but harmless.
//
// Only the chip is parked, not the nRF QSPI peripheral: deactivating the
// peripheral was tried and made idle WORSE (~9mA → ~11mA) via nRF52840 Errata
// 122, so it was removed. See firmware/POWER_OPTIMIZATION.md.
// ---------------------------------------------------------------------------
static bool qspiSimpleCmd(uint8_t opcode) {
  nrf_qspi_cinstr_conf_t cinstr = {
    .opcode    = opcode,
    .length    = NRF_QSPI_CINSTR_LEN_1B,
    .io2_level = true,
    .io3_level = true,
    .wipwait   = false,
    .wren      = false
  };
  return nrfx_qspi_cinstr_xfer(&cinstr, NULL, NULL) == NRFX_SUCCESS;
}

void qspiSleep() {
  if (!qspiReady || qspiAsleep) return;
  qspiSimpleCmd(0xB9);             // Deep Power Down
  qspiAsleep = true;
}

void qspiWake() {
  if (!qspiReady || !qspiAsleep) return;
  qspiSimpleCmd(0xAB);             // Release from Deep Power Down
  delayMicroseconds(30);           // tRES: let the chip settle before any access
  qspiAsleep = false;
}

bool qspiEraseSector(uint32_t addr) {
  nrfx_err_t err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] Erase failed at 0x");
    Serial.println(addr, HEX);
    return false;
  }
  return qspiWait();
}

// Erase one 64KB block, with error checking and an erase-sized busy-wait.
// A 64KB block erase can take up to ~1s on the P25Q16H — far longer than the
// ~100ms qspiWait() budget used for writes — so poll with a larger bound.
// Returning early (as qspiWait() would) while the chip is still busy makes the
// next erase land on a busy chip and get dropped, so the wait MUST cover the
// whole erase. Returns false on driver error or timeout so callers can abort.
bool qspiEraseBlock64k(uint32_t addr) {
  nrfx_err_t err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, addr);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] 64K erase failed at 0x");
    Serial.println(addr, HEX);
    return false;
  }
  uint32_t timeout = 200000;   // ~2s: 200000 * 10us
  while (nrfx_qspi_mem_busy_check() != NRFX_SUCCESS) {
    if (--timeout == 0) {
      Serial.print("[QSPI] 64K erase timeout at 0x");
      Serial.println(addr, HEX);
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

bool qspiWrite(uint32_t addr, const uint8_t* buf, size_t len) {
  nrfx_err_t err = nrfx_qspi_write(buf, len, addr);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] Write failed at 0x");
    Serial.println(addr, HEX);
    return false;
  }
  return qspiWait();
}

bool qspiRead(uint32_t addr, uint8_t* buf, size_t len) {
  size_t alignedLen = (len + 3) & ~3;
  nrfx_err_t err = nrfx_qspi_read(buf, alignedLen, addr);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] Read failed at 0x");
    Serial.println(addr, HEX);
    return false;
  }
  return qspiWait();
}

// ---------------------------------------------------------------------------
// Header persistence — survives power cycles
// ---------------------------------------------------------------------------
void saveHeader() {
  qspiEraseSector(LOG_HEADER_ADDR);
  alignas(4) uint8_t header[20] = {0};   // QSPI EasyDMA: 4-byte aligned
  uint32_t magic       = LOG_MAGIC;
  memcpy(header,     &magic,     4);
  memcpy(header + 4, &writeAddr, 4);
  qspiWrite(LOG_HEADER_ADDR, header, 20);

  // Read-back verify
  alignas(4) uint8_t verify[20] = {0};   // QSPI EasyDMA: 4-byte aligned
  uint32_t readMagic  = 0;
  uint32_t readBack   = 0;
  qspiRead(LOG_HEADER_ADDR, verify, 20);
  memcpy(&readMagic, verify,     4);
  memcpy(&readBack,  verify + 4, 4);

  if (readMagic != LOG_MAGIC || readBack != writeAddr) {
    Serial.println("[QSPI] WARNING: header verify FAILED");
    Serial.print("  expected magic: 0x"); Serial.println(LOG_MAGIC, HEX);
    Serial.print("  read magic:     0x"); Serial.println(readMagic, HEX);
    Serial.print("  expected addr:  0x"); Serial.println(writeAddr, HEX);
    Serial.print("  read addr:      0x"); Serial.println(readBack,  HEX);
  }
}

bool loadHeader() {
  alignas(4) uint8_t header[20] = {0};   // QSPI EasyDMA: 4-byte aligned
  if (!qspiRead(LOG_HEADER_ADDR, header, 20)) return false;

  uint32_t magic = 0;
  memcpy(&magic, header, 4);

  if (magic != LOG_MAGIC) {
    Serial.println("[QSPI] Fresh flash — initialising log");
    writeAddr = LOG_DATA_START;
    saveHeader();
  } else {
    memcpy(&writeAddr, header + 4, 4);
    if (writeAddr < LOG_DATA_START || writeAddr >= QSPI_FLASH_SIZE) {
      Serial.println("[QSPI] Bad write pointer — resetting log");
      writeAddr = LOG_DATA_START;
      saveHeader();
    }
    Serial.print("[QSPI] Resuming log at 0x");
    Serial.println(writeAddr, HEX);
  }

  writeCount = (writeAddr - LOG_DATA_START) / BYTES_PER_SAMPLE;
  return true;
}

// ---------------------------------------------------------------------------
// eraseLog() — wipes entire flash and resets write pointer.
// Wired to BLE control command 0x03 (see handleControl).
// ---------------------------------------------------------------------------
void eraseLog() {
  Serial.println("[QSPI] Erasing log — this takes ~10-30s...");

  // Erase the whole chip in 64KB blocks. Abort on the FIRST failure: the old
  // eraseLog() ignored every return value and unconditionally reset the write
  // pointer, so a silently-failed erase (dropped block-erase, chip busy) left
  // stale records on flash while reporting log=0. Bail without touching
  // writeAddr instead — status keeps reporting the real (still-populated) size.
  for (uint32_t addr = 0; addr < QSPI_FLASH_SIZE; addr += 64 * 1024) {
    if (!qspiEraseBlock64k(addr)) {
      Serial.println("[QSPI] Log erase FAILED — flash NOT wiped, pointer unchanged");
      return;
    }
  }

  // Verify the data region actually came back blank (0xFF) before trusting the
  // erase and resetting the pointer. A read-back is cheap insurance against a
  // block that reports success but did not physically clear.
  alignas(4) uint8_t check[16];   // QSPI EasyDMA: 4-byte aligned
  if (!qspiRead(LOG_DATA_START, check, sizeof(check))) {
    Serial.println("[QSPI] Log erase verify read FAILED — pointer unchanged");
    return;
  }
  for (size_t i = 0; i < sizeof(check); i++) {
    if (check[i] != 0xFF) {
      Serial.print("[QSPI] Log erase verify FAILED at data byte ");
      Serial.print(i);
      Serial.print(" (0x");
      Serial.print(check[i], HEX);
      Serial.println(") — pointer unchanged");
      return;
    }
  }

  writeAddr  = LOG_DATA_START;
  writeCount = 0;
  logging    = true;
  saveHeader();
  Serial.println("[QSPI] Log erased");
}

// ---------------------------------------------------------------------------
// writeLogRecord() — appends one 20-byte record to flash.
// Returns true if the write succeeded, false if capacity limit was hit
// or flash is unavailable. Caller uses the return value to trigger
// state transitions (e.g. flash full → IDLE).
// ---------------------------------------------------------------------------
bool writeLogRecord(const uint8_t* buf, size_t len) {
  if (!qspiReady || !logging) return false;

  // Capacity guard
  if (writeAddr + len > LOG_DATA_START + LOG_STOP_BYTES) {
    if (logging) {
      logging = false;
      LOG("[QSPI] Capacity limit — logging halted");
    }
    return false;
  }

  // Erase sector ahead when we reach a sector boundary
  if ((writeAddr % QSPI_SECTOR_SIZE) == 0) {
    qspiEraseSector(writeAddr);
  }

  qspiWrite(writeAddr, buf, len);
  writeAddr += len;
  writeCount++;

  // Periodically persist write pointer
  if (writeCount % FSYNC_EVERY == 0) {
    saveHeader();
    LOGF("FLASH: checkpoint — records: %lu  addr: 0x%lX", writeCount, writeAddr);
  }

  return true;
}


// =============================================================================
// SECTION 12b — BLE Helpers
// =============================================================================

// ---------------------------------------------------------------------------
// updateStatus() — writes current device state to the status characteristic.
// Phone reads this on connect to decide whether to trigger a sync.
//
// Layout:
//   Byte 0  : Device state (0=IDLE, 1=STATIC, 2=ACTIVE)
//   Byte 1  : Flags (bit 0 = streaming, bit 1 = time synced)
//   Byte 2-3: Log size in KB (little-endian uint16)
// ---------------------------------------------------------------------------
void updateStatus() {
  uint32_t logBytes = (writeAddr > LOG_DATA_START) ? (writeAddr - LOG_DATA_START) : 0;
  uint16_t logKB    = (uint16_t)(logBytes / 1024);

  uint8_t flags = 0;
  if (streaming)  flags |= 0x01;
  if (timeSynced) flags |= 0x02;

  uint8_t status[4] = {
    (uint8_t)currentState,
    flags,
    (uint8_t)(logKB & 0xFF),
    (uint8_t)((logKB >> 8) & 0xFF)
  };
  statChar.writeValue(status, 4);
}

// ---------------------------------------------------------------------------
// makeDeviceName() — builds a per-board unique advertised name into
// g_deviceName by appending the last 4 hex chars of the BLE MAC address.
// The MAC is fixed per board, so the suffix is stable across reboots and
// unique per physical board — no per-board reflash needed.
//
// MUST be called after BLE.begin() (BLE.address() is only valid once the
// stack is up). Uses only ArduinoBLE, so it needs no nRF MDK headers.
// ---------------------------------------------------------------------------
void makeDeviceName() {
  String addr = BLE.address();     // "xx:xx:xx:xx:xx:xx"
  addr.replace(":", "");           // "xxxxxxxxxxxx"
  addr.toUpperCase();
  String suffix = (addr.length() >= 4) ? addr.substring(addr.length() - 4)
                                       : addr;
  snprintf(g_deviceName, sizeof(g_deviceName),
           "%s-%s", DEVICE_NAME_PREFIX, suffix.c_str());
}

// ---------------------------------------------------------------------------
// updateSyncInfo() — packs the node's current clock state into the Time Info
// characteristic (A005). The central reads this from each node and computes
//   epoch_now_ms = sync_epoch_ms + (node_millis_now - sync_millis)
// then compares nodes to measure cross-node clock offset. Refreshed lazily
// on every read via the BLERead handler so byte 0-3 is always live.
//
// Layout (little-endian):
//   [0-3]   uint32 node millis() now
//   [4-11]  uint64 sync epoch ms (0 if never synced)
//   [12-15] uint32 node millis() captured at last sync
// ---------------------------------------------------------------------------
void updateSyncInfo() {
  uint8_t  buf[16];
  uint32_t nowMs = millis();
  memcpy(buf + 0,  &nowMs,       4);
  memcpy(buf + 4,  &syncEpochMs, 8);
  memcpy(buf + 12, &syncMillis,  4);
  syncInfoChar.writeValue(buf, 16);
}

// BLERead handler: refresh the live millis() field just before the value is
// served, so the central always reads a fresh timestamp.
void onSyncInfoRead(BLEDevice /*central*/, BLECharacteristic /*chr*/) {
  updateSyncInfo();
}

// ---------------------------------------------------------------------------
// offloadLog(central) — streams all logged records to phone via A004.
//
// Uses 200-byte notifications at 3ms pacing (~20s for 888KB vs ~7min at
// 20B/10ms). Each notification is framed with a 4-byte LE offset prefix
// (see OFFLOAD_* defines) so the host can detect and locate any dropped,
// unacknowledged notification; a header notification with a sentinel offset
// carries the exact total length up front. Pauses flash logging during
// transfer. Does NOT erase afterward — the phone sends 0x03 explicitly after
// confirming complete receipt, so an incomplete transfer can be re-offloaded.
// Rejected if not in IDLE state (enforced by handleControl).
// ---------------------------------------------------------------------------

// Send one offload notification WITH backpressure. offloadChar.writeValue()
// returns false when the BLE TX buffer is full (no ACL credits this connection
// event); the previous code ignored that and advanced anyway, silently dropping
// ~half the notifications when 3ms pacing outran the negotiated connection
// interval. Here we instead retry the SAME bytes, pumping the stack with
// BLE.poll() so a connection event can drain the queue, until it is accepted.
// This self-paces to whatever interval the central negotiated (fast or slow).
// Returns false only if the link drops or the packet is stuck past the timeout.
static bool offloadSend(BLEDevice& central, const uint8_t* buf, size_t len) {
  uint32_t start = millis();
  while (central.connected()) {
    if (offloadChar.writeValue(buf, len)) return true;
    BLE.poll();                                  // let a connection event transmit
    if (millis() - start > OFFLOAD_SEND_TIMEOUT_MS) return false;
  }
  return false;
}

void offloadLog(BLEDevice& central) {
  saveHeader();

  uint32_t totalBytes = (writeAddr > LOG_DATA_START) ? (writeAddr - LOG_DATA_START) : 0;
  if (totalBytes == 0) {
    Serial.println("[OFFLOAD] No data to send");
    return;
  }

  Serial.print("[OFFLOAD] Starting — ");
  Serial.print(totalBytes);
  Serial.print(" bytes (");
  Serial.print(totalBytes / 1024);
  Serial.println(" KB)");

  // Pause logging so flash writes don't interfere with reads
  bool wasLogging = logging;
  logging = false;

  uint32_t readAddr     = LOG_DATA_START;
  uint32_t bytesSent    = 0;
  uint32_t chunkCount   = 0;
  uint32_t lastProgress = 0;
  alignas(4) uint8_t chunk[OFFLOAD_CHUNK_SIZE];   // QSPI EasyDMA: 4-byte aligned

  // Header notification: sentinel offset + exact total length, so the host can
  // detect any dropped notification by the byte count / hole it leaves.
  uint32_t hdrOffset = OFFLOAD_OFFSET_HDR;
  memcpy(chunk, &hdrOffset, OFFLOAD_HEADER_SIZE);
  memcpy(chunk + OFFLOAD_HEADER_SIZE, &totalBytes, 4);
  offloadSend(central, chunk, OFFLOAD_HEADER_SIZE + 4);

  // Real wall-clock timing. offloadSend() blocks until the stack accepts each
  // notification, so the loop is paced by the actual connection interval —
  // measuring elapsed here captures the TRUE transfer time.
  uint32_t tStart = millis();

  while (central.connected() && readAddr < writeAddr) {
    uint32_t off       = readAddr - LOG_DATA_START;   // payload position in the log
    size_t   remaining = writeAddr - readAddr;
    size_t   dataSize  = (remaining >= OFFLOAD_DATA_SIZE) ? OFFLOAD_DATA_SIZE : remaining;

    memcpy(chunk, &off, OFFLOAD_HEADER_SIZE);         // 4-byte LE offset prefix
    if (!qspiRead(readAddr, chunk + OFFLOAD_HEADER_SIZE, dataSize)) break;
    // Backpressure: retry the same chunk until the stack accepts it, so a full
    // TX buffer stalls the loop instead of silently dropping the notification.
    if (!offloadSend(central, chunk, OFFLOAD_HEADER_SIZE + dataSize)) break;

    readAddr   += dataSize;
    bytesSent  += dataSize;
    chunkCount += 1;

    // Progress logging
    uint32_t sentKB = bytesSent / 1024;
    if (sentKB >= lastProgress + OFFLOAD_PROGRESS_KB) {
      lastProgress = sentKB;
      Serial.print("[OFFLOAD] ");
      Serial.print(sentKB);
      Serial.print(" / ");
      Serial.print(totalBytes / 1024);
      Serial.println(" KB");
    }

    delay(OFFLOAD_PACING_MS);
  }

  uint32_t elapsedMs = millis() - tStart;
  if (elapsedMs == 0) elapsedMs = 1;   // guard divide-by-zero on tiny transfers

  // Restore logging state
  logging = wasLogging;

  // Real throughput + per-chunk pace. Per-chunk ms ≈ the effective connection
  // interval (one notification is delivered per connection event), so this line
  // is also a live readout of what interval the central actually negotiated.
  uint32_t kbps_x100 = (uint32_t)(((uint64_t)bytesSent * 100000ULL) / elapsedMs / 1024);
  Serial.print("[OFFLOAD] Done — ");
  Serial.print(bytesSent);
  Serial.print(" bytes / ");
  Serial.print(chunkCount);
  Serial.print(" chunks in ");
  Serial.print(elapsedMs);
  Serial.print(" ms  (");
  Serial.print(kbps_x100 / 100); Serial.print('.'); Serial.print(kbps_x100 % 100);
  Serial.print(" KB/s, ");
  Serial.print((float)elapsedMs / (chunkCount ? chunkCount : 1), 1);
  Serial.print(" ms/chunk — pacing floor ");
  Serial.print(OFFLOAD_PACING_MS);
  Serial.println(" ms)");
}

// ---------------------------------------------------------------------------
// offloadRange(central, startOff, length) — re-send only a byte range of the
// log, for host gap recovery (control cmd 0x06). Same per-notification framing
// as offloadLog but NO total-header: the host already knows the total and is
// just filling holes a dropped notification left. Because whole-log re-offloads
// recreate the same fast burst — and so the same central-side drops (a chunk
// can go missing on every pass) — re-requesting just the missing bytes sends a
// small, non-bursty transfer the central can actually absorb. startOff/length
// are clamped to the logged region. IDLE-only (enforced by handleControl).
// ---------------------------------------------------------------------------
void offloadRange(BLEDevice& central, uint32_t startOff, uint32_t length) {
  uint32_t logBytes = (writeAddr > LOG_DATA_START) ? (writeAddr - LOG_DATA_START) : 0;
  if (startOff >= logBytes || length == 0) return;
  if (startOff + length > logBytes) length = logBytes - startOff;

  bool wasLogging = logging;
  logging = false;

  Serial.print("[OFFLOAD] range resend — offset ");
  Serial.print(startOff);
  Serial.print(" len ");
  Serial.println(length);

  alignas(4) uint8_t chunk[OFFLOAD_CHUNK_SIZE];   // QSPI EasyDMA: 4-byte aligned
  uint32_t readAddr = LOG_DATA_START + startOff;
  uint32_t endAddr  = LOG_DATA_START + startOff + length;

  while (central.connected() && readAddr < endAddr) {
    uint32_t off      = readAddr - LOG_DATA_START;   // absolute payload offset
    uint32_t remain   = endAddr - readAddr;
    size_t   dataSize = (remain >= OFFLOAD_DATA_SIZE) ? OFFLOAD_DATA_SIZE : remain;

    memcpy(chunk, &off, OFFLOAD_HEADER_SIZE);
    if (!qspiRead(readAddr, chunk + OFFLOAD_HEADER_SIZE, dataSize)) break;
    if (!offloadSend(central, chunk, OFFLOAD_HEADER_SIZE + dataSize)) break;
    readAddr += dataSize;
    delay(OFFLOAD_PACING_MS);
  }

  logging = wasLogging;
}

// ---------------------------------------------------------------------------
// handleControl(central) — dispatches BLE control commands.
//   0x00  stop BLE streaming (debug)
//   0x01  start BLE streaming (debug)
//   0x02  time sync (5-byte payload: cmd + 4-byte Unix epoch)
//   0x03  erase flash log
//   0x04  begin log offload (IDLE only)
//   0x06  re-send a byte range (IDLE only): [0x06, offset u32 LE, length u32 LE]
// ---------------------------------------------------------------------------
void handleControl(BLEDevice& central) {
  uint8_t cmd = ctrlChar.value()[0];
  switch (cmd) {
    case 0x01:
      streaming = true;
      Serial.println("[CTRL] BLE streaming START (debug)");
      break;

    case 0x00:
      streaming = false;
      Serial.println("[CTRL] BLE streaming STOP");
      break;

    case 0x02: {
      // Time sync (seconds, legacy): [0x02, epoch_b0, epoch_b1, epoch_b2, epoch_b3]
      if (ctrlChar.valueLength() >= 5) {
        const uint8_t* val = ctrlChar.value();
        memcpy(&syncEpoch, val + 1, 4);
        syncMillis  = millis();
        syncEpochMs = (uint64_t)syncEpoch * 1000ULL;   // keep ms mapping consistent
        timeSynced  = true;
        updateSyncInfo();
        Serial.print("[CTRL] Time sync (s) — epoch: ");
        Serial.print(syncEpoch);
        Serial.print("  millis: ");
        Serial.println(syncMillis);
      } else {
        Serial.println("[CTRL] Time sync — missing epoch payload (need 5 bytes)");
      }
      break;
    }

    case 0x05: {
      // Time sync (ms): [0x05, epoch_ms_b0 .. epoch_ms_b7]  (uint64 LE)
      // Millisecond resolution for multi-node alignment. syncMillis is
      // captured as close to receipt as possible; the central pairs this with
      // the send time to bound the residual sync error.
      if (ctrlChar.valueLength() >= 9) {
        const uint8_t* val = ctrlChar.value();
        syncMillis = millis();
        memcpy(&syncEpochMs, val + 1, 8);
        syncEpoch  = (uint32_t)(syncEpochMs / 1000ULL);
        timeSynced = true;
        updateSyncInfo();
        Serial.print("[CTRL] Time sync (ms) — epoch_ms: ");
        Serial.print((uint32_t)(syncEpochMs / 1000ULL));   // seconds part (printable)
        Serial.print("  millis: ");
        Serial.println(syncMillis);
      } else {
        Serial.println("[CTRL] Time sync (ms) — missing payload (need 9 bytes)");
      }
      break;
    }

    case 0x03:
      if (qspiReady) {
        eraseLog();
        timeSynced  = false;   // Sync epoch meaningless after log erase
        syncEpochMs = 0;
        updateSyncInfo();
      }
      break;

    case 0x04:
      // Enforce IDLE-only offload — protects against flash read/write
      // conflicts and ensures patient isn't losing motion data during sync.
      if (currentState != STATE_IDLE) {
        Serial.println("[CTRL] Offload REJECTED — device not in IDLE state");
        break;
      }
      if (qspiReady) offloadLog(central);
      break;

    case 0x06: {
      // Re-send a byte range for host gap recovery (IDLE only):
      //   [0x06, offset u32 LE, length u32 LE]
      if (currentState != STATE_IDLE) {
        Serial.println("[CTRL] Range resend REJECTED — device not in IDLE state");
        break;
      }
      if (ctrlChar.valueLength() >= 9 && qspiReady) {
        const uint8_t* val = ctrlChar.value();
        uint32_t offset, length;
        memcpy(&offset, val + 1, 4);
        memcpy(&length, val + 5, 4);
        offloadRange(central, offset, length);
      } else {
        Serial.println("[CTRL] Range resend — missing payload (need 9 bytes)");
      }
      break;
    }

    default:
      Serial.print("[CTRL] Unknown: 0x");
      Serial.println(cmd, HEX);
      break;
  }
  updateStatus();
}


// =============================================================================
// SECTION 13 — ISR
// =============================================================================

void onBNOInterrupt() {
  imuDataReady = true;
}


// =============================================================================
// SECTION 14 — Utility Helpers
// =============================================================================

// Motion = the classifier's genuine MOTION(4) value (>= keeps it robust if the
// library ever returns a higher code). Deliberately does NOT count UNKNOWN(0):
// the classifier emits Unknown right after it's enabled — i.e. on IDLE entry —
// so treating it as motion would bounce IDLE straight back to ACTIVE and it
// could never rest. Real motion onset reports MOTION(4), so it's still caught.
bool isMotion(uint8_t s) {
  return s >= STABILITY_MOTION;
}

void waitForIMUData() {
  if (!bleConnected) {
    // No BLE central — safe to sleep between INT pulses.
    //
    // The BNO INT line is LEVEL-meaningful (LOW = data waiting), but our ISR
    // is edge-triggered (FALLING). If a new report becomes ready while INT is
    // already asserted — e.g. it arrives during/just after the caller's
    // getSensorEvent() drain, before the line has risen — no fresh falling
    // edge is produced, imuDataReady is never set, and __WFE() would sleep
    // straight through the pending data. The BNO then retries (its ~10ms INT
    // timeout) and, after being ignored long enough, its internal watchdog
    // reboots the part (~6.6s) — the periodic reset that spikes idle current.
    //
    // Guard against the missed edge by checking the pin level directly: never
    // sleep while INT is already LOW, and treat a LOW level as a wake even if
    // the edge ISR didn't fire.
    if (digitalRead(BNO_INT_PIN) == LOW) {
      imuDataReady = false;
      return;                       // data already pending — go read it now
    }
    __SEV();
    __WFE();
    while (!imuDataReady && digitalRead(BNO_INT_PIN) == HIGH) {
      __WFE();
    }
    imuDataReady = false;
  } else {
    // BLE central connected — poll with timeout.
    // In IDLE, BNO INT events are sparse (~6.5s between SHTP watchdog
    // resets). Without a timeout we'd block here for seconds, starving
    // loop() and preventing BLE control command dispatch.
    // 100ms keeps the loop responsive while still catching INT edges.
    uint32_t start = millis();
    while (!imuDataReady && (millis() - start < 100)) {
      BLE.poll();
    }
    if (imuDataReady) {
      imuDataReady = false;
    }
    // On timeout: imuDataReady stays false. Caller's getSensorEvent()
    // returns false → handler exits cleanly back to loop().
  }
}


// =============================================================================
// SECTION 15 — BNO086 Mode Switching
// =============================================================================

// Enables the report that drives IDLE. Kept in one place so configureBNO_Idle()
// and handleIdle()'s reset-recovery path stay in sync; re-armed on every IDLE
// entry. See Section 3 (IDLE_WAKE_SOURCE) for the detector-vs-classifier choice.
void enableIdleReports() {
#if IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER
  // Reset-free path: the Stability Classifier (0x13) runs the MotionEngine
  // (accel+gyro), which keeps the hub active so it never enters the ~6.6s
  // self-rebooting idle state. It streams its classification (ON_TABLE /
  // STATIONARY / STABLE / MOTION) at IDLE_STABILITY_MS; handleIdle() wakes into
  // ACTIVE on MOTION.
  imu.enableStabilityClassifier(IDLE_STABILITY_MS);
#else
  // Accel-only wake (DETECTOR / SIGMOTION), hub awake: host stays in System-ON
  // (__WFE) sleep and wakes on the sensor's INT pulse.
  imu.enableReport(IDLE_WAKE_SENSOR_ID, IDLE_DETECTOR_INTERVAL_MS);    // ms!
#endif
}

void configureBNO_Idle() {
  LOGF("BNO: soft reset → IDLE mode (%s wake)", IDLE_WAKE_NAME);
  imu.softReset();
  delay(150);

  enableIdleReports();
  bnoRunningCfg = BNO_CFG_NONE;

  imu.wasReset();

  LOGF("BNO: IDLE %s armed as wake source", IDLE_WAKE_NAME);
}

// Configure the BNO for a RUNNING state. ACTIVE uses the fast fusion rate,
// STATIC the slow one; the classifier is identical in both. The delay(50) after
// each enableReport lets the SH-2 firmware ack before the next command, avoiding
// the buffer-overload/premature-reset seen without it (SparkFun issue #2).
void configureBNO_Running(BnoRunningCfg desired) {
  const char* name = (desired == BNO_CFG_STATIC) ? "STATIC" : "ACTIVE";

  if (bnoRunningCfg == desired) {
    LOGF("BNO: already in %s running mode — skipping reconfiguration", name);
    return;
  }

  uint16_t rvInterval = (desired == BNO_CFG_STATIC) ? BNO_RV_STATIC_INTERVAL_MS
                                                    : BNO_RV_INTERVAL_MS;
  enableFusionVector(rvInterval);   // RV or GRV — see Section 3b
  delay(50);

  if (bnoRunningCfg == BNO_CFG_NONE) {
    // From IDLE / post-reset the classifier isn't armed either; on an
    // ACTIVE↔STATIC switch it's already running at the same rate, so skip it.
    LOGF("BNO: configuring %s (%s @ %dms, Classifier @ %dms)",
         name, FUSION_NAME, rvInterval, ACTIVE_STABILITY_MS);
    imu.enableStabilityClassifier(ACTIVE_STABILITY_MS);
    delay(50);
  } else {
    LOGF("BNO: switching to %s fusion rate (%s @ %dms)", name, FUSION_NAME, rvInterval);
  }

  bnoRunningCfg = desired;
}


// =============================================================================
// SECTION 16 — State Transitions
// =============================================================================

void requestTransition(SystemState newState) {
  if (newState != currentState) {
    pendingState = newState;
  }
}

void applyPendingTransition() {
  if (pendingState == currentState) return;

  LOGF("STATE: %s → %s", stateName(currentState), stateName(pendingState));

  // Every transition writes the flash header (and RUNNING then logs), so wake the
  // flash first; the IDLE case re-parks it below. Persisting here also keeps the
  // write pointer safe if power is lost right after a transition.
  qspiWake();
  if (qspiReady) {
    saveHeader();
  }

  currentState = pendingState;

  switch (currentState) {
    case STATE_IDLE:
      configureBNO_Idle();
      // Offload is IDLE-only, so advertise only in IDLE (skip if connected).
      if (bleReady && !bleConnected) BLE.advertise();
      // IDLE never touches flash (a connecting central wakes it) — park it.
      if (!bleConnected) qspiSleep();
      break;
    case STATE_STATIC_POSTURE:
      configureBNO_Running(BNO_CFG_STATIC);
      if (bleReady) BLE.stopAdvertise();   // no offload outside IDLE
      break;
    case STATE_ACTIVE_RECORDING:
      configureBNO_Running(BNO_CFG_ACTIVE);
      if (bleReady) BLE.stopAdvertise();   // no offload outside IDLE
      break;
  }

  lastStabilityEvent = millis();
  consecutiveResets  = 0;
}


// =============================================================================
// SECTION 17 — Watchdog
// =============================================================================

void checkWatchdog() {
  if (currentState == STATE_IDLE) return;

  uint32_t elapsed = millis() - lastStabilityEvent;

  if (elapsed <= RUNNING_WATCHDOG_MS) return;

  consecutiveResets++;
  LOGF("WATCHDOG: no stability event in %lums — resetting BNO (attempt %d/%d)",
       elapsed, consecutiveResets, WATCHDOG_MAX_RESETS);

  if (consecutiveResets >= WATCHDOG_MAX_RESETS) {
    LOGF("WATCHDOG: %d consecutive resets with no recovery → forcing IDLE",
         WATCHDOG_MAX_RESETS);
    pendingState = STATE_IDLE;
    return;
  }

  imu.softReset();
  delay(150);
  // Soft reset cleared every report — re-arm from scratch for the current state.
  bnoRunningCfg = BNO_CFG_NONE;
  configureBNO_Running(currentState == STATE_STATIC_POSTURE ? BNO_CFG_STATIC
                                                            : BNO_CFG_ACTIVE);

  lastStabilityEvent = millis();
}


// =============================================================================
// SECTION 18 — Quaternion Sample Writer
//
// MODIFIED from Phase 3: now packs the 20-byte record buffer and calls
// writeLogRecord() from Phase 2b. Watermark-based activeHz throttling
// is derived from writeAddr instead of the old flashBytesUsed counter.
// =============================================================================

void writeQuaternionSample(float qi, float qj, float qk, float qr) {
  // ── Drop invalid quaternions ──
  // A valid orientation quaternion is unit-length (|q|^2 == 1). Mis-parsed
  // zero-length SHTP packets (SparkFun issue #21, unfixed as of lib v1.0.6) can
  // surface as garbage values; drop anything far from unit length so corrupt
  // readings aren't logged as real motion.
  float qmag2 = qi * qi + qj * qj + qk * qk + qr * qr;
  if (qmag2 < 0.5f || qmag2 > 1.5f) {
    LOGF("QUAT: dropped invalid sample (|q|^2=%.3f)", qmag2);
    return;
  }

  // ── Flash full → force IDLE ──
  if (!logging && qspiReady) {
    LOGF("FLASH: full — forcing IDLE to conserve power");
    requestTransition(STATE_IDLE);
    return;
  }

  // ── Watermark throttle — reduce write rate as flash fills ──
  if (qspiReady) {
    uint32_t usedBytes = writeAddr - LOG_DATA_START;
    if (usedBytes > LOG_WATERMARK_BYTES && activeHz > 2) {
      uint8_t prevHz = activeHz;
      activeHz = max(2, activeHz / 2);
      LOGF("FLASH: watermark — reducing write rate %dHz → %dHz", prevHz, activeHz);
    }
  }

  // ── Pack 20-byte record (matches Phase 2b format) ──
  // Layout: [ts:4][qw:4][qx:4][qy:4][qz:4]
  uint32_t ts = millis();
  memcpy(pktBuf,      &ts, 4);    // timestamp
  memcpy(pktBuf +  4, &qr, 4);    // quat_w (real)
  memcpy(pktBuf +  8, &qi, 4);    // quat_x (i)
  memcpy(pktBuf + 12, &qj, 4);    // quat_y (j)
  memcpy(pktBuf + 16, &qk, 4);    // quat_z (k)

  // ── Write to flash ──
  if (qspiReady) {
    if (!writeLogRecord(pktBuf, BYTES_PER_SAMPLE)) {
      // writeLogRecord returned false — flash full or unavailable
      LOGF("FLASH: write failed — forcing IDLE");
      requestTransition(STATE_IDLE);
      return;
    }
  }

  // ── BLE live stream (if enabled via 0x01 command) ──
  if (streaming && bleConnected) {
    quatChar.writeValue(pktBuf, 20);
  }

  LOGF("QUAT: w=%.4f x=%.4f y=%.4f z=%.4f | addr: 0x%lX",
       qr, qi, qj, qk, writeAddr);
}


// =============================================================================
// SECTION 19 — Stability Change Logger
// =============================================================================

void logStabilityIfChanged(uint8_t stability) {
  if (stability != lastLoggedStability) {
    LOGF("STABILITY: %s (%d)", stabilityName(stability), stability);
    lastLoggedStability = stability;
  }
}


// =============================================================================
// SECTION 20 — State: IDLE
// =============================================================================

void handleIdle() {
  if (imu.wasReset()) {
    // Periodic idle self-reboot (inherent — see Section 3). Re-arm the
    // detector and drain the post-reset advertisement the hub emits on boot.
    // Log ms-since-last-reset. The detector's inherent idle self-reset runs
    // ~6.7s apart (benign — IDLE logs nothing and re-arms); a useful field
    // diagnostic if that cadence ever changes.
    static uint32_t lastIdleResetMs = 0;
    uint32_t nowReset = millis();
    if (lastIdleResetMs) {
      LOGF("IDLE: BNO reset — %lums since last reset — re-arming detector",
           nowReset - lastIdleResetMs);
    } else {
      LOGF("IDLE: BNO reset (first) — re-arming detector");
    }
    lastIdleResetMs = nowReset;
    bnoRunningCfg = BNO_CFG_NONE;
    enableIdleReports();

    delay(50);
    while (imu.getSensorEvent()) { }   // drain post-reset advertisement

    imu.wasReset();
    imuDataReady = false;
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id = imu.getSensorEventID();

#if IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER
    // Stability Classifier drives IDLE: it streams ON_TABLE / STATIONARY /
    // STABLE / MOTION. MOTION = patient moving → wake into ACTIVE_RECORDING.
    // This is the SAME motion test STATIC_POSTURE / ACTIVE_RECORDING use.
    if (id == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
      lastStabilityEvent = millis();
      consecutiveResets  = 0;

      uint8_t s = imu.getStabilityClassifier();
      logStabilityIfChanged(s);
      if (isMotion(s)) {
        LOGF("CLASSIFIER: MOTION — patient moving → ACTIVE_RECORDING");
        activeHz         = DEFAULT_ACTIVE_HZ;
        lastMotionTime   = millis();
        onTableStartTime = 0;
        lastActiveSample = 0;
        requestTransition(STATE_ACTIVE_RECORDING);
        return;
      }
    }
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_SIGMOTION
    // Significant Motion (0x12) is a ONE-SHOT wake event: its mere arrival means
    // motion started, so there's no value to decode. The sensor auto-disables
    // after firing; enableIdleReports() re-arms it on the next IDLE entry/reset.
    if (id == SH2_SIGNIFICANT_MOTION) {
      lastStabilityEvent = millis();
      consecutiveResets  = 0;
      LOGF("SIGMOTION: significant motion → ACTIVE_RECORDING");
      activeHz         = DEFAULT_ACTIVE_HZ;
      lastMotionTime   = millis();
      onTableStartTime = 0;
      lastActiveSample = 0;
      requestTransition(STATE_ACTIVE_RECORDING);
      return;
    }
#else
    if (id == SH2_STABILITY_DETECTOR) {
      lastStabilityEvent = millis();
      consecutiveResets  = 0;

      // Detector reports enter/exit stability; the SparkFun lib exposes the
      // value through getStabilityClassifier(). EXITED = stability broken =
      // motion started → wake into ACTIVE_RECORDING. (ENTERED is ignored.)
      uint8_t val = imu.getStabilityClassifier();

      // DIAGNOSTIC: print the raw value on EVERY 0x1C event (not just EXITED),
      // so a bench shake shows whether val ever reaches DETECTOR_EXITED(2). If a
      // 0x1C event reports a steady non-2 value even while shaking, the detector
      // isn't delivering the EXITED edge on the channel we read.
      LOGF("DETECTOR: 0x1C val=%u (ENTERED=1 EXITED=2)", val);

      if (val == DETECTOR_EXITED) {
        LOGF("DETECTOR: EXITED — patient moving → ACTIVE_RECORDING");
        activeHz         = DEFAULT_ACTIVE_HZ;
        lastMotionTime   = millis();
        onTableStartTime = 0;
        lastActiveSample = 0;
        requestTransition(STATE_ACTIVE_RECORDING);
        return;
      }
    }
#endif
  }
}


// =============================================================================
// SECTION 21 — State: STATIC_POSTURE
// =============================================================================

void handleStaticPosture() {
  if (imu.wasReset()) {
    LOGF("IMU: reset in STATIC_POSTURE — reason: %u — reconfiguring", (unsigned)imu.getResetReason());
    bnoRunningCfg = BNO_CFG_NONE;
    configureBNO_Running(BNO_CFG_STATIC);
    lastStabilityEvent = millis();
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id  = imu.getSensorEventID();
    uint32_t now = millis();

    if (id == FUSION_REPORT_ID) {
      bool timeToSample = (now - lastStaticSample >= STATIC_SAMPLE_INTERVAL_MS);
      if (timeToSample) {
        writeQuaternionSample(
          fusionQuatI(), fusionQuatJ(),
          fusionQuatK(), fusionQuatReal()
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
        LOGF("TIMER: no motion for %lums → IDLE", now - lastMotionTime);
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
          LOGF("TIMER: ON_TABLE sustained %lums → IDLE", now - onTableStartTime);
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
// SECTION 22 — State: ACTIVE_RECORDING
// =============================================================================

void handleActiveRecording() {
  if (imu.wasReset()) {
    LOGF("IMU: reset in ACTIVE_RECORDING — reason: %u — reconfiguring", (unsigned)imu.getResetReason());
    bnoRunningCfg = BNO_CFG_NONE;
    configureBNO_Running(BNO_CFG_ACTIVE);
    lastStabilityEvent = millis();
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id   = imu.getSensorEventID();
    uint32_t now = millis();

    if (id == FUSION_REPORT_ID) {
      bool timeToSample = (now - lastActiveSample >= (1000u / activeHz));
      if (timeToSample) {
        writeQuaternionSample(
          fusionQuatI(), fusionQuatJ(),
          fusionQuatK(), fusionQuatReal()
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
          LOGF("TIMER: no motion for %lums → STATIC_POSTURE",
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
            LOGF("TIMER: ON_TABLE sustained %lums → STATIC_POSTURE",
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
// SECTION 22b — I2C Bus Recovery
// =============================================================================
// If the BNO086 resets in the middle of an I2C read (see the periodic idle
// reset investigated in state_machine_test/FINDINGS.md), it can be left holding
// SDA low, waiting for clock pulses that never come. The nRF52840 then cannot
// issue a START (START requires SDA high first), so every transfer fails — a
// permanent hang until power-cycle, the worst outcome for an unattended logger.
// Clear it the standard way: bit-bang up to 9 SCL pulses to clock the stuck
// slave out of its byte, then a manual STOP. Safe to call before Wire.begin();
// a no-op if the bus is already free. Uses INPUT_PULLUP to release lines high
// (open-drain) and OUTPUT-LOW to drive them low.
void i2cBusRecover() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);

  if (digitalRead(SDA) == HIGH) return;   // bus already free — nothing to do

  LOG("[I2C] SDA stuck low — clocking bus recovery");
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);              // drive clock low
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);          // release — pull-up brings it high
    delayMicroseconds(5);
  }

  // Manual STOP condition: SDA rising while SCL is high.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);            // SCL high
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);            // SDA released high => STOP
  delayMicroseconds(5);

  LOG("[I2C] bus recovery complete");
}


// =============================================================================
// SECTION 23 — setup()
// =============================================================================

void setup() {
  Serial.begin(115200);

  LOG("=== HULC Motion Shirt — Phase 3 + Flash ===");
  LOG("Initialising...");

  // Enable the nRF52840 internal DC/DC (buck) regulator — more efficient than
  // the default LDO in every state (the XIAO populates the REG1 inductor). Raw
  // register write; the mbed/Cordio stack doesn't manage it. See
  // firmware/POWER_OPTIMIZATION.md.
  NRF_POWER->DCDCEN = 1;

  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED,  OUTPUT);
  digitalWrite(PIN_LED_BLUE, HIGH);   // LEDs are active-low on XIAO
  digitalWrite(PIN_LED_RED,  HIGH);

  // ── External QSPI flash (init BEFORE IMU so flash errors are visible) ──
  Serial.print("[INIT] Flash (nrfx_qspi)... ");
  if (initQSPI() && loadHeader()) {
    qspiReady = true;
    Serial.print("OK  (existing records: ");
    Serial.print(writeCount);
    Serial.print("  writeAddr: 0x");
    Serial.print(writeAddr, HEX);
    Serial.println(")");
  } else {
    Serial.println("FAILED — logging disabled, state machine runs without flash");
  }

  // ── IMU ──
  pinMode(BNO_INT_PIN, INPUT_PULLUP);
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, HIGH);

  i2cBusRecover();   // clear a bus left stuck by a prior mid-transaction reset
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

  // ── Initialize timestamps ──
  lastMotionTime     = 0;
  lastStabilityEvent = millis();

  configureBNO_Idle();
  currentState = STATE_IDLE;
  pendingState = STATE_IDLE;

  // ── BLE (from Phase 2b) ──
  Serial.print("[INIT] BLE... ");
  if (!BLE.begin()) {
    Serial.println("FAILED — BLE disabled, state machine runs without sync");
  } else {
    bleReady = true;
    makeDeviceName();                    // unique per-board name (multi-node)
    BLE.setLocalName(g_deviceName);
    BLE.setDeviceName(g_deviceName);

    // Request a FAST connection interval (15–30 ms). Units are 1.25 ms, so
    // 12 = 15 ms and 24 = 30 ms. Without this the central often negotiates a
    // slow interval (hundreds of ms), which floored A005 read latency at
    // ~850 ms on a Windows host and throttles log offload throughput.
    //
    // 15 ms is the floor deliberately: Apple's Bluetooth Design Guidelines
    // require a requested Interval Min >= 15 ms (and a multiple of 15 ms), or
    // iOS rejects the connection-parameter update and falls back to its slow
    // default. 15 ms keeps this request honorable by iOS, Android, and BlueZ
    // alike. The central still has final say — this is a request, not a
    // guarantee. Must be set before advertise().
    BLE.setConnectionInterval(12, 24);
    BLE.setAdvertisingInterval(ADV_INTERVAL_UNITS);   // slow advertising to save radio power
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(quatChar);
    imuService.addCharacteristic(ctrlChar);
    imuService.addCharacteristic(statChar);
    imuService.addCharacteristic(offloadChar);
    imuService.addCharacteristic(syncInfoChar);
    BLE.addService(imuService);

    syncInfoChar.setEventHandler(BLERead, onSyncInfoRead);

    updateStatus();
    updateSyncInfo();
    BLE.advertise();

    Serial.println("OK");
    Serial.print("[BLE] Advertising as ");
    Serial.println(g_deviceName);
  }

  // Boot state is unconnected IDLE — park the external flash until it's needed.
  qspiSleep();

  LOG("Setup complete. State machine running — waiting for movement...\n");
}


// =============================================================================
// SECTION 24 — loop()
// =============================================================================

void loop() {
  // ── BLE connection management ──
  // Check at top of every loop iteration. ArduinoBLE's central() returns
  // the currently connected device (or an empty BLEDevice if none).
  BLEDevice central = BLE.central();

  if (central && central.connected() && !bleConnected) {
    // ── New connection ──
    bleConnected = true;
    Serial.print("[BLE] Connected: ");
    Serial.println(central.address());
    digitalWrite(PIN_LED_BLUE, LOW);    // Blue LED on = connected
    qspiWake();                         // offload/erase commands read/write flash
    updateStatus();
  }

  if (bleConnected && (!central || !central.connected())) {
    // ── Disconnection ──
    bleConnected = false;
    streaming    = false;
    Serial.println("[BLE] Disconnected");

    // Persist write pointer immediately — if device loses power after
    // disconnect, log resumes from the correct position on next boot.
    if (qspiReady) {
      saveHeader();
      Serial.println("[QSPI] Header saved on disconnect");
    }

    digitalWrite(PIN_LED_BLUE, HIGH);   // Blue LED off
    updateStatus();
    // Advertising is IDLE-gated; only re-advertise (and re-park the flash) when
    // in IDLE. Disconnected mid-recording stays dark until IDLE is re-entered.
    if (currentState == STATE_IDLE) {
      BLE.advertise();
      Serial.println("[BLE] Re-advertising...");
      qspiSleep();
    }
  }

  // ── BLE control command dispatch ──
  if (bleConnected && central && ctrlChar.written()) {
    handleControl(central);
  }

  // ── State machine (runs regardless of BLE state) ──
  applyPendingTransition();

  checkWatchdog();

  // Re-check in case watchdog forced a transition
  applyPendingTransition();

  switch (currentState) {
    case STATE_IDLE:             handleIdle();            break;
    case STATE_STATIC_POSTURE:   handleStaticPosture();   break;
    case STATE_ACTIVE_RECORDING: handleActiveRecording(); break;
  }
}
