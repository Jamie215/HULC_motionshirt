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
// BLE GATT layout:
//   Service  : IMU Motion Service         (UUID A0010000-...)
//   Char A001: Quaternion stream          NOTIFY  20 bytes   (debug only)
//   Char A002: Control                    WRITE    5 bytes
//   Char A003: Status                     READ     4 bytes
//   Char A004: Log offload                NOTIFY  200 bytes
//
// Status characteristic (A003) layout:
//   Byte 0  : Device state (0=IDLE, 1=STATIC_POSTURE, 2=ACTIVE_RECORDING)
//   Byte 1  : Flags (bit 0 = streaming, bit 1 = time synced)
//   Byte 2-3: Log size in KB (little-endian uint16)
//
// Control commands:
//   0x00  stop BLE streaming (debug)
//   0x01  start BLE streaming (debug)
//   0x02  time sync — payload: [0x02, epoch_b0, epoch_b1, epoch_b2, epoch_b3]
//         Firmware stores mapping between millis() and Unix epoch.
//         Records continue using raw millis(); phone applies correction.
//   0x03  erase flash log
//   0x04  begin log offload over A004 (IDLE only — rejected in other states)
//
// Offload protocol:
//   Phone connects → reads A003 → if state == IDLE, sends 0x02 (time sync)
//   then 0x04 (offload). Firmware streams flash in 200-byte chunks at 3ms
//   pacing. Phone confirms receipt, then sends 0x03 (erase).
//
// Connection behavior:
//   When no BLE central is connected, state handlers sleep via __WFE()
//   between BNO086 INT pulses (low power). When a central connects,
//   waitForIMUData() switches to BLE.poll() loop to keep the BLE stack
//   alive. State machine runs identically in both modes.
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
//   IDLE             — Selectable wake source (see Section 3, IDLE_WAKE_SOURCE):
//                      • CLASSIFIER (default) — Stability Classifier (0x13),
//                        accel+gyro / MotionEngine. Reset-FREE idle (fusion keeps
//                        the hub active) at the cost of higher idle current (gyro
//                        on, cannot hold devSleep). Motion = classifier == MOTION.
//                      • DETECTOR — Stability Detector (0x1C) wake + hub devSleep
//                        (~7mA). Lowest power but reboots ~6.6s while asleep
//                        (inherent) — see Section 3 and IDLE_USE_DEVSLEEP.
//   STATIC_POSTURE   — RV @ ~15Hz + Classifier, writes gated to 0.2Hz
//   ACTIVE_RECORDING — Same BNO config, writes gated to activeHz (10Hz default)
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

#define DEVICE_NAME    "HULC-IMU-01"
#define UUID_SERVICE   "A0010000-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_QUAT      "A0010001-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_CONTROL   "A0010002-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_STATUS    "A0010003-B0CE-4A4A-8F0B-0011223344FF"
#define UUID_OFFLOAD   "A0010004-B0CE-4A4A-8F0B-0011223344FF"

// Offload transfer tuning
#define OFFLOAD_CHUNK_SIZE    200    // bytes per BLE notification (up from 20)
#define OFFLOAD_PACING_MS     3      // ms delay between chunks (down from 10)
#define OFFLOAD_PROGRESS_KB   10     // print progress every N KB


// =============================================================================
// SECTION 3 — Stability Detector (0x1C)
// =============================================================================

#ifndef SH2_STABILITY_DETECTOR
#define SH2_STABILITY_DETECTOR 0x1C
#endif
#define DETECTOR_EXITED   2
#define DETECTOR_ENTERED  1

// ── BNO hub deep sleep (sh2_devSleep) ──────────────────────────────────────
// devSleep is REQUIRED for low power: without it the hub stays awake at ~22mA
// no matter which sensor is armed; with it the system drops to ~7mA. The cost
// is a periodic self-reboot while asleep (the BNO will not hold host-commanded
// devSleep with a wake sensor armed — it reboots on a config-dependent
// timeout). The armed sensor sets that timeout (Stability Detector ~6.6s).
// This reboot is INHERENT, not a bug we can code away; each
// one is a brief re-arm (no data is logged in IDLE), and it is already priced
// into the ~7mA average. Set to 0 only to fall back to the 22mA no-sleep path.
//
// AUTOMATIC per wake source: devSleep only applies to the DETECTOR. The
// CLASSIFIER runs the MotionEngine, which keeps the hub active and (per
// FINDINGS.md) cannot hold a host-commanded devSleep, so it is forced 0 there —
// the classifier's higher idle current is the known price of its reset-free idle.
//
// DIAGNOSTIC — DETECTOR_DIAG_NO_DEVSLEEP: set to 1 to run the DETECTOR build with
// the hub AWAKE (devSleep off). Purpose: test whether devSleep is what suppresses
// the detector's EXITED motion-wake. With devSleep the hub may deliver only the
// periodic heartbeat and coalesce the wake-channel change-event; running awake
// lets the detector emit change-events continuously. Shake in both settings and
// watch the per-event "DETECTOR: 0x1C val=" log (added in handleIdle): if it
// wakes awake but not asleep, devSleep is the blocker. No effect on the
// classifier build (never uses devSleep). Leave 0 for normal operation.
#define DETECTOR_DIAG_NO_DEVSLEEP  0

// ── IDLE wake source (compile-time A/B switch) ─────────────────────────────
// DETECTOR   — Stability Detector (0x1C), accelerometer only. Holds host
//              devSleep (~7mA idle) but self-reboots every ~6.6s (inherent —
//              a benign re-arm; IDLE logs nothing). Lowest power.
// CLASSIFIER — Stability Classifier (0x13), accel+gyro through the MotionEngine.
//              No idle reboot, but the running engine can't hold devSleep, so
//              idle current is higher. Reset-free middle ground; wakes IDLE on
//              the same MOTION classification STATIC_POSTURE/ACTIVE act on.
//
// See firmware/IDLE_WAKE_SOURCE.md and state_machine_test/FINDINGS.md for the
// full investigation. Flip IDLE_WAKE_SOURCE and reflash to A/B compare.
#define IDLE_WAKE_DETECTOR    0
#define IDLE_WAKE_CLASSIFIER  1
#define IDLE_WAKE_SOURCE      IDLE_WAKE_DETECTOR

#if   IDLE_WAKE_SOURCE == IDLE_WAKE_DETECTOR
  #define IDLE_WAKE_NAME "Stability Detector (0x1C)"
#elif IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER
  #define IDLE_WAKE_NAME "Stability Classifier (0x13)"
#else
  #error "IDLE_WAKE_SOURCE must be IDLE_WAKE_DETECTOR or IDLE_WAKE_CLASSIFIER"
#endif

#if IDLE_WAKE_SOURCE == IDLE_WAKE_DETECTOR && !DETECTOR_DIAG_NO_DEVSLEEP
  #define IDLE_USE_DEVSLEEP  1
#else
  #define IDLE_USE_DEVSLEEP  0
#endif

// Wake-sensor config flags for the devSleep path. Bench-established:
//   • alwaysOnEnabled MUST be 1 — "Sensor remains on in sleep state". With it 0
//     the hub collapses out of devSleep in ~2ms AND never wakes on motion, so
//     it is load-bearing, not the reboot cause (my earlier guess was backwards).
//   • wakeupEnabled = 1 — "Wake host on event": the detector's EXITED report is
//     delivered on the wake channel and pulls INT to wake the nRF.
// SparkFun's enableReport() hardcodes both false, so the devsleep path arms the
// sensor via sh2_setSensorConfig() directly (see enableIdleReports()).
#define IDLE_WAKE_WAKEUP_EN    1
#define IDLE_WAKE_ALWAYSON_EN  1


// =============================================================================
// SECTION 4 — Sampling Rates
// =============================================================================

#define DEFAULT_ACTIVE_HZ           10
#define STATIC_SAMPLE_INTERVAL_MS   5000
#define BNO_RV_INTERVAL_MS          65
#define DETECTOR_INTERVAL_MS        1000
#define ACTIVE_STABILITY_MS         500
// Classifier report interval when IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER.
// Also the worst-case IDLE->ACTIVE motion-onset latency for that build (the
// classifier reaches MOTION within one interval). Unused by the detector build.
#define IDLE_STABILITY_MS           1000

// IDLE arms the Stability DETECTOR (0x1C) as the wake source (see Section 3).
// Bench finding: the detector STREAMS a heartbeat report at this interval, and
// each one wakes the nRF — at 1s that kept IDLE at ~11mA (not the ~7mA target).
// Lengthening the interval cuts those wakeups. Unlike the SigMotion reboot
// cadence (which ignored interval), THIS heartbeat rate tracks the interval.
//
// CEILING: reportInterval_us is uint32 microseconds → hard max 4,294,967,295us
// ≈ 71.6 min. But there's a lower PRACTICAL cap: the hub reboots every ~5s and
// re-arms the detector, resetting the heartbeat timer — so any interval beyond
// the reboot period never fires (the reboot pre-empts it), and the ~5s reboot
// becomes the wakeup floor. Past ~10s, raising this does nothing for idle power.
//
// TRADEOFF TO VERIFY: if the detector only evaluates at the heartbeat (poll),
// a long interval also slows motion-onset detection. Watch the move→EXITED
// latency on the bench. 10s is chosen to sit just above the reboot interval.
#define IDLE_DETECTOR_INTERVAL_US   10000000UL   // 10s — see ceiling notes above


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
BLECharacteristic ctrlChar(UUID_CONTROL,    BLEWrite,    5);  // 1 cmd + 4 epoch
BLECharacteristic statChar(UUID_STATUS,     BLERead,     4);
BLECharacteristic offloadChar(UUID_OFFLOAD, BLENotify,  OFFLOAD_CHUNK_SIZE);
bool              bleConnected     = false;
bool              streaming        = false;

// ── Time Sync ──
uint32_t          syncEpoch        = 0;     // Unix epoch from phone
uint32_t          syncMillis       = 0;     // millis() at time of sync
bool              timeSynced       = false;

// ── State Machine ──
SystemState  currentState        = STATE_IDLE;
SystemState  pendingState        = STATE_IDLE;

uint8_t      activeHz            = DEFAULT_ACTIVE_HZ;

// ── QSPI Flash (from Phase 2b) ──
bool         qspiReady           = false;
bool         logging             = true;
uint32_t     writeAddr           = LOG_DATA_START;
uint32_t     writeCount          = 0;
uint8_t      pktBuf[20];          // Shared record packing buffer

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

bool qspiEraseSector(uint32_t addr) {
  nrfx_err_t err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
  if (err != NRFX_SUCCESS) {
    Serial.print("[QSPI] Erase failed at 0x");
    Serial.println(addr, HEX);
    return false;
  }
  return qspiWait();
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
  uint8_t  header[20]  = {0};
  uint32_t magic       = LOG_MAGIC;
  memcpy(header,     &magic,     4);
  memcpy(header + 4, &writeAddr, 4);
  qspiWrite(LOG_HEADER_ADDR, header, 20);

  // Read-back verify
  uint8_t  verify[20] = {0};
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
  uint8_t header[20] = {0};
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
  Serial.println("[QSPI] Erasing log — this takes ~30s...");

  for (uint32_t addr = 0; addr < QSPI_FLASH_SIZE; addr += 64 * 1024) {
    nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, addr);
    qspiWait();
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
// offloadLog(central) — streams all logged records to phone via A004.
//
// Uses 200-byte chunks at 3ms pacing (~20s for 888KB vs ~7min at 20B/10ms).
// Pauses flash logging during transfer. Does NOT erase afterward — the
// phone sends 0x03 explicitly after confirming receipt.
// Rejected if not in IDLE state (enforced by handleControl).
// ---------------------------------------------------------------------------
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
  uint32_t lastProgress = 0;
  uint8_t  chunk[OFFLOAD_CHUNK_SIZE];

  while (central.connected() && readAddr < writeAddr) {
    size_t remaining = writeAddr - readAddr;
    size_t chunkSize = (remaining >= OFFLOAD_CHUNK_SIZE) ? OFFLOAD_CHUNK_SIZE : remaining;

    if (!qspiRead(readAddr, chunk, chunkSize)) break;
    offloadChar.writeValue(chunk, chunkSize);

    readAddr  += chunkSize;
    bytesSent += chunkSize;

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

  // Restore logging state
  logging = wasLogging;

  Serial.print("[OFFLOAD] Done — ");
  Serial.print(bytesSent);
  Serial.print(" bytes sent in ");
  Serial.print((bytesSent / OFFLOAD_CHUNK_SIZE) * OFFLOAD_PACING_MS / 1000);
  Serial.println("s");
}

// ---------------------------------------------------------------------------
// handleControl(central) — dispatches BLE control commands.
//   0x00  stop BLE streaming (debug)
//   0x01  start BLE streaming (debug)
//   0x02  time sync (5-byte payload: cmd + 4-byte Unix epoch)
//   0x03  erase flash log
//   0x04  begin log offload (IDLE only)
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
      // Time sync: [0x02, epoch_b0, epoch_b1, epoch_b2, epoch_b3]
      if (ctrlChar.valueLength() >= 5) {
        const uint8_t* val = ctrlChar.value();
        memcpy(&syncEpoch, val + 1, 4);
        syncMillis = millis();
        timeSynced = true;
        Serial.print("[CTRL] Time sync — epoch: ");
        Serial.print(syncEpoch);
        Serial.print("  millis: ");
        Serial.println(syncMillis);
      } else {
        Serial.println("[CTRL] Time sync — missing epoch payload (need 5 bytes)");
      }
      break;
    }

    case 0x03:
      if (qspiReady) {
        eraseLog();
        timeSynced = false;   // Sync epoch meaningless after log erase
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
// entry. See Section 3 (IDLE_WAKE_SOURCE) for the detector-vs-classifier choice
// and what IDLE_USE_DEVSLEEP changes.
void enableIdleReports() {
#if IDLE_WAKE_SOURCE == IDLE_WAKE_CLASSIFIER
  // Reset-free path: the Stability Classifier (0x13) runs the MotionEngine
  // (accel+gyro), which keeps the hub active so it never enters the ~6.6s
  // self-rebooting idle state. It streams its classification (ON_TABLE /
  // STATIONARY / STABLE / MOTION) at IDLE_STABILITY_MS; handleIdle() wakes into
  // ACTIVE on MOTION. IDLE_USE_DEVSLEEP is forced 0 for this source (the running
  // MotionEngine can't hold devSleep), so no sh2 wake/always-on dance is needed.
  imu.enableStabilityClassifier(IDLE_STABILITY_MS);
#elif IDLE_USE_DEVSLEEP
  // Deep-sleep detector path: arm 0x1C as a WAKE + ALWAYS-ON sensor so it keeps
  // running while the hub is in devSleep and can pull INT to wake the host.
  // SparkFun's enableReport() forces both flags false, so configure the sh2
  // layer directly (sh2_setSensorConfig / sh2_SensorConfig_t come from sh2.h,
  // included by the library header). Falls back to the plain report if rejected.
  sh2_SensorConfig_t cfg = {};          // value-init: all fields zero/false
  cfg.wakeupEnabled     = IDLE_WAKE_WAKEUP_EN;    // "Wake host on event"
  cfg.alwaysOnEnabled   = IDLE_WAKE_ALWAYSON_EN;  // "Sensor remains on in sleep state"
  cfg.reportInterval_us = IDLE_DETECTOR_INTERVAL_US;
  int rc = sh2_setSensorConfig((sh2_SensorId_t)SH2_STABILITY_DETECTOR, &cfg);
  if (rc != SH2_OK) {
    LOGF("BNO: sh2_setSensorConfig(0x1C) FAILED rc=%d — using enableReport()", rc);
    imu.enableReport(SH2_STABILITY_DETECTOR, IDLE_DETECTOR_INTERVAL_US);
  }
#else
  // Detector without devSleep: host stays in System-ON (__WFE) sleep reading INT.
  imu.enableReport(SH2_STABILITY_DETECTOR, IDLE_DETECTOR_INTERVAL_US);
#endif
  idleArmedMs = millis();   // start-of-arm marker for the reset-hold stats
}

void configureBNO_Idle() {
  LOGF("BNO: soft reset → IDLE mode (%s wake)", IDLE_WAKE_NAME);
  imu.softReset();
  delay(150);

  enableIdleReports();
  bnoInRunningMode = false;

  imu.wasReset();

#if IDLE_USE_DEVSLEEP
  // Arm-then-sleep ordering matters: the wake sensor must be configured before
  // the hub sleeps. Let the feature config settle, then drop the hub.
  delay(20);
  if (imu.modeSleep()) {
    LOGF("BNO: devSleep engaged (wakeup=%d alwaysOn=%d) — hub asleep, 0x1C armed",
         IDLE_WAKE_WAKEUP_EN, IDLE_WAKE_ALWAYSON_EN);
  } else {
    LOGF("BNO: modeSleep() FAILED — detector running without devSleep");
  }
#endif

  LOGF("BNO: IDLE %s armed as wake source", IDLE_WAKE_NAME);
}

void configureBNO_Running() {
  if (bnoInRunningMode) {
    LOGF("BNO: already in running mode — skipping reconfiguration");
    return;
  }

  LOGF("BNO: configuring RUNNING mode (RV @ %dms, Classifier @ %dms)",
       BNO_RV_INTERVAL_MS, ACTIVE_STABILITY_MS);

  imu.enableRotationVector(BNO_RV_INTERVAL_MS);
  delay(50);   // let the SH-2 firmware ack each config frame before the next —
  imu.enableStabilityClassifier(ACTIVE_STABILITY_MS);   // avoids the enableReport
  delay(50);   // command-buffer overload / premature reset (SparkFun issue #2)
  bnoInRunningMode = true;
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

  // Persist flash header on any state transition — ensures write pointer
  // survives if the device loses power shortly after a transition.
  if (qspiReady) {
    saveHeader();
  }

  currentState = pendingState;

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
  bnoInRunningMode = false;
  configureBNO_Running();

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
    // Periodic devSleep self-reboot (inherent — see Section 3). Re-arm the
    // detector and drain the post-reset advertisement the hub emits on boot.
    LOGF("IDLE: BNO reset — re-arming detector");
    bnoInRunningMode = false;
    enableIdleReports();

    delay(50);
    while (imu.getSensorEvent()) { }   // drain post-reset advertisement

    imu.wasReset();
    imuDataReady = false;

#if IDLE_USE_DEVSLEEP
    // Reboot leaves the hub awake — re-enter devSleep so a rare idle reset
    // doesn't silently fall back to the higher-power (awake) detector path.
    delay(20);
    imu.modeSleep();
#endif
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id = imu.getSensorEventID();

    if (id == SH2_STABILITY_DETECTOR) {
      lastStabilityEvent = millis();
      consecutiveResets  = 0;

      // Detector reports enter/exit stability; the SparkFun lib exposes the
      // value through getStabilityClassifier(). EXITED = stability broken =
      // motion started → wake into ACTIVE_RECORDING. (ENTERED is ignored.)
      uint8_t val = imu.getStabilityClassifier();

      // DIAGNOSTIC: print the raw value on EVERY 0x1C event (not just EXITED),
      // so a bench shake shows whether val ever reaches DETECTOR_EXITED(2). If
      // the heartbeats stream a steady non-2 value even while shaking, the
      // detector isn't delivering the EXITED edge on the channel we read —
      // independent of devSleep. Compare with DETECTOR_DIAG_NO_DEVSLEEP flipped.
      LOGF("DETECTOR: 0x1C val=%u (ENTERED=1 EXITED=2)  devSleep=%d",
           val, IDLE_USE_DEVSLEEP);

      if (val == DETECTOR_EXITED) {
        LOGF("DETECTOR: EXITED — patient moving → ACTIVE_RECORDING");
#if IDLE_USE_DEVSLEEP
        // Hub was in devSleep — wake it before applyPendingTransition() runs
        // configureBNO_Running() (which issues enableRotationVector, etc.).
        imu.modeOn();
        delay(20);
#endif
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

  // Timeout instead of blocking forever — remove entirely before deployment
  unsigned long serialTimeout = millis();
  while (!Serial && (millis() - serialTimeout < 3000)) {
    delay(10);
  }

  LOG("=== HULC Motion Shirt — Phase 3 + Flash ===");
  LOG("Initialising...");

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
    BLE.setLocalName(DEVICE_NAME);
    BLE.setDeviceName(DEVICE_NAME);
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(quatChar);
    imuService.addCharacteristic(ctrlChar);
    imuService.addCharacteristic(statChar);
    imuService.addCharacteristic(offloadChar);
    BLE.addService(imuService);

    updateStatus();
    BLE.advertise();

    Serial.println("OK");
    Serial.print("[BLE] Advertising as ");
    Serial.println(DEVICE_NAME);
  }

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
    BLE.advertise();
    Serial.println("[BLE] Re-advertising...");
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
