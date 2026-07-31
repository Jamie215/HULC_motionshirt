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
// States (unchanged from Phase 3):
//   IDLE             — Significant Motion (0x12) wake only. One-shot low-power
//                      detector; BNO is silent until real motion. See Section 3.
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

// Significant Motion (0x12) — the IDLE wake source.
//
// This is CEVA's implementation of the Android SIGNIFICANT_MOTION sensor: a
// one-shot, low-power, self-disarming wake detector. When armed it runs only a
// low-rate accelerometer + a lightweight motion algorithm (no gyro, no fusion),
// emits a SINGLE report when significant motion is detected, then disables
// itself — so IDLE stays completely silent (no periodic reports, no periodic
// INT edges) until the patient actually moves. We re-arm it on every IDLE entry.
//
// WHY THIS OVER THE CLASSIFIER (0x13) IN IDLE:
//   The classifier emits a report every IDLE_STABILITY_MS (~1s), i.e. periodic
//   INT traffic even at rest. SigMotion is silent until motion, which both
//   (a) removes the periodic-idle-reporting behavior that the ~6.9s self-reset
//   fed on (see waitForIMUData notes), and (b) should draw less than the
//   continuously-classifying 0x13.
//
// MEASURE BEFORE YOU TRUST THE POWER CLAIM:
//   0x13 (measured ~7mA) is ALSO accelerometer-based, so the SigMotion win may
//   be a couple mA (continuous-classify+report vs armed-but-silent), not the
//   large drop the "sleep" framing implies. Confirm the real IDLE delta with a
//   bench ammeter — the datasheet number is not a substitute. Note too that the
//   bulk of system idle current is the nRF52840 (BLE advertising + always-on
//   QSPI + System-ON sleep), which this change does not touch.
#ifndef SH2_SIGNIFICANT_MOTION
#define SH2_SIGNIFICANT_MOTION 0x12
#endif


// =============================================================================
// SECTION 4 — Sampling Rates
// =============================================================================

#define DEFAULT_ACTIVE_HZ           10
#define STATIC_SAMPLE_INTERVAL_MS   5000
#define BNO_RV_INTERVAL_MS          65
#define DETECTOR_INTERVAL_MS        1000
#define ACTIVE_STABILITY_MS         500

// IDLE now arms Significant Motion (0x12) as the sole wake source (see
// Section 3). This is the interval hint passed to enableReport() — for a
// one-shot detector it governs the algorithm's evaluation cadence, not a
// report rate (SigMotion does not stream). Left at the library default;
// tune during the bench power/latency test if needed. Larger = lower power
// but slower/less-sensitive motion-onset detection.
#define IDLE_SIGMOTION_INTERVAL_US  10000UL

// Retained: previously the IDLE Stability Classifier report interval. No
// longer used for IDLE, kept for reference / easy rollback to the 0x13 path.
#define IDLE_STABILITY_MS           1000


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

#define STABILITY_ON_TABLE    0
#define STABILITY_STATIONARY  1
#define STABILITY_STABLE      2
#define STABILITY_MOTION      3


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
// Not called in this firmware (no BLE yet), but included for completeness.
// Will be wired to BLE control command 0x03 in Step 2.
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

// Enables the report that drives IDLE. Kept in one place so
// configureBNO_Idle() and handleIdle()'s reset-recovery path stay in sync.
// See Section 3 for why IDLE arms Significant Motion (0x12) — a one-shot,
// low-power, self-disarming wake detector — instead of a streaming classifier.
// SigMotion has no dedicated SparkFun helper, so we arm it via the generic
// enableReport(). NOTE: enableReport() sets wakeupEnabled=false, so this pairs
// with the host staying in System-ON sleep (__WFE) and reading INT — NOT with
// modeSleep()/sh2_devSleep, which would need a true wake-channel sensor.
void enableIdleReports() {
  imu.enableReport(SH2_SIGNIFICANT_MOTION, IDLE_SIGMOTION_INTERVAL_US);
}

void configureBNO_Idle() {
  LOGF("BNO: soft reset → IDLE mode (Significant Motion 0x12 wake)");
  imu.softReset();
  delay(150);

  enableIdleReports();
  bnoInRunningMode = false;

  imu.wasReset();

  LOGF("BNO: IDLE Significant Motion (0x12) armed — silent until motion");
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
    // NOTE: getResetReason() returns prodIds.entry[0].resetCause, which the
    // SparkFun library captures ONCE at begin() and does not refresh on later
    // resets — so this prints the *boot* cause, not this reset's. Treat it as
    // a weak hint only; the IDLE_WAKE_SOURCE A/B test is the real probe.
    LOGF("IMU reset detected in IDLE — boot-cached reason: %u", (unsigned)imu.getResetReason());

    bnoInRunningMode = false;
    enableIdleReports();

    delay(50);
    while (imu.getSensorEvent()) {}
    imu.wasReset();
    imuDataReady = false;
  }

  waitForIMUData();

  while (imu.getSensorEvent()) {
    uint8_t id = imu.getSensorEventID();

    // IDLE is silent by design (SigMotion doesn't stream), so ANY event here is
    // notable. Print the raw report ID to confirm on-bench that SparkFun's
    // getSensorEvent() surfaces 0x12 via getSensorEventID() — the library has
    // no named parser for Significant Motion, so this is the verification hook.
    LOGF("IDLE: BNO event id=0x%02X", id);

    if (id == SH2_SIGNIFICANT_MOTION) {
      // The event itself is the signal — no payload needed. SigMotion has now
      // auto-disarmed; configureBNO_Running() takes over on the transition.
      LOGF("SIGMOTION: significant motion detected → ACTIVE_RECORDING");
      lastStabilityEvent = millis();
      consecutiveResets  = 0;
      activeHz           = DEFAULT_ACTIVE_HZ;
      lastMotionTime     = millis();
      onTableStartTime   = 0;
      lastActiveSample   = 0;
      requestTransition(STATE_ACTIVE_RECORDING);
      return;
    }
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
