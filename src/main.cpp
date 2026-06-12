#ifndef UNIT_TEST

#include <Arduino.h>
#include <EEPROM.h>
#include <RadioLib.h>
#include <SPI.h>
#include <pico/time.h>
#include <string.h>

#include "lora_link/protocol.h"
#include "lora_link/rf/scheduler.h"

using namespace lora_link;

#ifndef DEFAULT_BINDING_PHRASE
#define DEFAULT_BINDING_PHRASE "default"
#endif

#if !defined(LORA_TX_ROLE) && !defined(LORA_RX_ROLE)
#error "Select LORA_TX_ROLE or LORA_RX_ROLE"
#endif

static DeviceConfig g_config;
static uint8_t g_uid[kUidSize];
static uint32_t g_uidCheck;
static uint16_t g_sequence = 0;
static uint16_t g_hop = 0;
static uint32_t g_lastRcInputMs = 0;
static uint32_t g_lastUplinkMs = 0;
static uint32_t g_lastSyncMs = 0;
static uint32_t g_lastLinkStatsMs = 0;
static bool g_configFault = false;
static bool g_rxWaiting = false;
static volatile bool g_dio1 = false;
static uint16_t g_channels[kRcChannelCount];
static int16_t g_lastRssi = 0;
static int8_t g_lastSnr = 0;
static uint8_t g_linkQuality = 0;
static bool g_radioReady = false;
static bool g_bindingRequiresReboot = false;
static uint32_t g_lastRxHopAdvanceUs = 0;
static uint32_t g_validOtaFrames = 0;
static uint32_t g_rejectedOtaFrames = 0;
static constexpr uint16_t kRcSpikeJumpThreshold = 160;
static constexpr uint16_t kRcSpikeConfirmTolerance = 80;
static constexpr uint32_t kHandsetTelemetryIntervalMs = 250;
static constexpr uint32_t kDownlinkTelemetryFreshMs = 2000;
static constexpr uint32_t kTelemetryReplyGuardUs = 1500;
static constexpr uint8_t kAcquisitionSyncIntervalFrames = 4;
static constexpr uint8_t kStartupCliGraceLoops = 3;
static constexpr int8_t kBenchTxPowerDbm = 0;
static constexpr bool kBenchSingleFrequency = true;
static constexpr uint8_t kSx1280OpWriteRegister = 0x18;
static constexpr uint8_t kSx1280OpReadRegister = 0x19;
static constexpr uint16_t kSx1280RegRxGain = 0x0891;
static constexpr uint16_t kSx1280RegLoraSfConfig = 0x0925;
static constexpr uint16_t kSx1280RegFreqErrorComp = 0x093C;
static constexpr uint8_t kSx1280RxGainHighSensitivity = 0xC0;
static constexpr uint8_t kSx1280RxGainKeepMask = 0x3F;
static constexpr uint8_t kSx1280LoraSfConfigSf5Sf6 = 0x1E;
static constexpr uint8_t kSx1280LoraSfConfigSf7Sf8 = 0x37;
static constexpr uint8_t kSx1280LoraSfConfigSf9Sf12 = 0x32;
static constexpr uint8_t kSx1280FreqErrorCompOn = 0x01;
#if LORA_RX_ROLE
struct RfTimerIsrEvent {
    uint32_t timestampUs;
    uint8_t half;
};
static alarm_id_t g_rfAlarm = 0;
static bool g_rfTimerRunning = false;
static volatile uint8_t g_rfTimerWrite = 0;
static volatile uint8_t g_rfTimerRead = 0;
static volatile RfTimerIsrEvent g_rfTimerRing[8];
static volatile bool g_nextRfTimerEventIsTock = false;
static volatile int32_t g_rfTimerFrequencyOffsetUs = 0;
#else
static repeating_timer_t g_rfTimer;
static bool g_rfTimerRunning = false;
static volatile uint32_t g_rfTimerEvents = 0;
static volatile uint32_t g_rfTimerLastTickUs = 0;
#endif
static uint32_t g_rfTimerEventsHandled = 0;
static uint32_t g_rfMissedTimerEvents = 0;
static uint8_t g_otaTxBytes[kOtaFrameSize];
static uint32_t g_radioTuneOk = 0;
static uint32_t g_radioTuneFail = 0;
enum class RadioOp : uint8_t {
    Idle = 0,
    Rx = 1,
    Tx = 2,
};
static RadioOp g_radioOp = RadioOp::Idle;
#if LORA_TX_ROLE
static rf::TxScheduler g_rfScheduler;
static uint32_t g_downlinkTelemetryFrames = 0;
static uint32_t g_uplinkTransmitAttempts = 0;
static uint32_t g_uplinkTransmitSuccesses = 0;
static uint32_t g_telemetryListenSlots = 0;
static uint32_t g_txRcFailsafeFrames = 0;
static uint32_t g_txSyncFrames = 0;
static uint32_t g_crsfInputFrames = 0;
static uint32_t g_crsfInputRejects = 0;
static uint32_t g_crsfInputSpikeHolds = 0;
static uint32_t g_crsfInputBytes = 0;
static uint32_t g_crsfBindingControlFrames = 0;
static uint32_t g_lastDownlinkTelemetryMs = 0;
static uint32_t g_lastHandsetTelemetryMs = 0;
static uint32_t g_telemetryDio1Events = 0;
static uint32_t g_telemetryReadAttempts = 0;
static uint32_t g_telemetryWrongTypeFrames = 0;
static uint32_t g_telemetryReadRejects = 0;
static uint16_t g_lastTelemetryListenHop = 0;
static uint8_t g_crsfByteRing[64];
static uint8_t g_crsfByteRingPos = 0;
static uint8_t g_telemetryListenSlotsRemaining = 0;
static RcSpikeGate g_txInputSpikeGate{};
#endif
#if LORA_RX_ROLE
static rf::RxScheduler g_rfScheduler;
static bool g_haveLastSequence = false;
static uint16_t g_lastSequence = 0;
static uint32_t g_uplinkDropCount = 0;
static uint16_t g_lqWindowFrames = 0;
static uint16_t g_lqWindowDrops = 0;
static uint32_t g_lqWindowStartMs = 0;
static uint32_t g_rxChannelGuardRejects = 0;
static uint32_t g_rxSpikeHolds = 0;
static uint32_t g_lastFcChannelWriteUs = 0;
static uint32_t g_fcChannelFrames = 0;
static uint32_t g_fcChannelMaxGapUs = 0;
static uint32_t g_rxTelemetryTransmitAttempts = 0;
static uint32_t g_rxTelemetryTransmitDone = 0;
static uint16_t g_lastTelemetryTransmitHop = 0;
static uint32_t g_rxAcquisitionChannelUntilMs = 0;
static RcSpikeGate g_rxOutputSpikeGate{};
#endif

SX1280 radio = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY);

static const RateConfig& activeRate() {
    return kRates[static_cast<uint8_t>(g_config.rate)];
}

static float rfFrequencyForHop(uint16_t hop) {
    if (kFixedRfChannel >= 0) return fhssFrequencyMHz(static_cast<uint8_t>(kFixedRfChannel));
    return fhssFrequencyMHz(fhssChannelFor(g_uid, hop));
}

static float syncChannelFrequencyMHz() {
    return fhssFrequencyMHz(rf::kSyncChannelFhssIndex);
}

#if LORA_RX_ROLE
static bool rxAcquisitionChannelActive(uint32_t nowMs) {
    const rf::SchedulerStats rfStats = g_rfScheduler.stats();
    return rfStats.connectionState == rf::ConnectionState::Disconnected ||
           static_cast<int32_t>(nowMs - g_rxAcquisitionChannelUntilMs) < 0;
}
#endif

static void onDio1() {
    g_dio1 = true;
}

#if LORA_TX_ROLE
static bool onRfTimerTick(repeating_timer_t*) {
    g_rfTimerLastTickUs = micros();
    ++g_rfTimerEvents;
    return true;
}

static void startRfTimer() {
    if (g_rfTimerRunning) cancel_repeating_timer(&g_rfTimer);
    g_rfTimerEvents = 0;
    g_rfTimerEventsHandled = 0;
    g_rfMissedTimerEvents = 0;
    g_rfTimerRunning = add_repeating_timer_us(-static_cast<int64_t>(activeRate().intervalUs), onRfTimerTick, nullptr, &g_rfTimer);
}

static void resyncRfTimerFromNow() {
    // WP3 will replace this coarse timer restart with ELRS-style PFD/tick-tock correction.
    startRfTimer();
}

static uint32_t drainRfTimerEvents() {
    const uint32_t events = g_rfTimerEvents;
    const uint32_t pending = events - g_rfTimerEventsHandled;
    if (pending > 1) g_rfMissedTimerEvents += pending - 1;
    g_rfTimerEventsHandled = events;
    return pending;
}
#else
static int64_t adjustedRxHalfIntervalUs(int32_t phaseShiftUs = 0) {
    int32_t delayUs = static_cast<int32_t>(activeRate().intervalUs / 2u) +
                      g_rfTimerFrequencyOffsetUs + phaseShiftUs;
    if (delayUs < 100) delayUs = 100;
    if (delayUs > static_cast<int32_t>(activeRate().intervalUs)) {
        delayUs = static_cast<int32_t>(activeRate().intervalUs);
    }
    return delayUs;
}

static int64_t onRfTimerAlarm(alarm_id_t, void*) {
    const uint8_t write = g_rfTimerWrite;
    const uint8_t nextWrite = static_cast<uint8_t>((write + 1u) & 0x07u);
    if (nextWrite == g_rfTimerRead) {
        ++g_rfMissedTimerEvents;
    } else {
        g_rfTimerRing[write].timestampUs = micros();
        g_rfTimerRing[write].half = g_nextRfTimerEventIsTock
                                        ? static_cast<uint8_t>(rf::RxTimerHalfEvent::Tock)
                                        : static_cast<uint8_t>(rf::RxTimerHalfEvent::Tick);
        g_rfTimerWrite = nextWrite;
    }
    g_nextRfTimerEventIsTock = !g_nextRfTimerEventIsTock;
    return -adjustedRxHalfIntervalUs();
}

static void startRfTimer() {
    if (g_rfTimerRunning) cancel_alarm(g_rfAlarm);
    g_rfTimerWrite = 0;
    g_rfTimerRead = 0;
    g_rfTimerEventsHandled = 0;
    g_rfMissedTimerEvents = 0;
    g_rfTimerFrequencyOffsetUs = 0;
    g_nextRfTimerEventIsTock = false;
    g_rfTimerRunning = true;
    g_rfAlarm = add_alarm_in_us(adjustedRxHalfIntervalUs(), onRfTimerAlarm, nullptr, true);
    if (g_rfAlarm <= 0) g_rfTimerRunning = false;
}

static void alignRfTimerTock(uint32_t tockUs) {
    if (g_rfTimerRunning) cancel_alarm(g_rfAlarm);
    g_rfTimerWrite = 0;
    g_rfTimerRead = 0;
    const uint32_t nowUs = micros();
    int32_t delayUs = static_cast<int32_t>(tockUs - nowUs);
    if (delayUs < 100) delayUs = 100;
    if (delayUs > static_cast<int32_t>(activeRate().intervalUs)) {
        delayUs = static_cast<int32_t>(activeRate().intervalUs);
    }
    g_nextRfTimerEventIsTock = true;
    g_rfTimerRunning = true;
    g_rfAlarm = add_alarm_in_us(delayUs, onRfTimerAlarm, nullptr, true);
    if (g_rfAlarm <= 0) g_rfTimerRunning = false;
}

static void applyRfTimerCorrection(const rf::RxTimerEventResult& result) {
    g_rfTimerFrequencyOffsetUs = result.frequencyOffsetUs;
    if (!result.hasPhaseShift || !g_rfTimerRunning) return;
    cancel_alarm(g_rfAlarm);
    g_rfAlarm = add_alarm_in_us(adjustedRxHalfIntervalUs(result.phaseShiftUs),
                                onRfTimerAlarm, nullptr, true);
    if (g_rfAlarm <= 0) g_rfTimerRunning = false;
}

static bool popRfTimerEvent(RfTimerIsrEvent& out) {
    const uint8_t read = g_rfTimerRead;
    if (read == g_rfTimerWrite) return false;
    out.timestampUs = g_rfTimerRing[read].timestampUs;
    out.half = g_rfTimerRing[read].half;
    g_rfTimerRead = static_cast<uint8_t>((read + 1u) & 0x07u);
    ++g_rfTimerEventsHandled;
    return true;
}
#endif

static void setDefaultChannels() {
    for (uint8_t i = 0; i < kRcChannelCount; ++i) g_channels[i] = 992;
    g_channels[2] = 172; // throttle low in CRSF 11-bit units
}

static bool saveConfig() {
    ConfigRecord record = makeConfigRecord(g_config);
    EEPROM.put(0, record);
    return EEPROM.commit();
}

static void loadConfig() {
    EEPROM.begin(256);
    ConfigRecord record{};
    EEPROM.get(0, record);
    if (!readConfigRecord(record, g_config)) {
        configDefaults(g_config, DEFAULT_BINDING_PHRASE);
        g_configFault = true;
        saveConfig();
    }
    deriveUid(g_config.bindingPhrase, g_uid);
    g_uidCheck = uidCheck(g_uid);
    g_bindingRequiresReboot = false;
}

static void copyBindingPhrase(const char* phrase) {
    memset(g_config.bindingPhrase, 0, sizeof(g_config.bindingPhrase));
    if (phrase) strncpy(g_config.bindingPhrase, phrase, sizeof(g_config.bindingPhrase) - 1);
}

static void restoreDefaultBindingPhrase() {
    copyBindingPhrase(DEFAULT_BINDING_PHRASE);
}

static bool sx1280WaitBusy(uint32_t timeoutUs = 10000) {
    const uint32_t startUs = micros();
    while (digitalRead(SX128X_SPI_BUSY)) {
        if (static_cast<uint32_t>(micros() - startUs) > timeoutUs) return false;
        yield();
    }
    return true;
}

static bool sx1280WriteRegister(uint16_t addr, const uint8_t* data, uint8_t len) {
    if (!sx1280WaitBusy()) return false;
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SX128X_SPI_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(kSx1280OpWriteRegister);
    SPI.transfer(static_cast<uint8_t>(addr >> 8));
    SPI.transfer(static_cast<uint8_t>(addr));
    for (uint8_t i = 0; i < len; ++i) SPI.transfer(data[i]);
    delayMicroseconds(2);
    digitalWrite(SX128X_SPI_CS, HIGH);
    SPI.endTransaction();
    return sx1280WaitBusy();
}

static bool sx1280ReadRegister(uint16_t addr, uint8_t* data, uint8_t len) {
    if (!sx1280WaitBusy()) return false;
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SX128X_SPI_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(kSx1280OpReadRegister);
    SPI.transfer(static_cast<uint8_t>(addr >> 8));
    SPI.transfer(static_cast<uint8_t>(addr));
    SPI.transfer(0x00);
    for (uint8_t i = 0; i < len; ++i) data[i] = SPI.transfer(0x00);
    delayMicroseconds(2);
    digitalWrite(SX128X_SPI_CS, HIGH);
    SPI.endTransaction();
    return sx1280WaitBusy();
}

static bool applySx1280LoraTuning() {
    uint8_t sfConfig = kSx1280LoraSfConfigSf9Sf12;
    if (activeRate().spreadingFactor <= 6) {
        sfConfig = kSx1280LoraSfConfigSf5Sf6;
    } else if (activeRate().spreadingFactor <= 8) {
        sfConfig = kSx1280LoraSfConfigSf7Sf8;
    }
    if (!sx1280WriteRegister(kSx1280RegLoraSfConfig, &sfConfig, 1)) return false;

    uint8_t freqErrorComp = 0;
    if (!sx1280ReadRegister(kSx1280RegFreqErrorComp, &freqErrorComp, 1)) return false;
    freqErrorComp |= kSx1280FreqErrorCompOn;
    if (!sx1280WriteRegister(kSx1280RegFreqErrorComp, &freqErrorComp, 1)) return false;

    uint8_t rxGain = 0;
    if (!sx1280ReadRegister(kSx1280RegRxGain, &rxGain, 1)) return false;
    rxGain = static_cast<uint8_t>((rxGain & kSx1280RxGainKeepMask) | kSx1280RxGainHighSensitivity);
    return sx1280WriteRegister(kSx1280RegRxGain, &rxGain, 1);
}

static bool configureRadio(float freqMHz) {
    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.begin();
    pinMode(SX128X_SPI_CS, OUTPUT);
    digitalWrite(SX128X_SPI_CS, HIGH);
    pinMode(SX128X_SPI_BUSY, INPUT);
    pinMode(SX128X_RXEN, OUTPUT);
    pinMode(SX128X_TXEN, OUTPUT);
    digitalWrite(SX128X_RXEN, HIGH);
    digitalWrite(SX128X_TXEN, LOW);

    const RateConfig& rate = activeRate();
    const int16_t state = radio.begin(freqMHz, rate.bandwidthKHz, rate.spreadingFactor,
                                      rate.codingRate, syncWordFromUid(g_uid), kBenchTxPowerDbm, 12);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("radio init failed: %d\n", state);
        return false;
    }
    radio.setDio1Action(onDio1);
    radio.setCRC(2);
    if (applySx1280LoraTuning()) {
        ++g_radioTuneOk;
    } else {
        ++g_radioTuneFail;
    }
    g_dio1 = false;
    radio.startReceive();
    g_rxWaiting = true;
    g_radioOp = RadioOp::Rx;
    g_lastRxHopAdvanceUs = micros();
    return true;
}

static void startReceiveOnHop() {
    if (!g_radioReady) return;
    digitalWrite(SX128X_TXEN, LOW);
    digitalWrite(SX128X_RXEN, HIGH);
    g_dio1 = false;
#if LORA_RX_ROLE
    const bool useAcquisitionChannel = rxAcquisitionChannelActive(millis());
    const float receiveFrequencyMHz = useAcquisitionChannel
                                          ? syncChannelFrequencyMHz()
                                          : rfFrequencyForHop(g_hop);
#elif LORA_TX_ROLE
    const bool useAcquisitionChannel = !isLinkFresh(millis(), g_lastDownlinkTelemetryMs, kDownlinkTelemetryFreshMs);
    const float receiveFrequencyMHz = useAcquisitionChannel
                                          ? syncChannelFrequencyMHz()
                                          : rfFrequencyForHop(g_hop);
#else
    const float receiveFrequencyMHz = rfFrequencyForHop(g_hop);
#endif
    radio.setFrequency(kBenchSingleFrequency ? syncChannelFrequencyMHz() : receiveFrequencyMHz);
    radio.startReceive();
    g_rxWaiting = true;
    g_radioOp = RadioOp::Rx;
    g_lastRxHopAdvanceUs = micros();
}

static bool startTransmitFrame(const OtaFrame& frame, float frequencyMHz) {
    if (!g_radioReady) return false;
    if (!encodeOtaFrame(frame, g_otaTxBytes)) return false;
    g_rxWaiting = false;
    digitalWrite(SX128X_RXEN, LOW);
    digitalWrite(SX128X_TXEN, HIGH);
    radio.setFrequency(frequencyMHz);
    g_dio1 = false;
    const int16_t state = radio.startTransmit(g_otaTxBytes, sizeof(g_otaTxBytes));
    g_radioOp = state == RADIOLIB_ERR_NONE ? RadioOp::Tx : RadioOp::Idle;
    return state == RADIOLIB_ERR_NONE;
}

static void finishTransmitAndReceive(float receiveFrequencyMHz) {
    radio.finishTransmit();
    digitalWrite(SX128X_TXEN, LOW);
    digitalWrite(SX128X_RXEN, HIGH);
    g_dio1 = false;
    radio.setFrequency(receiveFrequencyMHz);
    radio.startReceive();
    g_rxWaiting = true;
    g_radioOp = RadioOp::Rx;
}

static bool readOtaFrame(OtaFrame& out, uint32_t* beginProcessingUs = nullptr) {
    if (!g_radioReady) return false;
    if (!g_dio1) return false;
    const uint32_t beginUs = micros();
    if (beginProcessingUs) *beginProcessingUs = beginUs;
    g_dio1 = false;
    uint8_t bytes[kOtaFrameSize] = {};
    const int16_t state = radio.readData(bytes, sizeof(bytes));
    g_radioOp = RadioOp::Idle;
    if (state != RADIOLIB_ERR_NONE) {
        ++g_rejectedOtaFrames;
        return false;
    }
    g_lastRssi = static_cast<int16_t>(radio.getRSSI());
    g_lastSnr = static_cast<int8_t>(radio.getSNR());
    if (!decodeOtaFrame(bytes, g_uidCheck, out)) {
        ++g_rejectedOtaFrames;
        return false;
    }
    ++g_validOtaFrames;
    return true;
}

static void printStatus() {
    const rf::SchedulerStats rfStats = g_rfScheduler.stats();
    Serial.printf("role=%s phrase=%s rate=%s uid=%02X%02X%02X%02X%02X%02X%02X%02X sync_mhz=%.1f lq=%u rssi=%d snr=%d good=%lu bad=%lu tune_ok=%lu tune_fail=%lu",
#if LORA_TX_ROLE
                  "tx",
#else
                  "rx",
#endif
                  g_config.bindingPhrase, activeRate().name,
                  g_uid[0], g_uid[1], g_uid[2], g_uid[3], g_uid[4], g_uid[5], g_uid[6], g_uid[7],
                  static_cast<double>(syncChannelFrequencyMHz()),
                  g_linkQuality, g_lastRssi, g_lastSnr,
                  static_cast<unsigned long>(g_validOtaFrames),
                  static_cast<unsigned long>(g_rejectedOtaFrames),
                  static_cast<unsigned long>(g_radioTuneOk),
                  static_cast<unsigned long>(g_radioTuneFail));
#if LORA_TX_ROLE
    const uint32_t crsfAgeMs = g_lastRcInputMs ? millis() - g_lastRcInputMs : 0xFFFFFFFFu;
    Serial.printf(" ticks=%lu missed_ticks=%lu nonce=%u fhss=%u tx_done=%lu sync_tx=%lu tlm=%lu tlm_dio=%lu tlm_try=%lu tlm_bad=%lu tlm_type=%lu tlm_hop=%u tx_ok=%lu tx_try=%lu listen=%lu rc_failsafe=%lu crsf=%lu crsf_bind=%lu crsf_rej=%lu crsf_hold=%lu crsf_bytes=%lu crsf_age=%lu ch=%u,%u,%u,%u",
                  static_cast<unsigned long>(rfStats.timerTicks),
                  static_cast<unsigned long>(g_rfMissedTimerEvents + rfStats.missedTimerTicks),
                  rfStats.nonce,
                  rfStats.fhssIndex,
                  static_cast<unsigned long>(rfStats.txDone),
                  static_cast<unsigned long>(g_txSyncFrames),
                  static_cast<unsigned long>(g_downlinkTelemetryFrames),
                  static_cast<unsigned long>(g_telemetryDio1Events),
                  static_cast<unsigned long>(g_telemetryReadAttempts),
                  static_cast<unsigned long>(g_telemetryReadRejects),
                  static_cast<unsigned long>(g_telemetryWrongTypeFrames),
                  g_lastTelemetryListenHop,
                  static_cast<unsigned long>(g_uplinkTransmitSuccesses),
                  static_cast<unsigned long>(g_uplinkTransmitAttempts),
                  static_cast<unsigned long>(g_telemetryListenSlots),
                  static_cast<unsigned long>(g_txRcFailsafeFrames),
                  static_cast<unsigned long>(g_crsfInputFrames),
                  static_cast<unsigned long>(g_crsfBindingControlFrames),
                  static_cast<unsigned long>(g_crsfInputRejects),
                  static_cast<unsigned long>(g_crsfInputSpikeHolds),
                  static_cast<unsigned long>(g_crsfInputBytes),
                  static_cast<unsigned long>(crsfAgeMs),
                  g_channels[0], g_channels[1], g_channels[2], g_channels[3]);
#else
    Serial.printf(" ticks=%lu missed_ticks=%lu nonce=%u fhss=%u rx_done=%lu rf_miss=%lu phase_us=%ld tim=%u off=%ld dx=%ld trim=%ld sync_ok=%lu rc_no_sync=%lu pfd_ext=%lu pfd_int=%lu pfd_res=%lu pshift=%lu last_shift=%ld lost=%lu tent_lost=%lu conn=%lu lock=%lu prom=%lu prom_fr=%lu prom_off=%lu prom_dx=%lu max_rc=%u drops=%lu guard=%lu hold=%lu fc=%lu fc_gap_us=%lu",
                  static_cast<unsigned long>(rfStats.timerTicks),
                  static_cast<unsigned long>(g_rfMissedTimerEvents + rfStats.missedTimerTicks),
                  rfStats.nonce,
                  rfStats.fhssIndex,
                  static_cast<unsigned long>(rfStats.rxDone),
                  static_cast<unsigned long>(rfStats.missedRcFrames),
                  static_cast<long>(rfStats.phaseErrorUs),
                  static_cast<unsigned>(rfStats.rxTimerState),
                  static_cast<long>(rfStats.offsetUs),
                  static_cast<long>(rfStats.offsetDxUs),
                  static_cast<long>(rfStats.frequencyOffsetUs),
                  static_cast<unsigned long>(rfStats.syncAccepted),
                  static_cast<unsigned long>(rfStats.rcRejectedNoSync),
                  static_cast<unsigned long>(rfStats.pfdExternalEvents),
                  static_cast<unsigned long>(rfStats.pfdInternalEvents),
                  static_cast<unsigned long>(rfStats.pfdResults),
                  static_cast<unsigned long>(rfStats.phaseShiftRequests),
                  static_cast<long>(rfStats.lastPhaseShiftUs),
                  static_cast<unsigned long>(rfStats.lostConnections),
                  static_cast<unsigned long>(rfStats.tentativeLostConnections),
                  static_cast<unsigned long>(rfStats.connectedPromotions),
                  static_cast<unsigned long>(rfStats.lockedPromotions),
                  static_cast<unsigned long>(rfStats.promotionChecks),
                  static_cast<unsigned long>(rfStats.promotionFrameBlocks),
                  static_cast<unsigned long>(rfStats.promotionOffsetBlocks),
                  static_cast<unsigned long>(rfStats.promotionDxBlocks),
                  rfStats.maxConsecutiveRcFrames,
                  static_cast<unsigned long>(g_uplinkDropCount),
                  static_cast<unsigned long>(g_rxChannelGuardRejects),
                  static_cast<unsigned long>(g_rxSpikeHolds),
                  static_cast<unsigned long>(g_fcChannelFrames),
                  static_cast<unsigned long>(g_fcChannelMaxGapUs));
    Serial.printf(" tlm_tx=%lu tlm_done=%lu tlm_hop=%u",
                  static_cast<unsigned long>(g_rxTelemetryTransmitAttempts),
                  static_cast<unsigned long>(g_rxTelemetryTransmitDone),
                  g_lastTelemetryTransmitHop);
#endif
    Serial.printf(" config=%s\n", g_configFault ? "defaulted" : "ok");
}

static const char* bindingOpName(BindingControlOp op) {
    switch (op) {
        case BindingControlOp::Get:
            return "binding_get";
        case BindingControlOp::Set:
            return "binding_set";
        case BindingControlOp::Clear:
            return "binding_clear";
        case BindingControlOp::Verify:
            return "binding_verify";
        default:
            return "unknown";
    }
}

static const char* bindingResultName(BindingResult result) {
    switch (result) {
        case BindingResult::Ok:
            return "ok";
        case BindingResult::InvalidCommand:
            return "invalid_command";
        case BindingResult::InvalidPhrase:
            return "invalid_phrase";
        case BindingResult::PersistFailed:
            return "persist_failed";
        default:
            return "unknown";
    }
}

static void printUidHex(const uint8_t uid[kUidSize]) {
    for (uint8_t i = 0; i < kUidSize; ++i) Serial.printf("%02X", uid[i]);
}

static int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool percentDecodeToken(const char* value, size_t len, char* out, size_t outLen) {
    if (!out || outLen == 0) return false;
    size_t written = 0;
    for (size_t i = 0; i < len; ++i) {
        char c = value[i];
        if (c == '%') {
            if (i + 2 >= len) return false;
            const int hi = hexValue(value[i + 1]);
            const int lo = hexValue(value[i + 2]);
            if (hi < 0 || lo < 0) return false;
            c = static_cast<char>((hi << 4) | lo);
            i += 2;
        }
        if (written + 1 >= outLen) return false;
        out[written++] = c;
    }
    out[written] = '\0';
    return true;
}

static bool extractRcV1Arg(const char* line, const char* key, char* out, size_t outLen) {
    const size_t keyLen = strlen(key);
    const char* p = line;
    while (*p) {
        while (*p == ' ') ++p;
        const char* token = p;
        while (*p && *p != ' ') ++p;
        const size_t tokenLen = static_cast<size_t>(p - token);
        if (tokenLen > keyLen && strncmp(token, key, keyLen) == 0 && token[keyLen] == '=') {
            return percentDecodeToken(token + keyLen + 1, tokenLen - keyLen - 1, out, outLen);
        }
    }
    return false;
}

static const char* rcV1ArgsForCommand(const char* line, const char* command) {
    static constexpr const char* kPrefix = "rc.v1 ";
    const size_t prefixLen = strlen(kPrefix);
    const size_t commandLen = strlen(command);
    if (strncmp(line, kPrefix, prefixLen) != 0) return nullptr;
    const char* cursor = line + prefixLen;
    if (strncmp(cursor, command, commandLen) != 0) return nullptr;
    cursor += commandLen;
    if (*cursor != '\0' && *cursor != ' ') return nullptr;
    while (*cursor == ' ') ++cursor;
    return cursor;
}

static void printRcV1Encoded(const char* value) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    if (!value) return;
    for (size_t i = 0; value[i]; ++i) {
        const uint8_t c = static_cast<uint8_t>(value[i]);
        const bool unreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                                (c >= '0' && c <= '9') || c == '-' || c == '_' ||
                                c == '.' || c == '~';
        if (unreserved) {
            Serial.print(static_cast<char>(c));
        } else {
            Serial.print('%');
            Serial.print(kHex[(c >> 4) & 0x0F]);
            Serial.print(kHex[c & 0x0F]);
        }
    }
}

static const char* roleName() {
#if LORA_TX_ROLE
    return "tx";
#else
    return "rx";
#endif
}

static void printRcV1Err(const char* seq, const char* code, const char* message) {
    Serial.print("rc.v1 err");
    if (seq && seq[0]) {
        Serial.print(" seq=");
        Serial.print(seq);
    }
    Serial.print(" code=");
    printRcV1Encoded(code);
    Serial.print(" message=");
    printRcV1Encoded(message);
    Serial.println();
}

static void printBindingStatusRcV1(const char* seq, const char* target, const BindingStatus& status,
                                   BindingResult result, BindingControlOp op) {
    Serial.print("rc.v1 ok");
    if (seq && seq[0]) {
        Serial.print(" seq=");
        Serial.print(seq);
    }
    Serial.print(" target=");
    Serial.print(target);
    Serial.print(" op=");
    Serial.print(bindingOpName(op));
    Serial.print(" result=");
    Serial.print(bindingResultName(result));
    Serial.print(" phrase=");
    printRcV1Encoded(status.phrase);
    Serial.print(" uid=");
    printUidHex(status.uid);
    Serial.printf(" uid_check=%08lX persisted=%u requires_reboot=%u\n",
                  static_cast<unsigned long>(status.uidCheck),
                  status.persisted ? 1u : 0u,
                  status.requiresReboot ? 1u : 0u);
}

static BindingResult applyBindingControlRequest(const BindingControlRequest& request, BindingStatus& status) {
    switch (request.op) {
        case BindingControlOp::Get:
            makeBindingStatus(g_config.bindingPhrase, true, g_bindingRequiresReboot, status);
            return BindingResult::Ok;
        case BindingControlOp::Set:
            if (!validateBindingPhrase(request.phrase)) {
                makeBindingStatus(request.phrase, false, false, status);
                return BindingResult::InvalidPhrase;
            }
            copyBindingPhrase(request.phrase);
            {
                const bool saved = saveConfig();
                if (saved) g_bindingRequiresReboot = true;
                makeBindingStatus(g_config.bindingPhrase, saved, saved, status);
                return saved ? BindingResult::Ok : BindingResult::PersistFailed;
            }
        case BindingControlOp::Clear:
            restoreDefaultBindingPhrase();
            {
                const bool saved = saveConfig();
                if (saved) g_bindingRequiresReboot = true;
                makeBindingStatus(g_config.bindingPhrase, saved, saved, status);
                return saved ? BindingResult::Ok : BindingResult::PersistFailed;
            }
        case BindingControlOp::Verify:
            if (!validateBindingPhrase(request.phrase)) {
                makeBindingStatus(request.phrase, false, false, status);
                return BindingResult::InvalidPhrase;
            }
            makeBindingStatus(request.phrase, false, false, status);
            return BindingResult::Ok;
        default:
            makeBindingStatus(g_config.bindingPhrase, true, g_bindingRequiresReboot, status);
            return BindingResult::InvalidCommand;
    }
}

static bool startsWith(const char* value, const char* prefix) {
    return strncmp(value, prefix, strlen(prefix)) == 0;
}

static bool handleRcV1CliLine(const char* line) {
    if (strncmp(line, "rc.v1", 5) != 0) return false;

    BindingControlRequest request{};
    char seq[12] = {};
    extractRcV1Arg(line, "seq", seq, sizeof(seq));
    char target[8] = {};
    extractRcV1Arg(line, "target", target, sizeof(target));
    if (target[0] == '\0') strncpy(target, roleName(), sizeof(target) - 1);

    if (rcV1ArgsForCommand(line, "hello")) {
        Serial.print("rc.v1 ok");
        if (seq[0]) {
            Serial.print(" seq=");
            Serial.print(seq);
        }
        Serial.print(" role=");
        Serial.print(roleName());
        Serial.println(" fw=0.1 caps=binding");
        return true;
    }

    if (strcmp(target, roleName()) != 0) {
        printRcV1Err(seq, "unsupported_target", "target not hosted on this USB device");
        return true;
    }

    if (rcV1ArgsForCommand(line, "binding_get")) {
        request.op = BindingControlOp::Get;
    } else if (rcV1ArgsForCommand(line, "binding_set")) {
        request.op = BindingControlOp::Set;
        if (!extractRcV1Arg(line, "phrase", request.phrase, sizeof(request.phrase)) &&
            !extractRcV1Arg(line, "value", request.phrase, sizeof(request.phrase))) {
            printRcV1Err(seq, "missing_required_argument", "binding_set requires phrase");
            return true;
        }
    } else if (rcV1ArgsForCommand(line, "binding_clear")) {
        request.op = BindingControlOp::Clear;
    } else if (rcV1ArgsForCommand(line, "binding_verify")) {
        request.op = BindingControlOp::Verify;
        if (!extractRcV1Arg(line, "phrase", request.phrase, sizeof(request.phrase)) &&
            !extractRcV1Arg(line, "value", request.phrase, sizeof(request.phrase))) {
            printRcV1Err(seq, "missing_required_argument", "binding_verify requires phrase");
            return true;
        }
    } else {
        return false;
    }

    BindingStatus status{};
    const BindingResult result = applyBindingControlRequest(request, status);
    if (result == BindingResult::Ok) {
        printBindingStatusRcV1(seq, target, status, result, request.op);
    } else {
        printRcV1Err(seq, bindingResultName(result), "binding command failed");
    }
    return true;
}

static void handleCliLine(char* line) {
    while (*line == ' ') ++line;
    if (handleRcV1CliLine(line)) {
        return;
    } else if (strcmp(line, "bind get") == 0) {
        Serial.println(g_config.bindingPhrase);
    } else if (strncmp(line, "bind set ", 9) == 0) {
        const char* phrase = line + 9;
        if (!validateBindingPhrase(phrase)) {
            Serial.println("ERR phrase must be 1..32 printable bytes");
            return;
        }
        copyBindingPhrase(phrase);
        if (saveConfig()) {
            g_bindingRequiresReboot = true;
            Serial.println("OK reboot required");
        } else {
            Serial.println("ERR persist failed");
        }
    } else if (strcmp(line, "bind clear") == 0) {
        restoreDefaultBindingPhrase();
        if (saveConfig()) {
            g_bindingRequiresReboot = true;
            Serial.println("OK reboot required");
        } else {
            Serial.println("ERR persist failed");
        }
    } else if (strncmp(line, "rate ", 5) == 0) {
        if (strcmp(line + 5, "L250") == 0) g_config.rate = RateId::L250;
        else if (strcmp(line + 5, "L100") == 0) g_config.rate = RateId::L100;
        else {
            Serial.println("ERR rate must be L250 or L100");
            return;
        }
        if (saveConfig()) Serial.println("OK reboot required");
        else Serial.println("ERR persist failed");
    } else if (strcmp(line, "status") == 0) {
        printStatus();
    } else if (strcmp(line, "channels") == 0) {
        for (uint8_t i = 0; i < kRcChannelCount; ++i) {
            if (i) Serial.print(',');
            Serial.print(g_channels[i]);
        }
        Serial.println();
#if LORA_TX_ROLE
    } else if (strcmp(line, "crsf dump") == 0) {
        for (uint8_t i = 0; i < sizeof(g_crsfByteRing); ++i) {
            const uint8_t index = static_cast<uint8_t>((g_crsfByteRingPos + i) & 0x3F);
            if (i) Serial.print(' ');
            if (g_crsfByteRing[index] < 16) Serial.print('0');
            Serial.print(g_crsfByteRing[index], HEX);
        }
        Serial.println();
#endif
    } else if (strcmp(line, "reboot") == 0) {
        rp2040.reboot();
    } else if (*line) {
        Serial.println("commands: rc.v1 binding_get|binding_set <phrase>|binding_clear|binding_verify <phrase> | bind get | bind set <phrase> | bind clear | rate L250|L100 | status | channels | reboot");
    }
}

static void serviceCli() {
    static char line[128];
    static uint8_t pos = 0;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            line[pos] = '\0';
            handleCliLine(line);
            pos = 0;
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

static void setupCrsfUart() {
    Serial2.setTX(CRSF_UART_TX_PIN);
    Serial2.setRX(CRSF_UART_RX_PIN);
    Serial2.begin(kCrsfBaud);
}

#if LORA_TX_ROLE
static void serviceTxCrsfInput() {
    while (Serial2.available()) {
        const uint8_t byte = static_cast<uint8_t>(Serial2.read());
        ++g_crsfInputBytes;
        g_crsfByteRing[g_crsfByteRingPos] = byte;
        g_crsfByteRingPos = static_cast<uint8_t>((g_crsfByteRingPos + 1) & 0x3F);
        BindingControlRequest bindingRequest{};
        const bool gotBindingControl = parseCrsfBindingRequestFrame(byte, bindingRequest);
        uint16_t candidate[kRcChannelCount];
        const bool gotRcFrame = parseCrsfRcFrame(byte, candidate);
        if (gotBindingControl) {
            BindingStatus status{};
            const BindingResult result = applyBindingControlRequest(bindingRequest, status);
            uint8_t response[64];
            const size_t len = encodeCrsfBindingResponseFrame(status, result, bindingRequest.op,
                                                              response, sizeof(response));
            if (len) Serial2.write(response, len);
            ++g_crsfBindingControlFrames;
        }
        if (gotRcFrame) {
            if (!sanitizeRcChannels(g_channels, g_lastRcInputMs != 0, candidate)) {
                ++g_crsfInputRejects;
                continue;
            }
            if (!acceptRcChannelsWithSpikeGate(g_channels, g_lastRcInputMs != 0, candidate, g_txInputSpikeGate,
                                               kRcSpikeJumpThreshold, kRcSpikeConfirmTolerance)) {
                ++g_crsfInputSpikeHolds;
                continue;
            }
            memcpy(g_channels, candidate, sizeof(g_channels));
            g_lastRcInputMs = millis();
            ++g_crsfInputFrames;
        }
    }
}

static void sendTelemetryToHandset(uint32_t nowMs) {
    uint8_t frame[16];
    const uint8_t linkQuality = isLinkFresh(nowMs, g_lastDownlinkTelemetryMs, kDownlinkTelemetryFreshMs) ? g_linkQuality : 0;
    const size_t len = encodeCrsfLinkStats(g_lastRssi, g_lastSnr, linkQuality,
                                           static_cast<uint8_t>(g_config.rate), frame, sizeof(frame));
    if (len) {
        Serial2.write(frame, len);
        g_lastHandsetTelemetryMs = nowMs;
    }
}

static void serviceHandsetTelemetry() {
    const uint32_t nowMs = millis();
    if (g_lastHandsetTelemetryMs != 0 &&
        static_cast<uint32_t>(nowMs - g_lastHandsetTelemetryMs) < kHandsetTelemetryIntervalMs) {
        return;
    }
    sendTelemetryToHandset(nowMs);
}

static void txLoop() {
    serviceTxCrsfInput();
    serviceHandsetTelemetry();

    if (g_dio1) {
        if (g_radioOp == RadioOp::Tx) {
            g_rfScheduler.onTxDone();
            g_sequence = g_rfScheduler.sequence();
            g_hop = g_rfScheduler.fhss().index();
            const bool acquisitionMode = !isLinkFresh(millis(), g_lastDownlinkTelemetryMs, kDownlinkTelemetryFreshMs);
            const float receiveFrequencyMHz = acquisitionMode ? syncChannelFrequencyMHz()
                                                              : g_rfScheduler.fhss().frequencyMHz();
            finishTransmitAndReceive(kBenchSingleFrequency ? syncChannelFrequencyMHz() : receiveFrequencyMHz);
            ++g_uplinkTransmitSuccesses;
        } else if (g_radioOp == RadioOp::Rx) {
            OtaFrame rx{};
            ++g_telemetryDio1Events;
            ++g_telemetryReadAttempts;
            if (readOtaFrame(rx) && rx.type == OtaType::Telemetry) {
                const uint32_t nowMs = millis();
                g_linkQuality = rx.payload[0];
                if (rx.payloadLen > 1) g_lastRssi = -static_cast<int16_t>(rx.payload[1]);
                if (rx.payloadLen > 2) g_lastSnr = static_cast<int8_t>(rx.payload[2]);
                g_lastDownlinkTelemetryMs = nowMs;
                ++g_downlinkTelemetryFrames;
                sendTelemetryToHandset(nowMs);
            } else {
                if (rx.type != OtaType::Telemetry) ++g_telemetryWrongTypeFrames;
                ++g_telemetryReadRejects;
            }
            startReceiveOnHop();
        } else {
            g_dio1 = false;
        }
    }

    const uint32_t pendingTicks = drainRfTimerEvents();
    for (uint32_t i = 0; i < pendingTicks; ++i) {
        if (g_radioOp == RadioOp::Tx) return;
        if (!g_rfScheduler.onTimerTick()) {
            ++g_telemetryListenSlots;
            g_sequence = g_rfScheduler.sequence();
            g_hop = g_rfScheduler.fhss().index();
            g_lastTelemetryListenHop = g_hop;
            startReceiveOnHop();
            continue;
        }

        if (!isLinkFresh(millis(), g_lastRcInputMs)) {
            setDefaultChannels();
            ++g_txRcFailsafeFrames;
        }
        OtaFrame frame{};
        const uint16_t txSequence = g_rfScheduler.sequence();
        const bool downlinkFresh = isLinkFresh(millis(), g_lastDownlinkTelemetryMs, kDownlinkTelemetryFreshMs);
        const bool acquisitionMode = !downlinkFresh;
        const bool acquisitionSync = !kBenchSingleFrequency && acquisitionMode &&
                                     (txSequence % kAcquisitionSyncIntervalFrames) == 0;
        frame.type = (!kBenchSingleFrequency && (acquisitionSync || g_rfScheduler.shouldSendSyncFrame()))
                         ? OtaType::Sync
                         : OtaType::Rc;
        frame.sequence = txSequence;
        frame.uidCheck = g_uidCheck;
        float txFrequencyMHz = (kBenchSingleFrequency || acquisitionMode)
                                   ? syncChannelFrequencyMHz()
                                   : g_rfScheduler.fhss().frequencyMHz();
        if (frame.type == OtaType::Sync) {
            const OtaSyncPayload sync = g_rfScheduler.syncPayload();
            if (!encodeOtaSyncPayload(sync, frame.payload, frame.payloadLen)) continue;
            txFrequencyMHz = syncChannelFrequencyMHz();
            ++g_txSyncFrames;
        } else {
            frame.payloadLen = kOtaPayloadSize;
            packRcChannels11Bit(g_channels, frame.payload);
        }
        g_sequence = txSequence;
        g_hop = g_rfScheduler.fhss().index();
        ++g_uplinkTransmitAttempts;
        startTransmitFrame(frame, txFrequencyMHz);
    }
}
#endif

#if LORA_RX_ROLE
static void writeFcChannels() {
    uint8_t frame[32];
    const size_t len = encodeCrsfRcFrame(g_channels, frame, sizeof(frame));
    if (len) {
        const uint32_t nowUs = micros();
        if (g_lastFcChannelWriteUs != 0) {
            const uint32_t gapUs = nowUs - g_lastFcChannelWriteUs;
            if (gapUs > g_fcChannelMaxGapUs) g_fcChannelMaxGapUs = gapUs;
        }
        Serial2.write(frame, len);
        g_lastFcChannelWriteUs = nowUs;
        ++g_fcChannelFrames;
    }
}

static bool acceptRxOutputCandidate(const uint16_t candidate[kRcChannelCount], bool reacquiredLink) {
    if (reacquiredLink) {
        g_rxOutputSpikeGate.havePending = false;
        return true;
    }
    if (acceptRcChannelsWithSpikeGate(g_channels, true, candidate, g_rxOutputSpikeGate,
                                      kRcSpikeJumpThreshold, kRcSpikeConfirmTolerance)) {
        return true;
    }
    ++g_rxSpikeHolds;
    return false;
}

static void writeFcLinkStats() {
    uint8_t frame[16];
    const uint8_t lq = isLinkFresh(millis(), g_lastUplinkMs) ? g_linkQuality : 0;
    const size_t len = encodeCrsfLinkStats(g_lastRssi, g_lastSnr, lq,
                                           static_cast<uint8_t>(g_config.rate), frame, sizeof(frame));
    if (len) Serial2.write(frame, len);
}

static void updateRxSequenceStats(uint16_t rxSequence) {
    if (kBenchSingleFrequency) {
        g_haveLastSequence = true;
        g_lastSequence = rxSequence;
        return;
    }
    if (g_haveLastSequence) {
        const uint16_t expected = static_cast<uint16_t>(g_lastSequence + 1);
        if (rxSequence != expected) {
            const uint16_t missedFrames = static_cast<uint16_t>(rxSequence - expected);
            if (missedFrames < 1000) {
                g_uplinkDropCount += missedFrames;
                g_lqWindowDrops += missedFrames;
            }
        }
    }
    g_haveLastSequence = true;
    g_lastSequence = rxSequence;
}

static void updateRxLinkQuality(bool acceptedFrame) {
    const uint32_t nowMs = millis();
    if (g_lqWindowStartMs == 0) g_lqWindowStartMs = nowMs;
    if (acceptedFrame) ++g_lqWindowFrames;

    const uint32_t elapsedMs = nowMs - g_lqWindowStartMs;
    if (elapsedMs >= 1000) {
        const uint32_t expectedFrames = static_cast<uint32_t>(g_lqWindowFrames) + g_lqWindowDrops;
        uint32_t percent = expectedFrames ? (static_cast<uint32_t>(g_lqWindowFrames) * 100u) / expectedFrames : 0;
        if (percent > 100) percent = 100;
        g_linkQuality = static_cast<uint8_t>(percent);
        g_lqWindowFrames = 0;
        g_lqWindowDrops = 0;
        g_lqWindowStartMs = nowMs;
    }
}

static void resetRxSequenceTracking() {
    g_haveLastSequence = false;
    g_lqWindowFrames = 0;
    g_lqWindowDrops = 0;
    g_lqWindowStartMs = millis();
    g_rxOutputSpikeGate.havePending = false;
}

static void sendRxTelemetryNow() {
    OtaFrame tlm{};
    tlm.type = OtaType::Telemetry;
    tlm.sequence = g_sequence++;
    tlm.uidCheck = g_uidCheck;
    tlm.payloadLen = 4;
    tlm.payload[0] = g_linkQuality;
    tlm.payload[1] = static_cast<uint8_t>(g_lastRssi < 0 ? -g_lastRssi : g_lastRssi);
    tlm.payload[2] = static_cast<uint8_t>(g_lastSnr);
    tlm.payload[3] = static_cast<uint8_t>(g_config.rate);
    ++g_rxTelemetryTransmitAttempts;
    g_lastTelemetryTransmitHop = g_rfScheduler.fhss().index();
    busy_wait_us(kTelemetryReplyGuardUs);
    startTransmitFrame(tlm, syncChannelFrequencyMHz());
}

static void rxLoop() {
    bool acceptedFrameForLq = false;
    if (g_dio1) {
        if (g_radioOp == RadioOp::Tx) {
            g_rfScheduler.onTelemetryDone();
            ++g_rxTelemetryTransmitDone;
            const uint32_t nowMs = millis();
            const float receiveFrequencyMHz = rxAcquisitionChannelActive(nowMs)
                                                  ? syncChannelFrequencyMHz()
                                                  : g_rfScheduler.fhss().frequencyMHz();
            finishTransmitAndReceive(kBenchSingleFrequency ? syncChannelFrequencyMHz() : receiveFrequencyMHz);
        } else if (g_radioOp == RadioOp::Rx) {
            OtaFrame frame{};
            uint32_t beginProcessingUs = 0;
            if (readOtaFrame(frame, &beginProcessingUs) && frame.type == OtaType::Sync) {
                OtaSyncPayload sync{};
                const uint32_t nowMs = millis();
                if (decodeOtaSyncPayload(frame, sync) && sync.nonce == frame.sequence &&
                    g_rfScheduler.onValidSyncFrame(sync, beginProcessingUs)) {
                    g_lastSyncMs = nowMs;
                    g_hop = g_rfScheduler.fhss().index();
                    g_rxAcquisitionChannelUntilMs = nowMs + 1000u;
                    startReceiveOnHop();
                    alignRfTimerTock(g_rfScheduler.packetTockReferenceUs(beginProcessingUs));
                } else {
                    startReceiveOnHop();
                }
            } else if (frame.type == OtaType::Rc && frame.payloadLen == kOtaPayloadSize) {
                const uint16_t rxSequence = frame.sequence;
                uint16_t candidate[kRcChannelCount];
                const uint32_t nowMs = millis();
                const bool reacquired = !isLinkFresh(nowMs, g_lastUplinkMs);
                if (reacquired) resetRxSequenceTracking();
                const bool acquiring = kBenchSingleFrequency || rxAcquisitionChannelActive(nowMs);
                if (!(acquiring
                          ? g_rfScheduler.onAcquisitionRcFrame(rxSequence, beginProcessingUs, nowMs)
                          : g_rfScheduler.onValidRcFrame(rxSequence, beginProcessingUs, nowMs))) {
                    startReceiveOnHop();
                    return;
                }
                if (acquiring) alignRfTimerTock(g_rfScheduler.packetTockReferenceUs(beginProcessingUs));
                g_hop = g_rfScheduler.fhss().index();
                if (acquiring || reacquired) g_rxAcquisitionChannelUntilMs = nowMs + 1500u;
                unpackRcChannels11Bit(frame.payload, candidate);
                g_lastUplinkMs = nowMs;
                updateRxSequenceStats(rxSequence);
                acceptedFrameForLq = true;
                if (sanitizeRcChannels(g_channels, true, candidate)) {
                    if (acceptRxOutputCandidate(candidate, reacquired)) {
                        memcpy(g_channels, candidate, sizeof(g_channels));
                        writeFcChannels();
                    }
                } else {
                    ++g_rxChannelGuardRejects;
                }
                if (shouldRequestTelemetry(static_cast<uint16_t>(frame.sequence + 1), activeRate().telemetryRatio)) {
                    sendRxTelemetryNow();
                } else {
                    startReceiveOnHop();
                }
            } else {
                startReceiveOnHop();
            }
        } else {
            g_dio1 = false;
        }
    }

    updateRxLinkQuality(acceptedFrameForLq);
    const uint32_t nowMs = millis();
    const bool uplinkFresh = isLinkFresh(nowMs, g_lastUplinkMs);
    const bool linkFresh = uplinkFresh || isLinkFresh(nowMs, g_lastSyncMs);
    RfTimerIsrEvent timerEvent{};
    while (g_radioOp != RadioOp::Tx && popRfTimerEvent(timerEvent)) {
        if (kBenchSingleFrequency) {
            if (timerEvent.half == static_cast<uint8_t>(rf::RxTimerHalfEvent::Tock) && uplinkFresh) {
                writeFcChannels();
            }
            continue;
        }
        const rf::RxTimerEventResult timerResult = g_rfScheduler.onTimerEvent(
            static_cast<rf::RxTimerHalfEvent>(timerEvent.half), timerEvent.timestampUs, linkFresh, nowMs);
        g_hop = g_rfScheduler.fhss().index();
        applyRfTimerCorrection(timerResult);
        if (timerEvent.half == static_cast<uint8_t>(rf::RxTimerHalfEvent::Tock) && uplinkFresh) {
            writeFcChannels();
        }
        if (timerResult.event == rf::RxTimerTickEvent::ReceiveFrequencyChanged) startReceiveOnHop();
    }

    if (!isLinkFresh(millis(), g_lastUplinkMs)) g_linkQuality = 0;
    if (millis() - g_lastLinkStatsMs >= 1000) {
        g_lastLinkStatsMs = millis();
        writeFcLinkStats();
    }
}
#endif

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.begin(115200);
    Serial.ignoreFlowControl(true);
    for (uint8_t i = 0; i < kStartupCliGraceLoops; ++i) {
        serviceCli();
        delay(100);
    }
    setDefaultChannels();
    loadConfig();
#if LORA_USB_DIAG
    Serial.println();
    Serial.println("Clean LoRa Link USB diagnostic");
    printStatus();
    g_radioReady = false;
    return;
#endif
    setupCrsfUart();
#if LORA_CRSF_DIAG
    Serial.println();
    Serial.println("Clean LoRa Link CRSF diagnostic");
    printStatus();
    g_radioReady = false;
    return;
#endif
    Serial.println();
    Serial.println("Clean LoRa Link");
    printStatus();
    g_rfScheduler.begin(g_uid, activeRate());
    g_radioReady = configureRadio(
#if LORA_RX_ROLE
        syncChannelFrequencyMHz()
#else
        rfFrequencyForHop(0)
#endif
    );
    if (!g_radioReady) Serial.println("radio fault: CLI remains available");
    else {
#if LORA_TX_ROLE
        startRfTimer();
#endif
    }
}

void loop() {
    serviceCli();
#if LORA_TX_ROLE
    txLoop();
#else
    rxLoop();
#endif
    digitalWrite(STATUS_LED_PIN, isLinkFresh(millis(), g_lastUplinkMs) ? HIGH : LOW);
    yield();
}

#endif // UNIT_TEST
