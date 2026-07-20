// ============================================================
//  NURA LoRa 시리얼 브리지  (Teensy 4.1)
// ============================================================
//  이 펌웨어는 지상국(GCS) Teensy 를 "투명한 LoRa 다리"로 만든다.
//
//    PC(Python) --USB시리얼--> Teensy --LoRa--> 로켓
//    PC(Python) <--USB시리얼-- Teensy <--LoRa-- 로켓
//
//  - PC 가 시리얼로 보낸 nura 프레임 바이트 -> 파싱 후 LoRa 로 송신
//  - LoRa 로 받은 패킷 바이트            -> 그대로 USB 시리얼로 출력
//
//  즉 프로토콜(인증/CRC/재전송)은 전부 PC 의 Python 쪽이 담당하고,
//  Teensy 는 무선 송수신만 한다.
//
//  [설치 방법]
//   이 lora_serial_bridge 폴더를 2026-nura-avionics-main/ 레포 루트에
//   (receiver/, sender/ 폴더와 같은 위치) 두고:
//      pio run -d lora_serial_bridge -t upload
//
//  LoRa RF 설정은 receiver/sender 펌웨어와 완전히 동일해야 통신됨.
// ============================================================

#include <Arduino.h>

#if defined(NURA_LR900F_UART_BRIDGE)

#include "nura_protocol_v1_lite.h"

namespace
{
constexpr unsigned long kPcSerialBaud = 115200UL;
constexpr unsigned long kRadioSerialBaud = 57600UL;

HardwareSerial &RadioSerial = Serial1;

uint8_t uplinkBuffer[nura::kMaxFrameLen];
size_t uplinkCount = 0U;
size_t uplinkExpectedLen = 0U;

void sendFrameToRadio(const uint8_t *frame, size_t length)
{
    if (frame == nullptr || length == 0U || length > sizeof(uplinkBuffer))
    {
        return;
    }
    RadioSerial.write(frame, length);
}

void pumpSerialToRadio()
{
    while (Serial.available() > 0)
    {
        const int value = Serial.read();
        if (value < 0)
        {
            break;
        }

        const uint8_t byte = static_cast<uint8_t>(value);
        if (uplinkCount == 0U)
        {
            if (byte == nura::kSync0)
            {
                uplinkBuffer[uplinkCount++] = byte;
            }
            continue;
        }
        if (uplinkCount == 1U)
        {
            if (byte == nura::kSync1)
            {
                uplinkBuffer[uplinkCount++] = byte;
            }
            else if (byte != nura::kSync0)
            {
                uplinkCount = 0U;
            }
            continue;
        }
        if (uplinkCount >= sizeof(uplinkBuffer))
        {
            uplinkCount = 0U;
            uplinkExpectedLen = 0U;
            continue;
        }

        uplinkBuffer[uplinkCount++] = byte;
        if (uplinkCount == 3U)
        {
            const uint8_t payloadLen = nura::payloadLengthForType(nura::frameType(byte));
            if (nura::frameVersion(byte) != nura::kVersion || payloadLen == 0U)
            {
                uplinkCount = 0U;
                continue;
            }
            uplinkExpectedLen = static_cast<size_t>(nura::kFrameOverhead) + payloadLen;
        }

        if (uplinkExpectedLen != 0U && uplinkCount == uplinkExpectedLen)
        {
            sendFrameToRadio(uplinkBuffer, uplinkCount);
            uplinkCount = 0U;
            uplinkExpectedLen = 0U;
        }
    }
}

void pumpRadioToSerial()
{
    while (RadioSerial.available() > 0)
    {
        const int value = RadioSerial.read();
        if (value < 0)
        {
            break;
        }
        Serial.write(static_cast<uint8_t>(value));
    }
}
} // namespace

void setup()
{
    Serial.begin(kPcSerialBaud);
    RadioSerial.begin(kRadioSerialBaud);
    while (!Serial && millis() < 4000UL)
    {
    }

    Serial.println();
    Serial.println("NURA LR900-F UART bridge");
    Serial.println("role=bridge board=teensy41 radio=lr900f pc_baud=115200 radio_baud=57600");
}

void loop()
{
    pumpSerialToRadio();
    pumpRadioToSerial();
}

#else

#include <SPI.h>

#include "board_pinmap.h"
#include "nura_protocol_v1_lite.h"

#define private public
#include <LoRa.h>
#undef private

namespace
{
constexpr unsigned long kSerialBaud = 115200UL;
#if defined(NURA_DEV_SX1278)
constexpr long kLoraFrequencyHz = 433000000L;
constexpr uint32_t kLoraSpiFrequencyHz = 125000UL;
constexpr int kLoraTxPowerDbm = 10;
#elif defined(NURA_SPARKFUN_1W_SX1276) || defined(NURA_GROUND_SX1276_LEGACY_SPI0)
constexpr long kLoraFrequencyHz = 920900000L;
constexpr uint32_t kLoraSpiFrequencyHz = 250000UL;
constexpr int kLoraTxPowerDbm = 10;
#else
constexpr long kLoraFrequencyHz = 920900000L;
constexpr uint32_t kLoraSpiFrequencyHz = 8000000UL;
constexpr int kLoraTxPowerDbm = 17;
#endif
constexpr int kLoraSpreadingFactor = 7;
constexpr long kLoraSignalBandwidthHz = 125000L;
constexpr int kLoraCodingRateDenominator = 5;
constexpr long kLoraPreambleLength = 8L;
constexpr int kLoraSyncWord = 0x12;
constexpr uint8_t kLoraRegVersion = 0x42U;
constexpr uint8_t kLoraExpectedVersion = 0x12U;
constexpr uint8_t kLoraInitAttempts = 5U;

struct GroundLoraPinMap final
{
#if defined(NURA_GROUND_SX1276_LEGACY_SPI0)
    static constexpr uint8_t misoPin = 12U;
    static constexpr uint8_t mosiPin = 11U;
    static constexpr uint8_t sckPin = 13U;
    static constexpr uint8_t rxEnablePin = 4U;
    static constexpr uint8_t txEnablePin = 3U;
    static constexpr uint8_t dio0Pin = 2U;
    static constexpr uint8_t dio1Pin = 255U;
    static constexpr uint8_t resetPin = 9U;
    static constexpr int8_t libraryResetPin = -1;
    static constexpr uint8_t ssPin = 10U;

    static SPIClass &spi()
    {
        return SPI;
    }

    static const char *profileName()
    {
        return "sx1276_ground_legacy_spi0";
    }

    static const char *hardwareName()
    {
        return "legacy_sx1276";
    }
#else
    // Keep the GCS SparkFun harness locked to the current avionics source of
    // truth instead of maintaining a second copy of the final SPI1 pin map.
    static constexpr uint8_t misoPin = BoardPinMap::Spi1Bus::misoPin;
    static constexpr uint8_t mosiPin = BoardPinMap::Spi1Bus::mosiPin;
    static constexpr uint8_t sckPin = BoardPinMap::Spi1Bus::sckPin;
    static constexpr uint8_t rxEnablePin = BoardPinMap::Sx1276BreakoutLoRa::rxEnablePin;
    static constexpr uint8_t txEnablePin = BoardPinMap::Sx1276BreakoutLoRa::txEnablePin;
    static constexpr uint8_t dio0Pin = BoardPinMap::Sx1276BreakoutLoRa::dio0Pin;
    static constexpr uint8_t dio1Pin = BoardPinMap::kUnassignedPin;
    static constexpr uint8_t resetPin = BoardPinMap::Sx1276BreakoutLoRa::resetPin;
    static constexpr int8_t libraryResetPin = BoardPinMap::Sx1276BreakoutLoRa::libraryResetPin;
    static constexpr uint8_t ssPin = BoardPinMap::Sx1276BreakoutLoRa::ssPin;

    static SPIClass &spi()
    {
        return SPI1;
    }

    static const char *profileName()
    {
#if defined(NURA_SPARKFUN_1W_SX1276)
        return "sx1276_ground";
#else
        return "spi1_ground";
#endif
    }

    static const char *hardwareName()
    {
#if defined(NURA_SPARKFUN_1W_SX1276)
        return "sparkfun_spx18572_915m30s_1w";
#else
        return "generic_sx127x";
#endif
    }
#endif
};

#if defined(NURA_SPARKFUN_1W_SX1276)
static_assert(GroundLoraPinMap::misoPin == 1U &&
                  GroundLoraPinMap::mosiPin == 26U &&
                  GroundLoraPinMap::sckPin == 27U &&
                  GroundLoraPinMap::ssPin == 9U &&
                  GroundLoraPinMap::resetPin == 24U &&
                  GroundLoraPinMap::dio0Pin == 32U &&
                  GroundLoraPinMap::rxEnablePin == 30U &&
                  GroundLoraPinMap::txEnablePin == 31U,
              "SparkFun 1W ground pin map drifted from the final avionics pin map");
#endif

uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0U;
uint8_t lastRegVersionMode0 = 0U;
#if !defined(NURA_SPARKFUN_1W_SX1276)
uint8_t lastRegVersionMode1 = 0U;
uint8_t lastRegVersionMode2 = 0U;
uint8_t lastRegVersionMode3 = 0U;
#endif
bool radioReady = false;
uint8_t uplinkBuffer[nura::kMaxFrameLen];
size_t uplinkCount = 0U;
size_t uplinkExpectedLen = 0U;
uint32_t radioRxCount = 0UL;
uint32_t radioTxCount = 0UL;
uint32_t uplinkFormatDropCount = 0UL;
int lastPacketRssi = 0;
float lastPacketSnr = 0.0f;
constexpr char kStatusCommand[] = "NURA_STATUS\n";
size_t statusCommandMatch = 0U;

void setRadioReceiveMode()
{
    digitalWrite(GroundLoraPinMap::txEnablePin, LOW);
    digitalWrite(GroundLoraPinMap::rxEnablePin, HIGH);
}

void setRadioTransmitMode()
{
    digitalWrite(GroundLoraPinMap::rxEnablePin, LOW);
    digitalWrite(GroundLoraPinMap::txEnablePin, HIGH);
}

// ── LoRa 초기화 (receiver/sender 펌웨어와 동일) ──────────────
void beginSpi()
{
#if defined(CORE_TEENSY)
    GroundLoraPinMap::spi().setMOSI(GroundLoraPinMap::mosiPin);
    GroundLoraPinMap::spi().setMISO(GroundLoraPinMap::misoPin);
    GroundLoraPinMap::spi().setSCK(GroundLoraPinMap::sckPin);
#endif
    GroundLoraPinMap::spi().begin();
}

uint8_t readLoraRegisterRaw(uint8_t address, uint8_t spiMode)
{
    SPISettings settings(kLoraSpiFrequencyHz, MSBFIRST, spiMode);
    pinMode(GroundLoraPinMap::ssPin, OUTPUT);
    digitalWrite(GroundLoraPinMap::ssPin, HIGH);
    GroundLoraPinMap::spi().beginTransaction(settings);
    digitalWrite(GroundLoraPinMap::ssPin, LOW);
    delayMicroseconds(20);
    GroundLoraPinMap::spi().transfer(address & 0x7FU);
    const uint8_t value = GroundLoraPinMap::spi().transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(GroundLoraPinMap::ssPin, HIGH);
    GroundLoraPinMap::spi().endTransaction();
    return value;
}

void resetRadio()
{
    pinMode(GroundLoraPinMap::ssPin, OUTPUT);
    pinMode(GroundLoraPinMap::rxEnablePin, OUTPUT);
    pinMode(GroundLoraPinMap::txEnablePin, OUTPUT);
    pinMode(GroundLoraPinMap::dio0Pin, INPUT);
    if (GroundLoraPinMap::dio1Pin != 255U)
    {
        pinMode(GroundLoraPinMap::dio1Pin, INPUT);
    }
    digitalWrite(GroundLoraPinMap::ssPin, HIGH);
    setRadioReceiveMode();
    pinMode(GroundLoraPinMap::resetPin, OUTPUT);
    digitalWrite(GroundLoraPinMap::resetPin, LOW);
    delay(50);
    digitalWrite(GroundLoraPinMap::resetPin, HIGH);
    delay(500);
}

bool beginRadio()
{
    LoRa.setSPI(GroundLoraPinMap::spi());
    LoRa.setPins(GroundLoraPinMap::ssPin,
                 GroundLoraPinMap::libraryResetPin,
                 GroundLoraPinMap::dio0Pin);
    LoRa.setSPIFrequency(kLoraSpiFrequencyHz);

    for (uint8_t attempt = 1U; attempt <= kLoraInitAttempts; ++attempt)
    {
        beginSpi();
        resetRadio();

        lastRegVersionMode0 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE0);
#if defined(NURA_SPARKFUN_1W_SX1276)
        // SX1276 and the SparkFun reference firmware use CPOL=0/CPHA=0.
        // Do not accept a marginal read from a different mode on the long
        // breakout harness; the avionics firmware is also fixed to MODE0.
        if (lastRegVersionMode0 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else
        {
            delay(250);
            continue;
        }
#else
        lastRegVersionMode1 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE1);
        lastRegVersionMode2 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE2);
        lastRegVersionMode3 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE3);

        if (lastRegVersionMode1 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE1;
            selectedSpiModeNumber = 1U;
        }
        else if (lastRegVersionMode0 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else if (lastRegVersionMode2 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE2;
            selectedSpiModeNumber = 2U;
        }
        else if (lastRegVersionMode3 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE3;
            selectedSpiModeNumber = 3U;
        }
        else
        {
            delay(250);
            continue;
        }
#endif

        LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);

        if (LoRa.begin(kLoraFrequencyHz))
        {
            LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
            LoRa.setTxPower(kLoraTxPowerDbm);
            LoRa.setSpreadingFactor(kLoraSpreadingFactor);
            LoRa.setSignalBandwidth(kLoraSignalBandwidthHz);
            LoRa.setCodingRate4(kLoraCodingRateDenominator);
            LoRa.setPreambleLength(kLoraPreambleLength);
            LoRa.setSyncWord(kLoraSyncWord);
            LoRa.enableCrc();
            setRadioReceiveMode();
            LoRa.receive();
            return true;
        }

        LoRa.end();
        delay(250);
    }
    return false;
}

void printBridgeStatus()
{
    Serial.print("\nNURA_BRIDGE radio=");
    Serial.print(radioReady ? "ready" : "failed");
    Serial.print(" profile=");
    Serial.print(GroundLoraPinMap::profileName());
    Serial.print(" hardware=");
    Serial.print(GroundLoraPinMap::hardwareName());
    Serial.print(" pins=miso:");
    Serial.print(GroundLoraPinMap::misoPin);
    Serial.print(",mosi:");
    Serial.print(GroundLoraPinMap::mosiPin);
    Serial.print(",sck:");
    Serial.print(GroundLoraPinMap::sckPin);
    Serial.print(",cs:");
    Serial.print(GroundLoraPinMap::ssPin);
    Serial.print(",rst:");
    Serial.print(GroundLoraPinMap::resetPin);
    Serial.print(",dio0:");
    Serial.print(GroundLoraPinMap::dio0Pin);
    Serial.print(",rxen:");
    Serial.print(GroundLoraPinMap::rxEnablePin);
    Serial.print(",txen:");
    Serial.print(GroundLoraPinMap::txEnablePin);
    Serial.print(" transport=raw frequency_hz=");
    Serial.print(kLoraFrequencyHz);
    Serial.print(" spi_hz=");
    Serial.print(kLoraSpiFrequencyHz);
    Serial.print(" spi_mode=");
    Serial.print(selectedSpiModeNumber);
    Serial.print(" reg42_m0=0x");
    if (lastRegVersionMode0 < 16U)
    {
        Serial.print('0');
    }
    Serial.print(lastRegVersionMode0, HEX);
    Serial.print(" rx_packets=");
    Serial.print(radioRxCount);
    Serial.print(" tx_packets=");
    Serial.print(radioTxCount);
    Serial.print(" uplink_format_drop=");
    Serial.print(uplinkFormatDropCount);
    Serial.print(" last_rssi=");
    if (radioRxCount > 0UL)
    {
        Serial.print(lastPacketRssi);
    }
    else
    {
        Serial.print("na");
    }
    Serial.print(" last_snr=");
    if (radioRxCount > 0UL)
    {
        Serial.println(lastPacketSnr, 2);
    }
    else
    {
        Serial.println("na");
    }
}

void feedStatusCommand(uint8_t byte)
{
    if (byte == static_cast<uint8_t>(kStatusCommand[statusCommandMatch]))
    {
        ++statusCommandMatch;
        if (kStatusCommand[statusCommandMatch] == '\0')
        {
            statusCommandMatch = 0U;
            printBridgeStatus();
        }
        return;
    }
    statusCommandMatch =
        byte == static_cast<uint8_t>(kStatusCommand[0]) ? 1U : 0U;
}

// ── PC -> LoRa : 완성된 프레임을 무선 송신 ──────────────────
void sendFrameToLora(const uint8_t *frame, size_t length)
{
    if (!radioReady || frame == nullptr || length == 0U || length > sizeof(uplinkBuffer))
    {
        return;
    }

    LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
    setRadioTransmitMode();
    LoRa.idle();
    delay(2);
    if (!LoRa.beginPacket())
    {
        setRadioReceiveMode();
        LoRa.receive();
        return;
    }
    LoRa.write(frame, length);
    if (LoRa.endPacket() == 1)
    {
        ++radioTxCount;
    }
    setRadioReceiveMode();
    LoRa.receive();
}

// ── PC -> LoRa : USB 시리얼에서 들어온 바이트 처리 ──────────
void pumpSerialToLora()
{
    while (Serial.available() > 0)
    {
        const int value = Serial.read();
        if (value < 0)
        {
            break;
        }
        const uint8_t byte = static_cast<uint8_t>(value);
        if (uplinkCount == 0U)
        {
            feedStatusCommand(byte);
        }
        if (uplinkCount == 0U)
        {
            if (byte == nura::kSync0)
            {
                uplinkBuffer[uplinkCount++] = byte;
            }
            continue;
        }
        if (uplinkCount == 1U)
        {
            if (byte == nura::kSync1)
            {
                uplinkBuffer[uplinkCount++] = byte;
            }
            else if (byte != nura::kSync0)
            {
                uplinkCount = 0U;
            }
            continue;
        }
        if (uplinkCount >= sizeof(uplinkBuffer))
        {
            uplinkCount = 0U;
            uplinkExpectedLen = 0U;
            continue;
        }

        uplinkBuffer[uplinkCount++] = byte;
        if (uplinkCount == 3U)
        {
            const uint8_t payloadLen = nura::payloadLengthForType(nura::frameType(byte));
            if (nura::frameVersion(byte) != nura::kVersion || payloadLen == 0U)
            {
                ++uplinkFormatDropCount;
                uplinkCount = 0U;
                continue;
            }
            uplinkExpectedLen = static_cast<size_t>(nura::kFrameOverhead) + payloadLen;
        }

        if (uplinkExpectedLen != 0U && uplinkCount == uplinkExpectedLen)
        {
            sendFrameToLora(uplinkBuffer, uplinkCount);
            uplinkCount = 0U;
            uplinkExpectedLen = 0U;
        }
    }
}

// ── LoRa -> PC : 받은 패킷을 그대로 USB 시리얼로 출력 ───────
void pumpLoraToSerial()
{
    const int packetSize = LoRa.parsePacket();
    if (packetSize <= 0)
    {
        return;
    }
    ++radioRxCount;
    lastPacketRssi = LoRa.packetRssi();
    lastPacketSnr = LoRa.packetSnr();
    while (LoRa.available() > 0)
    {
        const int value = LoRa.read();
        if (value < 0)
        {
            break;
        }
        Serial.write(static_cast<uint8_t>(value));
    }
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    while (!Serial && millis() < 4000UL)
    {
    }

    // NURA_BRIDGE 줄은 Python이 진단 정보로만 수집한다. 이후 무선 데이터는
    // 인증 프레임 원문 그대로 내보내며 텔레메트리 텍스트와 섞지 않는다.
    Serial.println("\nNURA_BRIDGE boot=1 role=bridge board=teensy41 protocol=v2_lite_auth");

    radioReady = beginRadio();
    printBridgeStatus();
    Serial.println("NURA_BRIDGE raw=begin");
}

void loop()
{
    pumpSerialToLora();   // PC -> 로켓
    if (radioReady)
    {
        pumpLoraToSerial();   // 로켓 -> PC
    }
}

#endif
