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
#else
constexpr long kLoraFrequencyHz = 920900000L;
constexpr uint32_t kLoraSpiFrequencyHz = 8000000UL;
constexpr int kLoraTxPowerDbm = 17;
#endif
constexpr int kLoraSpreadingFactor = 7;
constexpr long kLoraSignalBandwidthHz = 125000L;
constexpr int kLoraCodingRateDenominator = 5;
constexpr int kLoraSyncWord = 0x12;
constexpr uint8_t kLoraRegVersion = 0x42U;
constexpr uint8_t kLoraExpectedVersion = 0x12U;
constexpr uint8_t kLoraInitAttempts = 5U;

struct GroundLoraPinMap final
{
    static constexpr uint8_t misoPin = 1U;
    static constexpr uint8_t mosiPin = 26U;
    static constexpr uint8_t sckPin = 27U;
    static constexpr uint8_t rxEnablePin = 30U;
    static constexpr uint8_t txEnablePin = 31U;
    static constexpr uint8_t dio0Pin = 32U;
    static constexpr uint8_t dio1Pin = 8U;
    static constexpr uint8_t resetPin = 24U;
    static constexpr int8_t libraryResetPin = -1;
    static constexpr uint8_t ssPin = 9U;

    static SPIClass &spi()
    {
        return SPI1;
    }
};

uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0U;
bool radioReady = false;
uint8_t uplinkBuffer[nura::kMaxFrameLen];
size_t uplinkCount = 0U;
size_t uplinkExpectedLen = 0U;

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
    pinMode(GroundLoraPinMap::dio1Pin, INPUT);
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

        const uint8_t m0 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE0);
        const uint8_t m1 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE1);
        const uint8_t m2 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE2);
        const uint8_t m3 = readLoraRegisterRaw(kLoraRegVersion, SPI_MODE3);

        if (m1 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE1;
            selectedSpiModeNumber = 1U;
        }
        else if (m0 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0U;
        }
        else if (m2 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE2;
            selectedSpiModeNumber = 2U;
        }
        else if (m3 == kLoraExpectedVersion)
        {
            selectedSpiMode = SPI_MODE3;
            selectedSpiModeNumber = 3U;
        }
        else
        {
            delay(250);
            continue;
        }

        LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);

        if (LoRa.begin(kLoraFrequencyHz))
        {
            LoRa._spiSettings = SPISettings(kLoraSpiFrequencyHz, MSBFIRST, selectedSpiMode);
            LoRa.setTxPower(kLoraTxPowerDbm);
            LoRa.setSpreadingFactor(kLoraSpreadingFactor);
            LoRa.setSignalBandwidth(kLoraSignalBandwidthHz);
            LoRa.setCodingRate4(kLoraCodingRateDenominator);
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

// ── PC -> LoRa : 완성된 프레임을 무선 송신 ──────────────────
void sendFrameToLora(const uint8_t *frame, size_t length)
{
    if (frame == nullptr || length == 0U || length > sizeof(uplinkBuffer))
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
    LoRa.endPacket();
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

    // 초기화 메시지(텍스트). PC 의 FrameParser 는 sync(0xAA 0x55) 만 찾으므로
    // 이 텍스트는 그냥 무시되어 안전함.
    Serial.println();
    Serial.println("NURA LoRa serial bridge");
    Serial.println("role=bridge board=teensy41 protocol=v2_lite_auth");

    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: bridge radio init failed");
        return;
    }
    Serial.println("PASS: bridge radio init OK");
}

void loop()
{
    if (!radioReady)
    {
        return;
    }
    pumpSerialToLora();   // PC -> 로켓
    pumpLoraToSerial();   // 로켓 -> PC
}

#endif
