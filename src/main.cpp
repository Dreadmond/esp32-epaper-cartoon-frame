#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/adc.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <math.h>

#ifndef PNG_MAX_BUFFERED_PIXELS
#define PNG_MAX_BUFFERED_PIXELS ((1872 * 4 + 1) * 2)
#endif

#include <PNGdec.h>
#include <FS.h>
#include <SPIFFS.h>
#include <algorithm>

#include "secrets.h"

#ifndef SECRET_GITHUB_OWNER
#define SECRET_GITHUB_OWNER ""
#endif
#ifndef SECRET_GITHUB_REPO
#define SECRET_GITHUB_REPO ""
#endif
#ifndef SECRET_GITHUB_FW_ASSET
#define SECRET_GITHUB_FW_ASSET "firmware.bin"
#endif
#ifndef SECRET_GITHUB_TOKEN
#define SECRET_GITHUB_TOKEN ""
#endif
#ifndef APP_VERSION
#define APP_VERSION "0.0.0"
#endif

#ifndef SECRET_NEXTCLOUD_CARTOON_103
#define SECRET_NEXTCLOUD_CARTOON_103 "nyc_cartoon.png"
#endif

#ifndef SECRET_NEXTCLOUD_PHOTO_103
#define SECRET_NEXTCLOUD_PHOTO_103 "immich_bobbie_aoife_epd103.png"
#endif

// FireBeetle 2 ESP32-E VSPI pins wired to Waveshare IT8951 10.3" panel
constexpr uint8_t PIN_EPD_SCK = 18;
constexpr uint8_t PIN_EPD_MOSI = 23;
constexpr uint8_t PIN_EPD_MISO = 19;
constexpr uint8_t PIN_EPD_CS = 4;
constexpr uint8_t PIN_EPD_RESET = 17;
constexpr uint8_t PIN_EPD_HRDY = 16;
constexpr uint8_t PIN_TPL5110_DONE = 13;  // Signal TPL5110 to cut power (GPIO13 - no boot glitch)
constexpr gpio_num_t PIN_BATTERY_ADC = GPIO_NUM_34; // FireBeetle BAT (A2 / ADC1_CH6)
constexpr char PNG_STORAGE_PATH[] = "/nyc_cartoon.png";
constexpr char PNG_TEMP_DOWNLOAD_PATH[] = "/.png_dl.tmp";

// Panel characteristics
constexpr uint16_t PANEL_WIDTH = 1872;
constexpr uint16_t PANEL_HEIGHT = 1404;

// IT8951 command codes and helpers
constexpr uint16_t IT8951_TCON_SYS_RUN = 0x0001;
constexpr uint16_t IT8951_TCON_SLEEP = 0x0003;
constexpr uint16_t IT8951_TCON_REG_RD = 0x0010;
constexpr uint16_t IT8951_TCON_REG_WR = 0x0011;
constexpr uint16_t IT8951_TCON_MEM_BST_WR = 0x0014;
constexpr uint16_t IT8951_TCON_MEM_BST_END = 0x0015;
constexpr uint16_t IT8951_TCON_LD_IMG = 0x0020;
constexpr uint16_t IT8951_TCON_LD_IMG_AREA = 0x0021;
constexpr uint16_t IT8951_TCON_LD_IMG_END = 0x0022;
constexpr uint16_t USDEF_I80_CMD_DPY_AREA = 0x0034;
constexpr uint16_t USDEF_I80_CMD_GET_DEV_INFO = 0x0302;

constexpr uint16_t IT8951_ROTATE_0 = 0;
constexpr uint16_t IT8951_4BPP = 2;
constexpr uint16_t IT8951_MODE_INIT = 0;  // INIT mode - full clear with flash
constexpr uint16_t IT8951_MODE_GC16 = 2;  // GC16 mode - 16 grayscale
constexpr uint16_t IT8951_LDIMG_L_ENDIAN = 0;

constexpr uint16_t DISPLAY_REG_BASE = 0x1000;
constexpr uint16_t LUTAFSR = DISPLAY_REG_BASE + 0x224;
constexpr uint16_t SYS_REG_BASE = 0x0000;
constexpr uint16_t I80CPCR = SYS_REG_BASE + 0x04;
constexpr uint16_t LISAR = 0x0200 + 0x0008;

// VCOM commands (user-defined IT8951 commands, not register writes)
constexpr uint16_t USDEF_I80_CMD_VCOM = 0x0039;  // Set VCOM command

// VCOM value from panel label (-1.52V)
constexpr int16_t PANEL_VCOM_MV = 1520;

struct IT8951DevInfo
{
    uint16_t usPanelW;
    uint16_t usPanelH;
    uint16_t usImgBufAddrL;
    uint16_t usImgBufAddrH;
    uint16_t usFWVersion[8];
    uint16_t usLUTVersion[8];
};

struct IT8951LdImgInfo
{
    uint16_t usEndianType;
    uint16_t usPixelFormat;
    uint16_t usRotate;
    uint32_t ulImgBufBaseAddr;
};

struct IT8951AreaImgInfo
{
    uint16_t usX;
    uint16_t usY;
    uint16_t usWidth;
    uint16_t usHeight;
};

struct OtaReleaseInfo
{
    String version;
    String downloadUrl;
};

static SPIClass spi(VSPI);
static PNG g_png;
static IT8951DevInfo g_devInfo{};
static uint32_t g_imgBufAddr = 0;
static bool g_streamActive = false;
static IT8951AreaImgInfo g_streamArea{};
static uint8_t g_lineGray[PANEL_WIDTH];

static constexpr uint64_t SLEEP_INTERVAL_US = 3600ULL * 1000000ULL;
static constexpr float BATTERY_VOLTAGE_DIVIDER = 2.0f; // Adjust to actual hardware divider ratio.
static constexpr char FIRMWARE_VERSION[] = APP_VERSION;
static const char *MQTT_DEVICE_ID = "cartoon_frame";
static const char *MQTT_DEVICE_NAME = "Cartoon Frame";
static const char *MQTT_STATE_TOPIC = "cartoon_frame/state";
static const char *MQTT_DISCOVERY_PREFIX = "homeassistant";

static WiFiClient mqttNetClient;
static PubSubClient mqttClient(mqttNetClient);
static bool mqttDiscoveryPublished = false;

// Boot counter for configurable refresh interval (stored in NVS)
static Preferences prefs;
static uint32_t g_bootCount = 0;
static uint32_t g_refreshInterval = 1;  // Refresh every N boots (1 = every boot)
static const char* MQTT_REFRESH_INTERVAL_TOPIC = "cartoon_frame/refresh_interval/set";
static const char* MQTT_REFRESH_INTERVAL_STATE_TOPIC = "cartoon_frame/refresh_interval";
static const char* MQTT_BOOT_COUNT_TOPIC = "cartoon_frame/boot_count";

// Image source selection (Cartoon or Photo)
static String g_imageSource = "Cartoon";  // Default to Cartoon
static const char* MQTT_IMAGE_SOURCE_TOPIC = "cartoon_frame/image_source/set";
static const char* MQTT_IMAGE_SOURCE_STATE_TOPIC = "cartoon_frame/image_source";

// Load boot counter, refresh interval, and image source from NVS
void loadSettings()
{
    prefs.begin("cartoon", false);
    g_bootCount = prefs.getUInt("boot_count", 0);
    g_refreshInterval = prefs.getUInt("refresh_int", 1);
    if (g_refreshInterval < 1) g_refreshInterval = 1;  // Minimum 1
    g_imageSource = prefs.getString("img_source", "Cartoon");
    Serial.printf("Boot count: %u, Refresh interval: %u boots, Image source: %s\n", 
                  g_bootCount, g_refreshInterval, g_imageSource.c_str());
}

// Increment and save boot counter
void incrementBootCounter()
{
    g_bootCount++;
    prefs.putUInt("boot_count", g_bootCount);
    Serial.printf("Boot count incremented to: %u\n", g_bootCount);
}

// Reset boot counter (after display refresh)
void resetBootCounter()
{
    g_bootCount = 0;
    prefs.putUInt("boot_count", g_bootCount);
    Serial.println("Boot count reset to 0");
}

// Set refresh interval and save to NVS
void setRefreshInterval(uint32_t interval)
{
    if (interval < 1) interval = 1;
    g_refreshInterval = interval;
    prefs.putUInt("refresh_int", g_refreshInterval);
    Serial.printf("Refresh interval set to: %u boots\n", g_refreshInterval);
}

// Set image source and save to NVS
void setImageSource(const String& source)
{
    if (source == "Cartoon" || source == "Photo") {
        g_imageSource = source;
        prefs.putString("img_source", g_imageSource);
        Serial.printf("Image source set to: %s\n", g_imageSource.c_str());
    } else {
        Serial.printf("Invalid image source: %s (must be 'Cartoon' or 'Photo')\n", source.c_str());
    }
}

// Check if we should refresh the display this boot
bool shouldRefreshDisplay()
{
    return (g_bootCount >= g_refreshInterval);
}

// Watchdog timer - forces shutdown after 2 minutes unless OTA is in progress
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 120000;  // 2 minutes
static uint32_t g_bootTime = 0;
static bool g_otaInProgress = false;

void initWatchdog()
{
    g_bootTime = millis();
    Serial.printf("Watchdog initialized - will force shutdown in %u seconds unless OTA\n", WATCHDOG_TIMEOUT_MS / 1000);
}

void checkWatchdog()
{
    if (g_otaInProgress) return;  // Don't trigger during OTA
    
    uint32_t elapsed = millis() - g_bootTime;
    if (elapsed >= WATCHDOG_TIMEOUT_MS) {
        Serial.printf("WATCHDOG TIMEOUT after %u ms - forcing shutdown!\n", elapsed);
        Serial.flush();
        // Force immediate shutdown
        pinMode(PIN_TPL5110_DONE, OUTPUT);
        digitalWrite(PIN_TPL5110_DONE, HIGH);
        delay(100);
        digitalWrite(PIN_TPL5110_DONE, LOW);
        while(1) { delay(1000); }
    }
}

void disableWatchdogForOTA()
{
    g_otaInProgress = true;
    Serial.println("Watchdog disabled for OTA update");
}

// Initialize TPL5110 DONE pin - must be LOW while running
void initTPL5110Done()
{
    pinMode(PIN_TPL5110_DONE, OUTPUT);
    digitalWrite(PIN_TPL5110_DONE, LOW);  // Keep LOW while code is running
}

// Signal TPL5110 that we're done - toggle HIGH then LOW to cut power
void signalTPL5110Done()
{
    Serial.println("Waiting 5s before signaling TPL5110...");
    delay(5000);  // Wait before signaling (allow display refresh to complete)
    
    Serial.println("Signaling TPL5110 DONE - power will be cut");
    Serial.flush();
    
    // Toggle HIGH then LOW to signal done
    digitalWrite(PIN_TPL5110_DONE, HIGH);
    delay(100);
    digitalWrite(PIN_TPL5110_DONE, LOW);
    delay(100);
    
    // Power will be cut by TPL5110 - we won't reach past here
    while(1) { delay(1000); }  // Wait for power cut
}

static void setupMqtt();
static bool ensureMqttConnected();
static void publishHomeAssistantDiscovery();
static void publishTelemetry(bool refreshSuccess, float refreshDurationMs, float batteryVoltage);
static float readBatteryVoltage();
static String buildDeviceDescriptorJson();
static bool displayTestPattern();
static bool maybePerformOtaUpdate();
static bool fetchLatestGitHubRelease(OtaReleaseInfo &info);
static bool extractJsonString(const String &json, const String &key, String &out);
static String normalizeVersionTag(const String &ver);

static constexpr bool ENABLE_TEST_PATTERN_FALLBACK = true;
static constexpr uint16_t SPI_PREAMBLE_CMD = 0x6000;
static constexpr uint16_t SPI_PREAMBLE_DATA = 0x0000;
static constexpr uint16_t SPI_PREAMBLE_READ = 0x1000;

bool waitForReady(uint32_t timeoutMs = 5000)
{
    const uint32_t deadline = millis() + timeoutMs;
    while (digitalRead(PIN_EPD_HRDY) == LOW)
    {
        if (millis() > deadline)
        {
            return false;
        }
        delayMicroseconds(50);
    }
    return true;
}

void spiWriteWord(uint16_t word)
{
    spi.transfer(static_cast<uint8_t>(word >> 8));
    spi.transfer(static_cast<uint8_t>(word & 0xFF));
}

uint16_t spiReadWord()
{
    const uint16_t hi = spi.transfer(0x00);
    const uint16_t lo = spi.transfer(0x00);
    return static_cast<uint16_t>((hi << 8) | lo);
}

bool lcdWriteCmd(uint16_t cmd)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_CMD);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    spiWriteWord(cmd);
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

bool lcdWriteData(uint16_t data)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_DATA);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    spiWriteWord(data);
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

// Bulk write - keeps CS LOW for all data (much faster, matches Waveshare LCDWriteNData)
bool lcdWriteDataBulk(const uint16_t *data, uint32_t count)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_DATA);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    // Write ALL data with CS held LOW
    for (uint32_t i = 0; i < count; i++)
    {
        spiWriteWord(data[i]);
    }
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

// Bulk write same value - for solid color fills
bool lcdWriteDataBulkSame(uint16_t value, uint32_t count)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_DATA);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    // Write same value with CS held LOW
    uint8_t hi = value >> 8;
    uint8_t lo = value & 0xFF;
    for (uint32_t i = 0; i < count; i++)
    {
        spi.transfer(hi);
        spi.transfer(lo);
    }
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

// Start bulk write - call this, then spiWriteWord for each word, then lcdEndBulkWrite
void lcdStartBulkWrite()
{
    waitForReady();
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_DATA);
    waitForReady();
}

void lcdEndBulkWrite()
{
    digitalWrite(PIN_EPD_CS, HIGH);
}

bool lcdReadData(uint16_t &data)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_READ);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    spiReadWord(); // dummy
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    data = spiReadWord();
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

bool lcdReadWords(uint16_t *buf, uint32_t count)
{
    if (!waitForReady())
        return false;
    digitalWrite(PIN_EPD_CS, LOW);
    spiWriteWord(SPI_PREAMBLE_READ);
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    spiReadWord(); // dummy
    if (!waitForReady())
    {
        digitalWrite(PIN_EPD_CS, HIGH);
        return false;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        buf[i] = spiReadWord();
    }
    digitalWrite(PIN_EPD_CS, HIGH);
    return true;
}

bool it8951WriteReg(uint16_t reg, uint16_t value)
{
    return lcdWriteCmd(IT8951_TCON_REG_WR) && lcdWriteData(reg) && lcdWriteData(value);
}

bool it8951ReadReg(uint16_t reg, uint16_t &value)
{
    return lcdWriteCmd(IT8951_TCON_REG_RD) && lcdWriteData(reg) && lcdReadData(value);
}

bool it8951SetImgBufAddr(uint32_t addr)
{
    const uint16_t high = static_cast<uint16_t>((addr >> 16) & 0xFFFF);
    const uint16_t low = static_cast<uint16_t>(addr & 0xFFFF);
    return it8951WriteReg(LISAR + 2, high) && it8951WriteReg(LISAR, low);
}

bool it8951SetVCOM(int16_t vcomMv)
{
    // VCOM is set via user-defined command 0x0039
    // Value is positive millivolts (e.g., 1520 for -1.52V)
    uint16_t vcomVal = static_cast<uint16_t>(abs(vcomMv));
    Serial.printf("Setting VCOM to -%d mV via command 0x0039\n", vcomVal);
    
    // Send VCOM command with value
    if (!lcdWriteCmd(USDEF_I80_CMD_VCOM))
        return false;
    if (!lcdWriteData(1))  // 1 = set VCOM
        return false;
    if (!lcdWriteData(vcomVal))
        return false;
    
    return true;
}

bool it8951GetVCOM(int16_t &vcomMv)
{
    // Read VCOM via user-defined command 0x0039
    if (!lcdWriteCmd(USDEF_I80_CMD_VCOM))
        return false;
    if (!lcdWriteData(0))  // 0 = read VCOM
        return false;
    
    uint16_t val;
    if (!lcdReadData(val))
        return false;
    
    vcomMv = static_cast<int16_t>(val);
    return true;
}

bool it8951WaitForDisplayReady()
{
    uint16_t busy = 1;
    const uint32_t deadline = millis() + 30000;  // Increased timeout for large displays (30s)
    while (millis() < deadline)
    {
        if (!it8951ReadReg(LUTAFSR, busy))
            return false;
        if (busy == 0)
            return true;
        delay(10);
    }
    return false;
}

bool it8951GetSystemInfo(IT8951DevInfo &info)
{
    if (!lcdWriteCmd(USDEF_I80_CMD_GET_DEV_INFO))
        return false;
    const uint32_t words = sizeof(IT8951DevInfo) / 2;
    return lcdReadWords(reinterpret_cast<uint16_t *>(&info), words);
}

bool it8951LoadImgAreaStart(const IT8951LdImgInfo &load, const IT8951AreaImgInfo &area)
{
    const uint16_t args[5] = {
        static_cast<uint16_t>((load.usEndianType << 8) | (load.usPixelFormat << 4) | load.usRotate),
        area.usX,
        area.usY,
        area.usWidth,
        area.usHeight};
    if (!lcdWriteCmd(IT8951_TCON_LD_IMG_AREA))
        return false;
    for (uint16_t arg : args)
    {
        if (!lcdWriteData(arg))
            return false;
    }
    return true;
}

bool it8951LoadImgEnd()
{
    return lcdWriteCmd(IT8951_TCON_LD_IMG_END);
}

bool it8951DisplayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode)
{
    // Matches Waveshare IT8951DisplayArea() exactly
    if (!lcdWriteCmd(USDEF_I80_CMD_DPY_AREA)) return false;  // 0x0034
    if (!lcdWriteData(x)) return false;
    if (!lcdWriteData(y)) return false;
    if (!lcdWriteData(w)) return false;
    if (!lcdWriteData(h)) return false;
    if (!lcdWriteData(mode)) return false;
    return true;
}

bool it8951DisplayAreaBuf(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode, uint32_t addr)
{
    return lcdWriteCmd(0x0037) && lcdWriteData(x) && lcdWriteData(y) && lcdWriteData(w) &&
           lcdWriteData(h) && lcdWriteData(mode) && lcdWriteData(static_cast<uint16_t>(addr)) &&
           lcdWriteData(static_cast<uint16_t>(addr >> 16));
}

bool it8951WriteImage4bpp(const IT8951LdImgInfo &load, const IT8951AreaImgInfo &area, const uint8_t *image8)
{
    if (!it8951SetImgBufAddr(load.ulImgBufBaseAddr))
        return false;
    if (!it8951LoadImgAreaStart(load, area))
        return false;

    for (uint32_t y = 0; y < area.usHeight; ++y)
    {
        const uint8_t *row = image8 + y * area.usWidth;
        uint32_t x = 0;
        while (x < area.usWidth)
        {
            uint8_t p0 = row[x];
            uint8_t p1 = (x + 1 < area.usWidth) ? row[x + 1] : p0;
            uint8_t p2 = (x + 2 < area.usWidth) ? row[x + 2] : p0;
            uint8_t p3 = (x + 3 < area.usWidth) ? row[x + 3] : p0;
            uint8_t n0 = p0 >> 4;
            uint8_t n1 = p1 >> 4;
            uint8_t n2 = p2 >> 4;
            uint8_t n3 = p3 >> 4;
            uint8_t b0 = (n1 << 4) | n0;
            uint8_t b1 = (n3 << 4) | n2;
            uint16_t packed = static_cast<uint16_t>((b1 << 8) | b0);
            if (!lcdWriteData(packed))
                return false;
            x += 4;
        }
    }
    return it8951LoadImgEnd();
}

// 8BPP mode constant (matches Waveshare IT8951.h)
constexpr uint16_t IT8951_8BPP = 3;

bool beginFrameStream(uint16_t width, uint16_t height)
{
    if (!it8951WaitForDisplayReady())
        return false;

    IT8951LdImgInfo load{};
    load.usEndianType = IT8951_LDIMG_L_ENDIAN;
    load.usPixelFormat = IT8951_8BPP;  // Use 8BPP like Waveshare example
    load.usRotate = IT8951_ROTATE_0;
    load.ulImgBufBaseAddr = g_imgBufAddr;

    g_streamArea.usX = 0;
    g_streamArea.usY = 0;
    g_streamArea.usWidth = width;
    g_streamArea.usHeight = height;

    if (!it8951SetImgBufAddr(g_imgBufAddr))
        return false;
    if (!it8951LoadImgAreaStart(load, g_streamArea))
        return false;

    // Start bulk write mode (CS stays LOW for all pixel data)
    lcdStartBulkWrite();
    
    g_streamActive = true;
    return true;
}

bool streamGrayRow(const uint8_t *row, uint16_t width)
{
    if (!g_streamActive)
        return false;
    
    // 8BPP mode: 2 pixels per 16-bit word
    // Each pixel is 1 byte (high nibble is used for gray level)
    // Bulk write entire row with CS held LOW
    for (uint32_t x = 0; x < width; x += 2)
    {
        uint8_t p0 = row[x];
        uint8_t p1 = (x + 1 < width) ? row[x + 1] : p0;
        
        // Pack 2 pixels into 16-bit word (8BPP mode)
        uint16_t packed = (p0 << 8) | p1;
        spiWriteWord(packed);
    }
    return true;
}

bool endFrameStream()
{
    if (!g_streamActive)
        return false;
    g_streamActive = false;
    
    // End bulk write mode (release CS)
    lcdEndBulkWrite();
    
    return it8951LoadImgEnd();
}

bool initDisplay()
{
    pinMode(PIN_EPD_CS, OUTPUT);
    pinMode(PIN_EPD_RESET, OUTPUT);
    pinMode(PIN_EPD_HRDY, INPUT);
    digitalWrite(PIN_EPD_CS, HIGH);

    // Use faster SPI speed for quicker image transfers (12MHz)
    spi.begin(PIN_EPD_SCK, PIN_EPD_MISO, PIN_EPD_MOSI, PIN_EPD_CS);
    spi.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

    // Hardware reset sequence
    digitalWrite(PIN_EPD_RESET, LOW);
    delay(100);
    digitalWrite(PIN_EPD_RESET, HIGH);
    delay(100);

    // CRITICAL: Wait for HRDY after reset before sending any commands
    Serial.println("Waiting for IT8951 ready after reset...");
    if (!waitForReady(10000))
    {
        Serial.println("IT8951 not ready after reset (HRDY timeout)");
        return false;
    }
    Serial.println("IT8951 HRDY asserted");

    // Wake up the controller
    if (!lcdWriteCmd(IT8951_TCON_SYS_RUN))
    {
        Serial.println("Failed to send SYS_RUN command");
        return false;
    }

    // Wait for system to be ready
    if (!waitForReady(5000))
    {
        Serial.println("IT8951 not ready after SYS_RUN");
        return false;
    }

    if (!it8951GetSystemInfo(g_devInfo))
    {
        Serial.println("Failed to read IT8951 dev info");
        return false;
    }

    g_imgBufAddr = g_devInfo.usImgBufAddrL | (static_cast<uint32_t>(g_devInfo.usImgBufAddrH) << 16);

    // Print firmware version for debugging
    char fwVersion[17] = {0};
    for (int i = 0; i < 8; i++)
    {
        fwVersion[i * 2] = g_devInfo.usFWVersion[i] >> 8;
        fwVersion[i * 2 + 1] = g_devInfo.usFWVersion[i] & 0xFF;
    }
    Serial.printf("IT8951 FW version: %s\n", fwVersion);

    // Read current VCOM from IT8951
    int16_t currentVcom = 0;
    if (it8951GetVCOM(currentVcom))
    {
        Serial.printf("Current VCOM stored in IT8951: -%d mV\n", currentVcom);
    }
    else
    {
        Serial.println("Failed to read VCOM");
    }

    // Only set VCOM if it differs significantly from panel label value
    // Note: IT8951 stores a calibration value that may differ from label
    if (abs(currentVcom - PANEL_VCOM_MV) > 100)
    {
        Serial.printf("Setting VCOM to panel label value: -%d mV\n", PANEL_VCOM_MV);
        if (!it8951SetVCOM(PANEL_VCOM_MV))
        {
            Serial.println("Warning: Failed to set VCOM");
        }
        
        // Verify
        int16_t verifyVcom;
        if (it8951GetVCOM(verifyVcom))
        {
            Serial.printf("VCOM after set: -%d mV\n", verifyVcom);
        }
    }
    else
    {
        Serial.println("VCOM already close to target, keeping current value");
    }

    // Enable packed write mode (1 pixel per byte packing for 4bpp)
    if (!it8951WriteReg(I80CPCR, 0x0001))
    {
        Serial.println("Failed to enable packed mode");
        return false;
    }

    Serial.printf("IT8951 panel %ux%u, imgBuf 0x%08lx\n",
                  g_devInfo.usPanelW, g_devInfo.usPanelH, static_cast<unsigned long>(g_imgBufAddr));
    return true;
}

// Clear display to white using INIT mode (required for first use or to reset panel state)
bool clearDisplay()
{
    Serial.println("Clearing display (INIT mode)...");
    
    if (!it8951WaitForDisplayReady())
    {
        Serial.println("Display busy before clear");
        return false;
    }

    // Fill the buffer with white using 8BPP mode (matches Waveshare)
    IT8951LdImgInfo load{};
    load.usEndianType = IT8951_LDIMG_L_ENDIAN;
    load.usPixelFormat = IT8951_8BPP;  // 8BPP like Waveshare
    load.usRotate = IT8951_ROTATE_0;
    load.ulImgBufBaseAddr = g_imgBufAddr;

    IT8951AreaImgInfo area{};
    area.usX = 0;
    area.usY = 0;
    area.usWidth = g_devInfo.usPanelW;
    area.usHeight = g_devInfo.usPanelH;

    if (!it8951SetImgBufAddr(g_imgBufAddr))
    {
        Serial.println("Failed to set image buffer address for clear");
        return false;
    }

    if (!it8951LoadImgAreaStart(load, area))
    {
        Serial.println("Failed to start clear image load");
        return false;
    }

    // Write white pixels using bulk write (8BPP: 2 pixels per word, 0xF0 = white)
    const uint32_t totalWords = (g_devInfo.usPanelW / 2) * g_devInfo.usPanelH;
    
    // Bulk write all white pixels
    lcdWriteDataBulkSame(0xF0F0, totalWords);  // 0xF0F0 = two white pixels

    if (!it8951LoadImgEnd())
    {
        Serial.println("Failed to end clear image load");
        return false;
    }

    // Use INIT mode (mode 0) to clear - this does a full hardware clear
    Serial.println("Sending INIT refresh command (0x0034)...");
    if (!it8951DisplayArea(0, 0, g_devInfo.usPanelW, g_devInfo.usPanelH, 0))
    {
        Serial.println("Clear display command failed");
        return false;
    }

    if (!it8951WaitForDisplayReady())
    {
        Serial.println("Clear display timeout");
        return false;
    }

    // Allow time for physical e-ink settling after clear
    Serial.println("Clear complete - allowing 2s for e-ink settling...");
    delay(2000);  // 2 seconds for clear (INIT mode is faster than GC16)

    Serial.println("Display cleared successfully");
    return true;
}

// Memory burst write functions for IT8951
bool it8951MemBurstWrite(uint32_t addr, const uint16_t* data, uint32_t wordCount)
{
    // Set target address via LISAR registers
    if (!it8951SetImgBufAddr(addr))
        return false;
    
    // Send burst write start command
    if (!lcdWriteCmd(IT8951_TCON_MEM_BST_WR))
        return false;
    
    // Write data words
    for (uint32_t i = 0; i < wordCount; ++i)
    {
        if (!lcdWriteData(data[i]))
            return false;
    }
    
    // End burst
    return lcdWriteCmd(IT8951_TCON_MEM_BST_END);
}

// Read memory from IT8951 (for verification)
bool it8951MemBurstRead(uint32_t addr, uint16_t* data, uint32_t wordCount)
{
    // Set address
    if (!it8951WriteReg(LISAR + 2, static_cast<uint16_t>((addr >> 16) & 0xFFFF)))
        return false;
    if (!it8951WriteReg(LISAR, static_cast<uint16_t>(addr & 0xFFFF)))
        return false;
    
    // Send burst read start command
    if (!lcdWriteCmd(0x0012))  // MEM_BST_RD_T
        return false;
    if (!lcdWriteData(addr & 0xFFFF))
        return false;
    if (!lcdWriteData((addr >> 16) & 0xFFFF))
        return false;
    if (!lcdWriteData(wordCount))
        return false;
    
    // Read data
    return lcdReadWords(data, wordCount);
}

// Simple test using 8BPP mode exactly like Waveshare IT8951DisplayExample()
bool simpleDisplayTest()
{
    Serial.println("=== WAVESHARE-STYLE DISPLAY TEST (8BPP) ===");
    
    // Step 1: Wait for display ready (like Waveshare IT8951WaitForDisplayReady)
    Serial.println("Step 1: Waiting for display ready...");
    if (!it8951WaitForDisplayReady())
    {
        Serial.println("ERROR: Display not ready");
        return false;
    }
    Serial.println("  Display ready");

    // Step 2: Set image buffer base address (like Waveshare IT8951SetImgBufBaseAddr)
    Serial.println("Step 2: Setting image buffer address...");
    Serial.printf("  Buffer address: 0x%08lX\n", (unsigned long)g_imgBufAddr);
    if (!it8951SetImgBufAddr(g_imgBufAddr))
    {
        Serial.println("ERROR: Failed to set buffer address");
        return false;
    }
    Serial.println("  Buffer address set");

    // Step 3: Prepare load image info (8BPP mode like Waveshare)
    Serial.println("Step 3: Starting image load (8BPP mode)...");
    Serial.printf("  Panel size: %u x %u\n", g_devInfo.usPanelW, g_devInfo.usPanelH);
    
    IT8951LdImgInfo load{};
    load.usEndianType = IT8951_LDIMG_L_ENDIAN;
    load.usPixelFormat = IT8951_8BPP;  // 8BPP like Waveshare example
    load.usRotate = IT8951_ROTATE_0;
    load.ulImgBufBaseAddr = g_imgBufAddr;

    IT8951AreaImgInfo area{};
    area.usX = 0;
    area.usY = 0;
    area.usWidth = g_devInfo.usPanelW;
    area.usHeight = g_devInfo.usPanelH;

    if (!it8951LoadImgAreaStart(load, area))
    {
        Serial.println("ERROR: Failed to start image load");
        return false;
    }
    Serial.println("  Image load started");

    // Step 4: Write pixel data - 8BPP means 2 pixels per 16-bit word
    // For WHITE: use 0xF0F0 (each byte is 0xF0 = white in 8bpp, high nibble used)
    // For BLACK: use 0x0000 (each byte is 0x00 = black)
    Serial.println("Step 4: Writing WHITE pixels (0xF0F0)...");
    
    // In 8BPP mode: width/2 words per row (2 pixels per word)
    const uint32_t wordsPerRow = g_devInfo.usPanelW / 2;
    const uint32_t totalWords = wordsPerRow * g_devInfo.usPanelH;
    Serial.printf("  Words per row: %lu, total words: %lu\n", wordsPerRow, totalWords);
    
    uint32_t written = 0;
    for (uint32_t i = 0; i < totalWords; ++i)
    {
        // 0xF0F0 = two white pixels (0xF0 each in 8bpp)
        if (!lcdWriteData(0xF0F0))
        {
            Serial.printf("ERROR: Write failed at word %lu\n", i);
            it8951LoadImgEnd();
            return false;
        }
        written++;
        if (written % 200000 == 0)
        {
            Serial.printf("  Written %lu words...\n", written);
        }
    }
    Serial.printf("  Wrote %lu words successfully\n", written);

    // Step 5: End image load
    Serial.println("Step 5: Ending image load...");
    if (!it8951LoadImgEnd())
    {
        Serial.println("ERROR: Failed to end image load");
        return false;
    }
    Serial.println("  Image load complete");

    // Step 6: Send display command (INIT mode for full refresh)
    Serial.println("Step 6: Sending display command (INIT mode 0)...");
    if (!it8951DisplayArea(0, 0, g_devInfo.usPanelW, g_devInfo.usPanelH, 0))
    {
        Serial.println("ERROR: Display command failed");
        return false;
    }
    Serial.println("  Display command sent");

    // Step 7: Wait for display to complete
    Serial.println("Step 7: Waiting for display refresh...");
    uint32_t refreshStart = millis();
    if (!it8951WaitForDisplayReady())
    {
        Serial.println("ERROR: Display refresh timeout");
        return false;
    }
    uint32_t refreshTime = millis() - refreshStart;
    Serial.printf("  Refresh completed in %lu ms\n", refreshTime);

    Serial.println("=== TEST COMPLETE - Screen should be WHITE ===");
    delay(3000);
    
    return true;
}

uint8_t rgb565ToGray(uint16_t c)
{
    uint8_t r = ((c >> 11) & 0x1F) << 3;
    uint8_t g = ((c >> 5) & 0x3F) << 2;
    uint8_t b = (c & 0x1F) << 3;
    uint8_t gray = static_cast<uint8_t>((r * 77 + g * 150 + b * 29) >> 8);
    // E-ink uses 0x0=black, 0xF=white in 4bpp mode
    // Our grayscale is 0=black, 255=white, so no inversion needed
    // But we need to ensure proper mapping to 4-bit range
    return gray;
}

// Convert 8-bit grayscale to 4-bit for e-ink (inverted: 0=black, 15=white)
uint8_t grayTo4bpp(uint8_t gray8)
{
    // For IT8951: 0x0 = black, 0xF = white
    // Input gray8: 0 = black, 255 = white
    // Just take the high nibble
    return gray8 >> 4;
}

int pngDrawCallback(PNGDRAW *pDraw)
{
    const int y = pDraw->y;
    if (!g_streamActive || y >= g_devInfo.usPanelH)
        return 1;

    static uint16_t lineBuf[1900];
    g_png.getLineAsRGB565(pDraw, lineBuf, PNG_RGB565_LITTLE_ENDIAN, 0);

    const int copyW = std::min<int>(g_devInfo.usPanelW, pDraw->iWidth);
    for (int x = 0; x < copyW; ++x)
    {
        g_lineGray[x] = rgb565ToGray(lineBuf[x]);
    }
    for (int x = copyW; x < g_devInfo.usPanelW; ++x)
    {
        g_lineGray[x] = 0xFF;
    }
    if (!streamGrayRow(g_lineGray, g_devInfo.usPanelW))
        return 0;
    return 1;
}

String buildImageUrl()
{
    String base = SECRET_NEXTCLOUD_URL;
    if (!base.endsWith("/"))
    {
        base += "/";
    }
    // Select image based on current source setting
    if (g_imageSource == "Photo") {
        base += SECRET_NEXTCLOUD_PHOTO_103;
        Serial.println("Using Photo image source");
    } else {
        base += SECRET_NEXTCLOUD_CARTOON_103;
        Serial.println("Using Cartoon image source");
    }
    return base;
}

bool extractJsonString(const String &json, const String &key, String &out)
{
    const String needle = "\"" + key + "\":\"";
    int pos = json.indexOf(needle);
    if (pos < 0)
        return false;
    pos += needle.length();
    int end = json.indexOf("\"", pos);
    if (end < 0)
        return false;
    out = json.substring(pos, end);
    return true;
}

String normalizeVersionTag(const String &ver)
{
    if (ver.length() > 0 && (ver[0] == 'v' || ver[0] == 'V'))
    {
        return ver.substring(1);
    }
    return ver;
}

bool fetchLatestGitHubRelease(OtaReleaseInfo &info)
{
    if (strlen(SECRET_GITHUB_OWNER) == 0 || strlen(SECRET_GITHUB_REPO) == 0)
    {
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(15000);

    HTTPClient https;
    // Use /releases instead of /releases/latest to include pre-releases
    // /releases/latest excludes pre-releases, but we want to include them
    String url = "https://api.github.com/repos/";
    url += SECRET_GITHUB_OWNER;
    url += "/";
    url += SECRET_GITHUB_REPO;
    url += "/releases?per_page=1";  // Get first page (most recent release, including pre-releases)
    Serial.println("Checking GitHub releases for OTA (including pre-releases)...");
    if (!https.begin(client, url))
    {
        Serial.println("Failed to start OTA release request");
        return false;
    }
    https.setUserAgent("CartoonFrame-OTA");
    https.addHeader("Accept", "application/vnd.github+json");
    if (strlen(SECRET_GITHUB_TOKEN) > 0)
    {
        https.addHeader("Authorization", String("Bearer ") + SECRET_GITHUB_TOKEN);
    }

    const int httpCode = https.GET();
    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("GitHub release query failed: %d\n", httpCode);
        https.end();
        return false;
    }

    String body = https.getString();
    https.end();

    // Parse the first release from the array (most recent, including pre-releases)
    // The response is an array, so we need to extract the first element
    int firstReleaseStart = body.indexOf('{');
    if (firstReleaseStart < 0)
    {
        Serial.println("GitHub release JSON missing releases array");
        return false;
    }
    
    // Find the end of the first release object
    int braceCount = 0;
    int firstReleaseEnd = firstReleaseStart;
    for (int i = firstReleaseStart; i < body.length(); i++)
    {
        if (body[i] == '{') braceCount++;
        if (body[i] == '}') braceCount--;
        if (braceCount == 0)
        {
            firstReleaseEnd = i + 1;
            break;
        }
    }
    
    String firstRelease = body.substring(firstReleaseStart, firstReleaseEnd);
    
    if (!extractJsonString(firstRelease, "tag_name", info.version))
    {
        Serial.println("GitHub release JSON missing tag_name");
        return false;
    }

    const String assetName = strlen(SECRET_GITHUB_FW_ASSET) > 0
                                 ? String(SECRET_GITHUB_FW_ASSET)
                                 : String("firmware.bin");
    const String assetNeedle = "\"name\":\"" + assetName + "\"";
    const int assetPos = firstRelease.indexOf(assetNeedle);
    if (assetPos < 0)
    {
        Serial.printf("GitHub release missing asset %s\n", assetName.c_str());
        return false;
    }

    const String urlKey = "\"browser_download_url\":\"";
    int urlPos = firstRelease.indexOf(urlKey, assetPos);
    if (urlPos < 0)
    {
        Serial.println("GitHub release missing browser_download_url");
        return false;
    }
    urlPos += urlKey.length();
    int urlEnd = firstRelease.indexOf("\"", urlPos);
    if (urlEnd < 0)
    {
        Serial.println("GitHub release URL parse failed");
        return false;
    }
    info.downloadUrl = firstRelease.substring(urlPos, urlEnd);
    return true;
}

bool maybePerformOtaUpdate()
{
    Serial.printf("OTA: Checking for updates (current version: %s)\n", FIRMWARE_VERSION);
    
    if (strlen(SECRET_GITHUB_OWNER) == 0 || strlen(SECRET_GITHUB_REPO) == 0)
    {
        Serial.println("OTA skipped; GitHub owner/repo not configured");
        return false;
    }
    if (strcmp(FIRMWARE_VERSION, "0.0.0") == 0)
    {
        Serial.println("OTA skipped; firmware version not set");
        return false;
    }

    OtaReleaseInfo releaseInfo;
    if (!fetchLatestGitHubRelease(releaseInfo))
    {
        Serial.println("OTA: Failed to fetch release info from GitHub");
        return false;
    }

    const String currentVersion = String(FIRMWARE_VERSION);
    const String normalizedCurrent = normalizeVersionTag(currentVersion);
    const String normalizedRemote = normalizeVersionTag(releaseInfo.version);
    
    Serial.printf("OTA: Current=%s (normalized: %s), Remote=%s (normalized: %s)\n",
                  currentVersion.c_str(), normalizedCurrent.c_str(),
                  releaseInfo.version.c_str(), normalizedRemote.c_str());

    if (normalizedRemote.equalsIgnoreCase(normalizedCurrent))
    {
        Serial.printf("OTA: already running latest release (%s)\n", normalizedCurrent.c_str());
        return false;
    }

    Serial.printf("OTA: update available (%s -> %s)\n",
                  currentVersion.c_str(), releaseInfo.version.c_str());

    // Disable watchdog during OTA (can take a while)
    disableWatchdogForOTA();

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(24000);

    HTTPUpdate httpUpdate;
    httpUpdate.rebootOnUpdate(true);
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    const t_httpUpdate_return result = httpUpdate.update(client, releaseInfo.downloadUrl);
    if (result != HTTP_UPDATE_OK)
    {
        Serial.printf("OTA update failed (%d): %s\n", result,
                      httpUpdate.getLastErrorString().c_str());
        return false;
    }
    Serial.println("OTA update applied successfully");
    return true;
}

bool connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);
    Serial.printf("Connecting to WiFi %s ...\n", SECRET_WIFI_SSID);

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connect failed");
        return false;
    }

    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

String makeBasicAuthHeader()
{
    String creds = String(SECRET_NEXTCLOUD_USER) + ":" + String(SECRET_NEXTCLOUD_PASS);
    const char *b64chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String out;
    int val = 0;
    int valb = -6;
    for (size_t jj = 0; jj < creds.length(); jj++)
    {
        unsigned char c = creds[jj];
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0)
        {
            out += b64chars[(val >> valb) & 0x3F];
            valb -= 6;
        }
    }
    if (valb > -6)
    {
        out += b64chars[((val << 8) >> (valb + 8)) & 0x3F];
    }
    while (out.length() % 4)
    {
        out += '=';
    }
    return String("Basic ") + out;
}

bool downloadPngToSpiffs(const char *path)
{
    // Delete temp file if exists
    if (SPIFFS.exists(PNG_TEMP_DOWNLOAD_PATH))
    {
        SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
    }
    
    // Delete old PNG first to make room for new one
    if (SPIFFS.exists(path))
    {
        Serial.println("Removing old PNG to make room for download...");
        SPIFFS.remove(path);
    }

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(60000);  // 60 second timeout for large files (Photo is ~1MB)

    HTTPClient https;
    const String url = buildImageUrl();
    Serial.print("Fetching image from: ");
    Serial.println(url);

    if (!https.begin(client, url))
    {
        Serial.println("HTTPS begin failed");
        return false;
    }

    https.addHeader("Authorization", makeBasicAuthHeader());
    https.setTimeout(60000);  // 60 second timeout
    
    // Collect headers we want to inspect
    const char* headerKeys[] = {"Content-Type", "Content-Length"};
    https.collectHeaders(headerKeys, 2);

    const int httpCode = https.GET();
    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("HTTP GET failed: %d\n", httpCode);
        https.end();
        return false;
    }
    
    const int32_t contentLength = https.getSize();
    String contentType = https.header("Content-Type");
    String contentLengthHeader = https.header("Content-Length");
    Serial.printf("HTTP %d\n", httpCode);
    Serial.printf("  Content-Length header: %s\n", contentLengthHeader.c_str());
    Serial.printf("  Content-Type header: %s\n", contentType.c_str());
    Serial.printf("  getSize() reports: %ld bytes\n", (long)contentLength);
    
    // Warn if not a PNG
    if (contentType.length() > 0 && contentType.indexOf("image/png") < 0 && contentType.indexOf("application/octet") < 0) {
        Serial.println("WARNING: Response may not be a PNG file!");
    }
    
    // Check SPIFFS free space
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    size_t freeBytes = totalBytes - usedBytes;
    Serial.printf("SPIFFS: %u used / %u total, %u free\n", usedBytes, totalBytes, freeBytes);
    
    if (contentLength > 0 && (size_t)contentLength > freeBytes) {
        Serial.printf("ERROR: File too large (%ld bytes) for available space (%u bytes)\n", 
                      (long)contentLength, freeBytes);
        https.end();
        return false;
    }

    File f = SPIFFS.open(PNG_TEMP_DOWNLOAD_PATH, FILE_WRITE);
    if (!f)
    {
        Serial.println("Failed to open SPIFFS file for write");
        https.end();
        return false;
    }

    WiFiClient *stream = https.getStreamPtr();
    uint8_t buf[1024];  // Keep buffer small to avoid stack overflow
    int32_t total = 0;
    const int32_t expected = https.getSize(); // -1 if unknown
    uint32_t lastProgressReport = 0;
    
    while (https.connected())
    {
        yield();  // Feed the watchdog and allow WiFi processing
        
        size_t chunk = sizeof(buf);
        if (expected > 0)
        {
            if (total >= expected)
                break;
            const int32_t remaining = expected - total;
            if (remaining < static_cast<int32_t>(chunk))
                chunk = static_cast<size_t>(remaining);
        }
        
        // Wait for data with timeout
        uint32_t waitStart = millis();
        while (stream->available() == 0 && https.connected()) {
            yield();
            if (millis() - waitStart > 30000) {  // 30 second read timeout
                Serial.println("HTTP read timeout");
                f.close();
                https.end();
                SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
                return false;
            }
            delay(10);
        }
        
        int len = stream->readBytes(reinterpret_cast<char *>(buf), chunk);
        if (len <= 0)
        {
            if (!https.connected())
                break;
            Serial.println("HTTP read stalled");
            break;
        }
        if (f.write(buf, len) != (size_t)len)
        {
            Serial.println("SPIFFS write failed");
            f.close();
            https.end();
            SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
            return false;
        }
        total += len;
        
        // Progress report every 100KB
        if (total - lastProgressReport >= 100000) {
            Serial.printf("Downloaded %ld / %ld bytes (%d%%)\n", 
                          (long)total, (long)expected, 
                          expected > 0 ? (int)(total * 100 / expected) : 0);
            lastProgressReport = total;
        }
    }
    f.close();
    https.end();

    if (total <= 0)
    {
        Serial.println("Failed to download PNG body");
        SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
        return false;
    }
    if (expected > 0 && total < expected)
    {
        Serial.printf("PNG download truncated (%ld/%ld)\n", (long)total, (long)expected);
        SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
        return false;
    }

    if (SPIFFS.exists(path) && !SPIFFS.remove(path))
    {
        Serial.println("Failed to remove previous PNG");
        SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
        return false;
    }
    if (!SPIFFS.rename(PNG_TEMP_DOWNLOAD_PATH, path))
    {
        Serial.println("Failed to move downloaded PNG into place");
        SPIFFS.remove(PNG_TEMP_DOWNLOAD_PATH);
        return false;
    }
    Serial.printf("Saved PNG to SPIFFS (%ld bytes)\n", (long)total);
    return true;
}

bool decodePngFromSpiffs(const char *path)
{
    File file = SPIFFS.open(path, FILE_READ);
    if (!file)
    {
        Serial.println("PNG read failed; file missing");
        return false;
    }

    const size_t fileSize = file.size();
    uint8_t *pngBuffer = nullptr;
    if (fileSize > 0)
    {
        if (psramFound())
        {
            pngBuffer = static_cast<uint8_t *>(ps_malloc(fileSize));
        }
        if (!pngBuffer)
        {
            pngBuffer = static_cast<uint8_t *>(malloc(fileSize));
        }
    }

    int rc = PNG_INVALID_FILE;
    if (pngBuffer)
    {
        size_t readBytes = file.readBytes(reinterpret_cast<char *>(pngBuffer), fileSize);
        file.close();
        if (readBytes != fileSize)
        {
            Serial.println("Failed to read full PNG into RAM");
            free(pngBuffer);
            return false;
        }
        rc = g_png.openFLASH(pngBuffer, fileSize, pngDrawCallback);
    }
    else
    {
        Serial.println("RAM allocation failed; streaming from SPIFFS");
        file.close();
        auto pngOpen = [](const char *filename, int32_t *pFileSize) -> void *
        {
            File *pf = new File(SPIFFS.open(filename, FILE_READ));
            if (!pf || !*pf)
            {
                Serial.println("PNG open failed (file)");
                if (pf)
                    delete pf;
                return nullptr;
            }
            *pFileSize = pf->size();
            return pf;
        };
        auto pngClose = [](void *handle) -> void
        {
            File *pf = static_cast<File *>(handle);
            if (pf)
            {
                pf->close();
                delete pf;
            }
        };
        auto pngRead = [](PNGFILE *pFile, uint8_t *pBuf, int32_t iLen) -> int32_t
        {
            File *pf = reinterpret_cast<File *>(pFile->fHandle);
            return pf->read(pBuf, iLen);
        };
        auto pngSeek = [](PNGFILE *pFile, int32_t iPosition) -> int32_t
        {
            File *pf = reinterpret_cast<File *>(pFile->fHandle);
            pf->seek(iPosition, SeekSet);
            return iPosition;
        };
        rc = g_png.open(path, pngOpen, pngClose, pngRead, pngSeek, pngDrawCallback);
    }

    if (rc != PNG_SUCCESS)
    {
        Serial.printf("PNG open failed: %d (last err %d)\n", rc, g_png.getLastError());
        g_png.close();
        if (pngBuffer)
            free(pngBuffer);
        return false;
    }

    Serial.printf("PNG size: %d x %d\n", g_png.getWidth(), g_png.getHeight());
    if (g_png.getWidth() != g_devInfo.usPanelW || g_png.getHeight() != g_devInfo.usPanelH)
    {
        Serial.println("PNG resolution mismatch; image may be padded");
    }

    if (!beginFrameStream(g_devInfo.usPanelW, g_devInfo.usPanelH))
    {
        Serial.println("Failed to begin frame stream");
        g_png.close();
        if (pngBuffer)
            free(pngBuffer);
        return false;
    }

    rc = g_png.decode(nullptr, 0);
    g_png.close();
    endFrameStream();

    if (pngBuffer)
    {
        free(pngBuffer);
        pngBuffer = nullptr;
    }

    if (rc != PNG_SUCCESS)
    {
        Serial.printf("PNG decode failed: %d (last err %d)\n", rc, g_png.getLastError());
        return false;
    }
    return true;
}

bool displayTestPattern()
{
    if (!beginFrameStream(g_devInfo.usPanelW, g_devInfo.usPanelH))
        return false;

    const uint16_t bands = 16;
    const uint16_t bandHeight = std::max<uint16_t>(1, g_devInfo.usPanelH / bands);
    for (uint32_t y = 0; y < g_devInfo.usPanelH; ++y)
    {
        const uint8_t gray = static_cast<uint8_t>((y / bandHeight) & 0x0F);
        for (uint32_t x = 0; x < g_devInfo.usPanelW; ++x)
        {
            const bool checker = ((x / 32) ^ (y / 32)) & 0x01;
            g_lineGray[x] = checker ? (gray * 17) : (15 - gray) * 17;
        }
        if (!streamGrayRow(g_lineGray, g_devInfo.usPanelW))
        {
            endFrameStream();
            return false;
        }
    }
    return endFrameStream();
}

bool refreshDisplay()
{
    Serial.printf("refreshDisplay: panel=%ux%u, imgBuf=0x%08lx\n", 
                  g_devInfo.usPanelW, g_devInfo.usPanelH, 
                  static_cast<unsigned long>(g_imgBufAddr));

    if (!it8951WaitForDisplayReady())
    {
        Serial.println("Display busy before refresh");
        return false;
    }

    Serial.println("Sending GC16 refresh command using 0x0034...");
    
    // Use the standard display area command (0x0034) - uses default image buffer
    if (!it8951DisplayArea(0, 0, g_devInfo.usPanelW, g_devInfo.usPanelH, IT8951_MODE_GC16))
    {
        Serial.println("Display refresh command failed");
        return false;
    }
    
    Serial.println("Waiting for display refresh to complete...");
    
    // Wait for the refresh to complete
    if (!it8951WaitForDisplayReady())
    {
        Serial.println("Display refresh timeout");
        return false;
    }
    
    // CRITICAL: Add settling delay after controller reports ready
    // The IT8951 controller reports ready when command processing is done,
    // but the physical e-ink particles need additional time to settle,
    // especially for large displays (10.3") using GC16 mode.
    // Without this delay, power can be cut mid-refresh causing smearing.
    Serial.println("Controller reports ready - allowing 3s for physical e-ink settling...");
    delay(3000);  // 3 second settling time for e-ink particles
    
    Serial.println("Display refresh done");
    return true;
}

// MQTT callback for handling settings
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    String topicStr = String(topic);
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.printf("MQTT received: %s = %s\n", topic, message.c_str());
    
    // Handle refresh interval from command topic only (state topic is for device to publish, not receive)
    if (topicStr == MQTT_REFRESH_INTERVAL_TOPIC) {
        uint32_t interval = message.toInt();
        Serial.printf("Received refresh interval command: %u\n", interval);
        if (interval >= 1 && interval <= 12) {
            setRefreshInterval(interval);
            // Publish updated state back to state topic (retained) - HA needs this to update the UI
            if (mqttClient.connected()) {
                String stateValue = String(g_refreshInterval);
                Serial.printf("Publishing refresh interval state: %s\n", stateValue.c_str());
                bool published = mqttClient.publish(MQTT_REFRESH_INTERVAL_STATE_TOPIC, stateValue.c_str(), true);
                if (!published) {
                    Serial.println("Failed to publish refresh interval state!");
                } else {
                    Serial.println("Refresh interval state published successfully");
                }
            }
        } else {
            Serial.printf("Invalid refresh interval: %u (must be 1-12)\n", interval);
        }
    }
    // Handle image source from command topic only (state topic is for device to publish, not receive)
    else if (topicStr == MQTT_IMAGE_SOURCE_TOPIC) {
        if (message == "Cartoon" || message == "Photo") {
            setImageSource(message);
            // Publish updated state back to state topic (retained)
            if (mqttClient.connected()) {
                mqttClient.publish(MQTT_IMAGE_SOURCE_STATE_TOPIC, g_imageSource.c_str(), true);
            }
        }
    }
}

void setupMqtt()
{
    mqttClient.setServer(SECRET_MQTT_BROKER, SECRET_MQTT_PORT);
    mqttClient.setBufferSize(1024);
    mqttClient.setCallback(mqttCallback);
}

bool ensureMqttConnected()
{
    if (mqttClient.connected())
        return true;

    Serial.println("Connecting to MQTT broker...");
    String clientId = String(MQTT_DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool connected = false;
    if (strlen(SECRET_MQTT_USERNAME) == 0)
    {
        connected = mqttClient.connect(clientId.c_str());
    }
    else
    {
        connected = mqttClient.connect(clientId.c_str(), SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD);
    }

    if (!connected)
    {
        Serial.printf("MQTT connect failed, state=%d\n", mqttClient.state());
    }
    else
    {
        mqttDiscoveryPublished = false;
        Serial.println("MQTT connected.");
        // Subscribe to command topics only (state topics are for device to publish, not receive)
        mqttClient.subscribe(MQTT_REFRESH_INTERVAL_TOPIC);
        mqttClient.subscribe(MQTT_IMAGE_SOURCE_TOPIC);
        Serial.printf("Subscribed to: %s, %s\n", 
                      MQTT_REFRESH_INTERVAL_TOPIC, MQTT_IMAGE_SOURCE_TOPIC);
        
        // Process any pending messages (including retained ones from broker)
        mqttClient.loop();
        
        // Publish current states immediately after connecting (before discovery)
        // This ensures HA has the correct state even if discovery hasn't run yet
        mqttClient.publish(MQTT_REFRESH_INTERVAL_STATE_TOPIC, String(g_refreshInterval).c_str(), true);
        mqttClient.publish(MQTT_IMAGE_SOURCE_STATE_TOPIC, g_imageSource.c_str(), true);
    }
    return connected;
}

String buildDeviceDescriptorJson()
{
    String json = "{\"identifiers\":[\"";
    json += MQTT_DEVICE_ID;
    json += "\"],\"name\":\"";
    json += MQTT_DEVICE_NAME;
    json += "\",\"manufacturer\":\"Cartoon\",\"model\":\"FireBeetle 2 ESP32-E\"}";
    return json;
}

void publishHomeAssistantDiscovery()
{
    if (mqttDiscoveryPublished || !mqttClient.connected())
        return;

    const String deviceJson = buildDeviceDescriptorJson();

    auto publishSensor = [&](const char *suffix, const char *friendlyName, const char *unit, const char *valueKey, const char *deviceClass, const char *stateClass)
    {
        String topic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + MQTT_DEVICE_ID + "_" + suffix + "/config";
        String payload = "{\"name\":\"";
        payload += friendlyName;
        payload += "\",\"uniq_id\":\"";
        payload += MQTT_DEVICE_ID;
        payload += "_";
        payload += suffix;
        payload += "\",\"state_topic\":\"";
        payload += MQTT_STATE_TOPIC;
        payload += "\",\"value_template\":\"{{ value_json.";
        payload += valueKey;
        payload += " }}\"";
        if (unit)
        {
            payload += ",\"unit_of_measurement\":\"";
            payload += unit;
            payload += "\"";
        }
        if (deviceClass)
        {
            payload += ",\"device_class\":\"";
            payload += deviceClass;
            payload += "\"";
        }
        if (stateClass)
        {
            payload += ",\"state_class\":\"";
            payload += stateClass;
            payload += "\"";
        }
        payload += ",\"device\":";
        payload += deviceJson;
        payload += "}";
        mqttClient.publish(topic.c_str(), payload.c_str(), true);
    };

    publishSensor("battery", "Cartoon Frame Battery", "V", "battery", "voltage", "measurement");
    publishSensor("wifi", "Cartoon Frame WiFi RSSI", "dBm", "wifi_rssi", "signal_strength", "measurement");
    publishSensor("refresh", "Cartoon Frame Refresh Time", "ms", "refresh_ms", nullptr, "measurement");
    publishSensor("boot_count", "Cartoon Frame Boot Count", nullptr, "boot_count", nullptr, "measurement");
    publishSensor("firmware", "Cartoon Frame Firmware Version", nullptr, "firmware_version", nullptr, nullptr);

    // Publish refresh interval as a number entity (configurable from HA)
    {
        String topic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + MQTT_DEVICE_ID + "_refresh_interval/config";
        String payload = "{\"name\":\"Cartoon Frame Refresh Interval\",\"uniq_id\":\"";
        payload += MQTT_DEVICE_ID;
        payload += "_refresh_interval\",\"state_topic\":\"";
        payload += MQTT_REFRESH_INTERVAL_STATE_TOPIC;
        payload += "\",\"command_topic\":\"";
        payload += MQTT_REFRESH_INTERVAL_TOPIC;
        payload += "\",\"min\":1,\"max\":12,\"step\":1,\"unit_of_measurement\":\"boots\",\"retain\":true,\"device\":";
        payload += deviceJson;
        payload += "}";
        mqttClient.publish(topic.c_str(), payload.c_str(), true);
    }

    // Publish image source as a select entity (dropdown in HA)
    {
        String topic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + MQTT_DEVICE_ID + "_image_source/config";
        String payload = "{\"name\":\"Cartoon Frame Image Source\",\"uniq_id\":\"";
        payload += MQTT_DEVICE_ID;
        payload += "_image_source\",\"state_topic\":\"";
        payload += MQTT_IMAGE_SOURCE_STATE_TOPIC;
        payload += "\",\"command_topic\":\"";
        payload += MQTT_IMAGE_SOURCE_TOPIC;
        payload += "\",\"options\":[\"Cartoon\",\"Photo\"],\"retain\":true,\"device\":";
        payload += deviceJson;
        payload += "}";
        mqttClient.publish(topic.c_str(), payload.c_str(), true);
    }

    mqttDiscoveryPublished = true;
    
    // Publish current states AFTER discovery config (HA needs config before state)
    // This ensures HA picks up the retained state correctly
    mqttClient.publish(MQTT_REFRESH_INTERVAL_STATE_TOPIC, String(g_refreshInterval).c_str(), true);
    mqttClient.publish(MQTT_IMAGE_SOURCE_STATE_TOPIC, g_imageSource.c_str(), true);
}

void publishTelemetry(bool refreshSuccess, float refreshDurationMs, float batteryVoltage)
{
    if (!ensureMqttConnected())
        return;

    publishHomeAssistantDiscovery();

    String payload = "{";
    if (isnan(batteryVoltage))
        payload += "\"battery\":null";
    else
    {
        payload += "\"battery\":";
        payload += String(batteryVoltage, 3);
    }
    payload += ",\"wifi_rssi\":";
    payload += String(WiFi.RSSI());
    payload += ",\"refresh_ms\":";
    payload += String(refreshDurationMs, 0);
    payload += ",\"refresh_success\":";
    payload += refreshSuccess ? "1" : "0";
    payload += ",\"boot_count\":";
    payload += String(g_bootCount);
    payload += ",\"refresh_interval\":";
    payload += String(g_refreshInterval);
    payload += ",\"firmware_version\":\"";
    payload += FIRMWARE_VERSION;
    payload += "\"}";

    mqttClient.publish(MQTT_STATE_TOPIC, payload.c_str(), true);
    mqttClient.loop();
}

float readBatteryVoltage()
{
    if (PIN_BATTERY_ADC == GPIO_NUM_NC)
    {
        Serial.println("Battery ADC pin not configured");
        return NAN;
    }

    const uint8_t adcPin = static_cast<uint8_t>(PIN_BATTERY_ADC);
    
    // Configure ADC once (these settings persist)
    static bool adcConfigured = false;
    if (!adcConfigured)
    {
        pinMode(adcPin, INPUT);
        analogSetPinAttenuation(adcPin, ADC_11db);
        analogReadResolution(12);
        delay(10);  // Let ADC settle after configuration
        adcConfigured = true;
    }
    
    // Take more samples with longer delays for better noise filtering
    // ESP32 ADC is noisy, especially when WiFi is active
    const uint8_t samples = 32;  // Increased from 8 to 32
    uint32_t acc = 0;
    for (uint8_t i = 0; i < samples; ++i)
    {
        acc += analogReadMilliVolts(adcPin);
        delay(5);  // Increased delay from 2ms to 5ms for better stability
    }
    float averageMv = acc / static_cast<float>(samples);
    
    if (averageMv <= 0)
    {
        Serial.println("Battery measurement invalid (<=0mV)");
        return NAN;
    }
    
    const float voltage = (averageMv / 1000.0f) * BATTERY_VOLTAGE_DIVIDER;
    Serial.printf("Battery ADC GPIO%d: %.0fmV (avg of %d samples) -> %.3fV\n", 
                  adcPin, averageMv, samples, voltage);
    return voltage;
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("FireBeetle 2 ESP32-E + IT8951 10.3\" EPD");

    // Initialize TPL5110 DONE pin LOW immediately
    initTPL5110Done();
    
    // Initialize watchdog (2 minute timeout unless OTA)
    initWatchdog();
    
    setupMqtt();

    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS mount failed");
    }

    // Load settings and increment boot counter
    loadSettings();
    
    // Default to Cartoon mode
    g_imageSource = "Cartoon";
    
    incrementBootCounter();
    
    // Check if we should refresh the display this boot
    bool shouldRefresh = shouldRefreshDisplay();
    Serial.printf("Should refresh display: %s (boot %u of %u)\n", 
                  shouldRefresh ? "YES" : "NO", g_bootCount, g_refreshInterval);

    bool pngOnDisk = SPIFFS.exists(PNG_STORAGE_PATH);
    bool refreshed = false;
    float refreshDurationMs = 0.0f;

    // Always connect to WiFi to check for OTA updates (on every boot)
    // This ensures firmware updates are available even when not refreshing display
    if (connectWiFi())
    {
        checkWatchdog();  // Check after WiFi connection
        
        // Check for OTA updates on every boot (not just when refreshing)
        maybePerformOtaUpdate();
        
        checkWatchdog();  // Check after OTA check
        
        // If OTA update was applied, device will reboot here
        // (maybePerformOtaUpdate calls httpUpdate.rebootOnUpdate(true))
    }

    if (shouldRefresh)
    {
        // Connect to WiFi if not already connected (should be, but just in case)
        if (connectWiFi())
        {
            
            // Process any pending MQTT messages (e.g., refresh interval, image source)
            if (ensureMqttConnected())
            {
                Serial.println("Waiting for MQTT messages (5 seconds)...");
                for (int i = 0; i < 100; i++) {  // 100 x 50ms = 5 seconds (longer to catch commands)
                    mqttClient.loop();
                    delay(50);
                }
                Serial.printf("After MQTT: Image source = %s, Refresh interval = %u\n", 
                              g_imageSource.c_str(), g_refreshInterval);
            }
            
            checkWatchdog();  // Check after MQTT
            
            if (downloadPngToSpiffs(PNG_STORAGE_PATH))
            {
                Serial.println("PNG downloaded to SPIFFS");
                pngOnDisk = true;
            }
            else
            {
                Serial.println("PNG download failed");
                // If Photo failed, try Cartoon as fallback
                if (g_imageSource == "Photo")
                {
                    Serial.println("Trying Cartoon as fallback...");
                    g_imageSource = "Cartoon";  // Temporarily switch to Cartoon
                    if (downloadPngToSpiffs(PNG_STORAGE_PATH))
                    {
                        Serial.println("Cartoon fallback downloaded successfully");
                        pngOnDisk = true;
                    }
                    else
                    {
                        Serial.println("Cartoon fallback also failed; keeping previous image");
                    }
                }
                else
                {
                    Serial.println("Keeping previous image");
                }
            }
            
            checkWatchdog();  // Check after PNG download
        }
        else
        {
            Serial.println("WiFi unavailable; keeping previous image");
        }

        checkWatchdog();  // Check before display init

        if (!initDisplay())
        {
            Serial.println("EPD init failed");
            publishTelemetry(false, 0.0f, readBatteryVoltage());
            signalTPL5110Done();
            return;
        }

        checkWatchdog();  // Check after display init

        bool imageStreamed = false;
        bool attemptedPng = false;
        
        if (pngOnDisk)
        {
            // Clear the screen first (INIT mode flashes to white, removes ghosting)
            // This must be done BEFORE streaming the image, otherwise it overwrites the image buffer
            clearDisplay();
            checkWatchdog();
            
            // Now stream the image to the buffer (after clearing)
            attemptedPng = true;
            if (decodePngFromSpiffs(PNG_STORAGE_PATH))
            {
                Serial.println("PNG decoded and streamed to panel buffer");
                imageStreamed = true;
            }
            else
            {
                Serial.println("PNG decode failed");
            }
        }

        checkWatchdog();  // Check after PNG decode

        if (!imageStreamed)
        {
            Serial.println("No image available; keeping previous display contents");
        }

        checkWatchdog();  // Check before display refresh

        if (imageStreamed)
        {
            const uint32_t refreshStart = millis();
            refreshed = refreshDisplay();
            refreshDurationMs = refreshed ? static_cast<float>(millis() - refreshStart) : 0.0f;
        }
        else
        {
            Serial.println("No image rendered; keeping previous display contents");
        }

        // Reset boot counter after successful refresh
        if (refreshed)
        {
            resetBootCounter();
        }
    }
    else
    {
        Serial.println("Skipping display refresh (not due yet)");
        // WiFi was already connected for OTA check above, so we can use it for telemetry
    }

    // Publish telemetry
    const float batteryVoltage = readBatteryVoltage();
    if (WiFi.status() == WL_CONNECTED)
    {
        ensureMqttConnected();
        publishTelemetry(refreshed, refreshDurationMs, batteryVoltage);
        
        // Process any pending MQTT commands (e.g., refresh interval, image source changes)
        // This allows HA to control the device even after it's done refreshing
        Serial.println("Processing MQTT commands (3 seconds)...");
        for (int i = 0; i < 60; i++) {  // 60 x 50ms = 3 seconds
            mqttClient.loop();
            delay(50);
        }
        Serial.printf("Final settings: Image source = %s, Refresh interval = %u\n", 
                      g_imageSource.c_str(), g_refreshInterval);
    }

    if (shouldRefresh && !refreshed)
    {
        Serial.println("Display update failed");
    }

    // Signal TPL5110 to cut power
    signalTPL5110Done();
}

void loop()
{
    delay(60000);
}
