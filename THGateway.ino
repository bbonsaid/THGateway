// THGateway bbd
// AtomMatrix用温湿度計ゲートウェイ
// ArduinoIDEスケッチ例ESP32 BLE ArduinoのBLE_scanを参考に
//
//近くの温湿度計から発せられるBLE信号をキャッチし、Ambientへデータを送ります。

#include <M5Atom.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "Ambient.h"

#define  RGB(r, g, b)  (((r) << 16) + ((g) << 8) + (b))

#define SECONDS(s)    ((s) * 1000)
#define MINUTES(m)    SECONDS(m * 60)

#define SWITCHBOT_SD_SIZE 6
#define CYALKIT_MD_SIZE 25

typedef struct {
  // Byte:0 Enc Type Dev Type
  uint8_t deviceType : 7;
  uint8_t reserved1 : 1;
  // Byte:1 Status
  uint8_t groupA : 1;
  uint8_t groupB : 1;
  uint8_t groupC : 1;
  uint8_t groupD : 1;
  uint8_t reserved2 : 4;
  // Byte:2 Update UTC Flag Battery
  uint8_t remainingBattery : 7;
  uint8_t reserved3 : 1;
  // Byte:3
  uint8_t decimalOfTheTemperature : 4;
  uint8_t humidityAlertStatus : 2;
  uint8_t temperatureAlertStatus : 2;
  // Byte:4
  uint8_t integerOfTheTemperature : 7;
  uint8_t posiNegaTemperatureFlag : 1;
  // Byte:5
  uint8_t humidityValue : 7;
  uint8_t temperatureScale : 1;
} SwitchBotServiceData;

typedef struct {
  // AD Data 25 bytes
  uint16_t  companyID;  // 0x004C
  uint8_t   deviceType; // 0x02
  uint8_t   length3;    // 0x15
  char      uuid[16];   // 00050001-0000-1000-8000-00805F9B0131 [hex]
  uint16_t  major;      // 0x0001
  uint8_t   humidity;   // 湿度[％] = 125 * (humidity * 256) / 65536 - 6
  uint8_t   temp;       // 温度[℃] = 175.72 * (temp * 256) / 65536 - 46.85
  uint8_t   rssi;       // RSSI
} CyalkitManufacturerData;

int scanTime = 5; //In seconds
int waitTime = 2; //In seconds
int intervalTime = 20 * 60; //In seconds
int skipResetCount = intervalTime / (scanTime + waitTime);
int skipCount = skipResetCount;

BLEScan* pBLEScan;

#define MAX_ADDRESS 8

int nAddresses = 0;
std::string addresses[MAX_ADDRESS];


boolean fConnect = false;
WiFiClient client;
// Wi-Fi こちらの値を適宜書き換えてご使用ください。
const char * ssid     = "XXXX";
const char * password = "XXXX";

Ambient ambient;
// こちらの値を適宜書き換えてご使用ください。
unsigned int channelId = 0; // AmbientのチャネルID
const char* writeKey = "xxxx"; // ライトキー

void
printAddressPos(void)
{
  Serial.printf("***** printAddressPos(%d) *****\n", nAddresses);
  int loopCount = (nAddresses < MAX_ADDRESS) ? nAddresses : MAX_ADDRESS;
  for (int i = 0; i < loopCount; i++)
    Serial.printf("address[%d]:%s\n", i, addresses[i]);
  Serial.printf("*****\n");
}

int getAddressPos(const std::string tAddress)
{
  // 見つけたMACアドレス毎に番号を振り、その番号でAmbientへデータを書き込む。
  // 最大数を超えた場合、-1を返す。
  bool fFound = false;
  int pos = -1;
  int loopCount = (nAddresses < MAX_ADDRESS) ? nAddresses : MAX_ADDRESS;
  for (int i = 0; i < loopCount; i++) {
    if (tAddress == addresses[i]) {
      fFound = true;
      pos = i;
      continue;
    }
  }
  if ((nAddresses < MAX_ADDRESS) && !fFound) {
    pos = nAddresses;
    addresses[nAddresses++] = tAddress;
  }
  return pos;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    //activeサイン
    static int ff = 1;
    M5.dis.drawpix(4, 4, (ff ^= 1) ? RGB(0x40, 0x00, 0x00) : 0);

    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());

    BLEAddress address = advertisedDevice.getAddress();

    if (advertisedDevice.haveServiceData()) {
      std::string d = advertisedDevice.getServiceData();
      Serial.printf("ServiceData(%d):", d.size());
      for (int i = 0; i < d.size(); i++) {
        Serial.printf("%02X", d[i]);
      }
      Serial.print("\n");
    }

    // Switchbot温湿度計を特定する。
    if (advertisedDevice.haveServiceData()
        && advertisedDevice.getServiceData().length() == SWITCHBOT_SD_SIZE) {
      SwitchBotServiceData sd;
      memcpy(&sd, advertisedDevice.getServiceData().data(),
          advertisedDevice.getServiceData().length());

      if (sd.deviceType == 'T') { // SwitchBot MeterTH (WoSensorTH) Normal Mode

        float temperature = ((sd.posiNegaTemperatureFlag) ? 1.0 : -1.0) *
                            (sd.decimalOfTheTemperature * 0.1 +
                             sd.integerOfTheTemperature);
        float humidity = (float)sd.humidityValue;

        Serial.printf("temperature:%.1f\n", temperature);
        Serial.printf("humidity:%.1f\n", humidity);

        //温度のみ送信する。
        if (fConnect) {
          int pos = getAddressPos(address.toString());
          if (pos >= 0) {
            ambient.set(pos + 1, temperature);
            Serial.printf("ambient.set(%d, %.1f);\n", pos + 1, temperature);
          }
        }

        Serial.printf("SwitchBotServiceData:\n");
        Serial.printf("   deviceType:%02x\n", sd.deviceType);
        Serial.printf("   reserved1:%02x\n", sd.reserved1);
        Serial.printf("   group A:%02x B:%02x C:%02x D:%02x\n", sd.groupA, sd.groupB, sd.groupC, sd.groupD);
        Serial.printf("   reserved2:%02x\n", sd.reserved2);
        Serial.printf("   remainingBattery:%02x(%d)\n", sd.remainingBattery, sd.remainingBattery);
        Serial.printf("   reserved3:%02x\n", sd.reserved3);
        Serial.printf("   decimalOfTheTemperature:%02x(%d)\n", sd.decimalOfTheTemperature, sd.decimalOfTheTemperature);
        Serial.printf("   humidityAlertStatus:%02x\n", sd.humidityAlertStatus);
        Serial.printf("   temperatureAlertStatus:%02x\n", sd.temperatureAlertStatus);
        Serial.printf("   integerOfTheTemperature:%02x(%d)\n", sd.integerOfTheTemperature, sd.integerOfTheTemperature);
        Serial.printf("   posiNegaTemperatureFlag:%02x\n", sd.posiNegaTemperatureFlag);
        Serial.printf("   humidityValue:%02x(%d)\n", sd.humidityValue, sd.humidityValue);
        Serial.printf("   temperatureScale:%02x\n", sd.temperatureScale);
      }
    }

    // CYALKIT温湿度計を特定する。
    if (advertisedDevice.haveManufacturerData() == true
        && advertisedDevice.getManufacturerData().length() == CYALKIT_MD_SIZE) {
      CyalkitManufacturerData md;
      memcpy(&md, advertisedDevice.getManufacturerData().data(),
          advertisedDevice.getManufacturerData().length());

      if (md.companyID == 0x004C && md.deviceType == 0x02) {
        float temperature = (float)(175.72 * (md.temp * 256.0) / 65536.0 - 46.85);
        float humidity = (float)(125.0 * (md.humidity * 256.0) / 65536.0 - 6.0);

        Serial.printf("temperature:%.1f\n", temperature);
        Serial.printf("humidity:%.1f\n", humidity);

        //温度のみ送信する。
        if (fConnect) {
          int pos = getAddressPos(address.toString());
          if (pos >= 0) {
            ambient.set(pos + 1, temperature);
            Serial.printf("ambient.set(%d, %.1f);\n", pos + 1, temperature);
          }
        }

        Serial.printf("CyalkitManufacturerData:\n");
        Serial.printf("   companyID:%04x\n", md.companyID);
        Serial.printf("   deviceType:%02x\n", md.deviceType);
        Serial.printf("   length3:%02x\n", md.length3);
        Serial.printf("   uuid:%s\n", md.uuid);
        Serial.printf("   humidity:%02x\n", md.humidity);
        Serial.printf("   temp:%02x\n", md.temp);
        Serial.printf("   rssi:%02x\n", md.rssi);
      }
    }
  }
};

void dispLCD(int skipCount)
{
  //Wifiの接続状況
  int c = (fConnect) ? RGB(0x00, 0x80, 0x00) : 0;
  for (int x = 0; x < 4; x++)
    M5.dis.drawpix(x, 4, c);

  //アドレス登録の状態
  c = RGB(0x20, 0x20, 0x20);
  for (int i = 0; i < nAddresses; i++)
    M5.dis.drawpix((i > 4) * 3 + (i % 2), (i % 4), c);

  //更新までの残り時間
  M5.dis.drawpix(2, 0, (skipCount > skipResetCount / 4 * 3) ? RGB(0x80, 0x80, 0x00) : 0);
  M5.dis.drawpix(2, 1, (skipCount > skipResetCount / 4 * 2) ? RGB(0x80, 0x80, 0x00) : 0);
  M5.dis.drawpix(2, 2, (skipCount > skipResetCount / 4) ? RGB(0x80, 0x80, 0x00) : 0);
  M5.dis.drawpix(2, 3, (skipCount > 0) ? RGB(0x80, 0x80, 0x00) : 0);
}

void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("M5Atom");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  WiFi.begin(ssid, password);  //  Wi-Fi APに接続
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {  //  Wi-Fi AP接続待ち
    Serial.print(".");
    delay(100);
  }
  Serial.print("\n");
  fConnect = (WiFi.status() == WL_CONNECTED);

  if (fConnect) {
    // チャネルIDとライトキーを指定してAmbientの初期化
    ambient.begin(channelId, writeKey, &client);
  }

  delay(SECONDS(1));
}

void loop() {
  dispLCD(skipCount);
  if (--skipCount < 0) {
    skipCount = skipResetCount;
    if (fConnect)
      ambient.send();
  }

  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

  printAddressPos();

  delay(SECONDS(waitTime));
}
