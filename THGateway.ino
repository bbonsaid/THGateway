// THGateway bbd
// AtomMatrix用温湿度計ゲートウェイ
// ArduinoIDEスケッチ例ESP32 BLE ArduinoのBLE_scanを参考に
//
//近くの温湿度計から発せられるBLE信号をキャッチし、Ambientへデータを送ります。

// ローカル用(Wifi, Ambient埋め込み)
// 2023.07
// 温湿度計Plusに対応する。
// 防水温湿度計に対応する。
// Wifi不安定対策をする。
// アドレス固定の対策をする。

// ArduinoIDE [ﾂｰﾙ]-[Partition Scheme:"NO OTA(Large APP)"]にする。

#include <M5Atom.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <FS.h>
#include <SPIFFS.h>
#include "Ambient.h"

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

#define RGB(r, g, b)  (((r) << 16) + ((g) << 8) + (b))

#define SECONDS(s)    ((s) * 1000)
#define MINUTES(m)    SECONDS(m * 60)

#define SWITCHBOT_SD_SIZE 6
#define CYALKIT_MD_SIZE 25
#define SWITCHBOT_MD_SIZE 14

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
  // Byte:0-1
  uint16_t  companyID;  // 0x0969
  // Byte:2-7
  char      mac[6];     // mac
  // Byte:8-9 ?
  uint8_t reserved1;
  uint8_t reserved2;
  // Byte:10
  uint8_t decimalOfTheTemperature : 4;
  uint8_t humidityAlertStatus : 2;    //?
  uint8_t temperatureAlertStatus : 2; //?
  // Byte:11
  uint8_t integerOfTheTemperature : 7;
  uint8_t posiNegaTemperatureFlag : 1;
  // Byte:12
  uint8_t humidityValue : 7;
  uint8_t temperatureScale : 1;
} SwitchBotManufacturerData;

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

boolean fConnect = false;
WiFiClient client;
// Wi-Fi こちらの値を適宜書き換えてご使用ください。
const char * ssid     = "xxxx";
const char * password = "xxxx";

Ambient ambient;
// こちらの値を適宜書き換えてご使用ください。
unsigned int channelId = 0; // AmbientのチャネルID
const char* writeKey = "xxxx"; // ライトキー

// MACアドレスの登録管理
#define MAX_ADDRESS 8
#define FNAME       "/addr.txt"

int nAddresses = 0;
String addresses[MAX_ADDRESS];

void initAddressPos(void)
{
  Serial.print("initAddressPos()...");
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }
  Serial.println("OK");

  nAddresses = 0;
  for (int i = 0; i < MAX_ADDRESS; i++)
    addresses[i] = String("");
}

void printAddressPos(void)
{
  Serial.printf("***** printAddressPos(%d) *****\n", nAddresses);
  int loopCount = (nAddresses < MAX_ADDRESS) ? nAddresses : MAX_ADDRESS;
  for (int i = 0; i < loopCount; i++)
    Serial.printf(" address[%d]:'%s'\n", i, addresses[i].c_str());
  Serial.printf("*****\n");
}

void loadAddressPos(void)
{
  Serial.println("loadAddressPos():");
  File file = SPIFFS.open(FNAME, FILE_READ);
  if(!file){
      Serial.println(" failed to open");
      nAddresses = 0;
      for (int i = 0; i < MAX_ADDRESS; i++)
        addresses[i] = String("");
      return;
  }
  nAddresses = file.readStringUntil('\n').toInt();
  Serial.printf(" nAddresses:%d\n", nAddresses);
  for (int i = 0; i < MAX_ADDRESS; i++) {
    String s = file.readStringUntil('\n');
    Serial.printf(" ***address[%d]:'%s'\n", i, s.c_str());
    addresses[i] = s;
  }
  file.close();
  printAddressPos();
}

void verifyAddressPos(void)
{
  Serial.println("verifyAddressPos():");
  File file = SPIFFS.open(FNAME, FILE_READ);
  if(!file){
      Serial.println(" failed to open");
      return;
  }
  int n = file.readStringUntil('\n').toInt();
  Serial.printf(" nAddresses:%d\n", n);
  for (int i = 0; i < MAX_ADDRESS; i++) {
    String s = file.readStringUntil('\n');
    Serial.printf(" ***address[%d]:'%s'\n", i, s.c_str());
  }
  file.close();
}

void saveAddressPos(void)
{
  Serial.println("saveAddressPos():");
  File file = SPIFFS.open(FNAME, FILE_WRITE);
  if(!file || file.isDirectory()){
      Serial.println(" failed to open");
      return;
  }
  file.println(nAddresses);
  Serial.printf(" nAddresses:%d\n", nAddresses);
  for (int i = 0; i < MAX_ADDRESS; i++) {
    String s = addresses[i];
    file.println(s);
    Serial.printf(" address[%d]:'%s'\n", i, s.c_str());
  }
  file.close();
}

void clearAddressPos(void)
{
  Serial.println("clearAddressPos()");
  SPIFFS.remove(FNAME);
  nAddresses = 0;
  for (int i = 0; i < MAX_ADDRESS; i++)
    addresses[i] = String("");
}

int getAddressPos(String tAddress)
{
  // 見つけた文字列毎に番号を振り、その番号を返す。
  // 最大数を超えた場合、-1を返す。
  Serial.printf("getAddressPos('%s'): nAddresses:%d\n", tAddress.c_str(), nAddresses);
  for (int i = 0; i < nAddresses; i++) {
    tAddress.trim();
    addresses[i].trim();
    Serial.printf(" tAddress:'%s' addresses[%d]:'%s'\n", tAddress.c_str(), i, addresses[i].c_str());
    if (tAddress.equals(addresses[i])) {
      // すでにストアされている場合、そのインデックスを返す
      Serial.printf(" Found:%d\n", i);
      return i;
    }
  }

  if (nAddresses < MAX_ADDRESS) {
    // 新たにストアして、インデックスを返す
    addresses[nAddresses++] = tAddress;
    Serial.printf(" New:%d\n", nAddresses - 1);
    return nAddresses - 1;
  }

  // 最大数を超えた場合は-1を返す
  Serial.println(" Not found:-1");
  return -1;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    //activeサイン
    static int ff = 1;
    M5.dis.drawpix(4, 4, (ff ^= 1) ? RGB(0x40, 0x00, 0x00) : 0);

    Serial.printf("AD:%s\n", advertisedDevice.toString().c_str());

    BLEAddress address = advertisedDevice.getAddress();

    if (advertisedDevice.haveServiceData()) {
      std::string d = advertisedDevice.getServiceData();
      Serial.printf("SD(%d):", d.size());
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

      if (sd.deviceType == 'T' || sd.deviceType == 'i') { // SwitchBot MeterTH or MeterPlus

        float temperature = ((sd.posiNegaTemperatureFlag) ? 1.0 : -1.0) *
                            (sd.decimalOfTheTemperature * 0.1 +
                             sd.integerOfTheTemperature);
        float humidity = (float)sd.humidityValue;

        Serial.printf("temp:%.1f\n", temperature);
        Serial.printf("humi:%.1f\n", humidity);

        //温度のみ送信する。
        int pos = getAddressPos(String(address.toString().c_str()));
        if (pos >= 0 && fConnect) {
          ambient.set(pos + 1, temperature);
          Serial.printf("ambient.set(%d, %.1f);\n", pos + 1, temperature);
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

    // Switchbot防水温湿度計を特定する。
    if (advertisedDevice.haveManufacturerData() == true
        && advertisedDevice.getManufacturerData().length() == SWITCHBOT_MD_SIZE) {

      SwitchBotManufacturerData md;
      memcpy(&md, advertisedDevice.getManufacturerData().data(),
          advertisedDevice.getManufacturerData().length());

      if (md.companyID == 0x0969) {
        float temperature = ((md.posiNegaTemperatureFlag) ? 1.0 : -1.0) *
                            (md.decimalOfTheTemperature * 0.1 +
                             md.integerOfTheTemperature);
        float humidity = (float)md.humidityValue;

        Serial.printf("temp:%.1f\n", temperature);
        Serial.printf("humi:%.1f\n", humidity);

        //温度のみ送信する。
        int pos = getAddressPos(String(address.toString().c_str()));
        if (pos >= 0 && fConnect) {
          ambient.set(pos + 1, temperature);
          Serial.printf("ambient.set(%d, %.1f);\n", pos + 1, temperature);
        }
      }

      Serial.printf("SwitchbotManufacturerData(%d):\n", advertisedDevice.getManufacturerData().length());
      Serial.printf("   companyID:%04x\n", md.companyID);
      Serial.printf("   mac:%02x:%02x:%02x:%02x\n", md.mac[0], md.mac[1], md.mac[2], md.mac[3]);
      Serial.printf("   reserved1:%02x\n", md.reserved1);
      Serial.printf("   reserved2:%02x\n", md.reserved2);
      Serial.printf("   decimalOfTheTemperature:%02x\n", md.decimalOfTheTemperature);
      Serial.printf("   humidityAlertStatus:%02x\n", md.humidityAlertStatus);
      Serial.printf("   temperatureAlertStatus:%02x\n", md.temperatureAlertStatus);
      Serial.printf("   integerOfTheTemperature:%02x\n", md.integerOfTheTemperature);
      Serial.printf("   posiNegaTemperatureFlag:%02x\n", md.posiNegaTemperatureFlag);
      Serial.printf("   humidityValue:%02x\n", md.humidityValue);
      Serial.printf("   temperatureScale:%02x\n", md.temperatureScale);
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

        Serial.printf("temp:%.1f\n", temperature);
        Serial.printf("humi:%.1f\n", humidity);

        //温度のみ送信する。
        int pos = getAddressPos(String(address.toString().c_str()));
        if (pos >= 0 && fConnect) {
          ambient.set(pos + 1, temperature);
          Serial.printf("ambient.set(%d, %.1f);\n", pos + 1, temperature);
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

  // Wifi不安定対策:大野さんのアドバイスによる
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);

  initAddressPos();
  loadAddressPos();
  
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
  M5.update();

  if (M5.Btn.wasReleasefor(3000)) {
    clearAddressPos();
    for (int i; i < 25; i++)
      M5.dis.drawpix(i, 0);
  }
  
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

  saveAddressPos();
  verifyAddressPos();
  printAddressPos();

  delay(SECONDS(waitTime));
}
