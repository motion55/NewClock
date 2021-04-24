
#include <Arduino.h>
#include <ArduinoJson.h>

#if defined(ESP8266)
#include <avr/pgmspace.h>
#endif

#define _USE_BME280_ 1
#define _USE_DS3231_ 0

#define _DEBUG_NTP_ 1

#if _USE_BME280_
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#endif

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Time.h>
#include <TimeLib.h>

#include <WiFiClientSecure.h>

#if _USE_BME280_
#define DEF_BME280_ADDR 0x76

Adafruit_BME280 bme; // I2C
boolean Use_bme280;
String BME280_str;
float Temperature, Pressure, Humidity;
int T_samples, P_samples, H_samples;
#endif

String sta_ssid("BST"); // your network SSID (name)
String sta_pass;        // your network password

unsigned int localPort = 2390; // local port to listen for UDP packets

const char *ntpServerName = "pool.ntp.org"; //"ntp.pagasa.dost.gov.ph";
//IPAddress timeServerIP(192, 168, 7, 1);     // IP address of

const int NTP_PACKET_SIZE =
    48; // NTP time stamp is in the first 48 bytes of the message

// buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

// Access point credentials
const char *ap_ssid = "MatrixClock";
const char *ap_password = "12345678";

///////////////////////////////////////////////////////////////////

const int numDevices = 8; // number of MAX7219s used
const int _SPI_CS = 12;   // D6 was D8 (15)
const int _SPI_MOSI = 13; // D7
const int _SPI_CLK = 14;  // D5;

#define ScrollBeginPos 32

extern int LoadPos;

void InitMax7219();
void UpdateTime(void);
int LoadMessage(const char *message);
void ResetScrollPos(void);
int LoadDisplayBuffer(int BufferLen);
void sendNTPpacket(IPAddress &address);

#if _USE_BME280_
void LoadDisplayBME280(void);
#endif

// https://api.coinmarketcap.com/v2/listings/ for find the {id} of the currency
// Change the name of the currency and put the {id} in " "
#define BITCOIN "1"
#define ETHEREUM "1027"
#define RIPPLE "52"
#define LITECOIN "2"
#define DASH "131"

// and change again the name here
String crypto[] = {BITCOIN, ETHEREUM, RIPPLE, LITECOIN, DASH};
int coin = 0;
String oldPrice[5];

void setup(void) {
  InitMax7219();

  Serial.begin(115200);

  WiFi.begin();

  IPAddress local_IP(192, 168, 25, 1);
  IPAddress gateway(192, 168, 25, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);

#if _USE_DS3231_
  DS3231_setup();
#endif

#if _USE_BME280_
  if (bme.begin(DEF_BME280_ADDR)) {
    Use_bme280 = true;
  } else if (bme.begin()) {
    Use_bme280 = true;
  } else {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    Use_bme280 = false;
  }

  if (Use_bme280) {
    Temperature = bme.readTemperature();
    T_samples = 1;
    Pressure = bme.readPressure();
    P_samples = 1;
    Humidity = bme.readHumidity();
    H_samples = 1;
  }
#endif

  delay(3000);

  InitColumnBuffer();
  LoadDisplayBuffer(ScrollBeginPos);

  String ConnectStr("Connecting... ");
  ResetScrollPos();
  int Len = LoadMessage(ConnectStr.c_str());
  for (int i = 0; i < 200; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("WiFi connected"));
      Serial.println(F("IP address: "));
      Serial.println(WiFi.localIP());
      break;
    }
    LoadDisplayBuffer(LoadPos);
    delay(50);
  }

  InitMax7219();

  ResetScrollPos();

  for (int i = 0; i < 400; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("WiFi connected"));
      Serial.println(F("IP address: "));
      Serial.println(WiFi.localIP());
      break;
    }
    delay(50);
  }

  Serial.println(F("Starting UDP"));
  udp.begin(localPort);
  Serial.print(F("Local port: "));
  Serial.println(udp.localPort());

  IPAddress timeServerIP(192, 168, 7, 1);
  IPAddress addr;
  if (WiFi.hostByName(ntpServerName, addr)) {
    timeServerIP = addr;
  }

  setSyncProvider(getNtpTime);
  setSyncInterval(300); // Update after 300 for the 1st hourt.

  webserver_setup();
}

#define _Scroll_Time_ 0

#if _Scroll_Time_
char TimeText[] = "00:00:00am\0";
// 01234567890
#endif

int LogoCount = 0;
int BufferEnd = ScrollBeginPos;

void loop(void) {
  UpdateTime();
#if 1
  if (LoadDisplayBuffer(BufferEnd) == ScrollBeginPos) {
 #if _Scroll_Time_
    String Timestr(TimeText);
    Timestr += " ";
    Timestr += GetDateStr();
 #else
    String Timestr(GetDateStr());
 #endif
    if (LogoOn()) {
      LogoCount++;
      if (LogoCount > 5) {
        LogoCount = 0;
        SetLogo(false);
      } else if ((LogoCount == 1)||(LogoCount == 3)||(LogoCount == 5)) {
        SetLogo(false);
      } else {
      }
    } else {
      SetLogo(true);
    }
#if _USE_BME280_
    LoadDisplayBME280();
    Timestr += " ";
    Timestr += BME280_str;
#endif
    BufferEnd = LoadMessage(Timestr.c_str());
    Serial.println(Timestr);
  } else {
#if _USE_BME280_
    Temperature += bme.readTemperature();
    T_samples++;
    Pressure += bme.readPressure();
    P_samples++;
    Humidity += bme.readHumidity();
    H_samples++;
#endif
#if _Scroll_Time_
    ReloadMessage(ScrollBeginPos, TimeText);
#endif    
  }
#endif
  //CheckBitCoin();

  webserver_loop();

  my_delay_ms(50);
}

/*///////////////////////////////////////////////////////////////////////////*/

String GetDateStr(void) {
  tmElements_t tm;
  breakTime(now(), tm);
  String DateStr(monthShortStr(tm.Month));
  DateStr += " " + String(tm.Day) + " " + dayShortStr(tm.Wday) + " ";
  return DateStr;
}

#if (ScrollBeginPos > 0)

#define NumWidth 4

#if (NumWidth == 4)
const unsigned char NumFont[] PROGMEM = {
    // 0
    0b00111110,
    0b01000001,
    0b01000001,
    0b00111110,
    // 1
    0b00000000,
    0b01000010,
    0b01111111,
    0b01000000,
    // 2
    0b01110010,
    0b01001001,
    0b01001001,
    0b01000110,
    // 3
    0b00100010,
    0b01001001,
    0b01001001,
    0b00110110,
    // 4
    0b00001111,
    0b00001000,
    0b00001000,
    0b01111111,
    // 5
    0b01001111,
    0b01001001,
    0b01001001,
    0b00110001,
    // 6
    0b00111110,
    0b01001001,
    0b01001001,
    0b00110010,
    // 7
    0b00000001,
    0b01110001,
    0b00001001,
    0b00000111,
    // 8
    0b00110110,
    0b01001001,
    0b01001001,
    0b00110110,
    // 9
    0b00100110,
    0b01001001,
    0b01001001,
    0b00111110,
};
#else
const unsigned char NumFont[] PROGMEM = {
    // 0
    0b00111110,
    0b01000001,
    0b00111110,
    // 1
    0b00000000,
    0b01111111,
    0b00000000,
    // 2
    0b01110010,
    0b01001001,
    0b01000110,
    // 3
//    0b01001001,
    0b00100010,
    0b01001001,
    0b00110110,
    // 4
    0b00001111,
    0b00001000,
    0b01111111,
    // 5
    0b01001111,
    0b01001001,
    0b00110001,
    // 6
    0b00111110,
    0b01001001,
    0b00110010,
    // 7
    0b01100011,
    0b00011001,
    0b00000111,
    // 8
    0b00110110,
    0b01001001,
    0b00110110,
    // 9
    0b00100110,
    0b01001001,
    0b00111110,
};
#endif
#endif

extern unsigned char ColumnBuffer[];

void UpdateTime(void) {
  time_t tm = now();

  int hrs = hourFormat12(tm);
  if (hrs < 10) {
#if _Scroll_Time_
    TimeText[0] = ' ';
#endif
#if (ScrollBeginPos > 0)
    ColumnBuffer[0] = 0;
    ColumnBuffer[1] = 0;
    ColumnBuffer[2] = 0;
    ColumnBuffer[3] = 0;
#endif
  } else {
    hrs -= 10;
#if _Scroll_Time_
    TimeText[0] = '1';
#endif
#if (ScrollBeginPos > 0)
    ColumnBuffer[0] = 0b01000010;
    ColumnBuffer[1] = 0b01111111;
    ColumnBuffer[2] = 0b01000000;
    ColumnBuffer[3] = 0;
#endif
  }

#if _Scroll_Time_
  TimeText[1] = '0' + hrs;
#endif
#if (ScrollBeginPos > 0)
  hrs *= NumWidth;

  memcpy_P(&ColumnBuffer[4], NumFont + hrs, NumWidth);
  ColumnBuffer[8] = 0;

  ColumnBuffer[9] = 0b00110110;
  ColumnBuffer[10] = 0;
#endif

  int mins = minute(tm);
  int min10 = mins / 10;
  mins = mins % 10;

#if _Scroll_Time_
  TimeText[3] = '0' + min10;
#endif
#if (ScrollBeginPos > 0)
  min10 *= NumWidth;
  memcpy_P(&ColumnBuffer[11], NumFont + min10, NumWidth);
  ColumnBuffer[15] = 0;
#endif

#if _Scroll_Time_
  TimeText[4] = '0' + mins;
#endif
#if (ScrollBeginPos > 0)
  mins *= NumWidth;
  memcpy_P(&ColumnBuffer[16], NumFont + mins, NumWidth);
  ColumnBuffer[20] = 0;

  ColumnBuffer[21] = 0b00110110;
  ColumnBuffer[22] = 0;
#endif

  int sec = second(tm);
  int sec10 = sec / 10;
  sec = sec % 10;

#if _Scroll_Time_
  TimeText[6] = '0' + sec10;
#endif
#if (ScrollBeginPos > 0)
  sec10 *= NumWidth;
  memcpy_P(&ColumnBuffer[23], NumFont + sec10, NumWidth);
  ColumnBuffer[27] = 0;
#endif

#if _Scroll_Time_
  TimeText[7] = '0' + sec;
#endif
#if (ScrollBeginPos > 0)
  sec *= NumWidth;
  memcpy_P(&ColumnBuffer[28], NumFont + sec, NumWidth);
#endif
#if _Scroll_Time_
  if (isAM(tm)) {
    TimeText[8] = 'a';
    TimeText[9] = 'm';
  } else {
    TimeText[8] = 'p';
    TimeText[9] = 'm';
  }
#endif
}

const int timeZone = 8 * SECS_PER_HOUR; // PHT
#define MAX_PACKET_DELAY 1500
uint32_t send_Timestamp;
uint32_t packet_delay = 1500;
bool first_hour = 0;

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("sending NTP packet..."));
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    send_Timestamp = millis();
    packet_delay = 1500;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); // NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
  }
}

time_t getNtpTime() {
  IPAddress timeServerIP(192, 168, 7, 1);
  IPAddress addr;
  if (WiFi.hostByName(ntpServerName, addr)) {
    timeServerIP = addr;
  }
#if _DEBUG_NTP_
  Serial.println(F("Transmit NTP Request"));
#endif
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  setSyncInterval(300);        // retry after 5 minutes

  return 0;
}

void my_delay_ms(int msec) {
  uint32_t delay_val = msec;
  uint32_t endWait = millis();
  uint32_t beginWait = endWait;
  while (endWait - beginWait < delay_val) {
    if ((endWait - send_Timestamp) < packet_delay) {
      int size = udp.parsePacket();

      if (size >= NTP_PACKET_SIZE) {
        Serial.println(F("Receive NTP Response"));
        udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 = (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];

        uint32_t pingTime = (endWait - send_Timestamp) / 2;
#ifdef _DEBUG_NTP_
        Serial.print("receive time = ");
        Serial.println(pingTime);
#endif
        uint32_t frac_sec = (unsigned long)packetBuffer[44] << 8;
        frac_sec += (unsigned long)packetBuffer[45];
        frac_sec *= 1000;
        frac_sec /= 65536;
        frac_sec += pingTime;

        if (frac_sec >= 500)
          secsSince1900 += 1;

        time_t tm = secsSince1900 - 2208988800UL + timeZone;

        if ((endWait > 3600000L) || first_hour) {
          setSyncInterval(4000); // Update after 1 hour.
          first_hour = 1;
        } else {
          setSyncInterval(600); // Update after 60 for the 1st hourt.
        }
      
        setTime(tm);
#if _USE_DS3231_
        DS3231_setTime(tm);
#endif
        packet_delay = 0;
      }
    }
    delay(1);
    endWait = millis();
  }
}

void LoadDisplayBME280(void) {
#if _USE_BME280_
  //String sampleStr = " ";
  if (Use_bme280) {
    if (T_samples > 0) {
      BME280_str = String(Temperature / T_samples, 1);
    }
    else {
      BME280_str = String(bme.readTemperature(), 1);
    }
    //sampleStr += String(T_samples) + " ";
    Temperature = 0.0f;
    T_samples = 0;
    BME280_str += String("C ");

    if (H_samples > 0) {
      BME280_str += String(Humidity / H_samples, 1);
    } else {
      BME280_str += String(bme.readHumidity(), 1);
    }
    //sampleStr += String(H_samples) + " ";
    Humidity = 0.0f;
    H_samples = 0;
    BME280_str += String("% ");

    if (P_samples > 0) {
      BME280_str += String(Pressure / (P_samples * 100), 2);
    } else {
      BME280_str += String(bme.readPressure() / 100.0, 2);
    }
    //sampleStr += String(P_samples) + " ";
    Pressure = 0.0f;
    P_samples = 0;
    BME280_str += String("hPa ");
  } else {
    BME280_str = String(" No BME280 detected! ");
  }
  //Serial.print(sampleStr);
  //Serial.println(BME280_str);
#endif
}

WiFiClientSecure *client = NULL;
const char host[] = "api.coinmarketcap.com";
const int httpsPort = 443;

void CheckBitCoin() {
  static bool Request = false;
  static uint32_t interval = 10000;
  static uint32_t interval2 = 5000;
  static uint32_t previousMillis;

  uint32_t currentMillis = millis();

  if (!Request) {
    if ((currentMillis - previousMillis) > interval) {
      previousMillis = currentMillis;
      interval = 60000;

      Serial.print(">>> Connecting to ");
      Serial.println(host);

      if (client) delete client;
      client = new WiFiClientSecure;
      if (client ==NULL) return;

      if (!client->connect(host, httpsPort)) {
        Serial.println(" Connection failed");
        //return;
      }
      Serial.print("Requesting URL: ");
      Serial.println("Connected to server!");

      String getStr = "GET /v1/ticker/" + crypto[coin] + "/ HTTP/1.1";
      Serial.println(getStr);
      client->println(getStr);
      client->println("Host: api.coinmarketcap.com");
      client->println("Connection: close");
      client->println();

      Request = true;
    }
  } else {
    if ((currentMillis - previousMillis) < interval2) {
      String data;
      if (client->available()) {
        while (client->available()) {
          data = client->readStringUntil('\r');
          Serial.println(" Response from URL:");
          Serial.println(data);
        }
        client->stop();
        delete client;
        client = NULL;
        
        Request = false;

        data.replace('[', ' ');
        data.replace(']', ' ');

        char buffer[data.length() + 1];
        data.toCharArray(buffer, sizeof(buffer));
        buffer[data.length() + 1] = '\0';

        const size_t bufferSize = JSON_OBJECT_SIZE(21) + 400;
        DynamicJsonDocument jsonDoc(bufferSize);

        //JsonObject &root = jsonDoc.parseObject(buffer);
        auto err = deserializeJson(jsonDoc, buffer);
        if (err) {
          Serial.print(F("deserializeJson() failed with code "));
          Serial.println(err.c_str());
          return;
        }
#if 0
        JsonObject &data0 = root["data"];
        String name = data0["name"];     // "Bitcoin"
        String symbol = data0["symbol"]; // "BTC"

        JsonObject &data1 = data0["quotes"]["USD"];
        String price = data1["price"];                         // "573.137"
        String percent_change_1h = data1["percent_change_1h"]; // "0.04"
        String last_updated =
            data0["last_updated"];    // "1472762067" <-- Unix Time Stamp
        String error = root["error"]; // id not found
#endif        
      }
    } else {
      Serial.println(" Request Timeout!");
      delete client;
      client = NULL;
      Request = false;
    }
  }
}
