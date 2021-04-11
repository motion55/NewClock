
//#include <Arduino.h>

#if defined(ESP8266)
#include <avr/pgmspace.h>
#endif

#define _USE_BMP280_  0
#define _USE_DS3231_  0

#define _DEBUG_NTP_   1 

#if _USE_BMP280_
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#endif

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Time.h>
#include <TimeLib.h>

#if _USE_BMP280_
Adafruit_BMP280 bme; // I2C
boolean Use_bmp280;
String bmp280_str;
float Temperature, Pressure;
int T_samples, P_samples;
float Ave_Temperature, Ave_Pressure;
int Count = 0;
#define	AVE_SAMPLES	128
#endif

String sta_ssid("BST");  //  your network SSID (name)
String sta_pass;            // your network password

unsigned int localPort = 2390;      // local port to listen for UDP packets

const char *ntpServerName = "pool.ntp.org"; //"ntp.pagasa.dost.gov.ph";
IPAddress timeServerIP(192, 168, 7, 1);     // IP address of

const int NTP_PACKET_SIZE = 48;	// NTP time stamp is in the first 48 bytes of the message

//buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];

//A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;


// Access point credentials
const char *ap_ssid = "MatrixClock";
const char *ap_password = "12345678";

///////////////////////////////////////////////////////////////////

const int numDevices = 8;      // number of MAX7219s used
const int _SPI_CS = 15;     //D8
const int _SPI_MOSI = 13;   //D7
const int _SPI_CLK = 14;    //D5;

#define ScrollBeginPos  32

extern int LoadPos;

void InitMax7219();
void UpdateTime(void);
int LoadMessage(const char *message);
void ResetScrollPos(void);
int LoadDisplayBuffer(int BufferLen);
void sendNTPpacket(IPAddress& address);

#if _USE_BMP280_
void LoadDisplayBMP280(void);
#endif

void setup(void) {
  InitMax7219();

	Serial.begin(115200);

  IPAddress local_IP(192,168,25,1);
  IPAddress gateway(192,168,25,1);
  IPAddress subnet(255,255,255,0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid,ap_password); 
  
#if _USE_DS3231_
	DS3231_setup();
#endif 

#if _USE_BMP280_
	if (!bme.begin(0x76))
	{
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
		Use_bmp280 = false;
	}
	else
	{
		for (int i = 0; i < 100 && Ave_Temperature<20.0 && Ave_Pressure<50000.0; i++)
		{
			delay(10);
			Ave_Temperature = bme.readTemperature();
			Ave_Pressure = bme.readPressure();
		}
		Temperature = Ave_Temperature * AVE_SAMPLES;
		Pressure = Ave_Pressure * AVE_SAMPLES;
		Count = 0;
		Use_bmp280 = true;
	}
#endif
  delay(3000);

  String ConnectStr("Connecting... ");
  ResetScrollPos();
  int Len = LoadMessage(ConnectStr.c_str());
  for (int i=0; i<200; i++)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
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

	for (int i = 0; i<400; i++)
	{
		if (WiFi.status() == WL_CONNECTED)
		{
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

	IPAddress addr;
	if (WiFi.hostByName(ntpServerName, addr))
	{
		timeServerIP = addr;
	}

	setSyncProvider(getNtpTime);

	webserver_setup();
}

char TimeText[] = "00:00:00am\0";
//                 01234567890

int LogoCount = 0;
int BufferEnd = ScrollBeginPos;

void loop(void) {
	UpdateTime();

  if (LoadDisplayBuffer(BufferEnd) == ScrollBeginPos) {
    if (LogoOn())
    {
      LogoCount++;
      if (LogoCount > 5) {
        LogoCount = 0;
        SetLogo(false);
        String Timestr(TimeText);
        Timestr += " ";
        Timestr += GetDateStr();
        BufferEnd = LoadMessage(Timestr.c_str());
      } else
      if (LogoCount == 3) {
        SetLogo(false);
        LoadDisplayBMP280();
        String Timestr(TimeText);
#if _USE_BMP280_
        Timestr += " ";
        Timestr += bmp280_str;
#endif
        BufferEnd = LoadMessage(Timestr.c_str());
      } else {
        BufferEnd = LoadMessage(TimeText);
      }
    }
    else
    {
      SetLogo(true);
      BufferEnd = LoadMessage(TimeText);
    }
  } else  {
#if _USE_BMP280_
    if (Use_bmp280)
    {
      Ave_Temperature = Temperature / AVE_SAMPLES;
      Temperature -= Ave_Temperature;
      Temperature += bme.readTemperature();
  
      Ave_Pressure = Pressure / AVE_SAMPLES;
      Pressure -= Ave_Pressure;
      Pressure += bme.readPressure();
    }
#endif
    ReloadMessage(ScrollBeginPos, TimeText);
  }

  webserver_loop();

	my_delay_ms(50);
}

/*///////////////////////////////////////////////////////////////////////////*/

String GetDateStr(void)
{
  tmElements_t tm;
  breakTime(now(),tm);
  String DateStr(monthShortStr(tm.Month));
  DateStr += " " + String(tm.Day) + " " + dayShortStr(tm.Wday) + " ";
  return DateStr;
}

#if (ScrollBeginPos>0)    
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
  0b01010110,
  // 3
  0b01001001,
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
  0b01000110,
  0b01001001,
  0b00111110,
};
#endif

extern unsigned char ColumnBuffer[];

void UpdateTime(void)
{
  time_t tm = now();

  int hrs = hourFormat12(tm);
  if (hrs < 10)
  {
    TimeText[0] = ' ';
#if (ScrollBeginPos>0)    
    ColumnBuffer[0] = 0;
#endif    
  }
  else
  {
    hrs -= 10;
    TimeText[0] = '1';
#if (ScrollBeginPos>0)    
    ColumnBuffer[0] = 0b01111111;
#endif  
  }
#if (ScrollBeginPos>0)    
  ColumnBuffer[1]= 0;
#endif  
  
  TimeText[1] = '0' + hrs;
#if (ScrollBeginPos>0)    
  hrs *= 3;
  
  memcpy_P(&ColumnBuffer[2], NumFont + hrs, 3);
  ColumnBuffer[5] = 0;

  ColumnBuffer[6] = 0b00110110;
  ColumnBuffer[7] = 0;
#endif  
  
  int mins = minute(tm);
  int min10 = mins / 10;
  mins = mins % 10;
  
  TimeText[3] = '0' + min10;
#if (ScrollBeginPos>0)    
  min10 *= 3;
  memcpy_P(&ColumnBuffer[8], NumFont + min10, 3);
  ColumnBuffer[11] = 0;
#endif  
  
  TimeText[4] = '0' + mins;
#if (ScrollBeginPos>0)    
  mins *= 3;
  memcpy_P(&ColumnBuffer[12], NumFont + mins, 3);
  ColumnBuffer[15] = 0;
  
  ColumnBuffer[16] = 0b00110110;
  ColumnBuffer[17] = 0;
#endif  

  int sec = second(tm);
  int sec10 = sec / 10;
  sec = sec % 10;
  
  TimeText[6] = '0' + sec10;
#if (ScrollBeginPos>0)    
  sec10 *= 3;
  memcpy_P(&ColumnBuffer[18], NumFont + sec10, 3);
  ColumnBuffer[21] = 0;
#endif  
  
  TimeText[7] = '0' + sec;
#if (ScrollBeginPos>0)    
  sec *= 3;
  memcpy_P(&ColumnBuffer[22], NumFont + sec, 3);
  ColumnBuffer[25] = 0;
#endif  

  if (isAM(tm))
  {
    TimeText[8] = 'a';
    TimeText[9] = 'm';
  }
  else
  {
    TimeText[8] = 'p';
    TimeText[9] = 'm';
  }
}

const int timeZone = 8 * SECS_PER_HOUR;     // PHT
#define MAX_PACKET_DELAY 1500
uint32_t send_Timestamp;
uint32_t packet_delay = 1500;


// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
	if (WiFi.status() == WL_CONNECTED)
	{
		Serial.println(F("sending NTP packet..."));
		// set all bytes in the buffer to 0
		memset(packetBuffer, 0, NTP_PACKET_SIZE);
		// Initialize values needed to form NTP request
		// (see URL above for details on the packets)
		packetBuffer[0] = 0b11100011;   // LI, Version, Mode
		packetBuffer[1] = 0;     // Stratum, or type of clock
		packetBuffer[2] = 6;     // Polling Interval
		packetBuffer[3] = 0xEC;  // Peer Clock Precision
								 // 8 bytes of zero for Root Delay & Root Dispersion
		packetBuffer[12] = 49;
		packetBuffer[13] = 0x4E;
		packetBuffer[14] = 49;
		packetBuffer[15] = 52;

    send_Timestamp = millis();
    packet_delay = 1500;
    
		// all NTP fields have been given values, now
		// you can send a packet requesting a timestamp:
		udp.beginPacket(address, 123); //NTP requests are to port 123
		udp.write(packetBuffer, NTP_PACKET_SIZE);
		udp.endPacket();
	}
}

time_t getNtpTime() {
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

void my_delay_ms(int msec)
{
	uint32_t delay_val = msec;
	uint32_t endWait = millis();
	uint32_t beginWait = endWait;
	while (endWait - beginWait < delay_val)
	{
    if ((endWait - send_Timestamp) < packet_delay)
		{
      int size = udp.parsePacket();
      
			if (size >= NTP_PACKET_SIZE)
			{
				Serial.println(F("Receive NTP Response"));
				udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
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

void LoadDisplayBMP280(void)
{
#if _USE_BMP280_
  if (Use_bmp280)
  {
    if (T_samples > 0)
    {
      bmp280_str = String(Temperature / T_samples, 1);
    }
    else
    {
      bmp280_str = String(bme.readTemperature(), 1);
    }
    Temperature = 0.0f;
    T_samples = 0;
    bmp280_str += String("C ");
    if (P_samples > 0)
    {
      bmp280_str += String(Pressure / (P_samples * 100), 2);
    }
    else
    {
      bmp280_str += String(bme.readPressure() / 100.0, 2);
    }
    Pressure = 0.0f;
    P_samples = 0;
    bmp280_str += String("hPa ");
  }
  else
  {
    bmp280_str = String(" No BMP280 detected! ");
  }
#endif
}
