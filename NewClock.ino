
//#include <Arduino.h>

#if defined(ESP8266)
#include <avr/pgmspace.h>
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Time.h>
#include <TimeLib.h>

Adafruit_BMP280 bme; // I2C
boolean Use_bmp280;
String bmp280_str;
float Temperature, Pressure;
int T_samples, P_samples;
float Ave_Temperature, Ave_Pressure;
int Count = 0;
#define	AVE_SAMPLES	128

String sta_ssid("BST");  //  your network SSID (name)
String sta_pass;            // your network password

unsigned int localPort = 2390;      // local port to listen for UDP packets

const char* ntpServerName = "ntp.pagasa.dost.gov.ph";
IPAddress timeServerIP(121, 58, 193, 100);	//IP address of "ntp.pagasa.dost.gov.ph"

const int NTP_PACKET_SIZE = 48;	// NTP time stamp is in the first 48 bytes of the message

//buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];

//A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;


// Access point credentials
const char *ap_ssid = "OLEDClock";
const char *ap_password = "bst142857";

///////////////////////////////////////////////////////////////////

const int numDevices = 8;      // number of MAX7219s used
const int SPI_CS = 15;
const int SPI_MOSI = 13;
const int SPI_CLK = 14;

char scrollText[] = "00:00:00am \0";
//                   01234567890

extern int LoadPos;

void InitMax7219();
void UpdateTime(void);
int LoadMessage(const char *message);
void ResetScrollPos(void);
int LoadDisplayBuffer(int BufferLen);
void sendNTPpacket(IPAddress& address);

void LoadDisplayBMP280(void);

void setup(void) {
	Serial.begin(115200);

  IPAddress local_IP(192,168,8,1);
  IPAddress gateway(192,168,8,1);
  IPAddress subnet(255,255,255,0);

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid,ap_password); 
  
	DS3231_setup();

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

  delay(3000);

  InitMax7219();

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
int BufferEnd = 0;

void loop(void) {
	UpdateTime();

  if (LoadDisplayBuffer(BufferEnd) == 0) {
    if (LogoOn())
    {
      LogoCount++;
      if (LogoCount > 5) {
        LogoCount = 0;
        SetLogo(false);
        String Timestr(scrollText);
        Timestr += GetDateStr();
        BufferEnd = LoadMessage(Timestr.c_str());
      } else
      if (LogoCount == 3) {
        SetLogo(false);
        LoadDisplayBMP280();
        String Timestr(scrollText);
        Timestr += bmp280_str;
        BufferEnd = LoadMessage(Timestr.c_str());
      } else {
        BufferEnd = LoadMessage(scrollText);
      }
    }
    else
    {
      SetLogo(true);
      BufferEnd = LoadMessage(scrollText);
    }
  } else  {
    
    if (Use_bmp280)
    {
      Ave_Temperature = Temperature / AVE_SAMPLES;
      Temperature -= Ave_Temperature;
      Temperature += bme.readTemperature();
  
      Ave_Pressure = Pressure / AVE_SAMPLES;
      Pressure -= Ave_Pressure;
      Pressure += bme.readPressure();
    }

    ReloadMessage(0, scrollText);
  }
 

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

void UpdateTime(void)
{
  time_t tm = now();

  int hour = hourFormat12(tm);
  if (hour < 10)
  {
    scrollText[0] = ' ';
    scrollText[1] = '0' + hour;
  }
  else
  {
    scrollText[0] = '1';
    scrollText[1] = '0' + (hour-10);
  }

  int min = minute(tm);
  int min10 = min / 10;
  scrollText[3] = '0' + min10;
  scrollText[4] = '0' + min - (min10 * 10);

  int sec = second(tm);
  int sec10 = sec / 10;
  scrollText[6] = '0' + sec10;
  scrollText[7] = '0' + sec - (sec10 * 10);

  if (isAM(tm))
  {
    scrollText[8] = 'a';
    scrollText[9] = 'm';
  }
  else
  {
    scrollText[8] = 'p';
    scrollText[9] = 'm';
  }
}

const int timeZone = 8 * SECS_PER_HOUR;     // PHT
int packet_delay = 0;

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

		// all NTP fields have been given values, now
		// you can send a packet requesting a timestamp:
		udp.beginPacket(address, 123); //NTP requests are to port 123
		udp.write(packetBuffer, NTP_PACKET_SIZE);
		udp.endPacket();

		packet_delay = 1500;		 // wait to see if a reply is available
	}
}

time_t getNtpTime()
{
	Serial.println(F("Transmit NTP Request"));
	sendNTPpacket(timeServerIP); // send an NTP packet to a time server

	return 0;
}

void my_delay_ms(int msec)
{
	uint32_t delay_val = msec;
	uint32_t endWait = millis();
	uint32_t beginWait = endWait;
	while (endWait - beginWait < delay_val)
	{
		webserver_loop();

		int size = udp.parsePacket();
		if (packet_delay > 0)
		{
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

				time_t tm = secsSince1900 - 2208988800UL + timeZone;
				setTime(tm);
				DS3231_setTime(tm);
				packet_delay = 0;
			}
		}
		delay(1);
		endWait = millis();
	}
	if (packet_delay > 0)
		packet_delay -= msec;

}

void LoadDisplayBMP280(void)
{
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
}
