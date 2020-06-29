
#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

/*////////////////////////////////////////////////////////////////////////////////*/

#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

class MAX7219Control {
private:
	void spiTransfer(int maxbytes, byte *spidata) {
		//enable the line 
		digitalWrite(SPI_CS, LOW);
		//Now shift out the data 
		for (int i = maxbytes; i > 0; i--)
		{
			shiftOut(SPI_MOSI, SPI_CLK, MSBFIRST, spidata[i - 1]);
			delay(0);
		}
		//latch the data onto the display
		digitalWrite(SPI_CS, HIGH);
	}

	void spiTransfer(int addr, volatile byte opcode, volatile byte data) {
		//Create an array with the data to shift out
		byte spidata[16];
		/* The array for shifting the data to the devices */
		int offset = addr * 2;
		int maxbytes = maxDevices * 2;

		for (int i = 0; i<maxbytes; i++)
			spidata[i] = (byte)0;
		//put our device data into the array
		spidata[offset + 1] = opcode;
		spidata[offset] = data;

		spiTransfer(maxbytes, spidata);
	}

	/* Data is shifted out of this pin*/
	int SPI_MOSI;
	/* The clock is signaled on this pin */
	int SPI_CLK;
	/* This one is driven LOW for chip selection */
	int SPI_CS;
	/* The maximum number of devices we use */
	int maxDevices;
public:
	MAX7219Control(int dataPin, int clkPin, int csPin, int numDevices) {
		SPI_MOSI = dataPin;
		SPI_CLK = clkPin;
		SPI_CS = csPin;
		if (numDevices <= 0 || numDevices > 8)
			numDevices = 8;
		maxDevices = numDevices;
		pinMode(SPI_MOSI, OUTPUT);
		pinMode(SPI_CLK, OUTPUT);
		pinMode(SPI_CS, OUTPUT);
		digitalWrite(SPI_CS, HIGH);
		pinMode(10, OUTPUT);
		digitalWrite(10, HIGH); //To enusre SS pin is HIGH
		SPI_MOSI = dataPin;
		for (int i = 0; i < maxDevices; i++) {
			spiTransfer(i, OP_DISPLAYTEST, 0);
			//scanlimit is set to max on startup
			setScanLimit(i, 7);
			//decode is done in source
			spiTransfer(i, OP_DECODEMODE, 0);
			clearDisplay(i);
			//we go into shutdown-mode on startup
			shutdown(i, true);
		}
	}

	void shutdown(int addr, bool b) {
		if (addr<0 || addr >= maxDevices)
			return;
		if (b)
			spiTransfer(addr, OP_SHUTDOWN, 0);
		else
			spiTransfer(addr, OP_SHUTDOWN, 1);
	}

	void setScanLimit(int addr, int limit) {
		if (addr<0 || addr >= maxDevices)
			return;
		if (limit >= 0 && limit<8)
			spiTransfer(addr, OP_SCANLIMIT, limit);
	}

	void setIntensity(int addr, int intensity) {
		if (addr<0 || addr >= maxDevices)
			return;
		if (intensity >= 0 && intensity<16)
			spiTransfer(addr, OP_INTENSITY, intensity);
	}

	void clearDisplay(int addr) {
		if (addr<0 || addr >= maxDevices)
			return;
		for (int i = 0; i<8; i++) {
			spiTransfer(addr, i + 1, 0);
		}
	}

	void setRow(int row, byte *RowData) {
		if (row < 0 || row>7)
			return;
		byte spidata[16];
		/* The array for shifting the data to the devices */

		//put our device data into the array
		for (int addr = 0; addr < maxDevices; addr++)
		{
			int offset = addr * 2;
			spidata[offset + 1] = row + 1;
			spidata[offset] = RowData[addr];
		}

		int maxbytes = maxDevices * 2;
		spiTransfer(maxbytes, spidata);
	}
};

/*////////////////////////////////////////////////////////////////////////////////*/

const unsigned char font7x5[] PROGMEM = {
	//   offset = 0
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	// ! offset = 6
	0b01001111,
	0b00000000,
	// " offset = 8
	0b00000111,
	0b00000000,
	0b00000111,
	0b00000000,
	// # offset = 12
	0b00010100,
	0b01111111,
	0b00010100,
	0b01111111,
	0b00010100,
	0b00000000,
	// $ offset = 18
	0b00100100,
	0b00101010,
	0b01111111,
	0b00101010,
	0b00010010,
	0b00000000,
	// % offset = 24
	0b00100011,
	0b00010011,
	0b00001000,
	0b01100100,
	0b01100010,
	0b00000000,
	// & offset = 30
	0b00110110,
	0b01001001,
	0b01010101,
	0b00100010,
	0b01010000,
	0b00000000,
	// ' offset = 36
	0b00000101,
	0b00000011,
	0b00000000,
	// ( offset = 39
	0b00011100,
	0b00100010,
	0b01000001,
	0b00000000,
	// ) offset = 43
	0b01000001,
	0b00100010,
	0b00011100,
	0b00000000,
	// * offset = 47
	0b00010100,
	0b00001000,
	0b00111110,
	0b00001000,
	0b00010100,
	0b00000000,
	// + offset = 53
	0b00001000,
	0b00001000,
	0b00111110,
	0b00001000,
	0b00001000,
	0b00000000,
	// , offset = 59
	0b01010000,
	0b00110000,
	0b00000000,
	// - offset = 62
	0b00001000,
	0b00001000,
	0b00001000,
	0b00001000,
	0b00001000,
	0b00000000,
	// . offset = 68
	0b01100000,
	0b01100000,
	0b00000000,
	// / offset = 71
	0b00100000,
	0b00010000,
	0b00001000,
	0b00000100,
	0b00000010,
	0b00000000,
	// 0 offset = 77
	0b00111110,
	0b01010001,
	0b01001001,
	0b01000101,
	0b00111110,
	0b00000000,
	// 1 offset = 83
	0b00000000,
	0b01000010,
	0b01111111,
	0b01000000,
	0b00000000,
	0b00000000,
	// 2 offset = 89
	0b01000010,
	0b01100001,
	0b01010001,
	0b01001001,
	0b01000110,
	0b00000000,
	// 3 offset = 95
	0b00100001,
	0b01000001,
	0b01000101,
	0b01001011,
	0b00110001,
	0b00000000,
	// 4 offset = 101
	0b00011000,
	0b00010100,
	0b00010010,
	0b01111111,
	0b00010000,
	0b00000000,
	// 5 offset = 107
	0b00100111,
	0b01000101,
	0b01000101,
	0b01000101,
	0b00111001,
	0b00000000,
	// 6 offset = 113
	0b00111100,
	0b01001010,
	0b01001001,
	0b01001001,
	0b00110000,
	0b00000000,
	// 7 offset = 119
	0b00000011,
	0b00000001,
	0b01110001,
	0b00001001,
	0b00000111,
	0b00000000,
	// 8 offset = 125
	0b00110110,
	0b01001001,
	0b01001001,
	0b01001001,
	0b00110110,
	0b00000000,
	// 9 offset = 131
	0b00000110,
	0b01001001,
	0b01001001,
	0b00101001,
	0b00011110,
	0b00000000,
	// : offset = 137
	0b00110110,
	0b00110110,
	0b00000000,
	// ; offset = 140
	0b01010110,
	0b00110110,
	0b00000000,
	// < offset = 143
	0b00001000,
	0b00010100,
	0b00100010,
	0b01000001,
	0b00000000,
	// = offset = 148
	0b00010100,
	0b00010100,
	0b00010100,
	0b00010100,
	0b00010100,
	0b00000000,
	// > offset = 154
	0b01000001,
	0b00100010,
	0b00010100,
	0b00001000,
	0b00000000,
	// ? offset = 159
	0b00000010,
	0b00000001,
	0b01010001,
	0b00001001,
	0b00000110,
	0b00000000,
	// @ offset = 165
	0b00110010,
	0b01001001,
	0b01111001,
	0b01000001,
	0b00111110,
	0b00000000,
	// A offset = 171
	0b01111110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b01111110,
	0b00000000,
	// B offset = 177
	0b01111111,
	0b01001001,
	0b01001001,
	0b01001001,
	0b00110110,
	0b00000000,
	// C offset = 183
	0b00111110,
	0b01000001,
	0b01000001,
	0b01000001,
	0b00100010,
	0b00000000,
	// D offset = 189
	0b01111111,
	0b01000001,
	0b01000001,
	0b00100010,
	0b00011100,
	0b00000000,
	// E offset = 195
	0b01111111,
	0b01001001,
	0b01001001,
	0b01001001,
	0b01000001,
	0b00000000,
	// F offset = 201
	0b01111111,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00000001,
	0b00000000,
	// G offset = 207
	0b00111110,
	0b01000001,
	0b01001001,
	0b01001001,
	0b01111010,
	0b00000000,
	// H offset = 213
	0b01111111,
	0b00001000,
	0b00001000,
	0b00001000,
	0b01111111,
	0b00000000,
	// I offset = 219
	0b01000001,
	0b01111111,
	0b01000001,
	0b00000000,
	// J offset = 223
	0b00100000,
	0b01000000,
	0b01000001,
	0b00111111,
	0b00000001,
	0b00000000,
	// K offset = 229
	0b01111111,
	0b00001000,
	0b00010100,
	0b00100010,
	0b01000001,
	0b00000000,
	// L offset = 235
	0b01111111,
	0b01000000,
	0b01000000,
	0b01000000,
	0b01000000,
	0b00000000,
	// M offset = 241
	0b01111111,
	0b00000010,
	0b00001100,
	0b00000010,
	0b01111111,
	0b00000000,
	// N offset = 247
	0b01111111,
	0b00000100,
	0b00001000,
	0b00010000,
	0b01111111,
	0b00000000,
	// O offset = 253
	0b00111110,
	0b01000001,
	0b01000001,
	0b01000001,
	0b00111110,
	0b00000000,
	// P offset = 259
	0b01111111,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00000110,
	0b00000000,
	// Q offset = 265
	0b00111110,
	0b01000001,
	0b01010001,
	0b00100001,
	0b01011110,
	0b00000000,
	// R offset = 271
	0b01111111,
	0b00001001,
	0b00011001,
	0b00101001,
	0b01000110,
	0b00000000,
	// S offset = 277
	0b01000110,
	0b01001001,
	0b01001001,
	0b01001001,
	0b00110001,
	0b00000000,
	// T offset = 283
	0b00000001,
	0b00000001,
	0b01111111,
	0b00000001,
	0b00000001,
	0b00000000,
	// U offset = 289
	0b00111111,
	0b01000000,
	0b01000000,
	0b01000000,
	0b00111111,
	0b00000000,
	// V offset = 295
	0b00011111,
	0b00100000,
	0b01000000,
	0b00100000,
	0b00011111,
	0b00000000,
	// W offset = 301
	0b00111111,
	0b01000000,
	0b00111000,
	0b01000000,
	0b00111111,
	0b00000000,
	// X offset = 307
	0b01100011,
	0b00010100,
	0b00001000,
	0b00010100,
	0b01100011,
	0b00000000,
	// Y offset = 313
	0b00000111,
	0b00001000,
	0b01110000,
	0b00001000,
	0b00000111,
	0b00000000,
	// Z offset = 319
	0b01100001,
	0b01010001,
	0b01001001,
	0b01000101,
	0b01000011,
	0b00000000,
	// [ offset = 325
	0b01111111,
	0b01000001,
	0b01000001,
	0b00000000,
	// \ offset = 329
	0b00000010,
	0b00000100,
	0b00001000,
	0b00010000,
	0b00100000,
	0b00000000,
	// ] offset = 335
	0b01000001,
	0b01000001,
	0b01111111,
	0b00000000,
	// ^ offset = 339
	0b00000100,
	0b00000010,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00000000,
	// _ offset = 345
	0b01000000,
	0b01000000,
	0b01000000,
	0b01000000,
	0b01000000,
	0b00000000,
	// ` offset = 351
	0b00000001,
	0b00000010,
	0b00000100,
	0b00000000,
	// a offset = 355
	0b00100000,
	0b01010100,
	0b01010100,
	0b01010100,
	0b01111000,
	0b00000000,
	// b offset = 361
	0b01111111,
	0b01001000,
	0b01000100,
	0b01000100,
	0b00111000,
	0b00000000,
	// c offset = 367
	0b00111000,
	0b01000100,
	0b01000100,
	0b01000100,
	0b00101000,
	0b00000000,
	// d offset = 373
	0b00111000,
	0b01000100,
	0b01000100,
	0b01001000,
	0b01111111,
	0b00000000,
	// e offset = 379
	0b00111000,
	0b01010100,
	0b01010100,
	0b01010100,
	0b00011000,
	0b00000000,
	// f offset = 385
	0b00001000,
	0b01111110,
	0b00001001,
	0b00000001,
	0b00000010,
	0b00000000,
	// g offset = 391
	0b00011000,
	0b10100100,
	0b10100100,
	0b10100100,
	0b01111100,
	0b00000000,
	// h offset = 397
	0b01111111,
	0b00001000,
	0b00000100,
	0b00000100,
	0b01111000,
	0b00000000,
	// i offset = 403
	0b01000100,
	0b01111101,
	0b01000000,
	0b00000000,
	// j offset = 407
	0b00100000,
	0b01000000,
	0b01000100,
	0b00111101,
	0b00000000,
	// k offset = 412
	0b01111111,
	0b00010000,
	0b00101000,
	0b01000100,
	0b00000000,
	// l offset = 417
	0b01000001,
	0b01111111,
	0b01000000,
	0b00000000,
	// m offset = 421
	0b01111100,
	0b00000100,
	0b00011000,
	0b00000100,
	0b01111000,
	0b00000000,
	// n offset = 427
	0b01111100,
	0b00001000,
	0b00000100,
	0b00000100,
	0b01111000,
	0b00000000,
	// o offset = 433
	0b00111000,
	0b01000100,
	0b01000100,
	0b01000100,
	0b00111000,
	0b00000000,
	// p offset = 439
	0b11111100,
	0b00100100,
	0b00100100,
	0b00100100,
	0b00011000,
	0b00000000,
	// q offset = 445
	0b00011000,
	0b00100100,
	0b00100100,
	0b00010100,
	0b11111100,
	0b00000000,
	// r offset = 451
	0b01111100,
	0b00001000,
	0b00000100,
	0b00000100,
	0b00001000,
	0b00000000,
	// s offset = 457
	0b01001000,
	0b01010100,
	0b01010100,
	0b01010100,
	0b00100000,
	0b00000000,
	// t offset = 463
	0b00000100,
	0b00111111,
	0b01000100,
	0b01000000,
	0b00100000,
	0b00000000,
	// u offset = 469
	0b00111100,
	0b01000000,
	0b01000000,
	0b00100000,
	0b01111100,
	0b00000000,
	// v offset = 475
	0b00011100,
	0b00100000,
	0b01000000,
	0b00100000,
	0b00011100,
	0b00000000,
	// w offset = 481
	0b00111100,
	0b01000000,
	0b00111000,
	0b01000000,
	0b00111100,
	0b00000000,
	// x offset = 487
	0b01000100,
	0b00101000,
	0b00010000,
	0b00101000,
	0b01000100,
	0b00000000,
	// y offset = 493
	0b00001100,
	0b01010000,
	0b01010000,
	0b01010000,
	0b00111100,
	0b00000000,
	// z offset = 499
	0b01000100,
	0b01100100,
	0b01010100,
	0b01001100,
	0b01000100,
	0b00000000,
	// { offset = 505
	0b00001000,
	0b00110110,
	0b01000001,
	0b00000000,
	// | offset = 509
	0b01111111,
	0b00000000,
	// } offset = 511
	0b01000001,
	0b00110110,
	0b00001000,
	0b00000000,
	// ~ offset = 515
	0b00010000,
	0b00001000,
	0b00001000,
	0b00010000,
	0b00001000,
	0b00000000,
	//  offset = 521
	0b00000110,
	0b00001001,
	0b00001001,
	0b00000110,
	0b00000000,
};

const char font7x5_kern[] PROGMEM = {
	6,2,4,6,6,6,6,3,
	4,4,6,6,3,6,3,6,
	6,6,6,6,6,6,6,6,
	6,6,3,3,5,6,5,6,
	6,6,6,6,6,6,6,6,
	6,4,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,
	6,6,6,4,6,4,6,6,
	4,6,6,6,6,6,6,6,
	6,4,5,5,4,6,6,6,
	6,6,6,6,6,6,6,6,
	6,6,6,4,2,4,6,5,
};

const int font7x5_offset[] = {
	0,  6,  8, 12, 18, 24, 30, 36,
	39, 43, 47, 53, 59, 62, 68, 71,
	77, 83, 89, 95,101,107,113,119,
	125,131,137,140,143,148,154,159,
	165,171,177,183,189,195,201,207,
	213,219,223,229,235,241,247,253,
	259,265,271,277,283,289,295,301,
	307,313,319,325,329,335,339,345,
	351,355,361,367,373,379,385,391,
	397,403,407,412,417,421,427,433,
	439,445,451,457,463,469,475,481,
	487,493,499,505,509,511,515,521,
};

MAX7219Control lc = MAX7219Control(SPI_MOSI, SPI_CLK, SPI_CS, numDevices);

void InitMax7219() 
{
	for (int x = 0; x<numDevices; x++)
	{
		lc.shutdown(x, false);       //The MAX72XX is in power-saving mode on startup
		lc.setIntensity(x, 0);       // Set the brightness to minimum value
		lc.clearDisplay(x);         // and clear the display
	}
}

const int ColumnBufferLen = 512;
unsigned char ColumnBuffer[ColumnBufferLen];
int LoadPos = 0;

void ResetColumnBuffer()
{
	for (int col = 0; col < ColumnBufferLen; col++)
	{
		ColumnBuffer[col] = 0;
	}
	LoadPos = 0;
}

char LoadColumnBuffer(char ascii)
{
	char kern = 0;
	if (ascii >= 0x20 && ascii <= 0x7f)
	{
		kern = pgm_read_byte_near(font7x5_kern + (ascii - 0x20));
#if defined(ESP8266)
		int offset = font7x5_offset[ascii - 0x20];
#else
		int offset = pgm_read_word_near(font7x5_offset + (ascii-0x20));
#endif

#if defined(ESP8266)
		if ((LoadPos + kern) > ColumnBufferLen) kern = ColumnBufferLen - LoadPos;
		memcpy_P(&ColumnBuffer[LoadPos], font7x5 + offset, kern);
#else
		for (int i = 0; i < kern; i++)
		{
			if (LoadPos >= ColumnBufferLen) return i;
			ColumnBuffer[LoadPos++] = pgm_read_byte_near(font7x5 + offset);
			offset++;
		}
#endif
		LoadPos += kern;
	}
	return kern;
}

int ReloadMessage(int Pos, const char *message) 
{
	LoadPos = Pos;
	for (int counter = 0; ; counter++)
	{
		// read back a char 
		unsigned char myChar = message[counter];
		if (myChar != 0)
		{
			LoadColumnBuffer(myChar);
		}
		else break;
	}
	return LoadPos;
}

int LoadMessage(const char *message)
{
	ResetColumnBuffer();
	for (int counter = 0; ; counter++)
	{
		// read back a char 
		unsigned char myChar = message[counter];
		if (myChar != 0)
		{
			LoadColumnBuffer(myChar);
		}
		else break;
	}
	return LoadPos;
}

int ScrollPos;

void ResetScrollPos(void)
{
	ScrollPos = 0;
}

int LoadDisplayBuffer(int BufferLen)
{
	unsigned char mask = 0x01;

	for (int row = 0; row < 8; row++)
	{
		unsigned char RowBuffer[numDevices];
		int Pos = ScrollPos;
		unsigned char dat = 0;

		for (int device = numDevices - 1; device >= 0; device--)
		{
			unsigned char dat = 0;
			for (int col = 0; col < 8; col++)
			{
				dat <<= 1;
				if (Pos >= BufferLen) Pos = 0;
				if (mask & ColumnBuffer[Pos++])
				{
					dat += 1;
				}
				RowBuffer[device] = dat;
			}
		}
		mask <<= 1;
		lc.setRow(row, RowBuffer);
	}
	ScrollPos++;
	if (ScrollPos >= BufferLen) ScrollPos = 0;

	return ScrollPos;
}

