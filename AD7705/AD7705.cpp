#include "AD7705.h"
#include "SPI.h"

AD7705::AD7705(int ssPin, double vref) {
    VRef = vref;
    pinSS = ssPin;
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinSPIClock, OUTPUT);
    pinMode(pinSS, OUTPUT);

    digitalWrite(pinSS, HIGH);
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA) | _BV(SPI2X) | _BV(SPR1) | _BV(SPR0);
}

void AD7705::init(byte channel, byte clkDivider, byte polarity, byte gain, byte updRate) {
    setNextOperation(REG_CLOCK, channel, Write);
    writeClockRegister(0, clkDivider, updRate);

    setNextOperation(REG_SETUP, channel, Write);
    writeSetupRegister(MODE_SELF_CAL, gain, polarity, 0, 0);

    while (!dataReady(channel)) {
    };
}

void AD7705::init(byte channel) {
    init(channel, CLK_DIV_1, UNIPOLAR, GAIN_1, UPDATE_RATE_20);
}

void AD7705::reset() {
    digitalWrite(pinSS, LOW);
    for (int i = 0; i < 100; i++)
        SPI.transfer(0xff);
    digitalWrite(pinSS, HIGH);
}

//write communication register for next operation
//   7        6      5      4      3      2      1      0
//0/DRDY(0) RS2(0) RS1(0) RS0(0) R/W(0) STBY(0) CH1(0) CH0(0)

void AD7705::setNextOperation(byte reg, byte channel, byte readWrite) {
    byte r = 0;
    r = reg << 4 | readWrite << 3 | channel;

    digitalWrite(pinSS, LOW);
    SPI.transfer(r);
    digitalWrite(pinSS, HIGH);
}


//Clock Register
//   7      6       5        4        3        2      1      0
//ZERO(0) ZERO(0) ZERO(0) CLKDIS(0) CLKDIV(0) CLK(1) FS1(0) FS0(1)
//
//CLKDIS: master clock disable bit
//CLKDIV: clock divider bit

void AD7705::writeClockRegister(byte CLKDIS, byte CLKDIV, byte outputUpdateRate) {
    byte r = CLKDIS << 4 | CLKDIV << 3 | outputUpdateRate;

    r &= ~(1 << 2); // clear CLK, use 20 25 100 200

    digitalWrite(pinSS, LOW);
    SPI.transfer(r);
    digitalWrite(pinSS, HIGH);
}

//Setup Register
//  7     6     5     4     3      2      1      0
//MD10) MD0(0) G2(0) G1(0) G0(0) B/U(0) BUF(0) FSYNC(1)

void AD7705::writeSetupRegister(byte operationMode, byte gain, byte polarity, byte buffered, byte fsync) {
    byte r = operationMode << 6 | gain << 3 | polarity << 2 | buffered << 1 | fsync;

    digitalWrite(pinSS, LOW);
    SPI.transfer(r);
    digitalWrite(pinSS, HIGH);
}


bool AD7705::dataReady(byte channel) {
    setNextOperation(REG_CMM, channel, Read);

    digitalWrite(pinSS, LOW);
    byte b1 = SPI.transfer(0x0);
    digitalWrite(pinSS, HIGH);

    return (b1 & 0x80) == 0x0;
}


unsigned int AD7705::readADResult() {
    digitalWrite(pinSS, LOW);
    byte b1 = SPI.transfer(0x0);
    byte b2 = SPI.transfer(0x0);
    digitalWrite(pinSS, HIGH);

    unsigned int r = b1 << 8 | b2;

    return r;
}

double AD7705::readValue(byte channel, float refOffset) {
    while (!dataReady(channel)) {
    };
    setNextOperation(REG_DATA, channel, 1);

    return readADResult() * 1.0 / 65536.0 * VRef - refOffset;
}
