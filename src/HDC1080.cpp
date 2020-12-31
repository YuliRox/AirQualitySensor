#include <Wire.h>

#include "HDC1080.h"

HDC1080::HDC1080(uint8_t address)
{
	_address = address;
	temperatureRaw = 0;
	humidityRaw = 0;
}

void HDC1080::setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature)
{
	HDC1080_Registers reg;
	reg.HumidityMeasurementResolution = 0;
	reg.TemperatureMeasurementResolution = 0;
	reg.ModeOfAcquisition = 0x01;

	if (temperature == HDC1080_RESOLUTION_11BIT)
		reg.TemperatureMeasurementResolution = 0x01;

	switch (humidity)
	{
	case HDC1080_RESOLUTION_8BIT:
		reg.HumidityMeasurementResolution = 0x02;
		break;
	case HDC1080_RESOLUTION_11BIT:
		reg.HumidityMeasurementResolution = 0x01;
		break;
	default:
		break;
	}

	writeRegister(reg);

	/*Wire.beginTransmission(_address);
	Wire.write(0x02);
	Wire.write(0x10);
	Wire.endTransmission();*/
}

HDC1080_SerialNumber HDC1080::readSerialNumber()
{
	HDC1080_SerialNumber sernum;
	sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
	sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
	sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
	return sernum;
}

HDC1080_Registers HDC1080::readRegister()
{
	HDC1080_Registers reg;
	reg.rawData = (readData(HDC1080_CONFIGURATION) >> 8);
	return reg;
}

void HDC1080::writeRegister(HDC1080_Registers reg)
{
	Wire.beginTransmission(_address);
	Wire.write(HDC1080_CONFIGURATION);
	Wire.write(reg.rawData);
	Wire.write(0x00);
	Wire.endTransmission();
	delay(10);
}

void HDC1080::heatUp(uint8_t seconds)
{
	HDC1080_Registers reg = readRegister();
	reg.Heater = 1;
	reg.ModeOfAcquisition = 1;
	writeRegister(reg);

	uint8_t buf[4];
	for (int i = 1; i < (seconds * 66); i++)
	{
		Wire.beginTransmission(_address);
		Wire.write(0x00);
		Wire.endTransmission();
		delay(20);
		Wire.requestFrom(_address, (uint8_t)4);
		Wire.readBytes(buf, (size_t)4);
	}
	reg.Heater = 0;
	reg.ModeOfAcquisition = 0;
	writeRegister(reg);
}

void HDC1080::readTemperature()
{
	temperatureRaw = readData(HDC1080_TEMPERATURE);
}

void HDC1080::readHumidity()
{
	humidityRaw = readData(HDC1080_HUMIDITY);
}

double HDC1080::getTemperature()
{
	return ((double)temperatureRaw) * 165.0 / 65536.0 - 40.0;
}

uint8_t HDC1080::getTemperatureAsCCS() {
	//TODO
	//Byte 2                 Byte 3
	//Temperature High Byte Temperature Low Byte
	// 7  6  5 4 3 2 1   0   7   6    5    4    3     2     1     0
	//64 32 16 8 4 2 1 1/2 1/4 1/8 1/16 1/32 1/64 1/128 1/256 1/512
	//Temperature 25°C   Temperature 25°C Fraction
	//Temperature is stored as an unsigned 16 bits integer in 1/512
	//degrees; there is an offset: 0 maps to -25°C. The default value is
	//25°C = 0x64, 0x00. As an example 23.5% temperature would be
	//0x61, 0x00.
	//The internal algorithm uses ENV_DATA values (or default values
	//if not set by the application) to compensate for changes in
	//relative humidity and ambient temperature.
	//For temperatures below-25°C the 7-bit temperature field in
	//Byte 2 above should be set to all zeros.
	return temperatureRaw;
}

double HDC1080::getHumidity()
{
	return ((double)humidityRaw) * 100 / 65536;
}

uint8_t HDC1080::getHumidityAsCCS() {
	//TODO
	//Byte 0 Byte 1
	//Humidity High Byte     Humidity Low Byte
	// 7  6  5 4 3 2 1   0   7   6    5    4    3     2     1     0
	//64 32 16 8 4 2 1 1/2 1/4 1/8 1/16 1/32 1/64 1/128 1/256 1/512
	//Humidity %         Humidity % Fraction
	//Humidity is stored as an unsigned 16 bits in 1/512%RH. The
	//default value is 50% = 0x64, 0x00. As an example 48.5%
	//humidity would be 0x61, 0x00.
	return humidityRaw;
}

void HDC1080::readTempHumid()
{
	Wire.beginTransmission(_address);
	Wire.write(0x00); // init meassure
	Wire.endTransmission();
	delay(15); // wait for meassurement
	Wire.requestFrom(_address, 4); // read meassurement
	temperatureRaw = temperatureRaw << 8 | Wire.read();
	temperatureRaw = temperatureRaw << 8 | Wire.read();
	humidityRaw = humidityRaw << 8 | Wire.read();
	humidityRaw = humidityRaw << 8 | Wire.read();
}

uint16_t HDC1080::readManufacturerId()
{
	return readData(HDC1080_MANUFACTURER_ID);
}

uint16_t HDC1080::readDeviceId()
{
	return readData(HDC1080_DEVICE_ID);
}

uint16_t HDC1080::readData(uint8_t pointer)
{
	Wire.beginTransmission(_address);
	Wire.write(pointer);
	Wire.endTransmission();

	delay(9);
	Wire.requestFrom(_address, (uint8_t)2);

	byte msb = Wire.read();
	byte lsb = Wire.read();

	return msb << 8 | lsb;
}