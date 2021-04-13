/*
 * bmp280.c
 *
 *  Created on: Mar 20, 2021
 *      Author: QB
 */

#include "main.h"
#include "bmp280.h"

// funkcja odczytująca rejestry 8-bitowe
uint8_t Read8(BMP280_t *bmp, uint8_t Register)
{
	uint8_t Value;

	// odczyt przez I2C - z bmp o adresie Address (przesunietym o 1 bit), z rejestru o adresie Register i rozmiarze 1 bajt
	// do zmiennej Value wczytuje 1 bajt danych, z Timeoutem (czasem dozwolonym na odpowiedz)
	HAL_I2C_Mem_Read(bmp->bmp_i2c, (bmp->Address << 1), Register, 1, &Value, 1, BMP280_I2C_TIMEOUT);

	return Value;
}

// funkcja wpisujaca do rejestru 8-bitowego
void Write8(BMP280_t *bmp, uint8_t Register, uint8_t Value)
{
	// zapis przez I2C - do bmp o adresie Address (przesunietym o 1 bit), do rejestru o adresie Register i rozmiarze 1 bajt
	// wpisuje ze zmiennej Value 1 bajt danych, z Timeoutem (czasem dozwolonym na odpowiedz)
	HAL_I2C_Mem_Write(bmp->bmp_i2c, (bmp->Address << 1), Register, 1, &Value, 1, BMP280_I2C_TIMEOUT);
}

// funkcja odczytująca rejestry 16-bitowe
uint16_t Read16(BMP280_t *bmp, uint8_t Register)
{
	uint8_t Value [2]; // dwie dane 8-bitowe

	// odczyt przez I2C - z bmp o adresie Address (przesunietym o 1 bit), z rejestru o adresie Register i rozmiarze 1 bajt
	// do zmiennej Value wczytuje 2 bajty danych, z Timeoutem (czasem dozwolonym na odpowiedz)
	HAL_I2C_Mem_Read(bmp->bmp_i2c, (bmp->Address << 1), Register, 1, Value, 2, BMP280_I2C_TIMEOUT);

	return (Value[1] << 8) | (Value[0]); // zwraca odczytane dane z dwoch rejestrow
}

// funkcja odczytująca rejestry 24-bitowe
uint32_t Read24(BMP280_t *bmp, uint8_t Register)
{
	uint8_t Value [3]; // trzy dane 8-bitowe

	// odczyt przez I2C - z bmp o adresie Address (przesunietym o 1 bit), z rejestru o adresie Register i rozmiarze 1 bajt
	// do zmiennej Value wczytuje 3 bajty danych, z Timeoutem (czasem dozwolonym na odpowiedz)
	HAL_I2C_Mem_Read(bmp->bmp_i2c, (bmp->Address << 1), Register, 1, Value, 3, BMP280_I2C_TIMEOUT);

	return ((Value[0] << 16) | (Value[1] << 8) | Value[2]); // zwraca odczytane dane z trzech rejestrow
}

// funkcja ustawiajaca tryb - w rejestrze; wczytuje rejestr, zmienia zawartosc i wpisuje ja do rejestru
void BMP280_SetMode(BMP280_t *bmp, uint8_t Mode)
{
	uint8_t Tmp;

	if (Mode > 3) Mode = 3; // moze byc 0-3; jesli dziwna wartosc, to ustawia 3 - Normal

	Tmp = Read8(bmp, BMP280_CONTROL); // odczytuje rejestr Control

	Tmp = Tmp & 0xFC; // maska - czysci 2 bity LSB	 	1111 1100
	Tmp |= Mode & 0x03; // wpisuje Mode na LSB (dobrze, zeby Mode byl 2-bitowy; dla pewności maskowane 2 LSB)

	Write8(bmp, BMP280_CONTROL, Tmp); // wpisuje slowo 8-bitowe do rejestru Control
}

// funkcja ustawiajaca nadprobkowanie (dokladnosc) pomiaru cisnienia
void BMP280_SetPressureOversampling(BMP280_t *bmp, uint8_t POversampling)
{
	uint8_t Tmp;

	if (POversampling > 5) POversampling = 5; // moze byc 0-5; jesli dziwna wartosc, to ustawia 5 - najwiekszy

	Tmp = Read8(bmp, BMP280_CONTROL); // odczytuje rejestr Control

	Tmp = Tmp & 0xE3; // maska - zeruje 3 bity		1110 0011
	Tmp |= (POversampling << 2 ) & 0x1C; // wpisuje 3 bity odpowiedzialne za Pressure Oversampling

	Write8(bmp, BMP280_CONTROL, Tmp); // wpisuje slowo 8-bitowe do rejestru Control
}

// funkcja ustawiajaca nadprobkowanie (dokladnosc) pomiaru temperatury
void BMP280_SetTemperatureOversampling(BMP280_t *bmp, uint8_t TOversampling)
{
	uint8_t Tmp;

	if (TOversampling > 5) TOversampling = 5; // moze byc 0-5; jesli dziwna wartosc, to ustawia 5 - najwiekszy

	Tmp = Read8(bmp, BMP280_CONTROL); // odczytuje rejestr Control

	Tmp = Tmp & 0x1F; // maska - zeruje 3 bity		0001 1111
	Tmp |= (TOversampling << 5 ) & 0xE0; // wpisuje 3 bity odpowiedzialne za Temperature Oversampling

	Write8(bmp, BMP280_CONTROL, Tmp); // wpisuje slowo 8-bitowe do rejestru Control
}

// funkcja odczytujaca "surowa" temperature (przesunieta juz o 4 bity)
int32_t BMP280_ReadTemperatureRaw(BMP280_t *bmp)
{
	uint32_t Tmp;

	Tmp = (int32_t)Read24(bmp, BMP280_TEMPDATA);
	Tmp >>= 4; // przesuniecie w prawo o 4 bity

	return Tmp; // zwraca "surowa" nieprzeliczona wartosc
}

// funkcja odczytujaca "surowe" cisnienie (przesunieta juz o 4 bity)
int32_t BMP280_ReadPressureRaw(BMP280_t *bmp)
{
	uint32_t Tmp;

	Tmp = (int32_t)Read24(bmp, BMP280_PRESSUREDATA);
	Tmp >>= 4; // przesuniecie w prawo o 4 bity

	return Tmp; // zwraca "surowa" nieprzeliczona wartosc
}

// funkcja zwracajaca temperature
float BMP280_ReadTemperature(BMP280_t *bmp)
{
	// zmienne pomocnicze
	int32_t var1, var2, T;
	int32_t adc_T = BMP280_ReadTemperatureRaw(bmp);

	var1 = ((((adc_T>>3) - ((int32_t)(bmp->t1)<<1))) * ((int32_t)(bmp->t2))) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)(bmp->t1))) * ((adc_T>>4) - ((int32_t)(bmp->t1)))) >> 12) *
			((int32_t)(bmp->t3))) >> 14;
	bmp->t_fine = var1 + var2;
	T = ((bmp->t_fine) * 5 + 128) >> 8;

	// algorytm z dokumentacji zwraca liczbe calkowita, ktora trzeba podzielic przez 100, aby dostac *C
	return (float)(T/100.0); // zwraca przeliczona temperature
}

// funkcja zwracajaca cisnienie
uint8_t BMP280_ReadPressureandTemperature(BMP280_t *bmp, float *Pressure, float *Temperature)
{
	*Temperature = BMP280_ReadTemperature(bmp);

	int32_t var1, var2;
	uint32_t p;

	var1 = (((int32_t)(bmp->t_fine))>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)(bmp->p6));
	var2 = var2 + ((var1*((int32_t)(bmp->p5)))<<1);
	var2 = (var2>>2)+(((int32_t)(bmp->p4))<<16);
	var1 = ((((bmp->p3) * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)(bmp->p2)) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)(bmp->p1)))>>15);

	if (var1 == 0)
	{
		return 1; // avoid exception caused by division by zero
	}

	int32_t adc_P = BMP280_ReadPressureRaw(bmp);

	p = (((int32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)(bmp->p9)) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)(bmp->p8)))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + (bmp->p7)) >> 4));

	*Pressure = (float)(p/100.0);

	return 0;
}

// funkcja inicjujaca strukture czujnika; argumenty: adres struktury, uchwyt I2C, adres I2C czujnika
uint8_t BMP280_Init(BMP280_t *bmp, I2C_HandleTypeDef *bmp_i2c, uint8_t Address)
{
	bmp->bmp_i2c = bmp_i2c;
	bmp->Address = Address;

	uint8_t ChipID = Read8(bmp, BMP280_CHIPID); // odczytuje ChipID - identyfikator urządzenia

	if (ChipID != 0x58)
	{
		return 1; // zwraca blad, gdy obslugiwane urzadzenie to nie BMP280 o ChipID = 0x58
	}

	// odczyt danych kalibracyjnych
	bmp->t1 = Read16(bmp, BMP280_DIG_T1); // czyta rejestr T1 - 0x88 i zinkremetowany -  0x89
	bmp->t2 = Read16(bmp, BMP280_DIG_T2);
	bmp->t3 = Read16(bmp, BMP280_DIG_T3);
	bmp->p1 = Read16(bmp, BMP280_DIG_P1); // czyta rejestr P1
	bmp->p2 = Read16(bmp, BMP280_DIG_P2);
	bmp->p3 = Read16(bmp, BMP280_DIG_P3);
	bmp->p4 = Read16(bmp, BMP280_DIG_P4);
	bmp->p5 = Read16(bmp, BMP280_DIG_P5);
	bmp->p6 = Read16(bmp, BMP280_DIG_P6);
	bmp->p7 = Read16(bmp, BMP280_DIG_P7);
	bmp->p8 = Read16(bmp, BMP280_DIG_P8);
	bmp->p9 = Read16(bmp, BMP280_DIG_P9);

	// wpis do mode z rejestru ctrl_meas
	BMP280_SetTemperatureOversampling(bmp, BMP280_TEMPERATURE_20BIT); // ustawia najwieksza dokladnosc temperatury
	BMP280_SetPressureOversampling(bmp, BMP280_ULTRAHIGHRES); // ustawia najwieksza dokladnosc cisnienia
	BMP280_SetMode(bmp, BMP280_NORMALMODE); // ustawia tryb Normal

	return 0; // dobrze jest
}
