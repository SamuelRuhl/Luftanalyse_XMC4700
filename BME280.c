/**
  ******************************************************************************
  * C Library for the BME280 Sensor
  ******************************************************************************
  * @author  Samuel Ruhl, Alexander Meier
  * @date	 2017-04-04
  * @file    BME280.c
  * @brief   Contains Functions for using the BME280
  ******************************************************************************
**/

#include "BME280def.h"


int32_t t_fine;
bme280_calib_data bme280_calib;

uint8_t rx_buff[4];
uint8_t xx = 0;

void BME280_select(void){
	PORT0->OUT &= ~(1<<3);
}

void BME280_deselect(void){
	for(int i = 0; i< 100;i++);
	PORT0->OUT |= 1<<3;
}

void spi_send8(uint8_t *data){
	SPI_MASTER_Transmit(&SPI_MASTER_1, &data, 1 );
	while(SPI_MASTER_1.runtime->tx_busy);
}

void spi_send16(uint8_t *data){
	SPI_MASTER_Transmit(&SPI_MASTER_1, &data, 2 );
	while(SPI_MASTER_1.runtime->tx_busy);
}

void spi_send24(uint8_t *data){
	SPI_MASTER_Transmit(&SPI_MASTER_1, &data, 3 );
	while(SPI_MASTER_1.runtime->tx_busy);
}

void spi_tx8_bme280(uint8_t addr, uint8_t data){
	//See Datasheet BME280 p.33		MSB of address is "0"
	addr &= 0x7F;
	uint8_t tx[2] = {addr, data};

	BME280_select();

	SPI_MASTER_Transmit(&SPI_MASTER_1, &tx[0], 2 );
	while(SPI_MASTER_1.runtime->tx_busy);

	BME280_deselect();
}

uint8_t spi_recive8(){
	uint8_t data = 0;
	SPI_MASTER_Receive(&SPI_MASTER_1, &rx_buff[0], 1 );
	while(SPI_MASTER_1.runtime->rx_busy);
	return rx_buff[0];
}

uint8_t spi_recive16(){
	uint8_t data = 0;
	SPI_MASTER_Receive(&SPI_MASTER_1, &rx_buff[0], 2 );
	while(SPI_MASTER_1.runtime->rx_busy);
	return (uint16_t)(rx_buff[0]<<8)|(rx_buff[1]);
}

uint32_t spi_recive24(){
	uint8_t data = 0;
	SPI_MASTER_Receive(&SPI_MASTER_1, &rx_buff[0], 3 );
	while(SPI_MASTER_1.runtime->rx_busy);
	return (uint32_t)(rx_buff[0]<<16)|(rx_buff[1]<<8)|(rx_buff[2]);
}

uint8_t spi_rx8_bme280(uint8_t addr){
	//See Datasheet BME280 p.33		MSB of address is "1"
	addr |= 0x80;
	uint8_t rx_data;
	BME280_select();	//CS Low active

	spi_send8(addr);
	rx_data = spi_recive8();

	BME280_deselect();
	return rx_data;
}

uint16_t spi_rx16_bme280(uint8_t addr){
	//See Datasheet BME280 p.33		MSB of address is "1"
	addr |= 0x80;
	//uint8_t rx_data[2];
	BME280_select();		//CS Low active
	spi_send8(addr);
	//rx_data[0] = spi_recive8();
	//rx_data[1] = spi_recive8();
	uint16_t rdata = spi_recive16();

	BME280_deselect();
	//return (uint16_t)((rx_data[0]<<8)|(rx_data[1])) ;
	return rdata;
}

uint32_t spi_rx24_bme280(uint8_t addr){
	//See Datasheet BME280 p.33		MSB of address is "1"
	addr |= 0x80;

	BME280_select();		//CS Low active

	spi_send8(addr);
	uint32_t rdata = spi_recive24();

	BME280_deselect();

	return rdata;
}

uint16_t spi_rx16_bme280_LE(uint8_t addr){
	uint16_t tmp = spi_rx16_bme280(addr);
	return (tmp >> 8) | (tmp << 8);
}

int16_t spi_rxS16_bme280(uint8_t addr){
	return (int16_t) spi_rx16_bme280(addr);
}

int16_t spi_rxS16_bme280_LE(uint8_t addr){
	return (int16_t) spi_rx16_bme280_LE(addr);
}


void BME280_readCoefficients(void)
{
    bme280_calib.dig_T1 = spi_rx16_bme280_LE(BME280_REGISTER_DIG_T1);
    bme280_calib.dig_T2 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_T2);
    bme280_calib.dig_T3 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_T3);

    bme280_calib.dig_P1 = spi_rx16_bme280_LE(BME280_REGISTER_DIG_P1);
    bme280_calib.dig_P2 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P2);
    bme280_calib.dig_P3 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P3);
    bme280_calib.dig_P4 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P4);
    bme280_calib.dig_P5 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P5);
    bme280_calib.dig_P6 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P6);
    bme280_calib.dig_P7 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P7);
    bme280_calib.dig_P8 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P8);
    bme280_calib.dig_P9 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_P9);

    bme280_calib.dig_H1 = spi_rx8_bme280(BME280_REGISTER_DIG_H1);
    bme280_calib.dig_H2 = spi_rxS16_bme280_LE(BME280_REGISTER_DIG_H2);
    bme280_calib.dig_H3 = spi_rx8_bme280(BME280_REGISTER_DIG_H3);
    bme280_calib.dig_H4 = (spi_rx8_bme280(BME280_REGISTER_DIG_H4) << 4) | (spi_rx8_bme280(BME280_REGISTER_DIG_H4+1) & 0xF);
    bme280_calib.dig_H5 = (spi_rx8_bme280(BME280_REGISTER_DIG_H5+1) << 4) | (spi_rx8_bme280(BME280_REGISTER_DIG_H5) >> 4);
    bme280_calib.dig_H6 = (int8_t)spi_rx8_bme280(BME280_REGISTER_DIG_H6);
}


void BME280_init(void){
	spi_tx8_bme280(0xF2,0x01) ;	//OSR for humidity
	spi_tx8_bme280(0xF4,0xFF) ; //MAX oversampling and normal Mode
	spi_tx8_bme280(0xF5,0xA0) ;	//Config: 1s, IIR off, SPI 3 wire disabled
}


/**************************************************************************/
/*!
    @brief  Formulas as described in Datasheet BME280 p.23 and p24
*/
/**************************************************************************/

float BME280_readTemperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = spi_rx24_bme280(BME280_REGISTER_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return 0;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)bme280_calib.dig_T1 <<1))) *
            ((int32_t)bme280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T>>4) - ((int32_t)bme280_calib.dig_T1)) *
              ((adc_T>>4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
            ((int32_t)bme280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}


float BME280_readPressure(void) {
    int64_t var1, var2, p;

    //readTemperature(); // must be done first to get t_fine

    int32_t adc_P = spi_rx24_bme280(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return 0;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)bme280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)bme280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bme280_calib.dig_P3)>>8) +
           ((var1 * (int64_t)bme280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib.dig_P7)<<4);
    return (float)p/256;
}



float  BME280_readHumidity(void) {
    //readTemperature(); // must be done first to get t_fine

    int32_t adc_H = spi_rx24_bme280(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return 0;

    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib.dig_H4) << 20) -
                    (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)bme280_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0;
}



