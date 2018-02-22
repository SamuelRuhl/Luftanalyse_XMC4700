/**
  ******************************************************************************
  * C Library for the BME280 Sensor
  ******************************************************************************
  * @author  Samuel Ruhl, Alexander Meier
  * @date	 2017-04-04
  * @file    BME280.c
  * @brief   Contains Functions for using the SPI
  ******************************************************************************
**/

#include <Dave.h>
#include "spi.h"
#include "xmc4700.h"


/*** SEND **************************************************************************/
uint8_t SPI_send(uint8_t data)
{    
	/*
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
    SPI_I2S_SendData(SPI1, data);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    
    return SPI_I2S_ReceiveData(SPI1);
    */
    //uint8_t rx_dat;

    SPI_MASTER_Transmit(&SPI_MASTER_0,&data,1);

    return 0;

}

/*** REC ***************************************************************************/
uint8_t SPI_rec(void)
{
	/*
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
    SPI_I2S_SendData(SPI1, address);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    SPI_I2S_ReceiveData(SPI1);
     
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
    SPI_I2S_SendData(SPI1, 0x00);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
     
    return  SPI_I2S_ReceiveData(SPI1);

    */
    uint8_t data;
    SPI_MASTER_Receive(&SPI_MASTER_0,&data,1);
    return data;

}


/*** FT800 SPI select **************************************************************/
void FT_spi_select(void)
{
	PORT3->OUT &= ~(1<<12);
}

/*** FT800 SPI deselect ************************************************************/
void FT_spi_deselect(void)
{
	for(int i = 0; i < 100; i++);
	PORT3->OUT |= 1<<12;
}




void ms_delay(uint32_t millisec){
	//Device running on 144MHz
	millisec *= 31100;
	while(millisec--){
		__NOP();		//No Operation
	}
}



void us_delay(uint32_t microsec){
	//Device running on 144MHz
	microsec *= 31;
	while(microsec--){
		__NOP();		//No Operation
	}
}

