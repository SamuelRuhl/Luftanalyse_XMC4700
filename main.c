/**
  ******************************************************************************
  * Main
  ******************************************************************************
  * @author  Samuel Ruhl, Alexander Meier
  * @date	 2017-04-04
  * @file    main.c
  * @brief   Initializing of the XMC4700 components and displaying Things on the LCD
  ******************************************************************************
**/


#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#include "spi.h"
#include "ft800.h"
#include "BME280def.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


uint8_t start = 0;
uint32_t temp = 0;
char buf[50];
char buf2[50];
char buf3[50];
char buf4[50];
char buf5[50];
char buf6[50];

float temprature, press, hum;

uint32_t result_CO2;
uint32_t result_CO;
uint32_t result_CH4;

int interrupt_sign ;

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

void sysDms(uint32_t millisec)
{
    for(int i = OSCHP_GetFrequency() * millisec /1000; i > 0 ;i--);
}



/* Init function for an 5" LCD display */
uint8_t initFT800(void){
	uint8_t dev_id = 0;                  // Variable for holding the read device id

	ms_delay(50);
	PORT3->OUT &= ~(1<<13);				// Set the PDN pin low
	ms_delay(50);                       // Delay 50 ms for stability
	PORT3->OUT |= (1<<13);				// Set the PDN pin high
	ms_delay(50);                       // Delay 50 ms for stability

	//WAKE
	HOST_CMD_ACTIVE();
	ms_delay(500);

	//Ext Clock
	HOST_CMD_WRITE(CMD_CLKEXT);         // Send CLK_EXT Command (0x44)// Set FT800 for external clock
	ms_delay(5);						//give some time to process

	//PLL (48M) Clock
	HOST_CMD_WRITE(CMD_CLK48M);         // Send CLK_48M Command (0x62)// Set FT800 for 48MHz PLL
	ms_delay(5);		//give some time to process


	ms_delay(5);

	HOST_CMD_WRITE(0x40);
	ms_delay(5);

	// Now FT800 can accept commands at up to 30MHz clock on SPI bus

	//Read Dev ID
	dev_id = HOST_MEM_RD8(REG_ID);      // Read device id		//REG_ID is read only and always 0x7C

	//while(HOST_MEM_RD8(REG_ID) != 0x7C);
	if(dev_id != 0x7C)                  // Device ID should always be 0x7C
	{
		return 1;
	}

	HOST_MEM_WR8(REG_GPIO, 0x00);			// Set REG_GPIO to 0 to turn off the LCD DISP signal
	HOST_MEM_WR8(REG_PCLK, 0x00);      		// Pixel Clock Output disable

	// End of Wake-up FT800

	//***************************************
	// Initialize Display

	HOST_MEM_WR16(REG_HCYCLE, 548);         // Set H_Cycle to 548
	HOST_MEM_WR16(REG_HOFFSET, 43);         // Set H_Offset to 43
	HOST_MEM_WR16(REG_HSYNC0, 0);           // Set H_SYNC_0 to 0
	HOST_MEM_WR16(REG_HSYNC1, 41);          // Set H_SYNC_1 to 41
	HOST_MEM_WR16(REG_VCYCLE, 292);         // Set V_Cycle to 292
	HOST_MEM_WR16(REG_VOFFSET, 12);         // Set V_OFFSET to 12
	HOST_MEM_WR16(REG_VSYNC0, 0);           // Set V_SYNC_0 to 0
	HOST_MEM_WR16(REG_VSYNC1, 10);          // Set V_SYNC_1 to 10
	HOST_MEM_WR8(REG_SWIZZLE, 0);           // Set SWIZZLE to 0
	HOST_MEM_WR8(REG_PCLK_POL, 1);          // Set PCLK_POL to 1
	HOST_MEM_WR8(REG_CSPREAD, 1);           // Set CSPREAD to 1
	HOST_MEM_WR16(REG_HSIZE, 480);          // Set H_SIZE to 480
	HOST_MEM_WR16(REG_VSIZE, 272);          // Set V_SIZE to 272

	/* configure touch & audio */
	HOST_MEM_WR8(REG_TOUCH_MODE, 0x03);     	//set touch on: continous
	HOST_MEM_WR8(REG_TOUCH_ADC_MODE, 0x01); 	//set touch mode: differential
	HOST_MEM_WR8(REG_TOUCH_OVERSAMPLE, 0x0F); 	//set touch oversampling to max
	HOST_MEM_WR16(REG_TOUCH_RZTHRESH, 5000);	//set touch resistance threshold
	HOST_MEM_WR8(REG_VOL_SOUND, 0xFF);      	//set the volume to maximum

	/* write first display list */
	HOST_MEM_WR32(RAM_DL+0, CLEAR_COLOR_RGB(0,0,0));  // Set Initial Color to BLACK
	HOST_MEM_WR32(RAM_DL+4, CLEAR(1,1,1));            // Clear to the Initial Color
	HOST_MEM_WR32(RAM_DL+8, DISPLAY());               // End Display List

	HOST_MEM_WR8(REG_DLSWAP, DLSWAP_FRAME);           // Make this display list active on the next frame

	HOST_MEM_WR8(REG_GPIO_DIR, 0x80);                 // Set Disp GPIO Direction
	HOST_MEM_WR8(REG_GPIO, 0x80);                     // Enable Disp (if used)
	HOST_MEM_WR16(REG_PWM_HZ, 0x00FA);                // Backlight PWM frequency
	HOST_MEM_WR8(REG_PWM_DUTY, 0x80);                 // Backlight PWM duty

	HOST_MEM_WR8(REG_PCLK, 0x05);                     // After this display is visible on the LCD

	return 0;
}

/* Clear Screen */
void clrscr(void)
{
	cmd(CMD_DLSTART);
	cmd(CLEAR_COLOR_RGB(0,0,0));
	cmd(CLEAR(1,1,1));
	cmd(DISPLAY());
	cmd(CMD_SWAP);
}



void screen_var(){

	//clrscr();
	cmd(CMD_DLSTART);
	cmd(CLEAR_COLOR_RGB(0,0,0));
	cmd(CLEAR(1,1,1));
	cmd_gradient(0,0,0xA1E1FF, 0,250,0x000080);
	start = 1;
	cmd(COLOR_RGB(0x00,0x00,0x00));
	cmd_text(240,35, 30,OPT_CENTERX, "Luftanalysesystem");


	cmd(COLOR_RGB(255,255,255));

	int vor = (int) temprature  ;
	int nach = (int)(temprature * 100) % 100;
	int vor2 =  press / 100;
	int nach2 = (int) press  % 100 ;

	int vor3 = (int) hum  ;
	int nach3 = (int)(hum * 100) % 100;

	sprintf(buf, "%d,%d Grad C", vor, nach);
	sprintf(buf2, "%d,%d hPa", vor2, nach2);
	sprintf(buf3, "%d,%d %%", vor3, nach3);

	cmd_text(10,200, 27,0, "Temperature");
	cmd_text(120,200, 27,0, buf);
	cmd_text(10,220, 27,0, "Luftdruck");
	cmd_text(120,220, 27,0, buf2);
	cmd_text(10,240, 27,0, "Luftfeuchte");
	cmd_text(120,240, 27,0, buf3);


	sprintf(buf4, "%d", result_CO2);
	sprintf(buf5, "%d", result_CO);
	sprintf(buf6, "%d", result_CH4);

	cmd_text(10,100, 27,0, "CO2");
	cmd_text(10,120, 27,0, "CO");
	cmd_text(10,140, 27,0, "CH4");
	cmd_text(120,100, 27,0, buf4);
	cmd_text(120,120, 27,0, buf5);
	cmd_text(120,140, 27,0, buf6);

	cmd(DISPLAY());
	cmd(CMD_SWAP);


}

//#############################################################################################################

void val_to_buf(void){
	int vor = (int) temprature ;
	int nach = (int)(temprature * 100) % 100;

	int vor2 =  press / 100;
	int nach2 = (int) press  % 100;

	int vor3 = (int) hum ;
	int nach3 = (int)(hum * 100) % 100;

	sprintf(buf, "%d,%d", vor, nach);
	sprintf(buf2, "%d,%d", vor2, nach2);
	sprintf(buf3, "%d,%d %%", vor3, nach3);

	sprintf(buf4, "%d", result_CO2);
	sprintf(buf5, "%d", result_CO);
	sprintf(buf6, "%d", result_CH4);
}


void luft_warm(void){
	val_to_buf();
	cmd(CMD_DLSTART);
	cmd(CLEAR_COLOR_RGB(0,0,0));
	cmd(CLEAR(1,1,1));
	cmd_gradient(362,134, 0xb7172c, 447,235, 0xf43b16);
	cmd(COLOR_RGB(243,234,249));
	cmd_text(178,15,28, 1536, "Meier - Ruhl'sche Luftanalyse");


	cmd(COLOR_RGB(243,228,238));
	cmd_text(80,72, 31, 1536, buf);
	cmd_text(224,72,31, 1536, "Grad C");



	cmd(COLOR_RGB(247,247,247));
	cmd_text(80,130, 27, 1536, buf2);
	cmd_text(129,130,27, 1536, "hPa");


	cmd(COLOR_RGB(137,7,9));
	cmd(LINE_WIDTH(16));
	cmd(BEGIN(RECTS));
	cmd(VERTEX2F(304,2496));
	cmd(VERTEX2F(4224,4080));
	cmd(END());


	cmd(COLOR_RGB(248,248,248));
	cmd_text(140,178, 28, 1536, buf4);
	cmd_text(49,178,28, 1536, "CO2");


	cmd(COLOR_RGB(248,248,248));
	cmd_text(140,236, 28, 1536, buf6);
	cmd_text(49,236,28, 1536, "CH4");


	cmd(COLOR_RGB(245,245,245));
	cmd_text(140,207, 28, 1536, buf5);
	cmd_text(44,207,28, 1536, "CO");


	cmd(COLOR_RGB(170,18,7));
	cmd(LINE_WIDTH(16));
	cmd(BEGIN(RECTS));
	cmd(VERTEX2F(-96,496));
	cmd(VERTEX2F(6144,512));
	cmd(END());


	cmd(POINT_SIZE(532));
	cmd(cmd(COLOR_RGB(112,15,7)));
	cmd(BEGIN(FTPOINTS));
	cmd(VERTEX2F(4304,4016));
	cmd(END());


	cmd(POINT_SIZE(462));
	cmd(COLOR_RGB(112,33,17));
	cmd(BEGIN(FTPOINTS));
	cmd(VERTEX2F(6608,1392));
	cmd(END());


	cmd(LINE_WIDTH(16));
	cmd(COLOR_RGB(112,12,8));
	cmd(BEGIN(LINES));
	cmd(VERTEX2F(6640,1312));
	cmd(VERTEX2F(6640,4352));


	cmd(POINT_SIZE(462));
	cmd(COLOR_RGB(112,33,17));
	cmd(BEGIN(FTPOINTS));
	cmd(VERTEX2F(5520,2768));
	cmd(END());


	cmd(LINE_WIDTH(16));
	cmd(COLOR_RGB(112,12,8));
	cmd(BEGIN(LINES));
	cmd(VERTEX2F(5552,2816));
	cmd(VERTEX2F(5552,5856));


	cmd(POINT_SIZE(462));
	cmd(COLOR_RGB(112,33,17));
	cmd(BEGIN(FTPOINTS));
	cmd(VERTEX2F(7680,336));
	cmd(END());


	cmd(DISPLAY());
	cmd(CMD_SWAP);
}

void luft_kalt(){
	val_to_buf();
	cmd(CMD_DLSTART);
	cmd(CLEAR_COLOR_RGB(0,0,0));
	cmd(CLEAR(1,1,1));

	cmd_gradient(332,141, 0x61cdff, 416,245, 0x002040);


	cmd(COLOR_RGB(5,30,67));
	cmd(LINE_WIDTH(16));
	cmd(BEGIN(RECTS));
	cmd(VERTEX2F(288,2608));
	cmd(VERTEX2F(4208,4192));
	cmd(END());


	cmd(POINT_SIZE(357));
	cmd(COLOR_RGB(34,43,205));
	cmd(BEGIN(FTPOINTS));
	cmd(VERTEX2F(6048,1600));
	cmd(END());


	cmd(COLOR_RGB(146,146,146));
	cmd_text(140,178, 28, 1536, buf4);
	cmd_text(49,178,28, 1536, "CO2");


	cmd(COLOR_RGB(141,141,141));
	cmd_text(140,207, 28, 1536, buf5);
	cmd_text(44,207,28, 1536, "CO");


	cmd(COLOR_RGB(142,142,142));
	cmd_text(140,236, 28, 1536, buf6);
	cmd_text(49,236,28, 1536, "CH4");


	cmd(COLOR_RGB(140,131,137));
	cmd_text(80,72, 31, 1536, buf);
	cmd_text(224,72,31, 1536, "Grad C");


	cmd(COLOR_RGB(134,134,134));
	cmd_text(80,130, 27, 1536, buf2);
	cmd_text(129,130,27, 1536, "hPa");


	cmd(COLOR_RGB(10,52,170));
	cmd(LINE_WIDTH(16));
	cmd(BEGIN(RECTS));
	cmd(VERTEX2F(-96,496));
	cmd(VERTEX2F(6144,512));
	cmd(END());


	cmd(COLOR_RGB(123,118,126));
	cmd_text(154,14,28, 1536, "Meier - Ruhl'sche Luftanalyse");


	cmd(LINE_WIDTH(16));
	cmd(COLOR_RGB(0,56,112));
	cmd(BEGIN(LINES));
	cmd(VERTEX2F(6080,1536));
	cmd(VERTEX2F(7664,16));


	cmd(DISPLAY());
	cmd(CMD_SWAP);
}


int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           /* Initialization of DAVE APPs  */
  PORT3->IOCR12 |= (0x10 << 3) ; //Init the GPIO CSS 3.12 (Displ) Pin as General Purpose Output (see RM p.2789 (chapter 28 table 26.5))
  PORT3->OUT |= 1<<12;

  PORT3->IOCR12 |= (0x10 << 11) ; //Init the GPIO PD_N 3.13 (Displ) Pin as General Purpose Output (see RM p.2789 (chapter 28 table 26.5))
  PORT3->OUT |= 1<<13;

  PORT0->IOCR0 |= (0x10<<27);	//Init the GPIO CSS (BME280) Pin as General Purpose Output (see RM p.2789 (chapter 28 table 26.5))
  PORT0->OUT |= 1<<3;

  BME280_init();
  BME280_readCoefficients();

  while(initFT800());
  sysDms(500);
  ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);	//Start ADC conversation


  clrscr();


  if(status != DAVE_STATUS_SUCCESS)
  {
    /* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  /* Placeholder for user application code. The while loop below can be replaced with user application code. */
  while(1U)
  {
	  temprature = BME280_readTemperature();
	  press = BME280_readPressure();
	  hum = BME280_readHumidity();

	  luft_warm();
	  ms_delay(50);

  }
}


void Adc_Measurement_Handler(void)
{
    /*Read out conversion results*/
	result_CO2   = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_CO2_handle);
	result_CO    = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_CO_handle);
	result_CH4   = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_CH4_handle);

    /*Re-trigger conversion sequence*/
    ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);

    interrupt_sign = 1 ;

}

