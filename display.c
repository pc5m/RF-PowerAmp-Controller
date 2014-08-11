/*
 * display.c
 *
 * Created: 21-4-2014 9:36:46
 *  Author: cmobach
 */ 

#include "display.h"
#include "lcd.h"
#include <stdlib.h>

#define BARGRAPH_COUNT 9 //nr of full bargraph's

volatile static const PROGMEM unsigned char BargraphElements[] =
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F,
	0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1F,
	0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F,
	0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F
};


void display_welcomeMessage()
{
    display_textLine(1,welcomeLine1);
	display_textLine(2,welcomeLine2);
	display_textLine(3,welcomeLine3);
	display_textLine(4,welcomeLine4);
}


void display_InitBargraph()
{
	uint8_t i;
	lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
	for(i=0; i<48; i++)
	{
		lcd_data(pgm_read_byte_near(&BargraphElements[i]));
	}
}


/*************************************************************************
Display float with width and nrOfDigits without auto linefeed, maximum width of 10
Input:    float to be displayed
          Total width of float
		  nrOfDigits of float
Returns:  none
*************************************************************************/
void display_Float( float f, uint8_t width, uint8_t nrOfdigits)
{
	char buffer[10];
	if (width > 10) width = 10;
	lcd_puts(dtostrf(f,width,nrOfdigits,buffer));
}/* lcd_printFloat */


/*************************************************************************
Display formatted string at line position 1 - 4 
Input:   line = line nr (first line is line 1
         preamble = preamble string
		 f_value = float value
		 width = width of float
		 nrOfdigits = nr of digits in float
		 postamble = postamble string
Returns:  none
*************************************************************************/
void display_FormattedLine (uint8_t line, char *preamble,float f_value, uint8_t width, uint8_t nrOfdigits, char *postamble)
{
	char buffer[10];
	lcd_gotoxy(0,--line);
	lcd_puts(preamble);
	lcd_puts(dtostrf(f_value,width,nrOfdigits,buffer));
	lcd_puts(postamble);
}/* lcd_formattedLine */



/*************************************************************************
Display text string at line position 1 - 4 
Input:   line = line nr (first line is line 1
         text = string
Returns:  none
*************************************************************************/
void display_textLine (uint8_t line, char *text)
{
	lcd_gotoxy(0,--line);
	lcd_puts(text);
}/* display_textLine */



/**
 @brief    Display menu
 @param    menu to be displayed                                        
 @return   none
*/
void display_Menu (enum activeMenus menu)
{
	if (menu != activeMenu) lcd_clrscr(); // clear full lcd when switching to new menu, else only refresh data part
	switch (menu)
	{
	case Gen_Menu:
			display_FormattedLine(1,"P  ",power.fwrd,4,0," W ");
			if (power.swr == 999) display_textLine(2,"SWR ----    "); else  display_FormattedLine(2,"SWR 1:",power.swr,1,1,"  ");
			display_FormattedLine(3,"I  ",current.moduleTotal,3,0," A  ");
			display_FormattedLine(4,"T  ",temp_values.tempMax,3,0," C  ");  //should be maximum temperature = todo !
			activeMenu = Gen_Menu;
		break;
	case Pall_menu:
			display_FormattedLine(1,"FWD ",power.fwrd,4,0," W");
			display_FormattedLine(2,"REF ",power.refl,3,0," W ");
			display_FormattedLine(3,"INP ",power.input,4,1," W");
			if (power.swr == 999) display_textLine(4,"SWR ----    "); else  display_FormattedLine(4,"SWR 1:",power.swr,1,1,"  ");
			// display_FormattedLine(4,"SWR 1:",power.swr,1,1,"  ");
			activeMenu = Pall_menu;
		break;
	case Temp_menu:
			display_FormattedLine(1,"T-A ",temp_values.tempA,2,0," C  ");
			display_FormattedLine(2,"T-B ",temp_values.tempB,2,0," C  ");
			display_FormattedLine(3,"T-C ",temp_values.tempC,2,0," C  ");
			display_FormattedLine(4,"T-D ",temp_values.tempD,2,0," C  ");
			activeMenu = Temp_menu;
			break;
	case Imod_menu:
	    	display_FormattedLine(1,"I-A ",current.moduleA,4,1," A");
	    	display_FormattedLine(2,"I-B ",current.moduleB,4,1," A");
			display_FormattedLine(3,"I-C ",current.moduleC,4,1," A");
			display_FormattedLine(4,"I-D ",current.moduleD,4,1," A");
			activeMenu = Imod_menu;
			break;
	}
}



/**
 @brief    Display bar for data on specific menu
 @param    16bit word for line nr (max 1024) , value to be displayed and max value                                      
 @return   none
*/
void display_Bar (uint8_t line, uint16_t val, uint16_t bar_max )
{
	int i,j;
	int valsPerChar = bar_max/BARGRAPH_COUNT;
	int valsPerSubChar = valsPerChar/5;
	lcd_gotoxy(11,--line);

	for (i = 0; i<BARGRAPH_COUNT; i++) 	
	{
		if (val > valsPerChar)
		{
			lcd_putc(5);
			val = val - valsPerChar;
		}
		else
		{
			valsPerSubChar = valsPerChar /5;
			if (val > 4 * valsPerSubChar) lcd_putc(5);
			else if (val > 3 * valsPerSubChar) lcd_putc(4);
			else if (val > 2 * valsPerSubChar) lcd_putc(3);
			else if (val > 1 * valsPerSubChar) lcd_putc(2);
			else lcd_putc(1);
			for (j=0; j<BARGRAPH_COUNT-1-i; j++)
			{
				lcd_putc(0);
			}
			break;
		}
	}
}


/**
 @brief    Display bargraphs for data on specific menu
 @param    menu to be displayed                                        
 @return   none
*/
void display_Bargraph (enum activeMenus menu)
{
	switch (menu)
	{
	case Gen_Menu:
			display_Bar(1,adc_values.pwrFwrd_ADC,trip_values.Pfwrd_trip_ADC);
			display_Bar(3,(adc_values.iModuleA_ADC+adc_values.iModuleB_ADC+adc_values.iModuleD_ADC+adc_values.iModuleA_ADC)/4, 
			                   (trip_values.ImodA_trip_ADC+trip_values.ImodB_trip_ADC+trip_values.ImodC_trip_ADC+trip_values.ImodD_trip_ADC)/4);
			break;
	case Pall_menu:
			display_Bar(1,adc_values.pwrFwrd_ADC,trip_values.Pfwrd_trip_ADC);
			display_Bar(2,adc_values.pwrRefl_ADC,trip_values.Prefl_trip_ADC);
			display_Bar(3,adc_values.pwrIn_ADC,trip_values.Pin_trip_ADC);
			break;
	case Imod_menu:
	    	display_Bar(1,adc_values.iModuleA_ADC,trip_values.ImodA_trip_ADC);
	    	display_Bar(2,adc_values.iModuleB_ADC,trip_values.ImodB_trip_ADC);
			display_Bar(3,adc_values.iModuleC_ADC,trip_values.ImodC_trip_ADC);
			display_Bar(4,adc_values.iModuleD_ADC,trip_values.ImodD_trip_ADC);
			break;
	default: break;
	}
}


/**
 @brief    Display specific error 
 @param    none                                        
 @return   none
*/
void display_error ()
{
	lcd_clrscr();
	switch (activeError)
	{
	case ImodA:
            display_textLine(1,"ERROR: CURRENT MOD A");
			display_FormattedLine(2,"IA:",current.moduleA,4,1," A");
			display_FormattedLine(3,"IA Trip:  ",trip_values.ImodA_trip_ADC * cal_values.ImodA_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case ImodB:
			display_textLine(1,"ERROR: CURRENT MOD B ");
			display_FormattedLine(2,"IB:",current.moduleB,4,1," A");
			display_FormattedLine(3,"IB Trip:  ",trip_values.ImodB_trip_ADC * cal_values.ImodB_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case ImodC:
			display_textLine(1,"ERROR: CURRENT MOD C ");
			display_FormattedLine(2,"IC:",current.moduleC,4,1," A");
			display_FormattedLine(3,"IC Trip:  ",trip_values.ImodC_trip_ADC * cal_values.ImodC_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;	
	case ImodD:
			display_textLine(1,"ERROR: CURRENT MOD D ");
			display_FormattedLine(2,"ID:",current.moduleD,4,1," A");
			display_FormattedLine(3,"ID Trip:  ",trip_values.ImodD_trip_ADC * cal_values.ImodD_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Pfwrd:
			display_textLine(1,"ERROR: POWER FORWARD ");
			display_FormattedLine(2,"P FWRD :",power.fwrd,4,0," W");
			display_FormattedLine(3,"P FWRD Trip:  ",trip_values.Pfwrd_trip_W,4,0," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Prefl:
			display_textLine(1,"ERROR: POWER REFL   ");
			display_FormattedLine(2,"P REFL:",power.refl,4,0," W");
			display_FormattedLine(3,"P REFL Trip: ",trip_values.Prefl_trip_W,4,0," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Pin:
			display_textLine(1,"ERROR: POWER INPUT  ");
			display_FormattedLine(2,"P INP: ",power.input,4,1," W");
			display_FormattedLine(3,"P INP Trip: ",trip_values.Pin_trip_W,4,1," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case TempA:
			display_textLine(1,"ERROR: TEMP MOD A   ");
			display_FormattedLine(2,"T A:",temp_values.tempA,2,0," C");
			display_FormattedLine(3,"T A Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case TempB:
			display_textLine(1,"ERROR: TEMP MOD B   ");
			display_FormattedLine(2,"T B:",temp_values.tempB,2,0," C");
			display_FormattedLine(3,"T B Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	case TempC:
			display_textLine(1,"ERROR: TEMP MOD C   ");
			display_FormattedLine(2,"T C:",temp_values.tempC,2,0," C");
			display_FormattedLine(3,"T C Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	case TempD:
			display_textLine(1,"ERROR: TEMP MOD D   ");
			display_FormattedLine(2,"T D:",temp_values.tempD,2,0," C");
			display_FormattedLine(3,"T D Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	case SWR:
			display_textLine(1,"ERROR: SWR          ");
			if (power.swr == 999) display_textLine(2,"SWR ----    "); else  display_FormattedLine(2,"SWR 1:",power.swr,1,1,"  ");
			display_FormattedLine(3,"SWR Trip 1:",trip_values.swr_trip,1,1," ");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	default: 
			display_textLine(2,"UNDEFINED ERROR");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	}
}

