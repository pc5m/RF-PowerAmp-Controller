/*
 * display.c
 *
 * Created: 21-4-2014 9:36:46
 *  Author: cmobach
 */ 

#include "display.h"
#include "lcd.h"
#include <stdlib.h>


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
			display_FormattedLine(2,"SWR 1:",power.swr,1,0,"   ");
			display_FormattedLine(3,"I  ",current.moduleTotal,3,0," A  ");
			
			display_FormattedLine(4,"T  ",temp_values.tempMax,3,0," C  ");  //should be maximum temperature = todo !
			activeMenu = Gen_Menu;
		break;
	case Pall_menu:
			display_FormattedLine(1,"FWD ",power.fwrd,4,0," W");
			display_FormattedLine(2,"REF ",power.refl,3,0," W ");
			display_FormattedLine(3,"INP ",power.input,4,1," W");
			display_FormattedLine(4,"SWR 1:",power.swr,1,0,"   ");
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
            display_textLine(1,"ERROR:CURRENT MOD A TRIPPED");
			display_FormattedLine(2,"IA:",current.moduleA,4,1," A");
			display_FormattedLine(3,"IA Trip:  ",trip_values.ImodA_trip_ADC * cal_values.ImodA_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case ImodB:
			display_textLine(1,"ERROR:CURRENT MOD B TRIPPED");
			display_FormattedLine(2,"IB:",current.moduleB,4,1," A");
			display_FormattedLine(3,"IB Trip:  ",trip_values.ImodB_trip_ADC * cal_values.ImodB_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case ImodC:
			display_textLine(1,"ERROR:CURRENT MOD C TRIPPED");
			display_FormattedLine(2,"IC:",current.moduleC,4,1," A");
			display_FormattedLine(3,"IC Trip:  ",trip_values.ImodC_trip_ADC * cal_values.ImodC_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;	
	case ImodD:
			display_textLine(1,"ERROR:CURRENT MOD D TRIPPED");
			display_FormattedLine(2,"ID:",current.moduleD,4,1," A");
			display_FormattedLine(3,"ID Trip:  ",trip_values.ImodD_trip_ADC * cal_values.ImodD_ADC2AMP,4,1," A");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Pfwrd:
			display_textLine(1,"ERROR:POWER OUTPUT TRIPPED");
			display_FormattedLine(2,"P FWRD :",power.fwrd,4,0," W");
			display_FormattedLine(3,"P FWRD Trip:  ",trip_values.Pfwrd_trip_W,4,0," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Prefl:
			display_textLine(1,"ERROR:POWER REFLECTED TRIPPED");
			display_FormattedLine(2,"P REFL:",power.refl,4,0," W");
			display_FormattedLine(3,"P REFL Trip: ",trip_values.Prefl_trip_W,4,0," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case Pin:
			display_textLine(1,"*** P-IN TRIPPED ***");
			display_FormattedLine(2,"P INP: ",power.input,4,1," W");
			display_FormattedLine(3,"P INP Trip: ",trip_values.Pin_trip_W,4,1," W");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case TempA:
			display_textLine(1,"ERROR:TEMP MOD A TRIPPED");
			display_FormattedLine(2,"T A:",temp_values.tempA,2,0," C");
			display_FormattedLine(3,"T A Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	case TempB:
			display_textLine(1,"ERROR:TEMP MOD B TRIPPED");
			display_FormattedLine(2,"T B:",temp_values.tempB,2,0," C");
			display_FormattedLine(3,"T B Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	case TempC:
			display_textLine(1,"ERROR:TEMP MOD C TRIPPED");
			display_FormattedLine(2,"T C:",temp_values.tempC,2,0," C");
			display_FormattedLine(3,"T C Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	case TempD:
			display_textLine(1,"ERROR:TEMP MOD D TRIPPED");
			display_FormattedLine(2,"T D:",temp_values.tempD,2,0," C");
			display_FormattedLine(3,"T D Trip: ",trip_values.temp_trip,2,0," C");
			display_textLine(4,"ON-OFF TO RESUME");
	break;
	default: 
			display_textLine(2,"UNDEFINED ERROR");
			display_textLine(4,"ON-OFF TO RESUME");
			break;
	}
}

