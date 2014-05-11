/*
 * display.h
 *
 * Created: 21-4-2014 9:36:27
 *  Author: cmobach
 */ 

#include <inttypes.h>
#include "generalDefine.h"


#ifndef DISPLAY_H_
#define DISPLAY_H_

/**
 @brief    Display float without auto linefeed
 @param    float to be displayed with width (max 10) and nr of digits                                       
 @return   none
*/
extern void display_Float( float f, uint8_t width, uint8_t nrOfdigits);

/**
 @brief    Display formatted string at line without auto linefeed
 @param    formatted string to be displayed including float value                                        
 @return   none
*/
extern void display_FormattedLine (uint8_t line, char *preamble,float f_value, uint8_t width, uint8_t nrOfdigits, char *postamble);

/**
 @brief    Display text string at line without auto linefeed
 @param    text to be displayed                                        
 @return   none
*/
extern void display_textLine (uint8_t line, char *text);

/**
 @brief    Display menu
 @param    menu to be displayed                                        
 @return   none
*/
extern void display_Menu (enum activeMenus menu);

/**
 @brief    Display active error
 @param    none                                        
 @return   none
*/
extern void display_error ();


#endif /* DISPLAY_H_ */