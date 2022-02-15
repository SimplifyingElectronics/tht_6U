/*
 * eeprom.c
 *
 * Created: 16-01-2022 17:21:03
 *  Author: HP
 */ 
#include "eeprom.h"

//uint16_t setTemp = 0, setKp = 0, setKi = 0, setKd = 0;

// void eeprom_init(void)
// {
// 	if((eeprom_read_word(EEPROM_CHECKSUM_ADD)) == EEPROM_CHECKSUM)
// 	{
// 		setTemp = eeprom_read_word(EEPROM_TEMP_ADD);
// 		setKp = eeprom_read_word(EEPROM_K_P_ADD);
// 		setKi = eeprom_read_word(EEPROM_K_I_ADD);
// 		setKd = eeprom_read_word(EEPROM_K_D_ADD);
// 		flagDebugMode = eeprom_read_word(EEPROM_FLAGDEBUG_ADD);
// 
// 		if((setTemp < TEMP_LOW) || (setTemp > TEMP_HIGH))
// 		{
// 			setTemp = TEMP_DEFAULT;
// 			eeprom_write_word(EEPROM_TEMP_ADD, setTemp);
// 		}
// 		
// 		if((setKp < K_P_LOW) || (setKp > K_P_HIGH))
// 		{
// 			setKp = K_P_DEFAULT;
// 			eeprom_write_word(EEPROM_K_P_ADD, setKp);
// 		}
// 		
// 		if((setKi < K_I_LOW) || (setKi > K_I_HIGH))
// 		{
// 			setKi = K_I_DEFAULT;
// 			eeprom_write_word(EEPROM_K_I_ADD, setKi);
// 		}
// 		
// 		if((setKd < K_D_LOW) || (setKd > K_D_HIGH))
// 		{
// 			setKd = K_D_DEFAULT;
// 			eeprom_write_word(EEPROM_K_D_ADD, setKd);
// 		}
// 		
// 		if((flagDebugMode != 0) && (flagDebugMode != 1))
// 		{
// 			flagDebugMode = 0;
// 			eeprom_write_word(EEPROM_FLAGDEBUG_ADD, flagDebugMode);
// 		}
// 		
// 	}
// 	
// 	else
// 	{
// 		setTemp = TEMP_DEFAULT;
// 		setKp = K_P_DEFAULT;
// 		setKi = K_I_DEFAULT;
// 		setKd = K_D_DEFAULT;
// 		flagDebugMode = 0;
// 		eeprom_write_word(EEPROM_TEMP_ADD, TEMP_DEFAULT);
// 		eeprom_write_word(EEPROM_K_P_ADD, K_P_DEFAULT);
// 		eeprom_write_word(EEPROM_K_I_ADD, K_I_DEFAULT);
// 		eeprom_write_word(EEPROM_K_D_ADD, K_D_DEFAULT);
// 		eeprom_write_word(EEPROM_FLAGDEBUG_ADD, FLAGDEBUG_DEFAULT);
// 		
// 		eeprom_write_word(EEPROM_CHECKSUM_ADD, EEPROM_CHECKSUM);
// 	}
// }