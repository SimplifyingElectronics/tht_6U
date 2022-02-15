/*
 * eeprom.h
 *
 * Created: 16-01-2022 17:21:21
 *  Author: HP
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

// #include <avr/eeprom.h>
// 
// #define EEPROM_CHECKSUM						7621
// #define EEPROM_CHECKSUM_ADD					(uint16_t *)110
// #define EEPROM_TEMP_ADD						(uint16_t *)130
// #define EEPROM_K_P_ADD						(uint16_t *)150
// #define EEPROM_K_I_ADD						(uint16_t *)170
// #define EEPROM_K_D_ADD						(uint16_t *)190
// #define EEPROM_FLAGDEBUG_ADD				(uint16_t *)210
// 
// #define TEMP_LOW							150
// #define TEMP_HIGH							450
// #define TEMP_DEFAULT						285
// 
// #define K_P_LOW								0
// #define K_P_HIGH							750
// #define K_P_DEFAULT							10	
// 
// #define K_I_LOW								50
// #define K_I_HIGH							500
// #define K_I_DEFAULT							0
// 
// #define K_D_LOW								0
// #define K_D_HIGH							500
// #define K_D_DEFAULT							0
// 
// #define FLAGDEBUG_DEFAULT					0
// 
// uint16_t setTemp, setKp, setKi, setKd, flagDebugMode;

void eeprom_init(void);


#endif /* EEPROM_H_ */