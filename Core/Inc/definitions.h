/*
 * definitions.h
 *
 *  Created on: Jan 26, 2024
 *      Author: kirill
 */

#ifndef INC_DEFINITIONS_H_
#define INC_DEFINITIONS_H_

#define DEBUG_APP

#define UID_BASE				0x1FFF7A10 // адрес в котором хранится уникальный айди процессора
#define LFS_BUF_SIZE			256 //должно быть кратно размеру страницы флеш памяти (256)

#define UPGRADE_PATTERN			0xA5 //просто случайное число его ожидает считать бутлоадер чтобы начать апгрейд прошивки.
#define UPGRADE_PATERN_FILE		"upgrade_stat"
#define FIRMWARE_FILE			"firmware.bin"
#define DEFAULT_FIRMWARE_FILE   "firmware.bin.def"
#define DEF_FIRMWARE_FILE_CRC	"firmware.bin.def.crc"
#define UPGRADE_PATTERN_LEN		8

#define INIT_VAL				0x22

#define COMPILE_DATE			__DATE__
#define COMPILE_TIME			__TIME__
//---------------_SPI FLASH DEF---------------------//
#define FLASH_BLOCK_CYCLES		10000  //more than 100 000 erase/program cycles
#define BOOTLOADER_VER			"v0.0.6"
#define APP_START 				(0x08010000)			//Origin + Bootloader size (64kB)
#define APP_END					(0x0803FFF0)
#define NUM_OF_PAGES			128
#define MAIN_PROGRAM_START_ADDRESS	APP_START
#define DELAY_MS				1500


#endif /* INC_DEFINITIONS_H_ */
