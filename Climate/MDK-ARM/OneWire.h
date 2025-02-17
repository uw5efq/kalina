/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  ���������� ��������� 1wire �� ���� ���������� libopencm3 ��� ���������������� STM32F103

            ��������, ���������� ����� ��������� �������� � �� ������ uK (��������� ��������).
            ����� ���� ����������� � ������������� ����������� USART uK ��� ��������� ������ 1wire.

            ����������� ��������� �������������� �� ��������� USART � TX ����, ������� ������ ���� �������� � ����� ������� �������������� 4.7�.
            ���������� ���������� ������������ ��������� RX �� TX ������ uK, �������� ����� RX ��������� ��� ������������� � ������ �������.

 */
#ifndef STM32_DS18X20_ONEWIRE_H
#define STM32_DS18X20_ONEWIRE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/usart.h>
//#include <libopencm3/cm3/nvic.h>

#define ONEWIRE_NOBODY 0xF0
#define ONEWIRE_SEARCH 0xF0
#define ONEWIRE_SKIP_ROM 0xCC
#define ONEWIRE_READ_ROM 0x33
#define ONEWIRE_MATCH_ROM 0x55
#define ONEWIRE_CONVERT_TEMPERATURE 0x44
#define ONEWIRE_READ_SCRATCHPAD 0xBE
#define ONEWIRE_WRITE_SCRATCHPAD 0x4E
#define ONEWIRE_COPY_SCRATCHPAD 0x48
#define ONEWIRE_RECALL_E2 0xB8

#ifndef MAXDEVICES_ON_THE_BUS
#define MAXDEVICES_ON_THE_BUS 10
#endif

#define DS18B20 0x28
#define DS18S20 0x10

#define WIRE_0    0x00 // 0x00 --default
#define WIRE_1    0xff
#define OW_READ   0xff

volatile uint8_t recvFlag;
volatile uint16_t rc_buffer[5];

typedef struct {
    int8_t inCelsus;
    uint8_t frac;
} Temperature;

typedef struct {
    uint8_t family;
    uint8_t code[6];
    uint8_t crc;
} RomCode;

typedef struct {
    uint8_t crc;
    uint8_t reserved[3];
    uint8_t configuration;
    uint8_t tl;
    uint8_t th;
    uint8_t temp_msb;
    uint8_t temp_lsb;
} Scratchpad_DS18B20;

typedef struct {
    uint8_t crc;
    uint8_t count_per;
    uint8_t count_remain;
    uint8_t reserved[2];
    uint8_t tl;
    uint8_t th;
    uint8_t temp_msb;
    uint8_t temp_lsb;
} Scratchpad_DS18S20;

typedef struct {
    uint32_t usart;
    RomCode ids[MAXDEVICES_ON_THE_BUS];
    int lastDiscrepancy;
    uint8_t lastROM[8];
} OneWire;

void usart_enable_halfduplex(uint32_t usart); /// ��������������� ������� �� ��������� HalfDuplex �� USART
void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity,
                 uint32_t flowcontrol);

uint16_t owResetCmd(OneWire *ow);

int owSearchCmd(OneWire *ow);

void owSkipRomCmd(OneWire *ow);

uint8_t owCRC8(RomCode *rom);

void owMatchRomCmd(OneWire *ow, RomCode *rom);

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom);

uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data);

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom);

void owRecallE2Cmd(OneWire *ow, RomCode *rom);

Temperature readTemperature(OneWire *ow, RomCode *rom, bool reSense);

void owSend(OneWire *ow, uint16_t data);

void owSendByte(OneWire *ow, uint8_t data);

uint16_t owEchoRead(OneWire *ow);

void owReadHandler(uint32_t usart);

#endif //STM32_DS18X20_ONEWIRE_H