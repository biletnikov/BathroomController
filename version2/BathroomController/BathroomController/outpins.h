/*
 * output_pins.h
 *
 * Created: 08.09.2019 19:38:10
 *  Author: sbiletnikov
 */ 


#ifndef OUTPINS_H_
#define OUTPINS_H_

#include <avr/io.h>

// Конфигурация управляющих выводов общего назначения
#define OUT_0_DDR DDRD
#define OUT_0_PORT PORTD
#define OUT_0_PORTIN PIND
#define OUT_0_PIN PIND5
#define OUT_0_INDEX 0

#define OUT_1_DDR DDRD
#define OUT_1_PORT PORTD
#define OUT_1_PORTIN PIND
#define OUT_1_PIN PIND6
#define OUT_1_INDEX 1

#define OUT_2_DDR DDRD
#define OUT_2_PORT PORTD
#define OUT_2_PORTIN PIND
#define OUT_2_PIN PIND7
#define OUT_2_INDEX 2

#define OUT_3_DDR DDRB
#define OUT_3_PORT PORTB
#define OUT_3_PORTIN PINB
#define OUT_3_PIN PINB0
#define OUT_3_INDEX 3

#define OUT_4_DDR DDRB
#define OUT_4_PORT PORTB
#define OUT_4_PORTIN PINB
#define OUT_4_PIN PINB1
#define OUT_4_INDEX 4

#define OUT_5_DDR DDRB
#define OUT_5_PORT PORTB
#define OUT_5_PORTIN PINB
#define OUT_5_PIN PINB2
#define OUT_5_INDEX 5

#define OUT_6_DDR DDRB
#define OUT_6_PORT PORTB
#define OUT_6_PORTIN PINB
#define OUT_6_PIN PINB3
#define OUT_6_INDEX 6

#define OUT_7_DDR DDRB
#define OUT_7_PORT PORTB
#define OUT_7_PORTIN PINB
#define OUT_7_PIN PINB4
#define OUT_7_INDEX 7

#define PIN_ON 1
#define PIN_OFF 0

// инициализация выходных портов
#define OUT_PINS_SETUP \
OUT_0_DDR|=(1<<OUT_0_PIN);\
OUT_1_DDR|=(1<<OUT_1_PIN);\
OUT_2_DDR|=(1<<OUT_2_PIN);\
OUT_3_DDR|=(1<<OUT_3_PIN);\
OUT_4_DDR|=(1<<OUT_4_PIN);\
OUT_5_DDR|=(1<<OUT_5_PIN);\
OUT_6_DDR|=(1<<OUT_6_PIN);\
OUT_7_DDR|=(1<<OUT_7_PIN);

// конфигурирование портов вывода в соотвествии с требуемыми логическими состояниями  ON = HIGL level or LOW
// каждый бит logic_level_config соответсвует порту вывода, наприме 0й бит ->  OUT_0 ,  0 - означает низкий уровень для ON
//extern void init_pins_output_logic_level(uint8_t logic_level_config);

// установка логического состояния порта по его индексу 
// on_value = OFF если == 0, иначе ON
// индекс от 0 до 7
// возвращает 1 если состояние порта была изменено, иначе 0
extern uint8_t set_output(uint8_t output_index, uint8_t on_value, uint8_t out_pin_level_config);
// установка логического состояния для всех портов
extern void set_output_for_all(uint8_t on_value, uint8_t out_pin_level_config);
// проверка текущего установленного логического состояния пина
// 0 если OFF, иначе ON
extern uint8_t check_output_on(uint8_t output_index, uint8_t out_pin_level_config);

#endif /* OUTPINS_H_ */