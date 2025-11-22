/*
 * output_pins.cpp
 *
 * Created: 08.09.2019 19:38:40
 *  Author: sbiletnikov
 */ 

#include "outpins.h"
#include <stdio.h>

#include "uart.h"


// если конфиг не найден, вернуть 0
uint8_t find_pin_config(uint8_t out_index, struct PinConfig * conf, uint8_t out_pin_level_config)
{
	volatile uint8_t * reg;
	volatile uint8_t * reg_portin;
	volatile uint8_t reg_pin;
	
	switch (out_index) {
		case 0 : reg = &OUT_0_PORT; reg_portin = &OUT_0_PORTIN; reg_pin = OUT_0_PIN; break;
		case 1 : reg = &OUT_1_PORT; reg_portin = &OUT_1_PORTIN; reg_pin = OUT_1_PIN; break;
		case 2 : reg = &OUT_2_PORT; reg_portin = &OUT_2_PORTIN; reg_pin = OUT_2_PIN; break;
		case 3 : reg = &OUT_3_PORT; reg_portin = &OUT_3_PORTIN; reg_pin = OUT_3_PIN; break;
		case 4 : reg = &OUT_4_PORT; reg_portin = &OUT_4_PORTIN; reg_pin = OUT_4_PIN; break;
		case 5 : reg = &OUT_5_PORT; reg_portin = &OUT_5_PORTIN; reg_pin = OUT_5_PIN; break;
		case 6 : reg = &OUT_6_PORT; reg_portin = &OUT_6_PORTIN; reg_pin = OUT_6_PIN; break;
		case 7 : reg = &OUT_7_PORT; reg_portin = &OUT_7_PORTIN; reg_pin = OUT_7_PIN; break;
		
		default: return 0;
	}
	
	conf->out_indx = out_index;
	conf->reg = reg;
	conf->reg_portin = reg_portin;
	conf->reg_pin = reg_pin;	
	conf->logic_level = out_pin_level_config & (1<<out_index);
	
	return 1;
}

extern uint8_t set_output(uint8_t output_index, uint8_t on_value, uint8_t out_pin_level_config)
{
	struct PinConfig conf;
	
	if (!find_pin_config(output_index, &conf, out_pin_level_config)) {
		return 0;
	}
	
	// сохран€ем текущее значение
	uint8_t current_value = *(conf.reg_portin) & (1<<conf.reg_pin);
	
	if (conf.logic_level)
	{
		
		// ¬ключение по высокому логическому уровню
		if (on_value == PIN_ON)
		{
			*(conf.reg)|=(1<<conf.reg_pin);
		} 
		else if (on_value == PIN_OFF)
		{
			*(conf.reg)&=~(1<<conf.reg_pin);
		}
	} else
	{
		// ¬ключение по низкому логическому уровню
		if (on_value == PIN_ON)
		{			
			*(conf.reg)&=~(1<<conf.reg_pin);
		}
		else if (on_value == PIN_OFF)
		{
			*(conf.reg)|=(1<<conf.reg_pin);
		}		
	}	
	// провер€ем, действительно ли изменилось значение
	uint8_t new_value = *(conf.reg_portin) & (1<< conf.reg_pin);
	return new_value != current_value;	
}

extern void set_output_for_all(uint8_t on_value, uint8_t out_pin_level_config)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		set_output(i, on_value, out_pin_level_config);
	}
}

extern uint8_t check_output_on(uint8_t output_index, uint8_t out_pin_level_config)
{
	struct PinConfig conf;
	
	if (!find_pin_config(output_index, &conf, out_pin_level_config)) {
		return 0;
	}
	
	uint8_t check_result = *(conf.reg) & (1<<conf.reg_pin);
	if (conf.logic_level)
	{
		return check_result;
	}
	return !check_result;
}
