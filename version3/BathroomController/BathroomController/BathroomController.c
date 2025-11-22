/*
 * Программа контроллера для управления ванной комнатой.
 * 
 *  
 Граф переходов между режимами:
 
 Режим ПРИСУТСВИЕ:
 дверь закрылась и после сработал датчик на движение. Имеет силу до открытия двери и пока датчик регистрирует движения. Вызывает режим ОТСУТСВИЕ, если магнитный датчик и датчик движения молчат более 20 мин.
 Режим ПЕРЕМЕЩЕНИЕ:
 дверь открылась, в течение 3 мин есть регистрация движения, по прошествию 3 мин в случае отсутвия регистраций движения режим переходит в ОТСУТСТВИЕ.
 Режим ОТСУТСТВИЕ:
 дверь открылась/закрылась/датчик - нет регистрации движения более 3 мин


 Действия в ответ на изменения режима

 В режиме ПЕРЕМЕЩЕНИЕ:
 1. включить свет на 3 мин
 2. выключить музыку
 3. выключить вентилятор
 4. Если режим подтверждается датчиком движения, то обнулять таймер (т.е. продлить ещё на 3 мин), иначе перевести в режим ОТСУТСВИЕ.


 В режиме ПРИСУТСВИЕ:
 1. включить свет
 2. включить музыку
 3. выключить вентилятор
 вести отсчет безопасного времени 20 мин без регистрации движения, после чего перевод в режим ОСТУТСТВИЕ.


 В режиме ОТСУТСВИЕ:
 1. отключить свет, музыку.
 2. сбрасываем счетчик активности
 3. запуск вентилятора
 4. ожидание
 
 Алгоритм работы вентилятора.
 Вентилятор запускается если соблюдаются 2 условия:
 - переход режима в ОТСУТСВИЕ
 - влажность воздуха превышает установленное значение
 
 Вентилятор работает, пока влажность воздуха не понизиться до установленного предела - 2%.
 Минимальное время работы вентилятора 2 минуты.
 Максимальное 20 минут, после чего вентилятор отключается независимо от текущей влажности воздуха.
 
 
 Дисплей должен отобржать информацию о текущем режиме, а также о включенных устройствах(с-мах).
 Также, дисплей отображает текущую влажность и установленный предел влажности при котором включается вентилятор.
 Значение предела можно установить в диапозоне :  50 - 100%

 * Author: Сергей Билетников
 */ 
// 12 Мгц
#define F_CPU 12000000UL

// Текущая версия приложения
#define AP_VERSION "1.2"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <math.h>
#include <avr/wdt.h>

#include "BathroomController.h"
// библиотека для работы с датчиком влажности
#include "am2302.h"
// библиотека для работы с UART
#include "uart.h"
// библиотека выводов
#include "outpins.h"

// скорость для работы с UART
#define BAUD 19200    

////////////////////////////////////////////////////////////////////////////////
// эти строки хранятся во флэш памяти программы
const char msg_hello[] PROGMEM = "Bathroom controller is started\n";
const char msg_humidity_sensor[] PROGMEM = "Humidity: %d %%\n";
const char msg_temperature_sensor[] PROGMEM = "Temperature: %d.%d C\n";
const char msg_humidity_sensor_error[] PROGMEM = "Humidity sensor error: %d\n";
const char msg_uptime[] PROGMEM = "Uptime: %d days and %d hours\n";
const char msg_total_uptime[] PROGMEM = "Total uptime : %d days and %d hours\n";
const char msg_relax_light_on[] PROGMEM = "[Command]: Relax Light ON\n";
const char msg_relax_mode_activated[] PROGMEM = "[Command]: Relax Mode Activated\n";
const char msg_relax_mode_deactivated[] PROGMEM = "[Command]: Relax Mode Deactivated\n";
const char msg_light_on[] PROGMEM = "[Command]: Light ON\n";
const char msg_light_off[] PROGMEM = "[Command]: Light OFF\n";
const char msg_ventilation_on[] PROGMEM = "[Command]: Ventilation ON\n";
const char msg_ventilation_off[] PROGMEM = "[Command]: Ventilation OFF\n";
const char msg_ventilation_blocked[] PROGMEM = "[State]: Ventilation blocked : can not decrease humidity issue\n";
const char msg_ventilation_done[] PROGMEM = "[State]: Ventilation done\n";

const char msg_music_on[] PROGMEM = "[Command]: Music ON\n";
const char msg_music_off[] PROGMEM = "[Command]: Music OFF\n";

const char msg_out_changed[] PROGMEM = "[State]: Output %d set to %s\n";

const char msg_door_opened[] PROGMEM = "[Event]: Door opened\n";
const char msg_door_closed[] PROGMEM = "[Event]: Door closed\n";
const char msg_motion_detected[] PROGMEM = "[Event]: Motion detected\n";

const char msg_room_mode_nobody_in[] PROGMEM = "[Room mode]: Nobody in the room\n";
const char msg_room_mode_somebody_in[] PROGMEM = "[Room mode]: Somebody in the room\n";
const char msg_room_mode_change[] PROGMEM = "[Room mode]: Investigation...\n";

const char msg_test_start[] PROGMEM = "Start testing\n";
const char msg_test_end[] PROGMEM = "End testing\n";
const char msg_test_main_light[] PROGMEM = "Main light test...\n";
const char msg_test_secondary_light[] PROGMEM = "Secondary light test...\n";
const char msg_test_main_secondary_light[] PROGMEM = "Main and secondary light test...\n";
const char msg_test_ventilation[] PROGMEM = "Ventilation test...\n";
const char msg_test_music[] PROGMEM = "Music test...\n";


// Помощь
const char msg_help[] PROGMEM = "Welcome to Bathroom Controller MCU, commands : \n \
HELP                     : Help guide, which you are reading now\n \
VERSION                  : version of the software\n \
T                        : current temperature\n \
H                        : current humidity\n \
UPTIME                   : days and hours of working time since last start\n \
RESTART or RESET         : command to restart MCU \n \
BOOTLOADER               : run bootloader to upgrade software \n \
TEST                     : test controller \n \
PARAMS                   : print parameters \n \
H-VENT-ON=80             : set humidity when ventilation is started, 80% by default, max 100%\n \
H-VENT-OFF=75            : set humidity when ventilation is stopped, 75% by default\n \
VENT-ENABLED=1           : enable ventilation, enabled 1, disabled 0 .Default is 1\n \
VENT-MAX-TIME=30         : set ventilation max time in minutes\n \
VENT-MIN-TIME=2          : set ventilation min time in minutes\n \
VENT-MIN-TEMP=18         : set minimal temperature when ventilation can be started\n \
VENT-AFTER=0             : run ventilation only if the room is empty. 1 - yes, 0 - no\n \
INV-WAIT-TIME=10         : set wait time in minutes when controller guess that somebody in without any signal from sensors\n \
SOMEBODY-IN-WAIT-TIME=40 : set wait time in minutes when somebody in already and no any signal from sensors\n \
MUSIC-ENABLED=1          : enable music, default is 1 (Enabled) \n \
LIGHT-MAIN-ENABLED=1     : enable primary light, default is 1 (Enabled) \n \
LIGHT-SEC-ENABLED=1      : enable secondary light, default is 1 (Enabled) \n \
LIGHT-RELAX-ENABLED=1    : enable relax light, default is 1 (Enabled) \n \
SET-HIGH-PIN-LEV=5       : set HIGH logic level for output pin with number : 0-7\n \
SET-LOW-PIN-LEV=3        : set HIGH logic level for output pin with number : 0-7\n \
PIN-LEVEL                : print pin logic level for output pins\n \
OUTPUT-STATE             : print output states\n \
SET-PIN-ON=2             : set pin to ON state using its index\n \
SET-PIN-OFF=3            : set pin to OFF state using its index\n \
\n \
OUTPUT PINS: Main Light 0, Secondary Light 1, Ventilation 2, Music 3\n \
-----------------------------------------------------------------------------------------\n \
";
///////////////////////////////////////////////////////////////////////////////////////////

#define UART_MSG_BUF_LEN 64
static char uart_msg_buf[UART_MSG_BUF_LEN] = "";
//static char debug_buf[64] = "";

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// проверка состояние закрытой двери, 1 дверь открыта, 0 дверь закрыта
#define CHECK_DOOR_OPENED (DOOR_SENSOR_PORT_PIN & (1<<DOOR_SENSOR_PIN))

// проверка движения для первого датчика, 1 движения нет, 0 есть движение
#define CHECK_MOTION_DETECTED1 !(CHECK_BIT(MOV_SENSOR1_PORT_PIN, MOV_SENSOR1_PIN))
// проверка движения для второго датчика, 1 движения нет, 0 есть движение
#define CHECK_MOTION_DETECTED2 !(CHECK_BIT(MOV_SENSOR2_PORT_PIN, MOV_SENSOR2_PIN)
// проверка датчика движения, 1 движения нет, 0 есть движение, проверка происходит по двум датчикам
#define CHECK_MOTION_DETECTED (CHECK_MOTION_DETECTED1) || (CHECK_MOTION_DETECTED2)


/************************************************************************/
/* Управление элементами, здесь можно установить необходимое реле       */
/************************************************************************/
// здесь хранится конфигурация логических уровней выходных пинов
uint8_t EEMEM OUT_PIN_CONFIG_LEVEL_ADDR;
// переменная хранит конфигурацию выводных пинов  (высокий или низкий уровень)
static uint8_t out_pin_level_config = 0XFF;

#define LOG_OUTPUT_STATE 1 // 0 отключить логирование
// включение, выключение света
#define MAIN_LIGHT_ON pin_set_output(MAIN_LIGHT_PIN_INDX, PIN_ON, LOG_OUTPUT_STATE)
#define MAIN_LIGHT_OFF pin_set_output(MAIN_LIGHT_PIN_INDX, PIN_OFF, LOG_OUTPUT_STATE)
#define MAIN_LIGHT_CHECK_ON check_output_on(MAIN_LIGHT_PIN_INDX, out_pin_level_config)

#define SEC_LIGHT_ON pin_set_output(SEC_LIGHT_PIN_INDX, PIN_ON, LOG_OUTPUT_STATE)
#define SEC_LIGHT_OFF pin_set_output(SEC_LIGHT_PIN_INDX, PIN_OFF, LOG_OUTPUT_STATE)
#define SEC_LIGHT_CHECK_ON check_output_on(SEC_LIGHT_PIN_INDX, out_pin_level_config)
// проверка, включен ли какой-либо свет
#define CHECK_ANY_LIGHT_ON ((MAIN_LIGHT_CHECK_ON) || (SEC_LIGHT_CHECK_ON))

#define DEBUG_PIN_ON pin_set_output(OUT_7_INDEX, PIN_ON, 0)
#define DEBUG_PIN_OFF pin_set_output(OUT_7_INDEX, PIN_OFF, 0)

uint8_t EEMEM LIGHT_MAIN_ENABLED_ADDR;
uint8_t EEMEM LIGHT_SEC_ENABLED_ADDR;
uint8_t EEMEM LIGHT_RELAX_ENABLED_ADDR;

static uint8_t light_main_enabled_settings = LIGHT_MAIN_ENABLED_DEFAULT;
static uint8_t light_sec_enabled_settings = LIGHT_SEC_ENABLED_DEFAULT;
static uint8_t light_relax_enabled_settings = LIGHT_RELAX_ENABLED_DEFAULT;

// включение, выключение музыки
uint8_t EEMEM MUSIC_ENABLED_ADDR;
static uint8_t music_enabled_settings = MUSIC_ENABLED_DEFAULT;

#define MUSIC_ON pin_set_output(MUSIC_PIN_INDX, PIN_ON, LOG_OUTPUT_STATE)
#define MUSIC_OFF pin_set_output(MUSIC_PIN_INDX, PIN_OFF, LOG_OUTPUT_STATE)
#define MUSIC_CHECK_ON pin_check_output_on(MUSIC_PIN_INDX)

// индекс статуса прерываний
#define PCINT_STATE_DOOR 2
#define PCINT_STATE_MOTION1 3
#define PCINT_STATE_MOTION2 4
#define STATE_RELAX_LIGHT 5
// общий статус прерываний
volatile static uint8_t InterState = 0b00011011;

// текущий режим комнаты
volatile static uint8_t RoomMode = 0x0;

// счетчик последней активности в секундах
volatile static uint16_t LastActivityTimeCounter = 0x0;
// таймер присутсвия в комнате в секундах
volatile static uint16_t SomebodyInTimeCounter = 0x0;
// таймер присутсвия в комнате без регистрации движения в секундах
volatile static uint16_t SomebodyInWithoutMotionDelayCounter = 0x0;
// таймер для подсчета длительности в 1 секунду
volatile static uint8_t milliseconds_timer_counter = 0x0;

/****************************************************************/
/* Сенсор влажности                                             */
/****************************************************************/
// адрес во внутренней памяти
uint8_t EEMEM HUMIDITY_VENT_ON_ADDR;
uint8_t EEMEM HUMIDITY_VENT_OFF_ADDR;

// переменная установки значения влажности
static uint8_t humidity_vent_on_value;
static uint8_t humidity_vent_off_value;


// данные собранные с сенсора влажности и температуры
#define HUMIDITY_SENSOR_OK 1
#define HUMIDITY_SENSOR_FAIL 0
#define HUMIDITY_SENSOR_NOT_READ -1

struct HT_Sensor_Data
{
	uint8_t humidity;
	uint16_t temperature;
	int8_t status;
	uint8_t error_code;
} ht_sensor;

// сохранение настроек вентиляции в EEPROM
uint8_t EEMEM VENT_ENABLED_ADDR;
uint8_t EEMEM VENT_MAX_TIME_ADDR;
uint8_t EEMEM VENT_MIN_TIME_ADDR;
uint8_t EEMEM VENT_MIN_TEMP_ADDR;
uint8_t EEMEM VENT_AFTER_ADDR;
// настройки
uint8_t vent_enabled_settings = 0;
uint8_t vent_max_time_settings = 0;
uint8_t vent_min_time_settings = 0;
uint8_t vent_min_temp_settings = 0;
uint8_t vent_after_settings = 0;

// счетчик работы вентилятора в секундах
volatile static uint16_t VentWorkingCounter = 0x0;
// блокировка работы вентилятора, чтобы исключить бесконечную работу, снимается только при изменении режима комнаты
volatile static int8_t vent_blocked = NO;


#define VENT_ON pin_set_output(VENT_PIN_INDX, PIN_ON, LOG_OUTPUT_STATE)
#define VENT_OFF pin_set_output(VENT_PIN_INDX, PIN_OFF, LOG_OUTPUT_STATE)
#define VENT_CHECK_ON pin_check_output_on(VENT_PIN_INDX)

/********************************************************/
/* Режимы и тайминги                                    */
/********************************************************/
// режимы комнаты
#define ROOM_MODE_NOBODY_IN 1 // бит режима отсутсвия, 1 режим установлен, 0 сброшен
#define ROOM_MODE_SOMEBODY_IN 2 // бит режима присутсвия, 1 режим установлен, 0 сброшен
#define ROOM_MODE_INVESTIGATION 3 // бит режима перемещения, 1 режим установлен, 0 сброшен

uint8_t EEMEM INVESTIGATION_WAIT_TIME_ADDR;
uint8_t EEMEM SOMEBODY_IN_WAIT_TIME_ADDR;
static uint8_t investigation_wait_time;
static uint8_t somebody_in_wait_time;

// Регистр событий, используется для отображения случившихся событий
volatile uint8_t EventRegister = 0x0;
#define DOOR_EVENT 0
#define SET_DOOR_EVENT EventRegister|=(1<<DOOR_EVENT); 
#define CLEAR_DOOR_EVENT EventRegister&=~(1<<DOOR_EVENT);

#define MOVEMENT_EVENT 1
#define SET_MOVEMENT_EVENT EventRegister|=(1<<MOVEMENT_EVENT); 
#define CLEAR_MOVEMENT_EVENT EventRegister&=~(1<<MOVEMENT_EVENT);

#define UPDATE_HUMIDITY_EVENT 2
#define SET_UPDATE_HUMIDITY_EVENT EventRegister|=(1<<UPDATE_HUMIDITY_EVENT);
#define CLEAR_UPDATE_HUMIDITY_EVENT EventRegister&=~(1<<UPDATE_HUMIDITY_EVENT);

#define RELAX_MODE_EVENT 3
#define SET_RELAX_MODE_EVENT EventRegister|=(1<<RELAX_MODE_EVENT);
#define CLEAR_RELAX_MODE_EVENT EventRegister&=~(1<<RELAX_MODE_EVENT);

// счетчики для отображения событий с помощью светодиодов
volatile uint8_t led_door_event_counter = 0x0;
volatile uint8_t led_mov_event_counter = 0x0;
volatile uint8_t led_hum_event_counter = 0x0;
#define LED_EVENT_DELAY 15 // 100 ms * 15 = 1.5 sec

// счетчик времени, позволяет устанавливать паузу между событием  двери и событием на движение
// это связано с тем, что датчик двжиения срабатывает с задержкой, что приводит к неправильному включению режима - например, человек быстро вышел, закрыл дверь, а датчик инициировал событие
volatile static uint8_t MotionSensorDelayCounter_1 = MOTION_SENSOR_DELAY_1;
volatile static uint8_t MotionSensorDelayCounter_2 = MOTION_SENSOR_DELAY_2;

// счетчик режимов комнаты
volatile static uint8_t OneSecondIntervalCounter = 0x0;
// счетчик разрешения включения режима RELAX освещение
volatile static uint8_t RelaxLightCommandCounter = 0x0;
volatile static uint8_t relax_light_timer_counter = 0; // отсчёт времерни для включения режима освещения Relax
#define RELAX_LIGHT_TIME_DELAY 20 // 20 * 100 мс = 2 секунды

// счетчик секунд, до 1 часа
volatile static uint16_t seconds_timer_counter = 0;
// счетчик часов без перезагрузки
volatile static uint16_t uptime_hours_counter = 0;

// Обработка поступающих комманд
#define COMMAND_BUF_LEN 64
static char command_buf[COMMAND_BUF_LEN]; // буфер для команды
static uint8_t command_read_index = 0;
static uint8_t command_read_timeout_counter = 0;
// Маркеры чтения команды
#define COMMAND_READ_MARKER_PROCESSING 2
#define COMMAND_READ_MARKER_READING 1
#define COMMAND_READ_MARKER_NOT_READING 0

static uint8_t command_read_marker = COMMAND_READ_MARKER_NOT_READING;
#define COMMAND_READ_TIMEOUT_100MS 2 // 100-200 ms

// перезапустить контроллер в течение 5 секунд
#define soft_reset()        \
do                          \
{                           \
  wdt_enable(WDTO_15MS);  \
  for(;;)                 \
  {                       \
  }                       \
} while(0)

/************************************************************************/
// Ф-ция проверки прерывания на ножке + сохранение текущего состояния в
// регистр предыдущего состояния.
// Возвращает:
// -1 если прерывание повторяется
/*  0 изменение по спаду фронта                                         */
/*  1 изменение по восходящему фронту                                   */
/************************************************************************/
int8_t check_pin_intterupt(const volatile uint8_t *inputreg, uint8_t inputbit,  volatile uint8_t *last_state, const uint8_t last_state_bit)
{
	int8_t regValue = (*inputreg >> inputbit) & 0x01;
	if (((*last_state >> last_state_bit)&1) == (regValue)) {
		return -1; /* no change */
	}
	
	// сохраняем значение в историю
	if (regValue) *last_state|=1<<last_state_bit; else *last_state&=~(1<<last_state_bit);
	return regValue;
}

uint8_t pin_check_output_on(uint8_t output_index)
{
	return check_output_on(output_index, out_pin_level_config);
}

uint8_t pin_set_output(uint8_t pin_index, uint8_t on_value, uint8_t log)
{
	uint8_t result = set_output(pin_index, on_value, out_pin_level_config);
	if (log && result)
	{
		if (result)
		{
			// произошло изменение состояния порта
			
			if (on_value == PIN_ON) {
				sprintf_P(uart_msg_buf, msg_out_changed, pin_index, "ON");
			} else {
				sprintf_P(uart_msg_buf, msg_out_changed, pin_index, "OFF");
			}
			uart_puts(uart_msg_buf);
		}		
	}
	
	return result;
}

// тестировать устройство
void run_device_test()
{
	wdt_disable(); // отключаем сторожевой таймер
	
	uart_puts_p(msg_test_start);	
	// проверка основого света	
	uart_puts_p(msg_test_main_light);
	MAIN_LIGHT_OFF;
	_delay_ms(1000);
	MAIN_LIGHT_ON;
	_delay_ms(1000);
	MAIN_LIGHT_OFF;
	_delay_ms(1000);
	MAIN_LIGHT_ON;
	_delay_ms(1000);
	MAIN_LIGHT_OFF;
	// прроверка дополнительного освещения
	uart_puts_p(msg_test_secondary_light);
	SEC_LIGHT_OFF;
	_delay_ms(1000);
	SEC_LIGHT_ON;
	_delay_ms(1000);
	SEC_LIGHT_OFF;
	_delay_ms(1000);
	SEC_LIGHT_ON;
	_delay_ms(1000);
	SEC_LIGHT_OFF;	
	// проверка основного и дополнительного вместе
	uart_puts_p(msg_test_main_secondary_light);
	MAIN_LIGHT_OFF; SEC_LIGHT_OFF;
	_delay_ms(1000);
	MAIN_LIGHT_ON; SEC_LIGHT_ON;
	_delay_ms(1000);
	MAIN_LIGHT_OFF; SEC_LIGHT_OFF;
	_delay_ms(1000);
	MAIN_LIGHT_ON; SEC_LIGHT_ON;
	_delay_ms(1000);
	MAIN_LIGHT_OFF; SEC_LIGHT_OFF;	
	// проверка вентиляции
	uart_puts_p(msg_test_ventilation);
	VENT_OFF;
	_delay_ms(1000);
	VENT_ON;
	_delay_ms(5000);
	VENT_OFF;
	_delay_ms(5000);
	VENT_ON;
	_delay_ms(5000);
	VENT_OFF;
	// проверка музыки	
	uart_puts_p(msg_test_music);
	MUSIC_OFF;
	_delay_ms(1000);
	MUSIC_ON;
	_delay_ms(5000);
	MUSIC_OFF;
	_delay_ms(5000);
	MUSIC_ON;
	_delay_ms(5000);
	MUSIC_OFF;	
	uart_puts_p(msg_test_end);	
	// инициализация сторожевого таймера
	wdt_enable(WDTO_2S);
}

// запустить/остановить вентиляцию
void ventilation(uint8_t on_flag)
{
	if (on_flag == ON)
	{
		if (vent_enabled_settings == ENABLED && !VENT_CHECK_ON)
		{
			VENT_ON;
			uart_puts_p(msg_ventilation_on);
			VentWorkingCounter = 0x0;
		}
	} else if (on_flag == OFF)
	{
		if (VENT_CHECK_ON)
		{
			VENT_OFF;
			uart_puts_p(msg_ventilation_off);
		}
	}
}

// включение / выключение света
void light(uint8_t on_flag)
{
	if (on_flag == ON)
	{			
		// включить свет
		if (CHECK_BIT(InterState, STATE_RELAX_LIGHT))
		{
			// интим свет
			MAIN_LIGHT_OFF;			
			SEC_LIGHT_ON;
		} else
		{
			// обычный	
			MAIN_LIGHT_ON;
			if (light_sec_enabled_settings == ENABLED) {
				SEC_LIGHT_ON;
			}	
			uart_puts_p(msg_light_on);
		}
	} else if (on_flag == OFF)
	{
		// выключить свет
		if (CHECK_ANY_LIGHT_ON) uart_puts_p(msg_light_off);
		MAIN_LIGHT_OFF;
		SEC_LIGHT_OFF;		
	}
}
// включение/выключение музыки
void music(uint8_t on_flag)
{
	if (on_flag == ON)
	{
		if (music_enabled_settings == ENABLED && !MUSIC_CHECK_ON)
		{
			MUSIC_ON;
			uart_puts_p(msg_music_on);
		}		
	}
	else if (on_flag == OFF)
	{
		if (MUSIC_CHECK_ON)
		{
			MUSIC_OFF;
			uart_puts_p(msg_music_off);
		}		
	}
}


void activate_relax_mode()
{
	// Активация релакс режима возможно только если в данный момент дверь закрыта
	if (!CHECK_DOOR_OPENED)
	{
		InterState|=(1<<STATE_RELAX_LIGHT);
		light(ON);
		uart_puts_p(msg_relax_mode_activated);
	}
}

void deactivate_relax_mode()
{
	InterState&=~(1<<STATE_RELAX_LIGHT);
	light(ON);
	uart_puts_p(msg_relax_mode_deactivated);
}

void try_relax_light_mode_activation()
{
	// 2 и более закрытий двери втечение 2 секунд - команда на активироание интимного освещения
	if (RelaxLightCommandCounter >= 2)
	{
		// активируем режим интимного освещения если вторичный свет разрешён
		if (light_relax_enabled_settings == ENABLED && light_sec_enabled_settings == ENABLED)
		{
			SET_RELAX_MODE_EVENT;
		}		
	}
				
	RelaxLightCommandCounter = 0;			
}

// изменить текущее состояние комнаты
static void change_room_mode(uint8_t mode)
{
	// проверить текущий режим	
	if (mode == RoomMode) return; // нет изменений, выходим
	
	if (mode == ROOM_MODE_SOMEBODY_IN)
	{
		uart_puts_p(msg_room_mode_somebody_in);
		// зажечь свет
		light(ON);
		// включить музыку
		music(ON);
	} else if (mode == ROOM_MODE_INVESTIGATION)
	{
		uart_puts_p(msg_room_mode_change);
		// зажечь свет
		light(ON);
		// выключить музыку
		music(OFF);		
	} else if (mode == ROOM_MODE_NOBODY_IN)
	{
		uart_puts_p(msg_room_mode_nobody_in);
		// выключить свет
		light(OFF);
		// выключить музыку
		music(OFF);
		uart_puts_p(msg_music_off);
	}
	RoomMode = mode;	
	vent_blocked = NO; // снимаем блокировку вентилятора
}

// установить исходное значение паузы для датчиков движения
static void setMotionSensorDelay()
{
	MotionSensorDelayCounter_1 = MOTION_SENSOR_DELAY_1;
	MotionSensorDelayCounter_2 = MOTION_SENSOR_DELAY_2;
}
// выполнить отсчет
static void updateMotionSensorCounter()
{
	if (MotionSensorDelayCounter_1 > 0)
	{
		MotionSensorDelayCounter_1--;
	}
	if (MotionSensorDelayCounter_2 > 0)
	{
		MotionSensorDelayCounter_2--;
	}
}


// установка значения переменной с валидацией на минимальное и максимальное значение, а также с использование значения по умолчанию
uint8_t set_config_value(uint8_t * var_addr, uint8_t new_value,  uint8_t set_default_value, uint8_t min_value, uint8_t max_value)
{
		uint8_t old_value = *var_addr;
		if (new_value < min_value || new_value > max_value)
		{
			if (set_default_value)
			{
				*var_addr = set_default_value; // значение по умолчанию
			}
		} else
		{
			*var_addr = new_value;
		}
		return (*var_addr) != old_value;
}

// установка конфигурации логическог уровня выходных пинов
void set_out_pin_level_config(uint8_t new_value, uint8_t eemem_store)
{		
	uint8_t result = set_config_value(&out_pin_level_config, new_value, 255, 0, 255);
	if (result && eemem_store) eeprom_update_byte(&OUT_PIN_CONFIG_LEVEL_ADDR, out_pin_level_config);
}

// установка таймингов для различных режимов комнаты
void set_investigation_wait_time(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&investigation_wait_time, new_value, INVESTIGATION_WAIT_TIME_DEFAULT, INVESTIGATION_WAIT_TIME_MIN_VALUE, INVESTIGATION_WAIT_TIME_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&INVESTIGATION_WAIT_TIME_ADDR, investigation_wait_time);
}

void set_somebody_in_wait_time(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&somebody_in_wait_time, new_value, SOMEBODY_IN_WAIT_TIME_DEFAULT, SOMEBODY_IN_WAIT_TIME_MIN_VALUE, SOMEBODY_IN_WAIT_TIME_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&SOMEBODY_IN_WAIT_TIME_ADDR, somebody_in_wait_time);	
}

// установка максимального времени работы вентилятора
void set_max_vent_time(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&vent_max_time_settings, new_value, MAX_VENT_TIME_DEFAULT, MAX_VENT_TIME_MIN_VALUE, MAX_VENT_TIME_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&VENT_MAX_TIME_ADDR, vent_max_time_settings);	
}

// установка минимально времени работы вентилятора
void set_min_vent_time(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&vent_min_time_settings, new_value, MIN_VENT_TIME_DEFAULT, MIN_VENT_TIME_MIN_VALUE, MIN_VENT_TIME_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&VENT_MIN_TIME_ADDR, vent_min_time_settings);	
}

// установка минимальной температуры, при которой срабатывае вентилятор
void set_min_vent_temperature(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&vent_min_temp_settings, new_value, MIN_VENT_TEMP_DEFAULT, MIN_VENT_TEMP_MIN_VALUE, MIN_VENT_TEMP_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&VENT_MIN_TEMP_ADDR, vent_min_temp_settings);	
}

// установка минимальной температуры, при которой срабатывает вентилятор
void set_vent_after(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&vent_after_settings, new_value, VENT_AFTER_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&VENT_AFTER_ADDR, vent_after_settings);
}

// разрешение работы вентилятор
void set_vent_enabled(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&vent_enabled_settings, new_value, VENT_ENABLED_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&VENT_ENABLED_ADDR, vent_enabled_settings);
}

// проверка и установка валидных значений для вентилятора
void set_humidity_vent_on_settings(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&humidity_vent_on_value, new_value, DEFAULT_HUMIDITY_VENT_ON_VALUE, HUMIDITY_MIN_VALUE, HUMIDITY_MAX_VALUE);
	if (result && eemem_store) eeprom_update_byte(&HUMIDITY_VENT_ON_ADDR, humidity_vent_on_value);	
}
// проверка и установка валидных значений для вентилятора
void set_humidity_vent_off_settings(uint8_t new_value, uint8_t eemem_store)
{
  uint8_t result = set_config_value(&humidity_vent_off_value, new_value, DEFAULT_HUMIDITY_VENT_OFF_VALUE, HUMIDITY_MIN_VALUE, HUMIDITY_MAX_VALUE);
  if (result && eemem_store) eeprom_update_byte(&HUMIDITY_VENT_OFF_ADDR, humidity_vent_off_value);	
}

// разрешение музыки
void set_music_enabled(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&music_enabled_settings, new_value, MUSIC_ENABLED_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&MUSIC_ENABLED_ADDR, music_enabled_settings);
}

// разрешение основного освещения
void set_light_main_enabled(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&light_main_enabled_settings, new_value, LIGHT_MAIN_ENABLED_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&LIGHT_MAIN_ENABLED_ADDR, light_main_enabled_settings);
}

// разрешение дополнительного освещения
void set_light_sec_enabled(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&light_sec_enabled_settings, new_value, LIGHT_SEC_ENABLED_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&LIGHT_SEC_ENABLED_ADDR, light_sec_enabled_settings);
}

// разрешение режима Релакс
void set_light_relax_enabled(uint8_t new_value, uint8_t eemem_store)
{
	uint8_t result = set_config_value(&light_relax_enabled_settings, new_value, LIGHT_RELAX_ENABLED_DEFAULT, 0, 1);
	if (result && eemem_store) eeprom_update_byte(&LIGHT_RELAX_ENABLED_ADDR, light_relax_enabled_settings);
}


// инициализация предустановленного значения влажности используя EEPROM память
void init_humidity_settings()
{
	// считываем значение установленной влажности с EEPROM
	uint8_t value = eeprom_read_byte(&HUMIDITY_VENT_ON_ADDR);
	set_config_value(&humidity_vent_on_value, value, DEFAULT_HUMIDITY_VENT_ON_VALUE, HUMIDITY_MIN_VALUE, HUMIDITY_MAX_VALUE);
    value = eeprom_read_byte(&HUMIDITY_VENT_OFF_ADDR);
    set_config_value(&humidity_vent_off_value, value, DEFAULT_HUMIDITY_VENT_OFF_VALUE, HUMIDITY_MIN_VALUE, HUMIDITY_MAX_VALUE);
	// инициализация данных сенсора
	ht_sensor.error_code = 0;
	ht_sensor.humidity = 0;
	ht_sensor.temperature = 0;
	ht_sensor.status = HUMIDITY_SENSOR_NOT_READ;
}
// инициализация предустановленного значения таймингов вентилятора, используя EEPROM память
void init_vent_settings()
{
	uint8_t value;
	value = eeprom_read_byte(&VENT_MAX_TIME_ADDR);
	set_max_vent_time(value, 0);
	value = eeprom_read_byte(&VENT_MIN_TIME_ADDR);  
	set_min_vent_time(value, 0);
	value = eeprom_read_byte(&VENT_MIN_TEMP_ADDR);      
	set_min_vent_temperature(value, 0);
	value = eeprom_read_byte(&VENT_AFTER_ADDR);  
	set_vent_after(value, 0);
	value = eeprom_read_byte(&VENT_ENABLED_ADDR);
	set_vent_enabled(value, 0);
}

// инициализация предустановлленых значений счетчиков для режима комнаты, используя EEPROM память
void init_mode_counter_setting()
{
	uint8_t value;
	value = eeprom_read_byte(&SOMEBODY_IN_WAIT_TIME_ADDR);
	set_somebody_in_wait_time(value, 0);
	value = eeprom_read_byte(&INVESTIGATION_WAIT_TIME_ADDR);
	set_investigation_wait_time(value, 0);		
}

void init_music_settings()
{
	uint8_t value;
	value = eeprom_read_byte(&MUSIC_ENABLED_ADDR);
	set_music_enabled(value, 0);
}

void init_light_settings()
{
	uint8_t value;
	value = eeprom_read_byte(&LIGHT_MAIN_ENABLED_ADDR);
	set_light_main_enabled(value, 0);
	value = eeprom_read_byte(&LIGHT_SEC_ENABLED_ADDR);
	set_light_sec_enabled(value, 0);
	value = eeprom_read_byte(&LIGHT_RELAX_ENABLED_ADDR);
	set_light_relax_enabled(value, 0);
}

void init_door_settings()
{
	// инициализация текущих состояний двери
	if (CHECK_DOOR_OPENED) {
		InterState|=(1<<PCINT_STATE_DOOR);
	}		
}

// обновить значения влажности воздуха
void update_humidity()
{
	// читаем датчик влажности, в ответ получаем значение влажности и статус чтения : ОК или ошибка
	uint16_t rawHumidityValue;	
	uint16_t rawTemperatureValue;	
	cli(); // запрещаем прерывания на время чтения
	// читаем датчик влажности
	uint8_t error = am2302(&rawHumidityValue, &rawTemperatureValue);		
	sei();
	// проверка чтения на ошибки
	if (error) 
	{
		ht_sensor.status = HUMIDITY_SENSOR_FAIL; // произошла ошибка чтения датчика
		ht_sensor.error_code = error;		
		
		memset(uart_msg_buf, '\0', UART_MSG_BUF_LEN); // очищаем буфер
		sprintf_P(uart_msg_buf, msg_humidity_sensor_error, error);
		uart_puts(uart_msg_buf);
	} 
	else 
	{
		ht_sensor.status = HUMIDITY_SENSOR_OK;
		// convert raw humidity value to humidity
		//CurrentHumidityValue = (uint8_t) round(rawHumidityValue / 10); // round требует достаточно много памяти 350 байт, в случае оптимазации можно упростить:
		ht_sensor.humidity = (uint8_t) (rawHumidityValue / 10);
		ht_sensor.temperature = rawTemperatureValue;
	}		
}

void control_ventilation()
{
	if (vent_blocked)
		return;  // вентилятор заблокирован
		
	int8_t normal_humidity = UNKNOWN; // индикатор нормы влжаности 
	if (ht_sensor.status == HUMIDITY_SENSOR_OK)
	{
		normal_humidity = YES; // Влажность в норме
		uint8_t temperature = (uint8_t)(ht_sensor.temperature/10);
		uint8_t humidity = ht_sensor.humidity;
		
		// сравниваем текущую и установленную влажность на включение вентилятора
		if (humidity > humidity_vent_on_value)
		{
			// достигнут предел допустимой влажности
			normal_humidity = NO; // влажность не в норме
						
			// проверяем допустимую минимальную температуру при которой можно включать вентилятор
			if (temperature > vent_min_temp_settings)
			{
				if (vent_after_settings == NO || (vent_after_settings == YES && RoomMode == ROOM_MODE_NOBODY_IN))
				{
					// температура в помещении достаточно высокая для включения вентилятора
					// если активирована ф-ция включение только в режиме ОТСУТСТВИЕ, проверить текущий режим перед включением, который должен соответсвовать режиму отсутсвие
					ventilation(ON); // включить вентилятор
				}
			} 
			else
			{
				// слишком холодно, чтобы включать вентилятор если там находится человек (т.е. режим присутсвие или исследование), поэтому включаем только если в комнате никого нет
				if (RoomMode == ROOM_MODE_NOBODY_IN)
				{
					// никого нет в комнате, можно включать
					ventilation(ON); // включить вентилятор
				}
			}
						
		}
	}
	
	// если вентилятор работает
	if (VENT_CHECK_ON)
	{
		// вентилятор работает
		if ((VentWorkingCounter >= (vent_min_time_settings * 60)) && (normal_humidity == YES))
		{
			// вентилятор отработал минимальное время работы, и влажность уже в норме, то его необходимо отключить
			uart_puts_p(msg_ventilation_done);
			ventilation(OFF);
		} 
		else if (VentWorkingCounter >= (vent_max_time_settings * 60))
		{
			// вентилятор максимальное время и его необходимо отключить			
			ventilation(OFF);
			vent_blocked = YES; // блокировка вентилятора, чтобы исключить его повторный запуск подряд			
			uart_puts_p(msg_ventilation_blocked);
		}
	}
}

// проверка режимов комнаты согласно установленным интервалам
void check_room_modes_and_intervals()
{
	if (RoomMode == ROOM_MODE_INVESTIGATION)
	{
		// проверка по таймеру на активность за прошедшее время в секундах
		uint16_t maxTimeForMotionWaiting = investigation_wait_time * 60;
		/*if (CHECK_DOOR_OPENED)
		{
			maxTimeForMotionWaiting = investigation_wait_time;
		}*/
		if (LastActivityTimeCounter >= maxTimeForMotionWaiting)
		{
			// время по таймеру истекло, необходим перевод в режим ОТСУТСТВИЕ
			change_room_mode(ROOM_MODE_NOBODY_IN);
		}
	}
	else if (RoomMode == ROOM_MODE_SOMEBODY_IN)
	{
		if (SomebodyInWithoutMotionDelayCounter > (somebody_in_wait_time * 60))
		{
			// так и не было зафиксировано сигналов от датчика движения за установленое время, значит переводим комнату в режим ОТСУТСВИЯ
			change_room_mode(ROOM_MODE_NOBODY_IN);
		}
	}
}

// сбрасываем операцию текущего чтения комманды
void reset_command_reading()
{
  command_read_index = 0;
  command_read_marker = COMMAND_READ_MARKER_NOT_READING;
  memset(command_buf, '\0', COMMAND_BUF_LEN);  
}
// обработка команды
void process_command(char * buf)
{  
  // проверяем, если команда GET типа, то она содержит только один параметр
  // если команда SET , то два параметра разделенных знаком =
  uint8_t index = 0;
  uint8_t param_name_index = 0;
  uint8_t param_value_index = 0;  
  char param_name[COMMAND_BUF_LEN/2+1] = "";
  char param_value[COMMAND_BUF_LEN/2+1] = "";    
  uint8_t command_setter_type = 0;
  uint8_t value_set = 0;
  
  while ((buf[index] != '\0') && (index < COMMAND_BUF_LEN))
  {
    char ch = buf[index];    
    if (ch == '=')
    {
      command_setter_type = 1;
      index++;
      continue;
    }    
    // читаем первый параметр
    if (!command_setter_type && (param_name_index < COMMAND_BUF_LEN/2))
    {
      param_name[param_name_index++] = ch;
    }
    // читаем второй параметр
    if (command_setter_type && (param_value_index < COMMAND_BUF_LEN/2))
    {
      param_value[param_value_index++] = ch;
    }
    
    index++;
  }
    
  uint8_t param_value_len = strlen(param_value);  
  
  if (strcasecmp("HELP", param_name) == 0)
  {
    uart_puts_p(msg_help);
  } else if (strcasecmp("BOOTLOADER", param_name) == 0)
  {
      // запускаем загрузчик	  
	uart_puts("Launch bootloader in 10 seconds");
	for (uint8_t i = 0; i < 10; i++)
	{
		wdt_reset();
		_delay_ms(1000);
	}
	soft_reset();
  } else if (strcasecmp("T", param_name) == 0)
  {
    // выводим температуру 
    sprintf_P(uart_msg_buf, msg_temperature_sensor, (uint8_t)(ht_sensor.temperature/10), (uint8_t)(ht_sensor.temperature % 10));
    uart_puts(uart_msg_buf);
    
  } else if (strcasecmp("H", param_name) == 0)
  {
    // выводим влажность
    sprintf_P(uart_msg_buf, msg_humidity_sensor, ht_sensor.humidity);
    uart_puts(uart_msg_buf);
  } else if (strcasecmp("VERSION", param_name) == 0)
  {
    uart_puts(AP_VERSION);
  } else if (strcasecmp("UPTIME", param_name) == 0)
  {
    // выводим время беспреревной работы
    sprintf_P(uart_msg_buf, msg_uptime, (uint16_t)(uptime_hours_counter/24), (uint8_t)(uptime_hours_counter%24));
    uart_puts(uart_msg_buf);
  } else if (strcasecmp("RESTART", param_name) == 0 || strcasecmp("RESET", param_name) == 0)
  {
	// перезагрузка
	uart_puts("Restarting...\n");
	soft_reset();
   }  
   else if ((strcasecmp("H-VENT-ON", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 3) {
    // установить значение	
    uint8_t new_value = atoi(param_value);
    set_humidity_vent_on_settings(new_value, 1);
	value_set = 1;
  } else if ((strcasecmp("H-VENT-ON", param_name) == 0) && !command_setter_type) {
    // выводим значение влажности при котором включается вентиляция
    itoa(humidity_vent_on_value, uart_msg_buf, 10);
    uart_puts(uart_msg_buf);
  } else if ((strcasecmp("H-VENT-OFF", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 3) {	
	uint8_t new_value = atoi(param_value);
	set_humidity_vent_off_settings(new_value, 1); // установить значение
	value_set = 1;
  } else if ((strcasecmp("H-VENT-OFF", param_name) == 0) && !command_setter_type) {
    // выводим значение параметра
	itoa(humidity_vent_off_value, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
  } else if ((strcasecmp("VENT-MAX-TIME", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 4) {
	// установить значение
	uint8_t new_value = atoi(param_value);
	set_max_vent_time(new_value, 1);
	value_set = 1;
 } else if ((strcasecmp("VENT-MAX-TIME", param_name) == 0) && !command_setter_type) {
	itoa(vent_max_time_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
 } else if ((strcasecmp("VENT-MIN-TIME", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 4) {
	// установить значение
	uint8_t new_value = atoi(param_value);
	set_min_vent_time(new_value, 1);
	value_set = 1;
 } else if ((strcasecmp("VENT-MIN-TIME", param_name) == 0) && !command_setter_type) {
	itoa(vent_min_time_settings, uart_msg_buf, 10);	
	uart_puts(uart_msg_buf);
 } else if ((strcasecmp("VENT-MIN-TEMP", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 3) {	  
	  uint8_t new_value = atoi(param_value);
	  set_min_vent_temperature(new_value, 1); // установить значение
	  value_set = 1;
 } else if ((strcasecmp("VENT-MIN-TEMP", param_name) == 0) && !command_setter_type) {
	  itoa(vent_min_temp_settings, uart_msg_buf, 10);
	  uart_puts(uart_msg_buf);
 } else if ((strcasecmp("VENT-AFTER", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 2) {	 
	 uint8_t param_value_int = atoi(param_value);
	 set_vent_after(param_value_int, 1); // установить значение  
	 value_set = 1;
 } else if ((strcasecmp("VENT-AFTER", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(vent_after_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf); 
 } else if ((strcasecmp("VENT-ENABLED", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t param_value_int = atoi(param_value);
	set_vent_enabled(param_value_int, 1); // установить значение
	value_set = 1;
 } else if ((strcasecmp("VENT-ENABLED", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(vent_enabled_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
 } else if ((strcasecmp("MUSIC-ENABLED", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t param_value_int = atoi(param_value);
	set_music_enabled(param_value_int, 1); // установить значение
	value_set = 1;
 } else if ((strcasecmp("MUSIC-ENABLED", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(music_enabled_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
 } else if ((strcasecmp("LIGHT-MAIN-ENABLED", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t param_value_int = atoi(param_value);
	set_light_main_enabled(param_value_int, 1); // установить значение
	value_set = 1;
 } else if ((strcasecmp("LIGHT-MAIN-ENABLED", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(light_main_enabled_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
} else if ((strcasecmp("LIGHT-SEC-ENABLED", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t param_value_int = atoi(param_value);
	set_light_sec_enabled(param_value_int, 1); // установить значение
	value_set = 1;
} else if ((strcasecmp("LIGHT-SEC-ENABLED", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(light_sec_enabled_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
} else if ((strcasecmp("LIGHT-RELAX-ENABLED", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t param_value_int = atoi(param_value);
	set_light_relax_enabled(param_value_int, 1); // установить значение
	value_set = 1;
} else if ((strcasecmp("LIGHT-RELAX-ENABLED", param_name) == 0) && !command_setter_type) {
	// выводим значение параметра
	itoa(light_relax_enabled_settings, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
} else if ((strcasecmp("INV-WAIT-TIME", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 4) {
	uint8_t new_value = atoi(param_value);
	set_investigation_wait_time(new_value, 1); // установить значение
	value_set = 1;
 } else if ((strcasecmp("INV-WAIT-TIME", param_name) == 0) && !command_setter_type) {
	itoa(investigation_wait_time, uart_msg_buf, 10);
	uart_puts(uart_msg_buf);
 } else if ((strcasecmp("SOMEBODY-IN-WAIT-TIME", param_name) == 0) && command_setter_type && param_value_len > 0 && param_value_len < 4) {
	 uint8_t new_value = atoi(param_value);
	 set_somebody_in_wait_time(new_value, 1); // установить значение
	 value_set = 1;
 } else if ((strcasecmp("SOMEBODY-IN-WAIT-TIME", param_name) == 0) && !command_setter_type) {
	 itoa(somebody_in_wait_time, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf);
 } else if ((strcasecmp("TEST", param_name) == 0) && !command_setter_type) {
	run_device_test();
 } else if ((strcasecmp("SET-HIGH-PIN-LEV", param_name) == 0) && command_setter_type && param_value_len == 1) {
	 uint8_t param_value_int = atoi(param_value);	 
	 if (param_value_int >= 0 && param_value_int < 8) {
		 uint8_t new_value = out_pin_level_config | (1<<param_value_int);
		 set_out_pin_level_config(new_value, 1);
		 value_set = 1;
	 } 	 
 } else if ((strcasecmp("SET-LOW-PIN-LEV", param_name) == 0) && command_setter_type && param_value_len == 1) {
	 uint8_t param_value_int = atoi(param_value);
	 if (param_value_int >= 0 && param_value_int < 8) {
		 uint8_t new_value = out_pin_level_config & ~(1<<param_value_int);
		 set_out_pin_level_config(new_value, 1);
		 value_set = 1;
	 }	 
 } else if ((strcasecmp("PIN-LEVEL", param_name) == 0) && !command_setter_type) {
	 for (uint8_t i = 0; i < 8; i++)
	 {
		 itoa(i, uart_msg_buf, 10);
		 uart_puts(uart_msg_buf);
		 if (CHECK_BIT(out_pin_level_config, i)) uart_puts("-HIGH\n"); else uart_puts("-LOW\n");
	 }	 
 } else if ((strcasecmp("OUTPUT-STATE", param_name) == 0) && !command_setter_type) {
	 for (uint8_t i = 0; i < 8; i++)
	 {
		 itoa(i, uart_msg_buf, 10);
		 uart_puts(uart_msg_buf);		 
		 if (CHECK_BIT(out_pin_level_config, i)) uart_puts("-HIGH "); else uart_puts("- LOW ");
		 if (pin_check_output_on(i)) uart_puts("ON\n"); else uart_puts("OFF\n");
	 }
 } else if ((strcasecmp("SET-PIN-ON", param_name) == 0) && command_setter_type && param_value_len == 1) {
	 uint8_t index = atoi(param_value);
	 pin_set_output(index, PIN_ON, LOG_OUTPUT_STATE);
	 value_set = 1;	 
 } else if ((strcasecmp("SET-PIN-OFF", param_name) == 0) && command_setter_type && param_value_len == 1) {
	uint8_t index = atoi(param_value);
	pin_set_output(index, PIN_OFF, LOG_OUTPUT_STATE);
	value_set = 1;
 }
 // FOR DEBUG
 else if ((strcasecmp("VentWorkingCounter", param_name) == 0) && !command_setter_type) {
	 // выводим значение параметра
	 itoa(VentWorkingCounter, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf); 
 }
 else if ((strcasecmp("SomebodyInTimeCounter", param_name) == 0) && !command_setter_type) {
	 // выводим значение параметра
	 itoa(SomebodyInTimeCounter, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf);
 }
 else if ((strcasecmp("SomebodyInWithoutMDCounter", param_name) == 0) && !command_setter_type) {
	 // выводим значение параметра
	 itoa(SomebodyInWithoutMotionDelayCounter, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf);
 }
 else if ((strcasecmp("LastActivityTimeCounter", param_name) == 0) && !command_setter_type) {
	 // выводим значение параметра
	 itoa(LastActivityTimeCounter, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf);
 }  
 else if ((strcasecmp("RoomMode", param_name) == 0) && !command_setter_type) {
	 // выводим значение параметра
	 itoa(RoomMode, uart_msg_buf, 10);
	 uart_puts(uart_msg_buf);
 }  
  
  if (value_set)
  {
    // если параметр был установлен    
    sprintf(uart_msg_buf, "%s=%s\n", param_name, param_value);
	uart_puts(uart_msg_buf);
  } else
  {
	  uart_putc('\n');
  }
  
  command_read_marker = COMMAND_READ_MARKER_NOT_READING;
}

// функци чтения команды из uart буфера 
void read_command_from_uart()
{
  //unsigned int result = uart_getc();
  //if (result != UART_NO_DATA) {
  //		uint8_t  data = result;
  //	}
  if (command_read_marker == COMMAND_READ_MARKER_PROCESSING)
  {
    // не читаем, пока идёт обработка комманды
    return;
  }

  for (;;)
  {
    int uart_resp = uart_getc();
    
    if (!(uart_resp >> 8))
    {
      command_read_marker = COMMAND_READ_MARKER_READING;
      
      // удачное считывание
      char read_data = uart_resp;            
      
      if (read_data == '\r')
      {
        // пропускаем этот символ
        continue;
      }
      
      if (read_data == '\n')
      {
        command_read_marker = COMMAND_READ_MARKER_PROCESSING;
        // символ ввода должен завершить команду, заполняем оставшуюся часть буфера нулевым символом
        for (uint8_t i = command_read_index; i < COMMAND_BUF_LEN; i++)
        {
          command_buf[i] = '\0';
        }
        // отправляем на обработку
        process_command(command_buf);
        break;
      }
      // новые данные получены
      if (command_read_index >= COMMAND_BUF_LEN)
      {
        // буфер заполнен - значит что-то не так, команду
        reset_command_reading();
        break;
      }
      
      // пишем данные в буфер
      command_buf[command_read_index++] = read_data;
    } else if (uart_resp == UART_NO_DATA)
    {
      // больше нет данных в данный момент
      break;
    }
    else
    {
      // произошла ошибка чтения останавливаем чтение команды
      reset_command_reading();
      break;
    }
  }
}

// функция инициализации портов
void init_port_pins()
{
	/************************************************************************/
	/* Инициализация портов ввода/вывода                                    */
	/************************************************************************/
	// загрузка конфигурации логических уровней порта из памяти
	set_out_pin_level_config(eeprom_read_byte(&OUT_PIN_CONFIG_LEVEL_ADDR), 0);
	// инициализация блоков реле
	OUT_PINS_SETUP;		
	// отключаем все пины вывода при инициализации
	set_output_for_all(PIN_OFF, out_pin_level_config);
	//
	
	// выводы работы с сенсором устанавливаем как вход с высоким импедансом, без резистора подтяжки
	MOV_SENSOR1_DDR&=~(1<<MOV_SENSOR1_PIN);
	MOV_SENSOR2_DDR&=~(1<<MOV_SENSOR2_PIN);
	DOOR_SENSOR_DDR&=~(1<<DOOR_SENSOR_PIN);
	
	// инициализация светодиодов
	LED_MCU_STATUS_SETUP; // статусный светодиод
	LED_EVENT_1_SETUP; // событие 1
	LED_EVENT_2_SETUP; // событие 2
	LED_EVENT_3_SETUP; // событие 3
	
	// устанавливаем резисторы подтяжки на неиспользуемые порты
	// TODO:			
	
	/************************************************************************/
	/* Настройка прерываний по изменению сигнала на ножке                   */
	/************************************************************************/
	PCICR|=(1<<PCIE0)|(1<<PCIE1)|(1<<PCIE2);
	//PCMSK0|=(1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2);
	//PCMSK2|=(1<<PCINT22)|(1<<PCINT23);
	PCMSK1|=(1<<PCINT11)|(1<<PCINT10)|(1<<PCINT9)|(1<<PCINT8);
	PCIFR=0x0; //очистить флаги пререваний по маске
}

/************************************************************************/
/* Настройка таймера                                                    */
/************************************************************************/
void initTimer() {
	TCCR1B|=(1<<CS10)|(1<<CS12)|(1<<WGM12); // пределитель 1024, и сбрасывать счетчик про совпадении регистра
	TCCR1A=0x0;
	//OCR1A=781;  // это значение дает задержку в 100 мили секунд, при частоте процессора 8 Мгц
    OCR1A=1172; // это значение дает задержку в 100 мили секунд при частоте процессора 12 Мгц
	//разрешаем таймер 1 в режиме прерывания по сравнению
	TCNT1=0x0;
	// активация таймера
	TIMSK1|=(1<<OCIE1A);
	// сброс счетчиков таймера
	milliseconds_timer_counter = 0;
}


// обработчик таймера, вызывается каждые 100 мс
void timer_on_each_100ms() 
{
	if (COMMAND_READ_MARKER_READING)
	{
		// в данный момент идёт считывание комманды с uart
		if (command_read_timeout_counter == 0)
		{
			// счетчик ещё не установлен, устанавливаем его
			command_read_timeout_counter = COMMAND_READ_TIMEOUT_100MS;
		} else
		{
			command_read_timeout_counter--;
			if (command_read_timeout_counter == 0)
			{
				// останавливаем считывание команды по таймауту
				reset_command_reading();
			}
		}
	}	
	
	if (relax_light_timer_counter > 0)
	{
		relax_light_timer_counter--;
		if (relax_light_timer_counter == 0)
		{
			try_relax_light_mode_activation();
		}
	}	
	
	// светодиоды событий
	if (led_door_event_counter > 0)
	{
		LED_EVENT_1_ON;
		led_door_event_counter--;
	} 
	else 
	{
		LED_EVENT_1_OFF;
	}
	// светодиоды событий
	if (led_mov_event_counter > 0)
	{
		LED_EVENT_2_ON;
		led_mov_event_counter--;
	}
	else
	{
		LED_EVENT_2_OFF;
	}
	if (led_hum_event_counter > 0)
	{
		
		LED_EVENT_3_ON;
		led_hum_event_counter--;
	}
	else
	{
		LED_EVENT_3_OFF;
	}
}

// таймер - каждые 10 секунд
void timer_on_each_10_sec() 
{
	// проверка влажности воздуха каждые 10 секунд
	SET_UPDATE_HUMIDITY_EVENT;
}

// обработчик таймера, вызывается каждые 2 секунды
void timer_on_each_2_sec()
{

}

// на каждую минуту
void timer_on_each_1_min() 
{
  
}

//на каждый час
void timer_on_each_1_hour()
{
  uptime_hours_counter++;
}

// обработчик таймера, вызывается каждую секунду
void timer_on_each_1sec() 
{
	seconds_timer_counter++;
  
	// мигать статусным светодиодом
	LED_MCU_STATUS_TOGGLE;
	
	// обновить интервалы комнаты
	if (RoomMode == ROOM_MODE_INVESTIGATION)
	{
		LastActivityTimeCounter++;
	}
	else if (RoomMode == ROOM_MODE_SOMEBODY_IN)
	{
		SomebodyInTimeCounter++;
		SomebodyInWithoutMotionDelayCounter++;
	}
		
	if (VENT_CHECK_ON)
	{
		VentWorkingCounter++;
	}	
			
	updateMotionSensorCounter();		
  
	if ((seconds_timer_counter % 2) == 0) {
		timer_on_each_2_sec();
	}    
  
	if ((seconds_timer_counter % 10) == 0)
	{
    
		timer_on_each_10_sec();
	}
  
	if ((seconds_timer_counter % 60) == 0) {
		timer_on_each_1_min();
	}
	
	if (seconds_timer_counter >= 3600) {
		timer_on_each_1_hour();
		seconds_timer_counter = 0; // сбросить счетчик
	} 
	
	// Debug
	//sprintf_P(debug_buf, msg_debug, VentWorkingCounter, (uint8_t)(ht_sensor.temperature/10), (uint8_t)(ht_sensor.humidity), vent_min_time_settings * 60, vent_max_time_settings * 60);
	//uart_puts(debug_buf);
}

// проверка PCINT прерываний на ножках
void check_pcint_intterupts()
{
	// события двери
	int8_t check_result = check_pin_intterupt(&DOOR_SENSOR_PORT_PIN, DOOR_SENSOR_PIN, &InterState, PCINT_STATE_DOOR);
	if (check_result != -1)
	{
		SET_DOOR_EVENT;
		//TODO :REMOVE
//		DEBUG_PIN_ON;
//		_delay_ms(1);
//		DEBUG_PIN_OFF;		
	}
	
	// событие на датчик движения 1
	check_result = check_pin_intterupt(&MOV_SENSOR1_PORT_PIN, MOV_SENSOR1_PIN, &InterState, PCINT_STATE_MOTION1);
	if (check_result == 0 && MotionSensorDelayCounter_1 == 0)
	{
		SET_MOVEMENT_EVENT;
	}
	
	// событие на датчик движения 2
	check_result = check_pin_intterupt(&MOV_SENSOR2_PORT_PIN, MOV_SENSOR2_PIN, &InterState, PCINT_STATE_MOTION2);	
	if (check_result == 0 && MotionSensorDelayCounter_2 == 0)
	{
		SET_MOVEMENT_EVENT;
	}
}

// регистрация на движение
static void onMovementEvent()
{
	// обнуляем таймер отсчета времени без регистрации движения
	SomebodyInWithoutMotionDelayCounter=0x0;
	
	// сбросить счетчик последней активности
	LastActivityTimeCounter = 0x0;
	
	led_mov_event_counter = LED_EVENT_DELAY;
	
	uart_puts_p(msg_motion_detected);
	
	if (CHECK_DOOR_OPENED)
	{
		// дверь открыта и зафиксировано движение, перевести в режим ПЕРЕМЕЩЕНИЕ
		change_room_mode(ROOM_MODE_INVESTIGATION);
	}
	else {
		// дверь заперта и зафиксировано движение, перевести в режим ПРИСУТСТВИЕ
		// сбросить счетчик присутсвия
		SomebodyInTimeCounter = 0x0;
		SomebodyInWithoutMotionDelayCounter = 0x0;
		change_room_mode(ROOM_MODE_SOMEBODY_IN);
	}
}

// событие на открытие/закрытие двери
static void onDoorEvent()
{
	// события магнитного датчика двери
	uint8_t door_opened = CHECK_DOOR_OPENED;
	
	// обнуляем таймер отсчета времени без регистрации движения
	SomebodyInWithoutMotionDelayCounter=0x0;
	
	// сбросить счетчик последней активности
	LastActivityTimeCounter = 0x0;
	// устанавливаем счётчик для индикатора светодиода
	led_door_event_counter = LED_EVENT_DELAY;
	
	// устанавливаем задержку для срабатывания датчиков движения
	setMotionSensorDelay();
	
	if (door_opened)
	{
		// очищаем бит RELAX режима
		if (CHECK_BIT(InterState, STATE_RELAX_LIGHT))
		{
			deactivate_relax_mode();
		}
	} else
	{
		// дверь закрылась - обновить счетчик
		if (RelaxLightCommandCounter == 0)
		{
			relax_light_timer_counter = RELAX_LIGHT_TIME_DELAY;
		}
		RelaxLightCommandCounter++;
	}
	
	if (door_opened) uart_puts_p(msg_door_opened); else uart_puts_p(msg_door_closed);
	
	// изменяем режим на ПЕРЕМЕЩЕНИЕ
	change_room_mode(ROOM_MODE_INVESTIGATION);
}

// получение событие на получение значение датчика влажностей (запрос на получение)
static void onUpdateHumidityEvent()
{
	led_hum_event_counter = LED_EVENT_DELAY;
	update_humidity();
}

// получено событие на включение режима Релакс
static void onRelaxModeEvent()
{
	activate_relax_mode();
}

// обработка событий
void check_events()
{	
	if (CHECK_BIT(EventRegister, DOOR_EVENT))
	{
		CLEAR_DOOR_EVENT;		
		onDoorEvent();
	}
			
	if (CHECK_BIT(EventRegister, MOVEMENT_EVENT))
	{
		CLEAR_MOVEMENT_EVENT;
		onMovementEvent();
	}
			
	if (CHECK_BIT(EventRegister, UPDATE_HUMIDITY_EVENT))
	{
		CLEAR_UPDATE_HUMIDITY_EVENT;
		onUpdateHumidityEvent();
	}
	
	if (CHECK_BIT(EventRegister, RELAX_MODE_EVENT))
	{
		CLEAR_RELAX_MODE_EVENT;
		onRelaxModeEvent();
	}	
}

// прерывания на ножках микроконтроллера
ISR(PCINT0_vect)
{
	check_pcint_intterupts();
}

ISR(PCINT1_vect)
{
	check_pcint_intterupts();
}
ISR(PCINT2_vect)
{
	check_pcint_intterupts();
}


// прерывания по сравнению 16 битного таймера 1
ISR(TIMER1_COMPA_vect) {
	timer_on_each_100ms();
	milliseconds_timer_counter++;
	if (milliseconds_timer_counter == 10) {
		timer_on_each_1sec();
		milliseconds_timer_counter = 0;
	}
}                        

int main(void)
{
	// инициализируем сторожевой таймер
	wdt_disable();
	// запрещаем прерывания на время инициализации
	cli();
	// инициализацмя устройств и портов
	init_port_pins();
	//Иницилизация UART
	uart_init(UART_BAUD_SELECT(BAUD, F_CPU));		
	// инициализация сторожевого таймера
	//wdt_enable(WDTO_2S);
		
	// инициализация настроек
	init_humidity_settings();
	init_vent_settings();	
	init_mode_counter_setting();
	init_music_settings();
	init_light_settings();
	init_door_settings();
	// инициализируем таймер
	initTimer();			
			
	// отключить компаратор
	ACSR|=(1<<ACD);
		
	// настроить режим сна (idle mode)
	SMCR|=(1<<SE);//IDLE

	// разрешаем глобальные прерывания
	sei();
  
    // Выводим привествие и версию прошивки
    uart_puts_p(msg_hello);
   
    while (1) 
    {		
		// сбрасываем сторожевой таймер
		wdt_reset();					
		
		// считывание данных с uart
		read_command_from_uart();

		// проверка рабочих интервалов, согласно режиму комнаты
		check_room_modes_and_intervals();
    
		// обработка событий
		check_events();     
		
		// алгоритм работы вентиляции
		control_ventilation();
		
		// уводим контроллер в сон
		asm("sleep");					
    }
}
