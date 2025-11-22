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


// рабочая частота процессора 8 Мгц
#define F_CPU 8000000UL

#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <math.h>
#include <avr/wdt.h>

#include "BathroomController.h"
// настройки дисплея
#include "lcd.h"
// датчик влажности
#include "am2302.h"

// версия программы
#define APP_VERSION 4
// длинна строки дисплея
#define LCD_LINE_LEN 16

// установка относительной влажности
#define MAX_HUMIDITY_VALUE 100
#define MIN_HUMIDITY_VALUE 50

// режимы комнаты
#define ROOM_MODE_NOBODY_IN 1 // бит режима отсутсвия, 1 режим установлен, 0 сброшен
#define ROOM_MODE_SOMEBODY_IN 2 // бит режима присутсвия, 1 режим установлен, 0 сброшен
#define ROOM_MODE_CHANGE 3 // бит режима перемещения, 1 режим установлен, 0 сброшен

// 0 если кнопка нажата
#define CHECK_BUTTON1_PRESSED BUTTON1_PORT_PIN & (1<<BUTTON1_PIN)
#define CHECK_BUTTON2_PRESSED BUTTON2_PORT_PIN & (1<<BUTTON2_PIN)

#define RELAY_1_ON RELAY_PORT|=(1<<RELAY_1_PIN)
#define RELAY_1_OFF RELAY_PORT&=~(1<<RELAY_1_PIN)

#define RELAY_2_ON RELAY_PORT|=(1<<RELAY_2_PIN)
#define RELAY_2_OFF RELAY_PORT&=~(1<<RELAY_2_PIN)

#define RELAY_3_ON RELAY_PORT|=(1<<RELAY_3_PIN)
#define RELAY_3_OFF RELAY_PORT&=~(1<<RELAY_3_PIN)

#define SOLID_RELAY_1_ON RELAY_PORT|=(1<<SOLID_RELAY_1_PIN)
#define SOLID_RELAY_1_OFF RELAY_PORT&=~(1<<SOLID_RELAY_1_PIN)

#define SOLID_RELAY_2_ON RELAY_PORT|=(1<<SOLID_RELAY_2_PIN)
#define SOLID_RELAY_2_OFF RELAY_PORT&=~(1<<SOLID_RELAY_2_PIN)

// проверка состояние закрытой двери, 1 дверь открыта, 0 дверь закрыта
#define CHECK_DOOR_OPENED (DOOR_SENSOR_PORT_PIN & (1<<DOOR_SENSOR_PIN))

// проверка движения для первого датчика, 1 движения нет, 0 есть движение
#define CHECK_MOTION_DETECTED1 !(MOV_SENSOR1_PORT_PIN & (1<<MOV_SENSOR1_PIN))
// проверка движения для второго датчика, 1 движения нет, 0 есть движение
#define CHECK_MOTION_DETECTED2 !(MOV_SENSOR2_PORT_PIN & (1<<MOV_SENSOR2_PIN))
// проверка датчика движения, 1 движения нет, 0 есть движение, проверка происходит по двум датчикам
#define CHECK_MOTION_DETECTED (CHECK_MOTION_DETECTED1) || (CHECK_MOTION_DETECTED2)


/************************************************************************/
/* Управление элементами, здесь можно установить необходимое реле       */
/************************************************************************/

// включить / выключить свет, 1 - включен;   SOLID_RELAY_1 - включает светильники, SOLID_RELAY_2 - включает зеркало
//#define LIGHT_ON SOLID_RELAY_1_ON; SOLID_RELAY_2_ON;
//#define LIGHT_OFF SOLID_RELAY_1_OFF; SOLID_RELAY_2_OFF;
#define CHECK_LIGHT_ON ((SOLID_RELAY_PORT_PIN & (1<<SOLID_RELAY_1_PIN)) || (SOLID_RELAY_PORT_PIN & (1<<SOLID_RELAY_2_PIN)))

// включить / выключить вентилятор, 1 - включен
#define VENT_ON RELAY_2_ON
#define VENT_OFF RELAY_2_OFF
#define CHECK_VENT_ON (RELAY_PORT_PIN & (1<<RELAY_2_PIN))

// включить / выключить музыку, 1 - включен
#define MUSIC_ON RELAY_1_ON
#define MUSIC_OFF RELAY_1_OFF
#define CHECK_MISIC_ON (RELAY_PORT_PIN & (1<<RELAY_1_PIN))

// индекс статуса прерываний
#define PCINT_STATE_BUTTON1 0
#define PCINT_STATE_BUTTON2 1
#define PCINT_STATE_DOOR 2
#define PCINT_STATE_MOTION1 3
#define PCINT_STATE_MOTION2 4
#define STATE_INTIM_LIGHT 5
// общий статус прерываний
volatile static uint8_t CommonState = 0b00011011;
// текущий режим комнаты
volatile static uint8_t RoomMode = 0x0; 

// счетчик последней активности
volatile static uint16_t LastActivityTimeCounter = 0x0;
// таймер присутсвия в комнате
volatile static uint16_t SomebodyInTimeCounter = 0x0;
// таймер присутсвия в комнате без регистрации движения
volatile static uint16_t SomebodyInWithoutMotionDelayCounter = 0x0;

// сохраняем некоторые фразы в EEPROM памяти, для экономии оперативной
const char HELLO_MSG[] = "Hi Bathroom :)";
const char VERSION_PREFIX[] = "VERSION: ";
const char MOTION_SUFFIX[] = " MOTION";
const char DOOR_SUFFIX[] = " DOOR";
const char ERROR_PREFIX[] = "ERR : ";
const char OUT_MODE_SUFFIX[] = "OUT";
const char IN_MODE_SUFFIX[] = " IN";
const char GO_MODE_SUFFIX[] = " GO";

/************************************************************************/
/* Сенсор влажности                                                     */
/************************************************************************/
// адрес во внутренней памяти
uint8_t EEMEM HumiditySettingsAddr;
// переменная установки значения влажности
volatile static uint8_t HumiditySettings;
uint8_t CurrentHumidityValue = 0x0;
uint16_t CurrentTemperatureValue = 0x0;
uint8_t CurrentHumiditySensorStatus = 0x0;
uint8_t CurrentHumiditySensorErrorCode = 0x0;
#define HUMIDITY_SENSOR_OK 1
#define HUMIDITY_SENSOR_FAIL 0

// Регистр событий, используется для отображения случившихся событий
volatile uint8_t EventRegister = 0x0;
#define DOOR_EVENT 0
#define MOVEMENT_EVENT 1
#define LCD_UPDATE_EVENT 5
#define DOOR_EVENT_DELAY 6
#define MOVEMENT_EVENT_DELAY 7
#define SET_DOOR_EVENT EventRegister|=(1<<DOOR_EVENT)|(1<<DOOR_EVENT_DELAY); 
#define SET_MOVEMENT_EVENT EventRegister|=(1<<MOVEMENT_EVENT)|(1<<MOVEMENT_EVENT_DELAY); 
#define SET_LCD_UPDATE_EVENT EventRegister|=(1<<LCD_UPDATE_EVENT);
#define CLEAR_DOOR_EVENT if ((EventRegister>>DOOR_EVENT_DELAY)&1) EventRegister&=~(1<<DOOR_EVENT_DELAY); else EventRegister&=~(1<<DOOR_EVENT);
#define CLEAR_MOVEMENT_EVENT if ((EventRegister>>MOVEMENT_EVENT_DELAY)&1) EventRegister&=~(1<<MOVEMENT_EVENT_DELAY); else EventRegister&=~(1<<MOVEMENT_EVENT);
#define CLEAR_LCD_UPDATE_EVENT EventRegister&=~(1<<LCD_UPDATE_EVENT);

// счетчик работы вентилятора
volatile static uint16_t VentWorkingCounter = 0x0;
// счетчик времени по прошествию которого происходит запись установленной влажности в EEPROM память
volatile static uint16_t HumidityEepromUpdateCounter = 0x0;
// счетчик времени, позволяет устанавливать паузу между событием  двери и событием на движение
// это связано с тем, что датчик двжиения срабатывает с задержкой, что приводит к неправильному включению режима - например, человек быстро вышел, закрыл дверь, а датчик инициировал событие
volatile static uint8_t MotionSensorDelayCounter_1 = MOTION_SENSOR_DELAY_1;
volatile static uint8_t MotionSensorDelayCounter_2 = MOTION_SENSOR_DELAY_2;

// timer 2
#define HUMIDITY_UPDATE_TIMEOUT 10 // 10 секунд
#define ONE_SECOND_TIMEOUT 1 // 1 секунда
#define EVENT_DISPLAY_TIMEOUT 3 // 3 секунды

// счетчик обновления ЖК дисплея
volatile static uint8_t LcdUpdateCounter = 0x0;
// счетчик обновления влажности
volatile static uint8_t HumidityUpdateCounter = 0x0;
// счетчик режимов комнаты
volatile static uint8_t OneSecondIntervalCounter = 0x0;
// счетчик отображения событий на дисплее
volatile static uint8_t EventDisplayCounter = 0x0;
// счетчик переиницализации дисплея
volatile static uint16_t ReinitLcdCounter = 0x0;
// счетчик разрешения включения режима ИНТИМ освещение
volatile static uint8_t IntimLightCommandCounter = 0x0;
// счетчик для прерывания каждые 2 секунды
volatile static uint8_t TwoSecondsCounter = 0x0;
// счетчик для прерывания каждые 5 секунд
volatile static uint8_t FiveSecondsCounter = 0x0;

/************************************************************************/
// Ф-ция проверки прерывания на ножке. 
// Возвращает:
// -1 если прерывание повторяется
/*  0 изменение по спаду фронта                                         */
/*  1 изменение по восходящему фронту                                   */
/************************************************************************/
int8_t checkPinIntterupt(const volatile uint8_t *inputreg, uint8_t inputbit,  volatile uint8_t *last_state, const uint8_t last_state_bit)
{
	uint8_t regValue = *inputreg & (1<<inputbit);
	if (((*last_state >> last_state_bit)&1) == (regValue)) {
		return -1; /* no change */
	}
	
	// сохраняем значение в историю
	if (regValue) *last_state|=1<<last_state_bit; else *last_state&=~(1<<last_state_bit);
	return regValue;	
}

// запустить вентиляцию
void startVentilation()
{
	VENT_ON;
	VentWorkingCounter = 0x0;
}

// остановить вентиляцию
void stopVentilation()
{
	VENT_OFF;
}

// включить свет
void lightOn()
{
	if ((CommonState >> STATE_INTIM_LIGHT) & 1)
	{
		// интим свет
		SOLID_RELAY_1_OFF;
		SOLID_RELAY_2_ON;
	} else
	{
		// обычный
		SOLID_RELAY_1_ON;
		SOLID_RELAY_2_ON;
	}		
}

// выключить свет
void lightOff()
{
	SOLID_RELAY_1_OFF;
	SOLID_RELAY_2_OFF;
}

// обновить режим освещения интим
void updateIntimLightMode()
{
	if (CHECK_LIGHT_ON)
	{
		lightOn(); // зажигаем свет ещё раз, чтобы переключить реле в необходимом порядке
	}	
}

// изменить текущее состояние комнаты
static void changeRoomMode(uint8_t mode)
{
	// проверить текущий режим
	
	if (mode == RoomMode) 
	return; // нет изменений, выходим
	
	if (mode == ROOM_MODE_SOMEBODY_IN)
	{
		// зажечь свет
		lightOn();
		// включить музыку
		MUSIC_ON;	
		// выключить вентилятор
		stopVentilation();
	} else if (mode == ROOM_MODE_CHANGE)
	{
		// зажечь свет
		lightOn();
		// выключить музыку
		MUSIC_OFF;		
	} else if (mode == ROOM_MODE_NOBODY_IN)
	{
		// выключить свет
		lightOff();
		// выключить музыку
		MUSIC_OFF;		
	}
	RoomMode = mode;
	// запрос на обновление ЖК
	SET_LCD_UPDATE_EVENT;
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

// регистрация на движение
static void onMovementRegister()
{
	SET_MOVEMENT_EVENT;
	
	// обнуляем таймер отсчета времени без регистрации движения
	SomebodyInWithoutMotionDelayCounter=0x0;
		
	// сбросить счетчик последней активности
	LastActivityTimeCounter = 0x0;	
	
	// сбросить счетчик отображения событий
	EventDisplayCounter = 0x0;
	
	// запрос на обновление ЖК
	SET_LCD_UPDATE_EVENT;
		
	if (CHECK_DOOR_OPENED)
	{
		// дверь открыта и зафиксировано движение, перевести в режим ПЕРЕМЕЩЕНИЕ
		changeRoomMode(ROOM_MODE_CHANGE);
	}
	else {
		// дверь заперта и зафиксировано движение, перевести в режим ПРИСУТСТВИЕ
		// сбросить счетчик присутсвия
		SomebodyInTimeCounter = 0x0;
		SomebodyInWithoutMotionDelayCounter = 0x0;
		changeRoomMode(ROOM_MODE_SOMEBODY_IN);
	}	
}

// событие на открытие/закрытие двери
static void onDoorEvent()
{
	// отмечаем событие в регистре учета событий
	EventRegister|=(1<<DOOR_EVENT)|(1<<DOOR_EVENT_DELAY);	
	// события магнитного датчика двери
	// обнуляем таймер отсчета времени без регистрации движения
	SomebodyInWithoutMotionDelayCounter=0x0;
		
	// сбросить счетчик последней активности
	LastActivityTimeCounter = 0x0;	
	// сбросить счетчик отображения событий
	EventDisplayCounter = 0x0;
	// запрос на обновление ЖК
	SET_LCD_UPDATE_EVENT;
	
	// устанавливаем задержку для срабатывания датчиков движения
	setMotionSensorDelay();
	
	// обновить счетчик
	if (!CHECK_DOOR_OPENED) 
	{
		IntimLightCommandCounter++;
	}	
	
	// очищаем бит интим режима
	CommonState&=~(1<<STATE_INTIM_LIGHT);
	
	updateIntimLightMode();
		
	// изменяем режим на ПЕРЕМЕЩЕНИЕ
	changeRoomMode(ROOM_MODE_CHANGE);
}

/************************************************************************/
/* Инициализация портов, порты LCD диспления устанавливаются в          */
/* заголовчном файле "lcd.h"                                            */
/************************************************************************/
void initLCD()
{
	//Initialize LCD module
	LCDInit(LS_NONE);
	LCDClear(); // очистка дисплея
	// запрос на обновление ЖК
	SET_LCD_UPDATE_EVENT;
}

// процедура отрисовки текущих данных на ЖК дисплей, вызывается после каждого события
// ################
// ################
// 90% : 95%    OUT
// LVM  MOTION DOOR

// L - свет включен
// V - вентилятор включен
// M - музыка включена
// MOTION - движение зарегистрировано
// DOOR - событие от двери (открытие/закрытие)
// OUT - никого нет в комнате,  IN - кто-то внутри, MOV - перемещение
// 90% : 95% - 90% текущее значение влажности, 95% - установленное, при котором включается вентилятор
static void updateLCD()
{
	char line[17]; // строка на дисплее
	char temp[4];
	
	if ((REINIT_LCD_IN_TIME > 0) && (ReinitLcdCounter >= REINIT_LCD_IN_TIME))
	{
		// время переинициализации ЖК
		ReinitLcdCounter = 0x0;
		initLCD();		
	}
	
	if ((EventRegister & (1<<LCD_UPDATE_EVENT))) {		
		CLEAR_LCD_UPDATE_EVENT;
	
		memset(line, 0, sizeof(line));
		memset(temp, 0, sizeof(temp));
	
		if (CurrentHumiditySensorStatus == HUMIDITY_SENSOR_OK) {
			itoa(CurrentHumidityValue, temp, 10);			
			strcat(line, temp);	
			strcat(line, "%");						
			
			strcat(line, " : ");
			itoa(HumiditySettings, temp, 10);
			strcat(line, temp);
			strcat(line, "%");
		} else {
			itoa(CurrentHumiditySensorErrorCode, temp, 10);
			strcat(line, ERROR_PREFIX);
			strcat(line, temp);				
		}						
		
		// состояние комнаты занимает последние 3 символа в строчке, проверка, в случае необходимости форматируем
		for (int i = 0; i < LCD_LINE_LEN - strlen(line) - 3; i++) {
			strcat(line, " "); // добавляем пробел(ы) перед названием 3 символьного режима
		}
	
		switch (RoomMode)
		{
			case ROOM_MODE_NOBODY_IN : strcat(line, OUT_MODE_SUFFIX); break;
			case ROOM_MODE_SOMEBODY_IN : strcat(line, IN_MODE_SUFFIX); break;
			case ROOM_MODE_CHANGE : strcat(line, GO_MODE_SUFFIX); break;
			default: strcat(line, "   ");
		}
		
		LCDClear();
		
		// выводим на ЖК первую строчку
		LCDWriteStringXY(0, 0, line);
	
		memset(line, 0, sizeof(line));
	
		if (CHECK_LIGHT_ON) strcat(line, "L"); else strcat(line, " ");
		if (CHECK_VENT_ON) strcat(line, "V"); else strcat(line, " ");
		if (CHECK_MISIC_ON) strcat(line, "M"); else strcat(line, " ");
	
		if ((EventRegister>>MOVEMENT_EVENT)&1) strcat(line, MOTION_SUFFIX); else strcat(line, "        "); 
		if ((EventRegister>>DOOR_EVENT)&1) strcat(line, DOOR_SUFFIX); else strcat(line, "     "); 
			
		// выводим на ЖК вторую строчку
		LCDWriteStringXY(0, 1, line);				
	}
}


// инициализация предустановленного значения влажности использую EEPROM память
void initHumidityValue()
{
	// считываем значение установленной влажности с EEPROM
	HumiditySettings = eeprom_read_byte(&HumiditySettingsAddr);
	if (HumiditySettings < MIN_HUMIDITY_VALUE || HumiditySettings > MAX_HUMIDITY_VALUE)
	{
		HumiditySettings = DEFAULT_HUMIDITY_VALUE;
	}
}

// проверка необоходимости сохранение установленного значения влажсности в EEPROM
void humiditySettingsUpdate()
{			
	if (HumidityEepromUpdateCounter > HUMIDITY_EEPROM_TIME)
	{
		eeprom_update_byte(&HumiditySettingsAddr, HumiditySettings);
		HumidityEepromUpdateCounter = 0;
	}
}

// обновить значения влажности воздуха
void updateHumidityValue()
{
	// читаем датчик влажности, в ответ получаем значение влажности и статус чтения : ОК или ошибка
	uint16_t rawHumidityValue;
	// читаем датчик влажности	
	uint8_t error = am2302(&rawHumidityValue, &CurrentTemperatureValue);
	
	// в случае ошибки чек суммы даем шанс считать данные повторно
	if (error == CHECKSUM_ERROR) {
		error = am2302(&rawHumidityValue, &CurrentTemperatureValue);
	}
	
	if (error) {
		CurrentHumiditySensorStatus = HUMIDITY_SENSOR_FAIL; // произошла ошибка чтения датчика
		CurrentHumiditySensorErrorCode = error;
	} else {
		CurrentHumiditySensorStatus = HUMIDITY_SENSOR_OK;
		// convert raw humidity value to humidity
		CurrentHumidityValue = (uint8_t) round(rawHumidityValue / 10); // round требует достаточно много памяти 350 байт, в случае оптимазации можно упростить: 
		//CurrentHumidityValue = rawHumidityValue / 10;
	}	
	
	// запрос на обновление ЖК
	SET_LCD_UPDATE_EVENT;			
}

// проверка работы вентиляции
void checkVentilation()
{	
	if (CHECK_VENT_ON) {						
		if (CurrentHumiditySensorStatus == HUMIDITY_SENSOR_OK) {
			if ((VentWorkingCounter >= VENT_MIN_WORKING_TIME) && (CurrentHumidityValue <= (HumiditySettings-HUMIDITY_HYSTERESIS))) {
				stopVentilation();
				// запрос на обновление ЖК
				SET_LCD_UPDATE_EVENT;
			}
		}
	} else {
		if (CurrentHumiditySensorStatus == HUMIDITY_SENSOR_OK) {			
			// влажность повышена
			if (CurrentHumidityValue >= HumiditySettings) {	
				// запустить вентилятор, если режим НЕ ОТСТУТСВИЕ и максимальная продолжительность работы вентилятора не достигнута
				if (VentWorkingCounter < VENT_MAX_WORKING_TIME || RoomMode != ROOM_MODE_NOBODY_IN) {
					startVentilation();
					// запрос на обновление ЖК
					SET_LCD_UPDATE_EVENT;
				}				
			}
		}
	}
			
	// проверка на необходимость остановки вентилятора
	if (VentWorkingCounter >= VENT_MAX_WORKING_TIME)
	{
		// вентилятор отработал свой рабочий лимит, теперь его необходимо остановить
		stopVentilation();
		// запрос на обновление ЖК
		SET_LCD_UPDATE_EVENT;
	}					
}

// проверка режимов комнаты согласно установленным интервалам
void checkRoomModesAndIntervals()
{	
	if (RoomMode == ROOM_MODE_CHANGE)
	{
		// проверка по таймеру на активность за прошедшее время
		uint16_t maxTimeForMotionWaiting = MAX_TIME_FOR_MOTION_WAITING_DOOR_CLOSED;
		if (CHECK_DOOR_OPENED)
		{
			maxTimeForMotionWaiting = MAX_TIME_FOR_MOTION_WAITING_DOOR_OPENNED;
		}
		if (LastActivityTimeCounter >= maxTimeForMotionWaiting)
		{
			// время по таймеру истекло, необходим перевод в режим ОТСУТСТВИЕ
			changeRoomMode(ROOM_MODE_NOBODY_IN);
		}
	}
	else if (RoomMode == ROOM_MODE_SOMEBODY_IN)
	{
		if (SomebodyInWithoutMotionDelayCounter > MAX_CHECKING_TIME_SINCE_LAST_MOTION)
		{
			// так и не было зафиксировано сигналов от датчика движения за установленое время, значит переводим комнату в режим ОТСУТСВИЯ
			changeRoomMode(ROOM_MODE_NOBODY_IN);
		}
	}
}


// вывод привествия и текущей версии программы на дислпее, используется при запуске устройства
void showHelloMessage()
{
	char line[17];
	char temp[4];
	memset(line, 0, sizeof(line));				
	
	// вывод приветсвия на ЖК
	LCDWriteStringXY(0,0, HELLO_MSG);
	// вывод версии
	itoa(APP_VERSION, temp, 10);
	strcpy(line, VERSION_PREFIX);
	strcat(line, temp);
	LCDWriteStringXY(0,1, line);
}

// функция инициализации портов и других устройств
void initDevices()
{
	/************************************************************************/
	/* Инициализация портов ввода/вывода                                    */
	/************************************************************************/
	// вывода на управление реле устанавливаем в режим выход
	RELAY_DDR|=(1<<RELAY_1_PIN)|(1<<RELAY_2_PIN)|(1<<RELAY_3_PIN)|(1<<SOLID_RELAY_1_PIN)|(1<<SOLID_RELAY_2_PIN);
	// выводы работы с сенсором устанавливаем как вход с высоким импедансом
	MOV_SENSOR1_DDR&=~(1<<MOV_SENSOR1_PIN);
	MOV_SENSOR2_DDR&=~(1<<MOV_SENSOR2_PIN);
	DOOR_SENSOR_DDR&=~(1<<DOOR_SENSOR_PIN);
	// вывода для работы с кнопками также устанавливаем как вход с высоким импедансом
	BUTTON1_DDR&=~(1<<BUTTON1_PIN);
	BUTTON2_DDR&=~(1<<BUTTON2_PIN);
	
	
	//Initialize LCD module
	initLCD();
	
	/************************************************************************/
	/* Настройка прерываний по изменению сигнала на ножке                   */
	/************************************************************************/
	PCICR|=(1<<PCIE0)|(1<<PCIE2);
	PCMSK0|=(1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2);
	PCMSK2|=(1<<PCINT22)|(1<<PCINT23);
	PCIFR=0x0; //очистить флаги пререваний по маске
	
	// инициализация текущих состояний двери
	if (CHECK_DOOR_OPENED) {
		CommonState|=(1<<PCINT_STATE_DOOR);
	}	
	
	// отключить компаратор
	ACSR|=(1<<ACD);
	
	// настроить режим сна (idle mode)
	SMCR|=(1<<SE);//IDLE
	//SMCR|=(1<<SE)|(1<<SM0)|(1<<SM1); // power-save
}

/************************************************************************/
/* Настройка таймера                                                    */
/************************************************************************/
void initTimer() {
	// инициализация Timer2 с поддержкой асинхронного режима
	//TCCR2B|=(1<<CS20)|(1<<CS21)|(1<<CS22); // пределитель 1024
	//TCNT2=0x0;
	//TIMSK2|=(1<<TOIE2); // 8000000/1024/256 = 30,5 Гц
	
	TCCR1B|=(1<<CS10)|(1<<CS12)|(1<<WGM12); // пределитель 1024, и сбрасывать счетчик про совпадении регистра
	TCCR1A=0x0;
	OCR1A=7812;  // это значение дает задержку в 1 секунду, при частоте процессора 8 Мгц
	//разрешаем таймер 1 в режиме прерывания по сравнению
	TCNT1=0x0;
	// активация таймера
	TIMSK1|=(1<<OCIE1A);
}


// прерывания на ножках микроконтроллера
ISR(PCINT0_vect)
{
	// события двери
	int8_t result = checkPinIntterupt(&DOOR_SENSOR_PORT_PIN, DOOR_SENSOR_PIN, &CommonState, PCINT_STATE_DOOR);
	if (result != -1)
	{		
		onDoorEvent();
	}
	
	// проверка нажатия кнопок
	result = checkPinIntterupt(&BUTTON1_PORT_PIN, BUTTON1_PIN, &CommonState, PCINT_STATE_BUTTON1);
	if (result == 0)
	{
		if (HumiditySettings < MAX_HUMIDITY_VALUE) {
			HumiditySettings++;
			} else {
			HumiditySettings = MAX_HUMIDITY_VALUE;
		}
		// запрос на обновление ЖК
		SET_LCD_UPDATE_EVENT;
	}
	
	result = checkPinIntterupt(&BUTTON2_PORT_PIN, BUTTON2_PIN, &CommonState, PCINT_STATE_BUTTON2);
	if (result == 0)
	{		
		if (HumiditySettings > MIN_HUMIDITY_VALUE) {
			HumiditySettings--;
			} else {
			HumiditySettings = MIN_HUMIDITY_VALUE;
		}
		// запрос на обновление ЖК
		SET_LCD_UPDATE_EVENT;
	}		
}

// прерывания на ножках микроконтроллера
ISR(PCINT2_vect)
{
	// событие на движение
	int8_t result1 = checkPinIntterupt(&MOV_SENSOR1_PORT_PIN, MOV_SENSOR1_PIN, &CommonState, PCINT_STATE_MOTION1);
	int8_t result2 = checkPinIntterupt(&MOV_SENSOR2_PORT_PIN, MOV_SENSOR2_PIN, &CommonState, PCINT_STATE_MOTION2);
	
	// события от датчика движения номер 1
	if (result1 == 0 && MotionSensorDelayCounter_1 == 0)
	{
		onMovementRegister();
	}
	// события от датчика движения номер 2
	if (result2 == 0 && MotionSensorDelayCounter_2 == 0)
	{
		onMovementRegister();
	}
}

// вызывается каждые 2 секунды
void onEachTwoSeconds() 
{		
	// 2 закрытия двери втечение 2 секунд - команда на активироание интимного освещения
	if (IntimLightCommandCounter == 2)
	{						
		if(INITM_LIGHT_MODE_ON)
		{
			// активируем режим интимного освещения
			CommonState|=(1<<STATE_INTIM_LIGHT);
			updateIntimLightMode();
		}
	}
	
	IntimLightCommandCounter = 0;
}
// вызывается каждые 5 секунд
void onEachFiveSeconds()
{
	// возможно, здесь будет код
}

// срабатывает каждую секунду
void onEachSecond()
{
		// обновить интервалы комнаты
		if (RoomMode == ROOM_MODE_CHANGE)
		{
			LastActivityTimeCounter++;
		}
		else if (RoomMode == ROOM_MODE_SOMEBODY_IN)
		{
			SomebodyInTimeCounter++;
			SomebodyInWithoutMotionDelayCounter++;
		}
		
		if (CHECK_VENT_ON)
		{
			VentWorkingCounter++;
		}

		// счетчик обновления установоленной влажности в EEPROM
		HumidityEepromUpdateCounter++;
		
		if (HumidityUpdateCounter > HUMIDITY_UPDATE_TIMEOUT) {
			HumidityUpdateCounter = 0;
			updateHumidityValue();
		}

		// очистка событий с дисплея
		if (EventDisplayCounter > EVENT_DISPLAY_TIMEOUT) {
			EventDisplayCounter = 0;
			if ((EventRegister & (1<<DOOR_EVENT)) || (EventRegister & (1<<MOVEMENT_EVENT))) {
				CLEAR_DOOR_EVENT;
				CLEAR_MOVEMENT_EVENT;
				SET_LCD_UPDATE_EVENT;
			}
		}
		
		if (REINIT_LCD_IN_TIME > 0 && ReinitLcdCounter < REINIT_LCD_IN_TIME)
		{
			ReinitLcdCounter++;
		}
		
		updateMotionSensorCounter();
		
		HumidityUpdateCounter++;
		EventDisplayCounter++;				
}

// Прерывания по сравнению, таймер 1
ISR(TIMER1_COMPA_vect)
{
	onEachSecond();
	
	// прерывания на заданные промежутки времени
	TwoSecondsCounter++;
	FiveSecondsCounter++;
	
	if (TwoSecondsCounter >= 2) 
	{
		TwoSecondsCounter = 0;
		onEachTwoSeconds();		
	}
	if (FiveSecondsCounter >=5)
	{
		FiveSecondsCounter = 0;
		onEachFiveSeconds();				
	}
}

int main(void)
{		
	// инициализируем сторожевой таймер
	wdt_disable();
	// запрещаем прерывания на время инициализации
	cli();
			
	// пауза инициализации
	_delay_ms(100);
		 
	// инициализация
	initDevices();			
	showHelloMessage();	
	_delay_ms(3000);
	
	// инициализация сторожевого таймера
	wdt_enable(WDTO_2S);
	
	// инициализация 2
	initHumidityValue();
	updateHumidityValue();
	initTimer();
			
    while(1)
    {												
		// отключаем прерывания на время выполнения цикла
		cli();
		//------------------------------------------------------
		// проверка рабочих интервалов, согласно режиму комнаты
		checkRoomModesAndIntervals();				
		// обновить установленое значение влажности в EEPROM
		humiditySettingsUpdate();
		//------------------------------------------------------
		// включаем прерывания						
		sei();
		
		// проверка вентиляции
		checkVentilation();
		
		// обновляем ЖК, если это необходимо
		updateLCD();
		
		asm("sleep");
		
		// сбрасываем сторожевой таймер
		wdt_reset();
    }	
}