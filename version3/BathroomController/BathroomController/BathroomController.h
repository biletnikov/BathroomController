/*
 * BathroomController.h
 *
 * Created: 29.09.2018 21:26:23
 *  Author: sbiletnikov
 */ 


#ifndef BATHROOMCONTROLLER_H_
#define BATHROOMCONTROLLER_H_


#define ENABLED 1
#define DISABLED 0

#define YES 1
#define NO 0
#define UNKNOWN -1

#define ON 1
#define OFF 0

// Светодиоды
// Индикация статуса работы микроконтроллера
// если светодиод мигает, значит выполняется работы основной программы
#define LED_MCU_STATUS_DDR DDRB
#define LED_MCU_STATUS_PORT PORTB
#define LED_MCU_STATUS_PIN PINB5
#define LED_MCU_STATUS_TOGGLE LED_MCU_STATUS_PORT^=(1<<LED_MCU_STATUS_PIN);
#define LED_MCU_STATUS_SETUP LED_MCU_STATUS_DDR|=(1<<LED_MCU_STATUS_PIN);

// светодиод событие 1
#define LED_EVENT_1_DDR DDRD
#define LED_EVENT_1_PORT PORTD
#define LED_EVENT_1_PIN PIND2
#define LED_EVENT_1_ON LED_EVENT_1_PORT|=(1<<LED_EVENT_1_PIN);
#define LED_EVENT_1_OFF LED_EVENT_1_PORT&=~(1<<LED_EVENT_1_PIN);
#define LED_EVENT_1_TOGGLE LED_EVENT_1_PORT^=(1<<LED_EVENT_1_PIN);
#define LED_EVENT_1_SETUP LED_EVENT_1_DDR|=(1<<LED_EVENT_1_PIN);

// светодиод событие 2
#define LED_EVENT_2_DDR DDRD
#define LED_EVENT_2_PORT PORTD
#define LED_EVENT_2_PIN PIND3
#define LED_EVENT_2_ON LED_EVENT_2_PORT|=(1<<LED_EVENT_2_PIN);
#define LED_EVENT_2_OFF LED_EVENT_2_PORT&=~(1<<LED_EVENT_2_PIN);
#define LED_EVENT_2_TOGGLE LED_EVENT_2_PORT^=(1<<LED_EVENT_2_PIN);
#define LED_EVENT_2_SETUP LED_EVENT_2_DDR|=(1<<LED_EVENT_2_PIN);

// светодиод событие 3
#define LED_EVENT_3_DDR DDRD
#define LED_EVENT_3_PORT PORTD
#define LED_EVENT_3_PIN PIND4
#define LED_EVENT_3_ON LED_EVENT_3_PORT|=(1<<LED_EVENT_3_PIN);
#define LED_EVENT_3_OFF LED_EVENT_3_PORT&=~(1<<LED_EVENT_3_PIN);
#define LED_EVENT_3_TOGGLE LED_EVENT_3_PORT^=(1<<LED_EVENT_3_PIN);
#define LED_EVENT_3_SETUP LED_EVENT_3_DDR|=(1<<LED_EVENT_3_PIN);


// Назначение устройств на модули реле
//
// основное освещение
#define MAIN_LIGHT_PIN_INDX OUT_0_INDEX
// дополинтельное освещение
#define SEC_LIGHT_PIN_INDX OUT_1_INDEX
// вентиляция
#define VENT_PIN_INDX OUT_2_INDEX
// музыка
#define MUSIC_PIN_INDX OUT_3_INDEX


// сенсор влажности воздуха настраивается в am2302.h


// датчик движения 1 (установливается ближе к входной двери)
#define MOV_SENSOR1_PORT PORTC
#define MOV_SENSOR1_DDR DDRC
#define MOV_SENSOR1_PIN PC2
#define MOV_SENSOR1_PORT_PIN PINC

// датчик движения 2
#define MOV_SENSOR2_PORT PORTC
#define MOV_SENSOR2_DDR DDRC
#define MOV_SENSOR2_PIN PC1
#define MOV_SENSOR2_PORT_PIN PINC

// магнитный датчик двери
#define DOOR_SENSOR_PORT PORTC
#define DOOR_SENSOR_PORT_PIN PINC
#define DOOR_SENSOR_DDR DDRC
#define DOOR_SENSOR_PIN PC3


// значение влажности при котором включается и выключается вентилятор по умолчанию
#define DEFAULT_HUMIDITY_VENT_OFF_VALUE 75
#define DEFAULT_HUMIDITY_VENT_ON_VALUE 80
// установка относительной влажности
#define HUMIDITY_MAX_VALUE 100
#define HUMIDITY_MIN_VALUE 50

// максимальное время работы вентилятора в минутах
#define MAX_VENT_TIME_MAX_VALUE 255
#define MAX_VENT_TIME_MIN_VALUE 1
#define MAX_VENT_TIME_DEFAULT 30
// минимальное время работы вентилятора в минутах
#define MIN_VENT_TIME_MAX_VALUE 255
#define MIN_VENT_TIME_MIN_VALUE 1
#define MIN_VENT_TIME_DEFAULT 2
// значения для минимальной температуры при котором разрешён запуск вентилятора
#define MIN_VENT_TEMP_MAX_VALUE 30
#define MIN_VENT_TEMP_MIN_VALUE 0
#define MIN_VENT_TEMP_DEFAULT 18 //18 C
#define VENT_AFTER_DEFAULT NO
#define VENT_ENABLED_DEFAULT YES

#define MUSIC_ENABLED_DEFAULT YES

// как долго контроллер предполагает работу режима INVESTIGATION, если нет активности с сенсоров, в минутах
#define INVESTIGATION_WAIT_TIME_MAX_VALUE 250
#define INVESTIGATION_WAIT_TIME_MIN_VALUE 1
#define INVESTIGATION_WAIT_TIME_DEFAULT 10

// как долго контроллер предполагает наличие внутри человека, если нет активности с сенсоров, в минутах
#define SOMEBODY_IN_WAIT_TIME_MAX_VALUE 250
#define SOMEBODY_IN_WAIT_TIME_MIN_VALUE 1
#define SOMEBODY_IN_WAIT_TIME_DEFAULT 10

// каждые 5 минут сохранение установленной влажности в EEPROM
#define HUMIDITY_EEPROM_TIME 300

// время задержки до первой регистрации события от датчика движения после события от двери, для того чтобы устранить проблему срабатывания датчика движения при быстром закрытии дверь уходящим человеком
#define MOTION_SENSOR_DELAY_1 2
#define MOTION_SENSOR_DELAY_2 1

#define LIGHT_MAIN_ENABLED_DEFAULT YES
#define LIGHT_SEC_ENABLED_DEFAULT YES
#define LIGHT_RELAX_ENABLED_DEFAULT NO


#endif /* BATHROOMCONTROLLER_H_ */