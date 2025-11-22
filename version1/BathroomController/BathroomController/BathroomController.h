/*
 * BathroomController.h
 *
 */ 


#ifndef BATHROOMCONTROLLER_H_
#define BATHROOMCONTROLLER_H_


// порт управления реле
#define RELAY_PORT PORTD
#define RELAY_DDR DDRD
// электромагнитные реле
#define RELAY_1_PIN PD0
#define RELAY_2_PIN PD1
#define RELAY_3_PIN PD2
#define RELAY_PORT_PIN PIND
// твердотельные реле
#define SOLID_RELAY_1_PIN PD3
#define SOLID_RELAY_2_PIN PD4
#define SOLID_RELAY_PORT_PIN PIND
// сенсор влажности воздуха
#define HUMIDITY_SENSOR_PORT PORTD
#define HUMIDITY_SENSOR_DDR DDRD
#define HUMIDITY_SENSOR_PIN PD5

// датчик движения 1 (установливается ближе к входной двери)
#define MOV_SENSOR1_PORT PORTD
#define MOV_SENSOR1_DDR DDRD
#define MOV_SENSOR1_PIN PD6
#define MOV_SENSOR1_PORT_PIN PIND

// датчик движения 2 
#define MOV_SENSOR2_PORT PORTD
#define MOV_SENSOR2_DDR DDRD
#define MOV_SENSOR2_PIN PD7
#define MOV_SENSOR2_PORT_PIN PIND

// магнитный датчик двери
#define DOOR_SENSOR_PORT PORTB
#define DOOR_SENSOR_PORT_PIN PINB
#define DOOR_SENSOR_DDR DDRB
#define DOOR_SENSOR_PIN PB0

// кнопка 1
#define BUTTON1_PORT PORTB
#define BUTTON1_PORT_PIN PINB
#define BUTTON1_DDR DDRB
#define BUTTON1_PIN PB2

// кнопка 2
#define BUTTON2_PORT PORTB
#define BUTTON2_PORT_PIN PINB
#define BUTTON2_DDR DDRB
#define BUTTON2_PIN PB1

// значение влажности по умолчанию
#define DEFAULT_HUMIDITY_VALUE 75

// время ожидания регистрация движения, после того, как дверь была открыта/закрыта
#define MAX_TIME_FOR_MOTION_WAITING_DOOR_CLOSED 180 // 3 минуты
#define MAX_TIME_FOR_MOTION_WAITING_DOOR_OPENNED 720 // 12 минут
// максимальное время ожидания следующей регистрации движения, с момента прошлой
#define MAX_CHECKING_TIME_SINCE_LAST_MOTION 3600 // 1 час
// максимальное время работы вентилятора 
#define VENT_MAX_WORKING_TIME 1200 // 20 мин
// минимальное время работы вентилятора после его запуска, это обусловлено тем, 
// что при работе вентилятора могут меняться потоки воздуха и влажность может на мгновение резко упасть
#define VENT_MIN_WORKING_TIME 90 // 1.5 минуты
// гистерезис датчика влажности, вентилятор будет работать при достижении заданной влажности до того как она не снизиться на значение гистерезиса
#define HUMIDITY_HYSTERESIS 5
// каждые 5 минут сохранение установленной влажности в EEPROM
#define HUMIDITY_EEPROM_TIME 300
// делать переинициализацию ЖК через каждое установленное время, если 0 , то переинициализация не будет происходить
#define REINIT_LCD_IN_TIME 3600 // каждый час
// режим приглушенного освещения, если 1, то разрешён режим интим, 0 запрещён
#define INITM_LIGHT_MODE_ON 1
// время задержки до первой регистрации события от датчика движения после события от двери, для того чтобы устранить проблему срабатывания датчика движения при быстром закрытии дверь уходящим человеком
#define MOTION_SENSOR_DELAY_1 2
#define MOTION_SENSOR_DELAY_2 1


#endif /* BATHROOMCONTROLLER_H_ */