/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "PLLDriver.h"
#include <stdint.h>
#include <stdio.h>
#include "AdcDriver.h"
#include "PLLDriver.h"
#include "string.h"
#include "PwmDriver.h"
#include <math.h>
#include "stdlib.h"

//pc7

#include "ExtiDriver.h"

void configPeripherals(void);
void parseCommands(char *ptrBufferReception);
void adjustPWMWithinRange(PWM_Handler_t *pwmHandler, int targetDutty,
		int diffPWM);
void logData(double adjustTheta, double adjustSpeed);

double integralTheta = 0.0;  // Integral accumulator for angle
double lastThetaError = 0.0; // Last theta error for derivative calculation

double integralSpeed = 0.0;  // Integral accumulator for speed
double lastSpeedError = 0.0; // Last speed error for derivative calculation

double Kp_theta = 0.0;//918.5; // Proportional gain for angle
double Ki_theta = 0.0;//.5;    // Integral gain for angle
double Kd_theta = 0.0;//362.5;   // Derivative gain for angle

double Kp_speed = 0.18;    // Proportional gain for speed
double Ki_speed = 0.03;     // Integral gain for speed
double Kd_speed = 0.01;     // Derivative gain for speed

//Handler para el control de la terminal
USART_Handler_t handlerTerminal = { 0 };
USART_Handler_t handlerConexion = { 0 };
//Para la recepcion de datos por la terminal
uint8_t rxData = 0;
uint8_t rxDataU1 = 0;
uint16_t adcLastData = 0;
uint16_t countTimer = 0;

uint64_t counterTimer2 = 0;
uint64_t lastBlue = 0;
uint64_t lastYlw = 0;

uint16_t counterTimer2R = 0;
uint16_t counterReception = 0;

char bufferReception[20] = { 0 };
char cmd[64] = { 0 };
char bufferData[250] = { 0 };
unsigned int firstParameter = 0;
unsigned int secondParameter = 0;
unsigned int thirdParameter = 0;

bool stringComplete = 0;
bool printData = 0;
GPIO_Handler_t tx2pin = { 0 };	//Pin para configurar la trasmision del USART2
GPIO_Handler_t rx2pin = { 0 };	//Pin para configurar la recepcion del USART2

#define STACK_SIZE 200;
#define start 100
float start2 = 99.0f;
void vTaskOne(void *pvParameters);
void vTaskTwo(void *pvParameters);
void configPeripherals(void);
void taskCreation(void);
double getAverage(uint16_t arr[], int size);
#define sizeArrays (40*4) -1
GPIO_Handler_t ledUsuario;
GPIO_Handler_t pwprueba;
int clamp(int value, int min, int max);
void updateMotorControl();

/*Definición de variables del sistema*/

uint8_t globalCounter = 0;
uint32_t SystemCoreClock = 16E6;

BasicTimer_Handler_t handlerTimer3;
BasicTimer_Handler_t handlerTimer2;

PWM_Handler_t pwmBlue = { 0 }; //Para configurar el PWM en el timer 3 para X
bool dirYellowVal = 0;
PWM_Handler_t pwmYellow = { 0 }; //Para configurar el PWM en el timer 3 para X
bool dirBlueVal = 0;

//Funcion para cuadrar el ADC
ADC_Config_t channnel_0 = { 0 };

GPIO_Handler_t PC7;
EXTI_Config_t PC7E;
int PC7Counter = 0;

GPIO_Handler_t counterYellow;
EXTI_Config_t counterYellowE;
int counterBlueCounter = 0;
int counterBlueCounterT = 0;
int counterBlueLastIns = 0;
int counterYellowLastIns = 0;

double printcounterBlue = 0;
uint16_t timeBlue = 0;

int lastValBlue;
int lastValYwl;

GPIO_Handler_t counterYwl;
EXTI_Config_t counterYwlE;
int counterYwlCounter = 0;
int counterYwlCounterT = 0;
int counterYwlCounterL[sizeArrays] = { 0 };
double printcounterYwl = 0;
uint16_t timeYwl = 0;

GPIO_Handler_t PC0;
EXTI_Config_t PC0E;
int PC0Counter = 0;

uint16_t prescaler = 400;
uint16_t periodo = 10000;
uint16_t duttyInBlue = 3237; //250  //350 32.370 30.190
uint16_t duttyIYwl = 3015; //3780; //264//364

uint16_t duttyGiroBlue = 3237; //250  //350
uint16_t duttyGiroYwl = 3015; //264//364

uint16_t duttyBlue = start;
uint16_t duttyYwl = start;
uint16_t distancia = 300;

bool upMode = 0;

GPIO_Handler_t dirPinYw;
GPIO_Handler_t dirPinBlue;

bool dir = 0;
bool moves = 0;
bool move90 = 0;
bool onMove = 0;

float recorridoBlue;
float recorridoYellow;

// Assuming these are global variables representing the robot's current state
double x = 0.0, y = 0.0, theta = 0.0; // Position (x, y) and orientation theta
float wheelbase = 10.359; // Distance between wheels
uint16_t counter10seg = 0;
uint8_t counter1seg = 0;
float wheelsize = 5.12;
float rotate = M_PI / 2;
uint8_t vueltasMsg = 1;
uint8_t modoVueltas = 0;

uint16_t timeBlueArray[100] = { 0 };
uint16_t timeYellowArray[100] = { 0 };
float timeBlueAvg, timeYwlAvg = 0;

bool newMoveBlue, newMoveYellow;
bool calibrate = 0;
bool initialCalibrate = 1;
int8_t dirIncrese = 1;
uint8_t vueltas = 1;
uint8_t vueltasYellow = 120;
uint8_t vueltasBlue = 120;
uint8_t counter10avg, counter250ms = 0;

float avgBlue, avgYwl = 0;
void updatePosition(void);
void stop(void);
double deltaTheta = 0;

int main(void) {
	//yellow 0, blue 1

	configPeripherals();

	/* Loop forever */
	while (1) {
		/* SI llegamos es que algo salio mal... */
		/* El caracter '@' nos indica que es el final de la cadena*/

		if (rxDataU1 != '\0') {

			if (rxDataU1 == 30) {
				dirIncrese = 1;
				rxDataU1 = 0;
			} else if (rxDataU1 == 31) {
				dirIncrese = -1;
				rxDataU1 = 0;
			} else if (rxDataU1 == 28) {

				duttyInBlue += 1 * dirIncrese;
				pwmBlue.config.duttyCicle = duttyInBlue;
				setDuttyCycle(&pwmBlue);
				rxDataU1 = 0;
			} else if (rxDataU1 == 29) {
				duttyIYwl += 1 * dirIncrese;
				pwmYellow.config.duttyCicle = duttyIYwl;
				setDuttyCycle(&pwmYellow);
				rxDataU1 = 0;
			}
//			writeChar(&handlerConexion, rxDataU1);
			bufferReception[counterReception] = rxDataU1;
			counterReception++;

			// If the incoming character is a newline, set a flag
			// so the main loop can do something about it
			if (rxDataU1 == '@') {
				stringComplete = 1;
				sprintf(bufferData, "\n");
				writeString(&handlerTerminal, bufferData);
				//Agrego esta linea para crear el string con el null al final
				bufferReception[counterReception] = '\0';
				counterReception = 0;
			}
			if (rxDataU1 == '\b') {
				counterReception--;
				counterReception--;
			}
			//Para que no vuelva entrar. Solo cambia debido a la interrupcion
			rxDataU1 = '\0';
		}

		//Hacemos un analisis de la cadena de datos obtenida
		if (stringComplete) {
			parseCommands(bufferReception);
			writeChar(&handlerConexion, '\n');
			stringComplete = 0;
			for (int i = 0;
					i < sizeof(bufferReception) / sizeof(bufferReception[0]);
					i++) {
				bufferReception[i] = 0;
			}
		}
	}
}

void configPeripherals(void) {

	//Activacion cooprocesador matematico(importante para esta tarea)
	SCB->CPACR |= (0xF << 20);

	configPLL(100);

	ledUsuario.pGPIOx = GPIOC;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_OUT;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinNumber = PIN_5;
	GPIO_Config(&ledUsuario);

	dirPinYw.pGPIOx = GPIOD;
	dirPinYw.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_OUT;
	dirPinYw.GPIO_PinConfig_t.GPIO_PinNumber = PIN_2;
	GPIO_Config(&dirPinYw);
	GPIO_WritePin(&dirPinYw, dirYellowVal);

	dirPinBlue.pGPIOx = GPIOC;
	dirPinBlue.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_OUT;
	dirPinBlue.GPIO_PinConfig_t.GPIO_PinNumber = PIN_12;
	GPIO_Config(&dirPinBlue);
	GPIO_WritePin(&dirPinBlue, dirBlueVal);

	//Pines necesarios para el uso del USART2
	tx2pin.pGPIOx = GPIOA;
	tx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_2;
	tx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	tx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	tx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; //Se usa en velocidad rapida
	tx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7;

	GPIO_Config(&tx2pin);

	rx2pin.pGPIOx = GPIOA;
	rx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_3;
	rx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	rx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	rx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	rx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7;

	GPIO_Config(&rx2pin);

	handlerTerminal.ptrUSARTx = USART2;
	handlerTerminal.USART_Config.USART_baudrate = 115200;
	handlerTerminal.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
	handlerTerminal.USART_Config.USART_mode = USART_MODE_RXTX;
	handlerTerminal.USART_Config.USART_parity = USART_PARITY_NONE;
	handlerTerminal.USART_Config.USART_stopbits = USART_STOPBIT_1;
	handlerTerminal.USART_Config.USART_RX_Int_Ena = ENABLE;

	USART_Config(&handlerTerminal);

	tx2pin.pGPIOx = GPIOA;
	tx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_9;
	tx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	tx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	tx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; //Se usa en velocidad rapida
	tx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7;

	GPIO_Config(&tx2pin);

	rx2pin.pGPIOx = GPIOA;
	rx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_10;
	rx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	rx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	rx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	rx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7;

	GPIO_Config(&rx2pin);

	handlerConexion.ptrUSARTx = USART1;
	handlerConexion.USART_Config.USART_baudrate = 19200;
	handlerConexion.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
	handlerConexion.USART_Config.USART_mode = USART_MODE_RXTX;
	handlerConexion.USART_Config.USART_parity = USART_PARITY_NONE;
	handlerConexion.USART_Config.USART_stopbits = USART_STOPBIT_1;
	handlerConexion.USART_Config.USART_RX_Int_Ena = ENABLE;

	USART_Config(&handlerConexion);

	PC7.pGPIOx = GPIOC;
	PC7.GPIO_PinConfig_t.GPIO_PinNumber = PIN_7;
	PC7.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IN;

	PC7E.pGPIOHandler = &PC7;
	PC7E.edgeType = EXTERNAL_INTERRUPT_BOTH_EDGE;

	extInt_Config(&PC7E);

	counterYellow.pGPIOx = GPIOC;
	counterYellow.GPIO_PinConfig_t.GPIO_PinNumber = PIN_1;
	counterYellow.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IN;

	counterYellowE.pGPIOHandler = &counterYellow;
	counterYellowE.edgeType = EXTERNAL_INTERRUPT_BOTH_EDGE;

	extInt_Config(&counterYellowE);

	counterYwl.pGPIOx = GPIOC;
	counterYwl.GPIO_PinConfig_t.GPIO_PinNumber = PIN_3;
	counterYwl.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IN;

	counterYwlE.pGPIOHandler = &counterYwl;
	counterYwlE.edgeType = EXTERNAL_INTERRUPT_BOTH_EDGE;

	extInt_Config(&counterYwlE);

	PC0.pGPIOx = GPIOC;
	PC0.GPIO_PinConfig_t.GPIO_PinNumber = PIN_0;
	PC0.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_IN;

	PC0E.pGPIOHandler = &PC0;
	PC0E.edgeType = EXTERNAL_INTERRUPT_BOTH_EDGE;

	extInt_Config(&PC0E);

	pwprueba.pGPIOx = GPIOA;
	pwprueba.GPIO_PinConfig_t.GPIO_PinNumber = PIN_0;
	pwprueba.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	pwprueba.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	pwprueba.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	pwprueba.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	pwprueba.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF2;

	GPIO_Config(&pwprueba);

	pwprueba.pGPIOx = GPIOA;
	pwprueba.GPIO_PinConfig_t.GPIO_PinNumber = PIN_1;
	pwprueba.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN;
	pwprueba.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	pwprueba.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	pwprueba.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	pwprueba.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF2;

	GPIO_Config(&pwprueba);

	pwmYellow.ptrTIMx = TIM5;
	pwmYellow.config.channel = PWM_CHANNEL_2;
	pwmYellow.config.duttyCicle = duttyYwl;
	pwmYellow.config.periodo = periodo;
	pwmYellow.config.prescaler = prescaler;
	pwmYellow.config.polarity = dirYellowVal;

	pwm_Config(&pwmYellow);
	enableOutput(&pwmYellow);
	startPwmSignal(&pwmYellow);

	pwmBlue.ptrTIMx = TIM5;
	pwmBlue.config.channel = PWM_CHANNEL_1;
	pwmBlue.config.duttyCicle = duttyBlue;
	pwmBlue.config.periodo = periodo;
	pwmBlue.config.prescaler = prescaler;
	pwmBlue.config.polarity = dirBlueVal;

	pwm_Config(&pwmBlue);
	enableOutput(&pwmBlue);
	startPwmSignal(&pwmBlue);

	handlerTimer3.ptrTIMx = TIM3; //El timer que se va a usar
	handlerTimer3.TIMx_Config.TIMx_interruptEnable = 1; //Se habilitan las interrupciones
	handlerTimer3.TIMx_Config.TIMx_mode = BTIMER_MODE_UP; //Se usara en modo ascendente
	handlerTimer3.TIMx_Config.TIMx_period = 500; //Se define el periodo en este caso el led cambiara cada 250ms
	handlerTimer3.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us; //Se define la "velocidad" que se usara

	BasicTimer_Config(&handlerTimer3); //Se carga la configuración.

	handlerTimer2.ptrTIMx = TIM2; //El timer que se va a usar
	handlerTimer2.TIMx_Config.TIMx_interruptEnable = 1; //Se habilitan las interrupciones
	handlerTimer2.TIMx_Config.TIMx_mode = BTIMER_MODE_UP; //Se usara en modo ascendente
	handlerTimer2.TIMx_Config.TIMx_period = 250; //Se define el periodo en este caso el led cambiara cada 250ms
	handlerTimer2.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us; //Se define la "velocidad" que se usara

	BasicTimer_Config(&handlerTimer2); //Se carga la configuración.

}

void parseCommands(char *ptrBufferReception) {

	/* Lee la cadena de caracteres a la que apunta el "ptrBufferReception
	 * y almacena en tres elementos diferentes: un string llamado "cmd",
	 * y dos integer llamados "firstParameter" y "secondParameter"
	 * De esta forma podemos introducir informacion al micro desde el puerto
	 */
	sscanf(ptrBufferReception, "%s %u %u %u", cmd, &firstParameter,
			&secondParameter, &thirdParameter);
	if (strcmp(cmd, "help") == 0) {
		writeString(&handlerTerminal, "Help Menu CMDs: \n");
		writeString(&handlerTerminal, "1)  Help -> Print this menu \n");

	}

	else if (strcmp(cmd, "dir") == 0) {
		if (firstParameter == 0) {
			dirBlueVal = secondParameter;
			pwmUpdatePolarity(&pwmYellow, secondParameter);
			GPIO_WritePin(&dirPinYw, secondParameter);
		} else {
			dirYellowVal = secondParameter;
			pwmUpdatePolarity(&pwmBlue, secondParameter);
			GPIO_WritePin(&dirPinBlue, secondParameter);
		}
	}

	else if (strcmp(cmd, "pwm") == 0) {
		if (firstParameter == 0) {
			duttyIYwl = secondParameter;
			pwmYellow.config.duttyCicle = duttyIYwl;
			setDuttyCycle(&pwmYellow);
		} else if (firstParameter == 1) {
			duttyInBlue = secondParameter;
			pwmBlue.config.duttyCicle = duttyInBlue;
			setDuttyCycle(&pwmBlue);
		} else {
			pwmYellow.config.duttyCicle = secondParameter;
			pwmBlue.config.duttyCicle = secondParameter;
			setDuttyCycle(&pwmYellow);
			setDuttyCycle(&pwmBlue);
		}

	} else if (strcmp(cmd, "upMode") == 0) {
		upMode = firstParameter;
	} else if (strcmp(cmd, "reset") == 0) {
		x = 0;
		theta = 0;
		y = 0;
	} else if (strcmp(cmd, "rotate") == 0) {
		if (firstParameter != 0) {
			rotate = firstParameter / 90 * (M_PI / 2);
		}
		move90 = 1;
		theta = 0;
		x = 0;
		y = 0;
		pwmBlue.config.duttyCicle = duttyGiroBlue;
		setDuttyCycle(&pwmBlue);
		pwmYellow.config.duttyCicle = duttyGiroYwl;
		setDuttyCycle(&pwmYellow);
		dirBlueVal = 1;
		pwmUpdatePolarity(&pwmBlue, dirBlueVal);
		GPIO_WritePin(&dirPinBlue, dirBlueVal);
		dirYellowVal = 0;
		pwmUpdatePolarity(&pwmYellow, dirYellowVal);
		GPIO_WritePin(&dirPinYw, dirYellowVal);
		newMoveBlue = 1;
		newMoveYellow = 1;
		onMove = 1;
	} else if (strcmp(cmd, "movex") == 0) {
		if (firstParameter != 0) {
			distancia = firstParameter;
		}
		moves = 1;
		theta = 0;
		x = 0;
		y = 0;
		pwmBlue.config.duttyCicle = duttyInBlue;
		setDuttyCycle(&pwmBlue);
		pwmYellow.config.duttyCicle = duttyIYwl;
		setDuttyCycle(&pwmYellow);
		dirBlueVal = 0;
		pwmUpdatePolarity(&pwmBlue, dirBlueVal);
		GPIO_WritePin(&dirPinBlue, dirBlueVal);
		dirYellowVal = 0;
		pwmUpdatePolarity(&pwmYellow, dirYellowVal);
		GPIO_WritePin(&dirPinYw, dirYellowVal);
		onMove = 1;
		newMoveBlue = 1;
		newMoveYellow = 1;
		counterYwlCounterT = 0;
		counterBlueCounterT = 0;

	}

	else if (strcmp(cmd, "wheel") == 0) {
		wheelbase = firstParameter / 10.0f;
		sprintf(bufferData, "Wheel = %.2f", wheelbase);
		writeString(&handlerConexion, bufferData);
	}

	else if (strcmp(cmd, "wheelSize") == 0) {
		wheelbase = firstParameter / 100.0f;
		sprintf(bufferData, "Wheel = %.2f", wheelbase);
		writeString(&handlerConexion, bufferData);
	}

	else if (strcmp(cmd, "vueltas") == 0) {
		modoVueltas = firstParameter;
		vueltasMsg = secondParameter;
		pwmBlue.config.duttyCicle = duttyInBlue;
		setDuttyCycle(&pwmBlue);
		pwmYellow.config.duttyCicle = duttyIYwl;
		setDuttyCycle(&pwmYellow);
		dirBlueVal = 0;
		pwmUpdatePolarity(&pwmBlue, dirBlueVal);
		GPIO_WritePin(&dirPinBlue, dirBlueVal);
		dirYellowVal = 0;
		pwmUpdatePolarity(&pwmYellow, dirYellowVal);
		GPIO_WritePin(&dirPinYw, dirYellowVal);
		onMove = 1;
	}

	else if (strcmp(cmd, "theta") == 0) {
		Kp_theta = firstParameter;
		Ki_theta = secondParameter;
		Kd_theta = thirdParameter;
	} else if (strcmp(cmd, "speed") == 0) {
		Kp_speed = firstParameter;
		Ki_speed = secondParameter;
		Kd_speed = thirdParameter;
	}

	else if (strcmp(cmd, "calibrate") == 0) {
		initialCalibrate = firstParameter;
	}

	else if (strcmp(cmd, "stop") == 0) {
		stop();
	}

	else {
		// Se imprime el mensaje "Wrong CMD" si la escritura no corresponde a los CMD implementados
		writeString(&handlerConexion, "Wrong CMD \n");
	}
	firstParameter = 0;
	secondParameter = 0;
	thirdParameter = 0;

}
void BasicTimer2_Callback(void) {

	counterTimer2++;
	if (modoVueltas == 0 && move90 == 0 && onMove) {
		updatePosition();
		counterBlueLastIns = counterBlueCounter;
		counterYellowLastIns = counterYwlCounter;
		counterYwlCounter = 0;
		counterBlueCounter = 0;
	}
	if (move90) {
		updatePosition();
		int aux = 61;
		if (counterYwlCounter > aux || counterBlueCounter > aux) {
			counterYwlCounter = 0;
			counterBlueCounter = 0;
			stop();
			move90 = 0;
		}
	}

}
//Calback del timer3 para el blinking
void BasicTimer3_Callback(void) {
	counter250ms++;
	if (counter250ms >= 5) {
		GPIOxTooglePin(&ledUsuario);
		counter10seg++;
		counter250ms = 0;
	}

	avgBlue = (avgBlue * counter10avg + counterBlueCounter)
			/ (counter10avg + 1.0f);

	avgYwl = (avgYwl * counter10avg + counterYwlCounter)
			/ (counter10avg + 1.0f);
	counter10avg++;

	if (counter10avg >= 5 * 2 && onMove && initialCalibrate && moves) {
		updateMotorControl();  // Call the motor control update function
		counter10avg = 0;      // Reset the counter for the next period
	}

	if (counter10seg > 4 - 1 && onMove) {

		sprintf(bufferData,
				"%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%d\t%d\t%.1f\t%.1f\t%.1f\t%.4f\t%.4f\t%.4f\n",
				x / 100, y / 100, theta, (avgBlue), (avgYwl),
				pwmBlue.config.duttyCicle / 100.0f,
				pwmYellow.config.duttyCicle / 100.0f, (int) counterBlueCounterT,
				(int) counterYwlCounterT, Kp_theta, Ki_theta, Kd_theta,
				Kp_speed, Ki_speed, Kd_speed);
		writeString(&handlerConexion, bufferData);
		lastValBlue = avgBlue;
		lastValYwl = avgYwl;
		counter10seg = 0;
	}

}

/**
 * Adjust PWM settings within specified range and apply.
 */
void adjustPWMWithinRange(PWM_Handler_t *pwmHandler, int targetDutty,
		int diffPWM) {
	if (pwmHandler->config.duttyCicle >= targetDutty - diffPWM
			&& pwmHandler->config.duttyCicle <= targetDutty + diffPWM) {
		setDuttyCycle(pwmHandler);
	} else if (pwmHandler->config.duttyCicle < targetDutty - diffPWM) {
		pwmHandler->config.duttyCicle = targetDutty - diffPWM;
		setDuttyCycle(pwmHandler);
	} else {
		pwmHandler->config.duttyCicle = targetDutty + diffPWM;
		setDuttyCycle(pwmHandler);
	}
}

void USART1Rx_Callback(void) {
	rxDataU1 = (uint8_t) USART1->DR;
}

void callback_extInt0(void) {
	PC0Counter++;

}

void callback_extInt7(void) {

	PC7Counter++;
}

void callback_extInt1(void) {
	counterYwlCounterT++;
	counterYwlCounter++;
	if (modoVueltas == 1) {
		if (counterYwlCounter >= vueltasYellow * vueltasMsg) {
			counterYwlCounter = 0;
			stop();
			modoVueltas = 3;
		}
	}

}

void callback_extInt3(void) {
	counterBlueCounterT++;
	counterBlueCounter++;
	if (modoVueltas == 2) {
		if (counterBlueCounter >= vueltasBlue * vueltasMsg) {
			counterBlueCounter = 0;
			stop();
			modoVueltas = 3;
		}
	}

}

double getAverage(uint16_t arr[], int size) {
	int sum = 0;
	double average;

// Sum all elements in the array
	for (int i = 0; i < size; i++) {
		sum += arr[i];
	}

// Calculate average
	average = (double) sum / size;

	return average;
}

void updatePosition(void) {
	double dLeft, dRight, dCenter, deltaX, deltaY;

	recorridoBlue = (counterBlueCounter) * M_PI * 5.117 / vueltasBlue;
	recorridoYellow = (counterYwlCounter) * M_PI * 5.124 / vueltasYellow;

	// Convert directions into forward (1) or backward (-1) multipliers
	int multiplierBlue = (dirBlueVal == 0) ? 1 : -1;
	int multiplierYellow = (dirYellowVal == 0) ? 1 : -1;

	// Calculate distances traveled by each wheel, adjusted by direction
	dLeft = recorridoBlue * multiplierBlue;
	dRight = recorridoYellow * multiplierYellow;

	// Average distance traveled by the robot
	dCenter = (dLeft + dRight) / 2.0;
	deltaTheta = (dRight - dLeft) / wheelbase;
	// Calculate change in orientation

	theta += deltaTheta;

//	if ((theta >= rotate) && move90 == 1) {
//		stop();
//		move90 = 0;
//	}

	// Ensure theta stays within [-pi, pi]
	if (theta > M_PI)
		theta -= 2 * M_PI;
	else if (theta < -M_PI)
		theta += 2 * M_PI;

	// Calculate change in position
	deltaX = dCenter * cos(theta);
	deltaY = dCenter * sin(theta);

	// Update position
	x += deltaX;
	y += deltaY;

	if ((x > distancia) && moves == 1) {
		stop();
		moves = 0;
	}
}

void stop(void) {
	onMove = 0;
	pwmYellow.config.duttyCicle = start;
	pwmBlue.config.duttyCicle = start;
	setDuttyCycle(&pwmYellow);
	setDuttyCycle(&pwmBlue);
	counterBlueLastIns = 0;
	counterYellowLastIns = 0;
	integralTheta = 0;
	lastThetaError = 0.0;
	integralSpeed = 0.0;  // Integral accumulator for speed
	lastSpeedError = 0.0;
}

void updateMotorControl() {
	// Angle control
	double thetaError = theta; // assuming theta is updated elsewhere in the code
	double derivativeTheta = thetaError - lastThetaError;
	integralTheta += thetaError;

	double adjustTheta = (Kp_theta * thetaError) + (Ki_theta * integralTheta)
			+ (Kd_theta * derivativeTheta);

	// Speed control
	double speedError = counterYwlCounter - counterBlueCounter;
	double derivativeSpeed = speedError - lastSpeedError;
	integralSpeed += speedError;

	double adjustSpeed = (Kp_speed * speedError) + (Ki_speed * integralSpeed)
			+ (Kd_speed * derivativeSpeed);

	uint16_t limit = 500;

	uint16_t aux = clamp(
			pwmYellow.config.duttyCicle - (int) (adjustTheta + adjustSpeed),
			duttyIYwl - limit, duttyIYwl + limit);

	// Adjust PWM settings for both motors
	pwmYellow.config.duttyCicle = aux;
	aux = clamp(pwmBlue.config.duttyCicle + (int) (adjustTheta + adjustSpeed),
			duttyInBlue - limit, duttyInBlue + limit);
	pwmBlue.config.duttyCicle = aux;

	// Update PWM outputs
	setDuttyCycle(&pwmYellow);
	setDuttyCycle(&pwmBlue);

	// Store current errors for next cycle's derivative calculations
	lastThetaError = thetaError;
	lastSpeedError = speedError;
}

// Helper function to ensure PWM values are within specified limits
int clamp(int value, int min, int max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

