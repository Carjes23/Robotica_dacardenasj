/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Carjes
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

// Necessary libs
#include "GPIOxDriver.h" // GPIO driver library
#include "BasicTimer.h" // Basic timer library
#include "USARTxDriver.h" // USART driver library
#include "PLLDriver.h" // PLL driver library
#include <stdint.h> // Standard integer types
#include "AdcDriver.h" // ADC driver library
#include "PwmDriver.h" // PWM driver library
#include "string.h" // String manipulation library
#include <stdio.h> // Standard input/output library
#include <math.h> // Mathematical functions library
#include "stdlib.h" // Standard library
#include "I2CxDriver.h" // I2C driver library
#include "MPU6050.h" // MPU6050 driver library
#include "SysTick.h" // System tick library
#include "ExtiDriver.h" // EXTI driver library

// Constants
#define START 100 // Value to start the motors
#define YELLOW_DISTANCE_INT M_PI * 5.124 / 120
#define BLUE_DISTANCE_INT M_PI * 5.117 / 120

// System core clock
uint32_t SystemCoreClock = 16E6;

// Function headers
void configPeripherals(void);
void parseCommands(char *ptrBufferReception);
void configPeripherals(void);
int clamp(int value, int min, int max);
void updateMotorControl();
void getAccelMeasure(int16_t *array);
void rotateG(void);

// PID constants
double integralTheta = 0.0; // Integral accumulator for angle
double lastThetaError = 0.0; // Last theta error for derivative calculation

double Kp_theta = 34.00; // Proportional gain for angle
double Ki_theta = 0.08; // Integral gain for angle
double Kd_theta = 95.00; // Derivative gain for angle

// General handlers
GPIO_Handler_t ledUsuario; // LED user handler

// Handlers to control the terminal
USART_Handler_t handlerTerminal = { 0 }; // Terminal handler
USART_Handler_t handlerConexion = { 0 }; // Connection handler
GPIO_Handler_t tx2pin, rx2pin = { 0 }; // Pins to configure USART2

// Movement variables
GPIO_Handler_t pin_PWMConfig; // PWM configuration handler
BasicTimer_Handler_t handlerTimer3; // Timer 3 handler
BasicTimer_Handler_t handlerTimer2; // Timer 2 handler
PWM_Handler_t pwmBlue = { 0 }; // PWM blue handler
bool dirYellowVal = 0; // Yellow direction value
PWM_Handler_t pwmYellow = { 0 }; // PWM yellow handler
bool dirBlueVal = 0; // Blue direction value

// Other variables
uint16_t prescaler = 400;
uint16_t periodo = 10000;
uint16_t duttyInBlue = 3237; //250  //350 32.370 30.190
uint16_t duttyIYwl = 3015; //3780; //264//364



uint16_t duttyGiroBlue = 3237; // 250; //350
uint16_t duttyGiroYwl = 3015; // 264//364

uint16_t duttyBlue = START;
uint16_t duttyYwl = START;
uint16_t distancia = 300;

bool dir = 0;
bool moves = 0;
bool move90 = 0;
bool onMove = 0;

float recorridoBlue;
float recorridoYellow;

// Data reception and transmission terminal
uint8_t rxData = 0;
uint8_t rxDataU1 = 0;
uint16_t counterReception = 0;
char bufferReception[20] = { 0 };
char cmd[64] = { 0 };
char bufferData[250] = { 0 };
unsigned int firstParameter = 0;
unsigned int secondParameter = 0;
unsigned int thirdParameter = 0;
bool stringComplete = 0;
bool printData = 0;

bool upMode = 0;

GPIO_Handler_t dirPinYw;
GPIO_Handler_t dirPinBlue;

// Measure time
uint64_t counterTimer2 = 0;

// Interruptions variables
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

int lastIntYellow, lastIntBlue = 0;

GPIO_Handler_t counterYwl;
EXTI_Config_t counterYwlE;
int counterYwlCounter = 0;
int counterYwlCounterT = 0;
double printcounterYwl = 0;
uint16_t timeYwl = 0;

GPIO_Handler_t PC0;
EXTI_Config_t PC0E;
int PC0Counter = 0;

GPIO_Handler_t mpu6050_sdaPin = { 0 }; // Pin SDA of MPU6050
GPIO_Handler_t mpu6050_sclPin = { 0 }; // Pin SCL of MPU6050

// Handler for MPU6050 (sensor) using I2C
I2C_Handler_t i2c1_mpu6050 = { 0 };
float gyroZOffset = 0;
float accelX, accelY = 0;

// Global variables representing the robot's current state
double x = 0.0, y = 0.0, theta = 0.0; // Position (x, y) and orientation theta
float wheelbase = 10.359; // Distance between wheels
uint16_t counter10seg = 0;
uint8_t counter1seg = 0;
float wheelsize = 5.12;
float rotate = M_PI / 2;
uint8_t vueltasMsg = 1;
uint8_t modoVueltas = 0;
bool movesq = 0;

bool newMoveBlue, newMoveYellow;
bool calibrate = 0;
bool initialCalibrate = 1;
int8_t dirIncrese = 1;
uint8_t vueltas = 1;
uint8_t vueltasYellow = 120;
uint8_t vueltasBlue = 120;
uint8_t counter250ms = 0;
uint8_t accelActivate = 0;
double yellowInstantVelocity, blueInstantVelocity = 0;

//Accell
bool measure = 0;
int measureCount = 0;
float sumGyroOffset = 0;
float gyroZ = 0;
float accelXOffset = 0;
float accelYOffset = 0;
float sumAccelXOffset = 0;
float sumAccelYOffset = 0;
int accelMeasureCount = 0;
int16_t accelMeasurements[3] = { 0 };
uint8_t regsToRead[6] = { ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
ACCEL_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L };

float avgBlue, avgYwl = 0;
void updatePosition(void);
void stop(void);
double deltaTheta = 0;
double thetaG = 0;
double thetaObjective = 0;
bool move90G = 0;
uint8_t squareSide = 0;
bool enableCounter = 0;
uint8_t counter1SegundoPausa = 0;
bool banderaLed = 0;

void moveForward(void);

// Redirection of printf() to USART2
int __io_putchar(int ch) {
	// Send the character by USART1
	writeChar(&handlerConexion, (char) ch);
	return ch;
}

int main(void) {
	// Calibration mode: yellow = 0, blue = 1

	// Initialize peripherals
	configPeripherals();

	// Wait for 1 second
	delay_ms(1000);

	// Perform calibration of MPU6050
	i2c_writeSingleRegister(&i2c1_mpu6050, PWR_MGMT_1, 0x00);
	uint8_t mpuErr = i2c_readSingleRegister(&i2c1_mpu6050, PWR_MGMT_1);
	if (mpuErr) {
		printf("Error initializing MPU. Check connection\n");
		while (1)
			; // Loop forever on error
	} else {
		printf("MPU6050 initialized correctly\n");
		accelActivate = 1; // Set acceleration activation flag
	}

	/* Loop forever */
	while (1) {
		/* If we reach this point, something went wrong... */
		/* The '@' character indicates the end of the string */

		if (banderaLed) {
			GPIOxTooglePin(&ledUsuario);
			banderaLed = 0;
		}

		if (movesq) {
			uint8_t auxTime = 4;
			if (squareSide == 0 && x >= distancia) {
				squareSide++;
				stop();
				enableCounter = 1;
				counter1SegundoPausa = 0;

			} else if (squareSide == 1 && counter1SegundoPausa >= auxTime) {

				rotateG();
				enableCounter = 0;
				counter1SegundoPausa = 0;
			}

			else if (squareSide == 2 && counter1SegundoPausa >= auxTime) {
				counter1SegundoPausa = 0;
				moveForward();
				if (y >= distancia) {
					squareSide++;
					stop();
					enableCounter = 1;
				}
			} else if (squareSide == 3 && counter1SegundoPausa >= auxTime) {
				rotateG();
				enableCounter = 0;
				counter1SegundoPausa = 0;
			} else if (squareSide == 4 && counter1SegundoPausa >= auxTime) {
				counter1SegundoPausa = 0;
				moveForward();
				if (x <= 0) {
					squareSide++;
					stop();
					enableCounter = 1;
				}
			} else if (squareSide == 5 && counter1SegundoPausa >= auxTime) {
				rotateG();
				enableCounter = 0;
				counter1SegundoPausa = 0;
			} else if (squareSide == 6 && counter1SegundoPausa >= auxTime) {
				counter1SegundoPausa = 0;
				moveForward();
				if (y <= 0) {
					squareSide++;
					stop();
					enableCounter = 1;
					movesq = 0;
				}
			}
		}

		// Calibration mode
		if (accelActivate == 1) {
			if (measure) {
				float aux = 150.0f;
				if (measureCount < aux) {
					getAccelMeasure(accelMeasurements);
					sumAccelXOffset += accelX;
					sumAccelYOffset += accelY;

					sumGyroOffset += gyroZ;
					measureCount++;
				} else if (measureCount == aux) {
					accelXOffset += sumAccelXOffset / aux;
					accelYOffset += sumAccelYOffset / aux;

					gyroZOffset += sumGyroOffset / aux;
					measureCount++;
				} else if (measureCount < aux * 1.5f) {
					getAccelMeasure(accelMeasurements);
					if (fabs(accelX) > 0.3f || fabs(accelY) > 0.3f
							|| fabs(gyroZ) > 0.3f) {
						printf("Calibration continues\n");
						measureCount = 0;
						sumAccelXOffset = 0;
						sumAccelYOffset = 0;
						sumGyroOffset = 0;
					}
					measureCount++;

				} else {
					accelActivate = 2;
					printf("MPU calibrated\n");
				}
				measure = 0;
			}
		}

		// Process received data
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
			// writeChar(&handlerConexion, rxDataU1);
			bufferReception[counterReception] = rxDataU1;
			counterReception++;

			// If the incoming character is a newline, set a flag
			// so the main loop can do something about it
			if (rxDataU1 == '@') {
				stringComplete = 1;
				sprintf(bufferData, "\n");
				writeString(&handlerTerminal, bufferData);
				// Add null terminator to the received string
				bufferReception[counterReception] = '\0';
				counterReception = 0;
			}
			if (rxDataU1 == '\b') {
				counterReception--;
				counterReception--;
			}
			// Reset rxDataU1 to prevent re-entry
			rxDataU1 = '\0';
		}

		// Analyze the received string
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
	// Enable co-processor (important for this task)
	SCB->CPACR |= (0xF << 20);

	// Configure PLL (Phase-Locked Loop) for clock generation
	configPLL(100);

	// Configure SysTick (system timer) for timing operations
	config_SysTick();

	/* Configure I2C1 pins */
	// SCL (clock) pin
	mpu6050_sclPin.pGPIOx = GPIOB;
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_8;
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinOPType =
	GPIO_OTYPE_OPENDRAIN; // Open-drain output
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinPuPdControl =
	GPIO_PUPDR_NOTHING; // No pull-up/pull-down
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	mpu6050_sclPin.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF4; // Alternate function 4
	GPIO_Config(&mpu6050_sclPin);

	// SDA (data) pin
	mpu6050_sdaPin.pGPIOx = GPIOB;
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_9;
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinOPType =
	GPIO_OTYPE_OPENDRAIN; // Open-drain output
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinPuPdControl =
	GPIO_PUPDR_NOTHING; // No pull-up/pull-down
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	mpu6050_sdaPin.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF4; // Alternate function 4
	GPIO_Config(&mpu6050_sdaPin);

	/* Configure I2C1 */
	i2c1_mpu6050.ptrI2Cx = I2C1; // I2C1 peripheral
	i2c1_mpu6050.modeI2C = I2C_MODE_FM; // Fast mode
	i2c1_mpu6050.slaveAddress = 0b1101000;  // Slave address
	i2c_config(&i2c1_mpu6050);

	// Configure LED user pin
	ledUsuario.pGPIOx = GPIOC;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_OUT;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinNumber = PIN_5;
	GPIO_Config(&ledUsuario);

	// Configure direction pins
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

	// Configure USART2 pins
	tx2pin.pGPIOx = GPIOA;
	tx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_2;
	tx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	tx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP; // Pull-up resistor
	tx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	tx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7; // Alternate function 7
	GPIO_Config(&tx2pin);

	rx2pin.pGPIOx = GPIOA;
	rx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_3;
	rx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	rx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP; // Pull-up resistor
	rx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	rx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7; // Alternate function 7
	GPIO_Config(&rx2pin);

	// Configure USART handler
	handlerTerminal.ptrUSARTx = USART2; // USART2 peripheral
	handlerTerminal.USART_Config.USART_baudrate = 115200;
	handlerTerminal.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
	handlerTerminal.USART_Config.USART_mode = USART_MODE_RXTX; // Receive and transmit mode
	handlerTerminal.USART_Config.USART_parity = USART_PARITY_NONE;
	handlerTerminal.USART_Config.USART_stopbits = USART_STOPBIT_1;
	handlerTerminal.USART_Config.USART_RX_Int_Ena = ENABLE; // Enable receive interrupt
	USART_Config(&handlerTerminal);

	// Configure additional USART pins
	tx2pin.pGPIOx = GPIOA;
	tx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_9;
	tx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	tx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP; // Pull-up resistor
	tx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	tx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7; // Alternate function 7
	GPIO_Config(&tx2pin);

	rx2pin.pGPIOx = GPIOA;
	rx2pin.GPIO_PinConfig_t.GPIO_PinNumber = PIN_10;
	rx2pin.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	rx2pin.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP; // Pull-up resistor
	rx2pin.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	rx2pin.GPIO_PinConfig_t.GPIO_PinAltFunMode = 7; // Alternate function 7
	GPIO_Config(&rx2pin);

	handlerConexion.ptrUSARTx = USART1; // USART1 peripheral
	handlerConexion.USART_Config.USART_baudrate = 19200;
	handlerConexion.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
	handlerConexion.USART_Config.USART_mode = USART_MODE_RXTX; // Receive and transmit mode
	handlerConexion.USART_Config.USART_parity = USART_PARITY_NONE;
	handlerConexion.USART_Config.USART_stopbits = USART_STOPBIT_1;
	handlerConexion.USART_Config.USART_RX_Int_Ena = ENABLE; // Enable receive interrupt
	USART_Config(&handlerConexion);

	// Configure external interrupts
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

	// Configure PWM pins
	pin_PWMConfig.pGPIOx = GPIOA;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinNumber = PIN_0;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinPuPdControl =
	GPIO_PUPDR_NOTHING;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF2;
	GPIO_Config(&pin_PWMConfig);

	pin_PWMConfig.pGPIOx = GPIOA;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinNumber = PIN_1;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinPuPdControl =
	GPIO_PUPDR_NOTHING;
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinSpeed = GPIO_OSPEED_FAST; // Fast output speed
	pin_PWMConfig.GPIO_PinConfig_t.GPIO_PinAltFunMode = AF2;
	GPIO_Config(&pin_PWMConfig);

	// Configure PWM timers
	pwmYellow.ptrTIMx = TIM5; // Timer 5
	pwmYellow.config.channel = PWM_CHANNEL_2;
	pwmYellow.config.duttyCicle = duttyYwl;
	pwmYellow.config.periodo = periodo;
	pwmYellow.config.prescaler = prescaler;
	pwmYellow.config.polarity = dirYellowVal;

	pwm_Config(&pwmYellow);
	enableOutput(&pwmYellow);
	startPwmSignal(&pwmYellow);

	pwmBlue.ptrTIMx = TIM5; // Timer 5
	pwmBlue.config.channel = PWM_CHANNEL_1;
	pwmBlue.config.duttyCicle = duttyBlue;
	pwmBlue.config.periodo = periodo;
	pwmBlue.config.prescaler = prescaler;
	pwmBlue.config.polarity = dirBlueVal;

	pwm_Config(&pwmBlue);
	enableOutput(&pwmBlue);
	startPwmSignal(&pwmBlue);

	// Configure timer 3
	handlerTimer3.ptrTIMx = TIM3; // Timer 3
	handlerTimer3.TIMx_Config.TIMx_interruptEnable = 1; // Enable interrupt
	handlerTimer3.TIMx_Config.TIMx_mode = BTIMER_MODE_UP; // Up-counting mode
	handlerTimer3.TIMx_Config.TIMx_period = 250; // Period
	handlerTimer3.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us; // Speed

	BasicTimer_Config(&handlerTimer3);

	handlerTimer2.ptrTIMx = TIM2; // Timer 2
	handlerTimer2.TIMx_Config.TIMx_interruptEnable = 1; // Enable interrupt
	handlerTimer2.TIMx_Config.TIMx_mode = BTIMER_MODE_UP; // Up-counting mode
	handlerTimer2.TIMx_Config.TIMx_period = 10; // Period
	handlerTimer2.TIMx_Config.TIMx_speed = 10; // Speed

	BasicTimer_Config(&handlerTimer2);
}

void parseCommands(char *ptrBufferReception) {
	// Parse the string pointed to by ptrBufferReception into three parts:
	// a command string (cmd), and two float parameters (firstParameterF and secondParameterF)
	float firstParameterF, secondParameterF, thirdParameterF;
	char cmd[20];  // assuming max command length is 20 characters
	sscanf(ptrBufferReception, "%s %f %f %f", cmd, &firstParameterF,
			&secondParameterF, &thirdParameterF);

	int firstParameter = (int) firstParameterF;
	int secondParameter = (int) secondParameterF;
	int thirdParameter = (int) thirdParameterF;
	(void) thirdParameter;  // unused variable warning suppression

	if (strcmp(cmd, "help") == 0) {
		// Print help menu
		writeString(&handlerTerminal, "Help Menu CMDs: \n");
		writeString(&handlerTerminal, "1)  Help -> Print this menu \n");
	} else if (strcmp(cmd, "dir") == 0) {
		// Handle "dir" command
		if (firstParameter == 0) {
			dirBlueVal = secondParameter;
			pwmUpdatePolarity(&pwmYellow, secondParameter);
			GPIO_WritePin(&dirPinYw, secondParameter);
		} else {
			dirYellowVal = secondParameter;
			pwmUpdatePolarity(&pwmBlue, secondParameter);
			GPIO_WritePin(&dirPinBlue, secondParameter);
		}
	} else if (strcmp(cmd, "pwm") == 0) {
		// Handle "pwm" command
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
		// Handle "upMode" command
		upMode = firstParameter;
	} else if (strcmp(cmd, "reset") == 0) {
		// Handle "reset" command
		x = 0;
		theta = 0;
		y = 0;
	} else if (strcmp(cmd, "rotate") == 0) {
		// Handle "rotate" command
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
		// Handle "movex" command
		if (firstParameter != 0) {
			distancia = firstParameter;
		}
		moves = 1;
		theta = 0;
		thetaG = 0;
		thetaObjective = 0;
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
	} else if (strcmp(cmd, "wheel") == 0) {
		// Handle "wheel" command
		wheelbase = firstParameterF;
		sprintf(bufferData, "Wheel = %.2f", wheelbase);
		writeString(&handlerConexion, bufferData);
	} else if (strcmp(cmd, "wheelSize") == 0) {
		// Handle "wheelSize" command
		wheelbase = firstParameterF / 10.0f;
		sprintf(bufferData, "Wheel = %.2f", wheelbase);
		writeString(&handlerConexion, bufferData);
	} else if (strcmp(cmd, "vueltas") == 0) {
		// Handle "vueltas" command
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
	} else if (strcmp(cmd, "theta") == 0) {
		// Handle "theta" command
		Kp_theta = firstParameterF;
		Ki_theta = secondParameterF;
		Kd_theta = thirdParameterF;
	} else if (strcmp(cmd, "calibrate") == 0) {
		// Handle "calibrate" command
		initialCalibrate = firstParameter;
	} else if (strcmp(cmd, "stop") == 0) {
		// Handle "stop" command
		stop();
	} else if (strcmp(cmd, "reset") == 0) {
		// Handle "reset" command
		thetaG = 0;
	} else if (strcmp(cmd, "calibrateG") == 0) {
		// Handle "calibrateG" command
		measureCount = 0;
		sumGyroOffset = 0;
		accelActivate = 1;
	} else if (strcmp(cmd, "rotateG") == 0) {
		// Handle "rotateG" command
		move90G = 1;
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
		thetaObjective += 90;
		onMove = 1;
	} else if (strcmp(cmd, "square") == 0) {
		// Handle "movex" command
		if (firstParameter != 0) {
			distancia = firstParameter;
			thetaObjective = 0;
			squareSide = 0;
			movesq = 1;
			theta = 0;
			thetaG = 0;
			x = 0;
			y = 0;
			moveForward();
		} else {
			stop();
			thetaObjective = 0;
			squareSide = 0;
			movesq = 0;
			theta = 0;
			thetaG = 0;
			x = 0;
			y = 0;
		}
	}

	else {
		// If the command does not match any of the above commands, print a "Wrong CMD" message
		writeString(&handlerConexion, "Wrong CMD \n");
	}

	// Reset parameters for next command
	firstParameter = 0;
	secondParameter = 0;
	thirdParameter = 0;
}

// Callback for timer 2, increments a counter
void BasicTimer2_Callback(void) {
	// Increment the timer 2 counter
	counterTimer2++;
}

// Callback for timer 3, handles blinking, motion control, and data transmission
void BasicTimer3_Callback(void) {
	// Increment the 250ms counter
	counter250ms++;

	// Check if accelerometer activation is triggered
	if (accelActivate == 1) {
		// Set measurement flag
		measure = 1;
	} else if (accelActivate == 2) {
		// Get and process gyroscope measurement
		getAccelMeasure(accelMeasurements);
//		if(fabs(gyroZ) < 0.3f){
//			gyroZ = 0;
//		}
		if (onMove || movesq) {
			thetaG += gyroZ * 0.025f;
		}

		// Check if the robot has reached the target angle
		if (move90G && thetaG > thetaObjective * 0.98f) {
			if (movesq) {
				// Stop the robot
				stop();
				move90G = 0;
				squareSide++;
				enableCounter = 1;
			} else {
				// Stop the robot
				stop();
				move90G = 0;
			}
		}
	}

	// Check if the robot is moving and needs to stop
	if (onMove && (modoVueltas == 0 && move90G == 0)) {
		// Update the robot's position
		updatePosition();

	}

	// Toggle the user LED every second
	if (counter250ms >= 10) {
		// Toggle the LED
		banderaLed = 1;
		counter250ms = 0;
		counter10seg++;
		if (enableCounter) {
			counter1SegundoPausa++;
		}

	}

	// Periodically update the motor control
	if (counter250ms == 7 && onMove) {
		// Update motor control
		updateMotorControl();

	}

	// Handle data transmission and reset counters
	if (counter10seg > 1 && accelActivate == 2
			&& ((onMove || move90G) && movesq)) {
		// Format data for transmission
		sprintf(bufferData,
				"%.4f\t%.4f\t%.4f\t%d\t%.4f\t%.3f\t%.3f\t%d\t%d\t%.2f\t%.2f\t%.2f\n",
				x / 100, y / 100, thetaG, squareSide, thetaObjective,
				pwmBlue.config.duttyCicle / 100.0f,
				pwmYellow.config.duttyCicle / 100.0f, (int) counterBlueCounterT,
				(int) counterYwlCounterT, Kp_theta, Ki_theta, Kd_theta);
		// Transmit data
		writeString(&handlerConexion, bufferData);
		// Reset counters
		counter10seg = 0;
	}
}

// USART1 receive callback
void USART1Rx_Callback(void) {
	// Get received data
	rxDataU1 = (uint8_t) USART1->DR;
}

// External interrupt callback for PC0
void callback_extInt0(void) {
	// Increment PC0 counter
	PC0Counter++;
}

// External interrupt callback for PC7
void callback_extInt7(void) {
	// Increment PC7 counter
	PC7Counter++;
}

// External interrupt callback for PC1
void callback_extInt1(void) {
	counterYwlCounterT++;
	counterYwlCounter++;

	if (modoVueltas == 1) {
		// Check if the yellow counter has reached the target vueltas
		if (counterYwlCounter >= vueltasYellow * vueltasMsg) {
			counterYwlCounter = 0;
			stop();
			modoVueltas = 3;
		}
	}
}

// External interrupt callback for PC3
void callback_extInt3(void) {
	counterBlueCounterT++;
	counterBlueCounter++;

	if (modoVueltas == 2) {
		// Check if the blue counter has reached the target vueltas
		if (counterBlueCounter >= vueltasBlue * vueltasMsg) {
			counterBlueCounter = 0;
			stop();
			modoVueltas = 3;
		}
	}
}

// Function to update the robot's position
void updatePosition(void) {
	double dLeft, dRight, dCenter, deltaX, deltaY;

	// Calculate the distance traveled by each wheel
	recorridoBlue = (counterBlueCounter) * M_PI * wheelsize / vueltasBlue;
	counterBlueCounter = 0;
	recorridoYellow = (counterYwlCounter) * M_PI * wheelsize / vueltasYellow;
	counterYwlCounter = 0;

	// Convert directions into moveForward (1) or backward (-1) multipliers
	int multiplierBlue = (dirBlueVal == 0) ? 1 : -1;
	int multiplierYellow = (dirYellowVal == 0) ? 1 : -1;

	// Calculate distances traveled by each wheel, adjusted by direction
	dLeft = recorridoBlue * multiplierBlue;
	dRight = recorridoYellow * multiplierYellow;

	// Average distance traveled by the robot
	dCenter = (dLeft + dRight) / 2.0;
	// Calculate change in orientation

	double thetaGRad = thetaG * (M_PI / 180.0f);

	// Calculate change in position
	deltaX = dCenter * cos(thetaGRad);
	deltaY = dCenter * sin(thetaGRad);

	// Update position
	x += deltaX;
	y += deltaY;

}

// Function to stop the robot's movement
void stop(void) {
	onMove = 0;
	pwmYellow.config.duttyCicle = START;
	pwmBlue.config.duttyCicle = START;
	setDuttyCycle(&pwmYellow);
	setDuttyCycle(&pwmBlue);
}

// Function to update motor control
void updateMotorControl() {
	// Angle control
	double thetaError = thetaG - thetaObjective; // theta is updated on other place
	double derivativeTheta = thetaError - lastThetaError;
	integralTheta += thetaError;

	double adjustTheta = (Kp_theta * thetaError) + (Ki_theta * integralTheta)
			+ (Kd_theta * derivativeTheta);

	uint16_t limit = 750.0f;

	uint16_t aux = clamp(pwmYellow.config.duttyCicle - (int) (adjustTheta),
			duttyIYwl - limit, duttyIYwl + limit);

	// Adjust PWM settings for both motors
	pwmYellow.config.duttyCicle = aux;
	aux = clamp(pwmBlue.config.duttyCicle + (int) (adjustTheta),
			duttyInBlue - limit, duttyInBlue + limit);
	pwmBlue.config.duttyCicle = aux;

	// Update PWM outputs
	setDuttyCycle(&pwmYellow);
	setDuttyCycle(&pwmBlue);

	// Store current errors for next cycle's derivative calculations
	lastThetaError = thetaError;
}

// Helper function to ensure PWM values are within specified limits
int clamp(int value, int min, int max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

//function than update the accelerometer measurements

void getAccelMeasure(int16_t *array) {
	// Create a temporary array to store the I2C read data
	uint8_t auxArray[6];
	// Read the 4 registers corresponding to the accelerometers
	i2c_readMulRegister2(&i2c1_mpu6050, regsToRead, 6, auxArray);

	// Organize the measurements in the input array
	// Combine high-byte and low-byte values to form 16-bit integers
	array[0] = (auxArray[0] << 8) | auxArray[1];  // Accel X
	array[1] = (auxArray[2] << 8) | auxArray[3];  // Accel Y
	array[2] = (auxArray[4] << 8) | auxArray[5];

	// Calculate the corrected acceleration values
	// Use the offset values to correct the measurements
	accelX = array[0] / 16384.0f - accelXOffset;
	// Note: It seems there is an error in the original code, accelY is calculated using array[0] instead of array[1]
	accelY = array[1] / 16384.0f - accelYOffset;

	gyroZ = array[2] / 131.0f - gyroZOffset;
}

void rotateG(void) {
	// Handle "rotateG" command
	move90G = 1;

	dirBlueVal = 1;
	pwmUpdatePolarity(&pwmBlue, dirBlueVal);
	GPIO_WritePin(&dirPinBlue, dirBlueVal);
	dirYellowVal = 0;
	pwmUpdatePolarity(&pwmYellow, dirYellowVal);
	GPIO_WritePin(&dirPinYw, dirYellowVal);
	pwmBlue.config.duttyCicle = duttyGiroBlue;
	setDuttyCycle(&pwmBlue);
	pwmYellow.config.duttyCicle = duttyGiroYwl;
	setDuttyCycle(&pwmYellow);
	newMoveBlue = 1;
	newMoveYellow = 1;
	thetaObjective += 90;
}

void moveForward(void) {
	lastThetaError = 0;
	integralTheta = 0;
	lastThetaError = 0;
	dirBlueVal = 0;
	pwmUpdatePolarity(&pwmBlue, dirBlueVal);
	GPIO_WritePin(&dirPinBlue, dirBlueVal);
	dirYellowVal = 0;
	pwmUpdatePolarity(&pwmYellow, dirYellowVal);
	GPIO_WritePin(&dirPinYw, dirYellowVal);
	pwmBlue.config.duttyCicle = duttyInBlue;
	setDuttyCycle(&pwmBlue);
	pwmYellow.config.duttyCicle = duttyIYwl;
	setDuttyCycle(&pwmYellow);
	onMove = 1;
}
