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
#include "PLLDriver.h"
#include "USARTxDriver.h" // USART driver library
#include "SysTick.h"
#include <string.h>
#include "BasicTimer.h"

// Necessary variables
char bufferReception[100];
char bufferData[100];
int counterReception = 0;
int stringComplete = 0;
char rxData = '\0';
USART_Handler_t handlerTerminal;
GPIO_Handler_t ledUsuario, tx2pin, rx2pin;
BasicTimer_Handler_t handlerTimer3;
int banderaLed = 0;
int counter1 = 0, counter2 = 0, counter3= 0, counter4 = 0, counter5 = 0;

// Function headers
void parseCommands(char *ptrBufferReception);
void configPeripherals(void);

// Redirection of printf() to USART2
int __io_putchar(int ch) {
	// Send the character by USART1
	writeChar(&handlerTerminal, (char) ch);
	return ch;
}

int main(void) {
	// Initialize peripherals
	configPeripherals();

	/* Loop forever */
	while (1) {
		if (banderaLed) {
			GPIOxTooglePin(&ledUsuario);
			banderaLed = 0;
			counter1++;
			counter2+= 2;
			counter3 += 3;
			counter4 += 4;
			counter5 += 5;
			printf("%d\t%d\t%d\t%d\t%d\n",counter1,counter2,counter3,counter4,counter5);
		}

		// Process received data
		if (rxData != '\0') {
			bufferReception[counterReception] = rxData;
			counterReception++;

			// If the incoming character is a newline, set a flag
			// so the main loop can do something about it
			if (rxData == '@') {
				stringComplete = 1;
				writeString(&handlerTerminal, bufferData);
				// Add null terminator to the received string
				bufferReception[counterReception] = '\0';
				counterReception = 0;
			}
			if (rxData == '\b') {
				counterReception--;
				counterReception--;
			}
			// Reset rxData to prevent re-entry
			rxData = '\0';
		}

		// Analyze the received string
		if (stringComplete) {
			parseCommands(bufferReception);
			writeChar(&handlerTerminal, '\n');
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

	ledUsuario.pGPIOx = GPIOA;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinNumber = PIN_5;
	ledUsuario.GPIO_PinConfig_t.GPIO_PinMode = GPIO_MODE_OUT;

	GPIO_Config(&ledUsuario);

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
	handlerTerminal.USART_Config.USART_baudrate = 19200;
	handlerTerminal.USART_Config.USART_datasize = USART_DATASIZE_8BIT;
	handlerTerminal.USART_Config.USART_mode = USART_MODE_RXTX; // Receive and transmit mode
	handlerTerminal.USART_Config.USART_parity = USART_PARITY_NONE;
	handlerTerminal.USART_Config.USART_stopbits = USART_STOPBIT_1;
	handlerTerminal.USART_Config.USART_RX_Int_Ena = ENABLE; // Enable receive interrupt
	USART_Config(&handlerTerminal);

	// Configure timer 3
	handlerTimer3.ptrTIMx = TIM3; // Timer 3
	handlerTimer3.TIMx_Config.TIMx_interruptEnable = 1; // Enable interrupt
	handlerTimer3.TIMx_Config.TIMx_mode = BTIMER_MODE_UP; // Up-counting mode
	handlerTimer3.TIMx_Config.TIMx_period = 2500; // Period
	handlerTimer3.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us; // Speed

	BasicTimer_Config(&handlerTimer3);
}

void parseCommands(char *ptrBufferReception) {
	// Parse the string pointed to by ptrBufferReception into command and parameter
	char cmd[20];  // assuming max command length is 20 characters
	int param;     // integer parameter
	sscanf(ptrBufferReception, "%s %d", cmd, &param);

	if (strcmp(cmd, "hello") == 0) {
		// Handle "hello" command
		char response[50];
		sprintf(response, "hello for command %d \n", param);
		writeString(&handlerTerminal, response);
	} else {
		// If the command does not match "hello", print a "Wrong CMD" message
		writeString(&handlerTerminal, "Wrong CMD \n");
	}
}

// Callback for timer 3, handles blinking, motion control, and data transmission
void BasicTimer3_Callback(void) {
	banderaLed = 1;
}

void USART2_IRQHandler(void) {
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	USART2->SR &= ~USART_SR_RXNE;
	//Auxiliar
	rxData = (uint8_t) USART2->DR;
}
