// Necessary libs
#include "GPIOxDriver.h" // GPIO driver library
#include "PLLDriver.h"
#include "USARTxDriver.h" // USART driver library
#include "SysTick.h"
#include <string.h>
#include "BasicTimer.h"
#include "arm_math.h" // CMSIS-DSP library

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
float32_t sinVal = 0, cosVal = 0, squareVal = 0, triangleVal = 0, sawtoothVal = 0, pwmVal = 0;
int signals[6] = {0, 0, 0, 0, 0, 0}; // Array to keep track of active signals

// Function headers
void parseCommands(char *ptrBufferReception);
void configPeripherals(void);
void generateSignals(void);

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
            generateSignals();
            printf("sin: %f\tcos: %f\tsquare: %f\ttriangle: %f\tsawtooth: %f\tpwm: %f\n",
                   signals[0] ? sinVal : 0,
                   signals[1] ? cosVal : 0,
                   signals[2] ? squareVal : 0,
                   signals[3] ? triangleVal : 0,
                   signals[4] ? sawtoothVal : 0,
                   signals[5] ? pwmVal : 0);
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
            for (int i = 0; i < sizeof(bufferReception) / sizeof(bufferReception[0]); i++) {
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
        if (param >= 0 && param <= 5) {
            signals[param] = 1; // Set the signal as active
            char response[50];
            sprintf(response, "hello for command %d \n", param);
            writeString(&handlerTerminal, response);
        } else {
            writeString(&handlerTerminal, "Invalid signal number \n");
        }
    } else if (strcmp(cmd, "bye") == 0) {
        // Handle "bye" command
        if (param >= 0 && param <= 5) {
            signals[param] = 0; // Clear the signal
            char response[50];
            sprintf(response, "Signal %d closed, number printed is 0 \n", param);
            writeString(&handlerTerminal, response);
        } else {
            writeString(&handlerTerminal, "Invalid signal number \n");
        }
    } else {
        // If the command does not match "hello" or "bye", print a "Wrong CMD" message
        writeString(&handlerTerminal, "Wrong CMD \n");
    }
}

// Generate different signals
void generateSignals(void) {
    static float32_t angle = 0;
    const float32_t increment = 0.1; // Adjust the increment to change the frequency

    // Generate sine and cosine signals
    sinVal = arm_sin_f32(angle);
    cosVal = arm_cos_f32(angle);

    // Generate square wave signal
    squareVal = (angle - (int)angle) > 0.5 ? 1.0f : -1.0f;

    // Generate triangle wave signal
    triangleVal = 2.0f * fabsf(angle / (2.0f * PI) - floorf(angle / (2.0f * PI) + 0.5f));

    // Generate sawtooth wave signal
    sawtoothVal = 2.0f * (angle / (2.0f * PI) - floorf(angle / (2.0f * PI) + 0.5f));

    // Generate a simple PWM signal
    pwmVal = (angle - (int)angle) > 0.5 ? 1.0f : 0.0f;

    angle += increment;
    if (angle >= 2 * PI) {
        angle -= 2 * PI;
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
