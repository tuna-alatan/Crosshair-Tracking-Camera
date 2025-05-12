#include "MKL25Z4.h"
#include <stdio.h>  // For sscanf
#include <stdbool.h>

// PWM setup
#define PWM_FREQUENCY 50  // 50 Hz for servo PWM
#define TPM1_MOD (21000000 / 128 / PWM_FREQUENCY)  // MOD value for TPM1 at 21 MHz with /128 prescaler

// Joystick ADC input channels
#define JOYSTICK_X_CHANNEL 8  // PTB0 = ADC0_SE8
#define JOYSTICK_Y_CHANNEL 9  // PTB1 = ADC0_SE9

// PWM output pins for servos
#define SERVO_X_PIN 20  // PTE20 = TPM1_CH0
#define SERVO_Y_PIN 21  // PTE21 = TPM1_CH1

// Servo pulse width range in TPM ticks (at 50 Hz)
#define SERVO_MIN_PULSE 162   // ~1 ms pulse
#define SERVO_MAX_PULSE 328   // ~2 ms pulse

// ADC reading bounds (can be tuned per joystick)
#define ADC_MIN 0
#define ADC_MAX 4095

#define BUTTON_PIN 3  // PTD3

volatile bool locked = false; // If true, disable manual (joystick) control

// Struct for cleaner GPIO pin definitions
typedef struct {
    GPIO_Type *port;
    uint8_t pin;
} gpio_pin_t;

// 8x8 LED matrix row and column pin mappings
gpio_pin_t row_pins[8] = {
    {GPIOA, 4}, {GPIOC, 6}, {GPIOC, 5}, {GPIOA, 2},
    {GPIOC, 3}, {GPIOD, 4}, {GPIOC, 8}, {GPIOC, 9}
};

gpio_pin_t col_pins[8] = {
    {GPIOC, 4}, {GPIOC, 10}, {GPIOC, 0}, {GPIOC, 11},
    {GPIOA, 12}, {GPIOC, 7}, {GPIOA, 5}, {GPIOA, 1}
};

// Initialize GPIO as digital output and set to LOW
void init_pin(gpio_pin_t pin) {
    uint32_t mask = (1 << pin.pin);

    // Enable clock to port
    if (pin.port == GPIOA) SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    if (pin.port == GPIOB) SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    if (pin.port == GPIOC) SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    if (pin.port == GPIOD) SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    if (pin.port == GPIOE) SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set pin as GPIO
    PORT_Type *port_base;
    if (pin.port == GPIOA) port_base = PORTA;
    if (pin.port == GPIOB) port_base = PORTB;
    if (pin.port == GPIOC) port_base = PORTC;
    if (pin.port == GPIOD) port_base = PORTD;
    if (pin.port == GPIOE) port_base = PORTE;

    port_base->PCR[pin.pin] = PORT_PCR_MUX(1);  // MUX=1: GPIO

    pin.port->PDDR |= mask;   // Output
    pin.port->PCOR = mask;    // Start low
}

// Set a GPIO pin high or low
void set_pin(gpio_pin_t pin, bool high) {
    uint32_t mask = (1 << pin.pin);
    if (high)
        pin.port->PSOR = mask;
    else
        pin.port->PCOR = mask;
}

// Initialize LED matrix pins
void matrix_init(void) {
    for (int i = 0; i < 8; i++) {
        init_pin(row_pins[i]);
        init_pin(col_pins[i]);

        set_pin(row_pins[i], 0);  // Rows low (inactive)
        set_pin(col_pins[i], 1);  // Columns high (inactive)
    }
}

// Light up a specific LED by activating one row and one column
void light_led(int row, int col) {
    for (int i = 0; i < 8; i++) {
        set_pin(row_pins[i], 1);  // Set all rows high
        set_pin(col_pins[i], 0);  // Set all columns low
    }

    // Activate selected LED (current flows from row → col)
    set_pin(row_pins[row], 0);  // Drive selected row low
    set_pin(col_pins[col], 1);  // Drive selected column high
}

// Initialize TPM1 for PWM output on PTE20 and PTE21
void initPWM(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);  // Use MCGFLLCLK as TPM clock

    PORTE->PCR[SERVO_X_PIN] = PORT_PCR_MUX(3);  // TPM1_CH0
    PORTE->PCR[SERVO_Y_PIN] = PORT_PCR_MUX(3);  // TPM1_CH1

    TPM1->SC = 0;
    TPM1->MOD = TPM1_MOD - 1;

    TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM1->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

    TPM1->CONTROLS[0].CnV = 0;
    TPM1->CONTROLS[1].CnV = 0;

    TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);  // Enable TPM1 with /128 prescaler
}

// ADC initialization for joystick input
void initADC(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    ADC0->CFG1 = ADC_CFG1_ADIV(3) |    // Divide by 8
                 ADC_CFG1_MODE(1) |    // 12-bit resolution
                 ADC_CFG1_ADICLK(0);   // Bus clock

    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;  // Software trigger
    ADC0->SC3 |= ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);  // 32x HW average
}

// Blocking read from given ADC channel
uint16_t readADC(uint8_t channel) {
    ADC0->SC1[0] = channel;
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

// Linearly remap ADC value to a valid servo pulse width
uint16_t mapADCToPulse(uint16_t raw) {
    if (raw < ADC_MIN) raw = ADC_MIN;
    if (raw > ADC_MAX) raw = ADC_MAX;

    return SERVO_MIN_PULSE + ((raw - ADC_MIN) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / (ADC_MAX - ADC_MIN);
}

// Write position to both servos
void setServoPosition(uint16_t x_raw, uint16_t y_raw) {
    TPM1->CONTROLS[0].CnV = mapADCToPulse(x_raw);
    TPM1->CONTROLS[1].CnV = mapADCToPulse(y_raw);
}

// UART1 initialization on PTE0/PTE1
void initUART1(uint32_t baud_rate) {
    SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[0] = PORT_PCR_MUX(3);  // TX
    PORTE->PCR[1] = PORT_PCR_MUX(3);  // RX

    uint16_t divisor = (10485760 / (baud_rate * 16));
    UART1->BDH = (divisor >> 8) & 0x1F;
    UART1->BDL = divisor & 0xFF;

    UART1->C1 = 0;
    UART1->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}

// Blocking character read from UART1
char uart1_readchar(void) {
    while (!(UART1->S1 & UART_S1_RDRF_MASK));
    return UART1->D;
}

// Read a full line from UART1
int uart1_readline(char *buf, int maxlen) {
    int i = 0;
    while (i < maxlen - 1) {
        char c = uart1_readchar();
        if (c == '\n') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
    return i;
}

// Setup external button with interrupt
void initButton(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[BUTTON_PIN] = PORT_PCR_MUX(1);
    PTD->PDDR &= ~(1 << BUTTON_PIN);  // Input
    PORTD->PCR[BUTTON_PIN] |= PORT_PCR_IRQC(0xA);  // Falling edge
    NVIC_EnableIRQ(PORTD_IRQn);
}

// Toggle mode on button press
void PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1 << BUTTON_PIN)) {
        locked = !locked;
        PORTD->ISFR |= (1 << BUTTON_PIN);  // Clear interrupt flag
    }
}

int main(void) {
    char buffer[32];
    int led_x = 3, led_y = 3;

    initPWM();
    initADC();
    initButton();
    initUART1(9600);
    matrix_init();

    while (1) {
        // Read joystick and control servos if unlocked
        uint16_t x_adc = ADC_MAX - readADC(JOYSTICK_X_CHANNEL);
        uint16_t y_adc = readADC(JOYSTICK_Y_CHANNEL);
        if (!locked) {
            setServoPosition(x_adc, y_adc);
        }

        // Parse UART command (e.g., "45,78") to update LED target
        if (UART1->S1 & UART_S1_RDRF_MASK) {
            if (uart1_readline(buffer, sizeof(buffer))) {
                int x_new, y_new;
                if (sscanf(buffer, "%d,%d", &x_new, &y_new) == 2) {
                    if (x_new >= 0 && x_new < 160 && y_new >= 0 && y_new < 120) {
                        led_x = (x_new * 8) / 160;
                        led_y = (y_new * 8) / 120;
                    }
                }
            }
        }

        // Show selected LED
        light_led(led_y, led_x);
        for (volatile int i = 0; i < 10; i++);  // Small delay to reduce flicker
    }
}
