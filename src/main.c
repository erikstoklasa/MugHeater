#include "stm32f3xx.h"
#include <math.h>

// Constants for temperature calculation
#define V25 1.43f      // V25 = 1.43V
#define AVG_SLOPE 4.3f // Avg_Slope = 4.3 mV/°C
#define VREFINT 3.3f   // VDDA value

#define ADC_RESOLUTION 4096  // 12-bit ADC
#define VREF 3.3             // Reference voltage in volts
#define R_PULLUP 10000.0     // Pull-up resistor in ohms (10kΩ)
#define R25 10000.0          // Thermistor resistance at 25°C (10kΩ)
#define B_COEFFICIENT 4050.0 // Beta coefficient (B)
#define T_REF 298.15         // Reference temperature in Kelvin (25°C)

#define T_DESIRED 40.0f // Desired temperature in Celsius

volatile uint16_t is_enabled_heating = 0;
volatile uint16_t has_reached_temp = 0;
/*
 * Initialize the load switch on PA11
 */
void Load_Switch_Init(void)
{
    // enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // set PA11 as output
    GPIOA->MODER |= GPIO_MODER_MODER11_0;
}

/*
 * Initialize the ADC to read the temperature sensor on PA3
 */
void ADC_TempSensor_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    // Configure PA3 as analog input
    GPIOA->MODER |= GPIO_MODER_MODER3;  // Analog mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR3; // No pull-up, pull-down
    // Enable ADC clock
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;

    ADC1_2_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC1_2_COMMON->CCR |= ADC_CCR_CKMODE_1; // Set CKMODE to HCLK/2

    // ADC voltage regulator enable
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADVREGEN_0;

    // Wait for voltage regulator startup time (10us required)
    for (uint32_t i = 0; i < 1000; i++)
        ;

    // ADC configuration
    ADC1->CR &= ~ADC_CR_ADEN; // Disable ADC
    while ((ADC1->CR & ADC_CR_ADEN) != 0)
        ; // Wait until ADC is disabled

    // Calibrate ADC
    ADC1->CR &= ~ADC_CR_ADCALDIF; // Single-ended calibration
    ADC1->CR |= ADC_CR_ADCAL;     // Start calibration
    while (ADC1->CR & ADC_CR_ADCAL)
        ; // Wait for calibration to complete

    // Configure ADC:
    ADC1->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_ALIGN); // single conversion, right-aligned
    ADC1->CFGR &= ADC_CFGR_RES;                      // 12-bit resolution = 00

    // // Configure channel 4 PA3
    ADC1->SQR1 = (4U << ADC_SQR1_SQ1_Pos);
    ADC1->SQR1 &= ~ADC_SQR1_L; // Length = 1 conversion

    // Set sample time for channel 4 to 601.5 cycles = 111
    ADC1->SMPR1 |= (7U << ADC_SMPR1_SMP4_Pos);

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;

    // Wait for ADC to be ready
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    {
    }
}

uint16_t ADC1_Read(void)
{
    ADC1->CR |= ADC_CR_ADSTART; // Start conversion
    while (!(ADC1->ISR & ADC_ISR_EOC))
        ;            // Wait for conversion to complete
    return ADC1->DR; // Read conversion result
}

/**
 * Converts an ADC reading to temperature in Celsius for an NTC thermistor.
 * @param adc_value: The raw ADC value from the thermistor voltage divider.
 * @return Temperature in Celsius.
 */
float Convert_NTC_Temperature(uint16_t adc_value)
{
    // Avoid division by zero or invalid ADC values
    if (adc_value == 0 || adc_value == (ADC_RESOLUTION - 1))
    {
        return -273.15; // Invalid temperature
    }

    // Step 1: Calculate thermistor resistance
    float v_out = (float)adc_value / (ADC_RESOLUTION - 1) * VREF;
    float r_ntc = R_PULLUP * v_out / (VREF - v_out);

    // Step 2: Apply the simplified Steinhart-Hart equation
    float temp_kelvin = 1.0 / ((1.0 / T_REF) + (1.0 / B_COEFFICIENT) * log(r_ntc / R25));
    float temp_celsius = temp_kelvin - 273.15; // Convert to Celsius

    return temp_celsius;
}

void Init_Button(void)
{
    // Init button on PA7
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    // Configure PA7 as input
    GPIOA->MODER &= ~GPIO_MODER_MODER7;  // Input mode
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_0; // Pull-up
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Connect EXTI7 to PA7
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI7;   // Clear EXTI7 bits
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA; // Map PA7 to EXTI7

    // Configure EXTI7 to trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR7;
    // Unmask EXTI7
    EXTI->IMR |= EXTI_IMR_IM7;
    // Enable EXTI7 interrupt
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 0);
}

void Init_LED(void)
{
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA4 as output
    GPIOA->MODER &= ~GPIO_MODER_MODER4_1; // Set PA4 to output mode
    GPIOA->MODER |= GPIO_MODER_MODER4_0;

    // Ensure LED is initially off
    GPIOA->ODR &= ~GPIO_ODR_4;
}

volatile uint32_t sys_tick_count = 0;

void SysTick_Handler(void)
{
    sys_tick_count++; // Increment the millisecond counter
}

// Function to get the current time in milliseconds
uint32_t GetSysTick(void)
{
    return sys_tick_count;
}

// Delay function using SysTick
void Delay_ms(uint32_t delay)
{
    uint32_t start_time = GetSysTick();
    while ((GetSysTick() - start_time) < delay)
        ;
}

volatile uint32_t last_interrupt_time = 0; // Track the last interrupt time

// Button press handler
void EXTI9_5_IRQHandler(void)
{
    // Check if EXTI7 triggered the interrupt
    if (EXTI->PR & EXTI_PR_PR7)
    {
        // Clear the pending interrupt flag for EXTI7
        EXTI->PR |= EXTI_PR_PR7;

        // Get the current system tick in milliseconds (from sys_tick_count)
        uint32_t current_time = sys_tick_count;

        // Debounce: Ensure at least 50ms between interrupts
        if ((current_time - last_interrupt_time) >= 50)
        {                                       // 50ms debounce threshold
            last_interrupt_time = current_time; // Update last interrupt time

            // Toggle the load switch on PA11
            GPIOA->ODR ^= GPIO_ODR_11;
            is_enabled_heating = !is_enabled_heating;
            // Toggle the LED on PA4
            GPIOA->ODR ^= GPIO_ODR_4;
        }
    }
}

void Init_Buzzer(void)
{
    // Init buzzer on PA5
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    // Configure PA5 as output
    GPIOA->MODER &= ~GPIO_MODER_MODER5_1; // Output mode
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
}

/**
 * Convert temperature to blinks on the LED
 * @param temp: Temperature in Celsius
 * @note 1 blink per 10 degrees
 */
void Convert_Temp_To_Blinks(float temp)
{
    // Convert temp to number of blinks 1 blink per 10 degrees
    int num_blinks = (int)(temp / 10);
    // Blink LED num_blinks times
    for (int i = 0; i < num_blinks; i++)
    {
        GPIOA->ODR |= GPIO_ODR_4;  // Turn on LED
        Delay_ms(500);             // Delay 500ms
        GPIOA->ODR &= ~GPIO_ODR_4; // Turn off LED
        Delay_ms(500);             // Delay 500ms
    }
}

int main(void)
{
    // Initialize the load switch
    Load_Switch_Init();
    // Initialize temperature sensor ADC
    ADC_TempSensor_Init();
    Init_LED();
    // Initialize button
    Init_Button();
    // Initialize buzzer and systick for it
    Init_Buzzer();

    // Setup systick timer for 1ms, use core clock
    SysTick_Config(SystemCoreClock / 1000);
    float temperature = 0.0f;
    uint16_t raw_value;
    while (1)
    {
        raw_value = ADC1_Read();
        temperature = Convert_NTC_Temperature(raw_value);
        if ((temperature < T_DESIRED) && is_enabled_heating)
        {
            GPIOA->ODR |= GPIO_ODR_11; // Turn on load
        }
        else
        {
            GPIOA->ODR &= ~GPIO_ODR_11; // Turn off load
            if (!has_reached_temp && (temperature >= T_DESIRED))
            {
                has_reached_temp = 1;
                for (int i = 0; i < 200; ++i)
                {
                    GPIOA->ODR |= GPIO_ODR_5; // Turn on buzzer
                    Delay_ms(1);
                    GPIOA->ODR &= ~GPIO_ODR_5; // Turn off buzzer
                    Delay_ms(1);
                }
            }
        }
        // Convert_Temp_To_Blinks(temperature);
        Delay_ms(10000); // Check every 10s
    }
}
