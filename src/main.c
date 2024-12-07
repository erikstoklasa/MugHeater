#include "stm32f3xx.h"
#include <math.h>

// Constants for temperature calculation
#define V25 1.43f      // V25 = 1.43V
#define AVG_SLOPE 4.3f // Avg_Slope = 4.3 mV/°C
#define VREFINT 3.3f   // VDDA value

// ADC register bit positions for F303
#define ADC_SQR1_SQ1_Pos 6U
#define ADC_SMPR1_SMP16_Pos 16U

#define ADC_RESOLUTION 4096  // 12-bit ADC
#define VREF 3.3             // Reference voltage in volts
#define R_PULLUP 10000.0     // Pull-up resistor in ohms (10kΩ)
#define R25 10000.0          // Thermistor resistance at 25°C (10kΩ)
#define B_COEFFICIENT 4050.0 // Beta coefficient (B)
#define T_REF 298.15         // Reference temperature in Kelvin (25°C)

/*
 * Initialize the load switch on PA11
 */
void Load_Switch_Init(void)
{
    // enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // set pa11 as output
    GPIOA->MODER |= GPIO_MODER_MODER11_0;
}

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

int main(void)
{
    Load_Switch_Init();
    // Initialize temperature sensor ADC
    ADC_TempSensor_Init();
    volatile float temperature = 0.0f;

    while (1)
    {
        uint16_t raw_value = ADC1_Read();
        temperature = Convert_NTC_Temperature(raw_value);
        if (temperature > 60.0f)
        {
            GPIOA->ODR &= ~GPIO_ODR_11;
        }
        else
        {
            GPIOA->ODR |= GPIO_ODR_11;
        }

        // Delay between readings
        for (volatile uint32_t i = 0; i < 100000; i++);
    }
}
