# Mug heater project
- Nucleo STM32-F303K8
## Electornic schema
![Schematic](schematic.png)

## Peripherals connected
- Load switching MOSFET gate on `PF1`
- NTC Thermistor on `PA3`
- Buzzer on `PA5` which indicates a reached temperature
- Button on `PA7` which turns `off` and `on` the heating
- LED on `PA4` which indicates mode of the heating
  - Flashes and then `on` indicates starting heating
  - Flashes and then `off` indicates stopping heating

## Functionalities
- Every 1 minute reads AD and either turns off or on the load based on the specified temperature `Default: 60°C`
- Button turns on or off the heating
- Plays a sound when specified temperature reached



Notes:
- Needs a 500 ohm short from NRST to 3.3V in order to operate from only one cable. Stlink pulls down the NRST pin, which causes the MC to be in reset state when stlink is not connected.