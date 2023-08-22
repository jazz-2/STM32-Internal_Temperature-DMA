Internal temperature reading using Nucleo-64 STM32F446RE
===================================
[*github.com/jazz-2*](https://github.com/jazz-2)

## Rise in CPU temperature starts the DC motor and LED
-----------------------------------
![TempSensorPWMMotor](https://github.com/jazz-2/STM32-Internal_Temperature-DMA/assets/141406828/789cf69a-f03d-417c-bc8b-074f0aeb4c9b)

### Average of temperatures from a given number of samples 
-----------------------------------
![temperatureSamples](https://github.com/jazz-2/STM32-Internal_Temperature-DMA/assets/141406828/a6a62b26-eeb3-4213-93a4-e591b3d3f831)
### Circuit
-----------------------------------
![circuitPhoto](https://github.com/jazz-2/STM32-Internal_Temperature-DMA/assets/141406828/30086d00-d25d-4dad-8318-a31519af7625)



#### Description
This project contains reading data from internal `Temperature Sensor Channel` and `Vrefint Channel` using ADC and DMA. It controls a DC motor and LED using PWM.

If the temperature exceeds a certain threshold, the PWM duty cycle will be increased/decreased, consequently increasing/decreasing LED brightness and DC motor speed.

Electronic elements: 
* transistor BJT NPN BD135
* resistor 1 kΩ
* a Schottky diode was used as a flyback diode.
* capacitor 1000 µF
    * The capacitor was added to improve the smoothness of DC motor work and to boost performance at low `PWM_DutyCycle`.

You can find `Temperature` calculations in `stm32f4xx_it.c` file.

