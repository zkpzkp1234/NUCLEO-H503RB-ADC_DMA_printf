# NUCLEO-H503RB-ADC_DMA_printf
USE STM32CUBEIDE 1.15.1 and STM32CUBEMX 6.11.1
Direct clone the project may not be able to compile, you should double-click the .ioc file and change anything, then change it back and save, let the IDE download the library for you.

Use NUCLEO-H503RB to do the ADC conversion and use DMA to save the data, then print the last one of the data to the com port.

The MCU is STM32H503.

ADC uses 256 oversamples, AD1 channel.
