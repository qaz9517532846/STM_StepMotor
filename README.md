# STM_StepMotor
How to control a step motor using NUCLEO-H743ZI2 board.

- Hardware:  NUCLEO-H743ZI2 board, step motor(28BYJ-48) and ULN2003(IC).

- Software: [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) and [Keil uVision5](https://www2.keil.com/mdk5/uvision/).

------

### Circuit

------

### Velocity control

This step motor takes 4096 steps to complete 1 revolution so it has 512 sequences per revolution.

``` bash
for(int i = 0; i < 512; i++)
{
	for(int i = 0; i < 8; i++)
	{
		stepper_half_drive(i);
		stepper_set_rpm(5);
	}
}
```

------

### Position control

Function stepper_step_angle(float angle, int direction, int rpm) is position control for a step motor.

- angle, position(Incremental, unit: degree).

- direction, 0 is clockwise and 1 is anti-clockwise.

- rpm, step motor velocity(unit: rpm).

------

### Reference

[1]. Interface Stepper Motor With STM32. https://controllerstech.com/interface-stepper-motor-with-stm32/

[2]. Interfacing Stepper Motor with STM32F103C8T6. https://www.electronicshub.org/interfacing-stepper-motor-with-stm32f103c8t6/

[3]. STM32H743ZI MCU. https://www.st.com/en/evaluation-tools/nucleo-h743zi.html

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
