# STM32_MOTOR

## Introduction

​	The main functions of this project are: to control 42 and 57 stepper motors to rotate through tb6600 driver, which can rotate clockwise and counterclockwise. The rotation speed and rotation distance can be customized. At the same time, four limit metal sensors are added to limit the position. When the machine body moves to the limit sensor, it stops running.

## Hardware 

​	57 stepper motor (model 57cm18), 42 stepper motor, driver tb6600, development board stm32f407zgt6, sn-4ndo limit metal sensor, the detection surface is about 9mm.

## Software

​	Keil uVision5

## How to use

- Configure IO port and related interrupt line according to the actual situation

- Turndown, turnup, turnright and turnleft functions control two motors to rotate in four directions. Parameters can be adjusted in the functions
- Locate_ RleLR、Locate_ Rleud controls two motors respectively. The first parameter is transferred into PWM wave, that is, displacement, the second parameter is transferred into frequency, that is, rotation speed, and the third parameter is transferred into direction


​	Communication resumebb@163.com Or leave a message on the blog https://blog.csdn.net/qq_ 41573860/article/details/107254090
