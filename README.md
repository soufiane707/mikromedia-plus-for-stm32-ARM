# MikroE Mikromedia Plus for STM32F4 – HAL Test Project

This repository contains testing code for the Mikromedia Plus for STM32F4 ARM development board.  
The project is based on the STM32 HAL library and developed using STM32CubeIDE.

---

## Board Information

| Specification | Details |
|----------------|----------|
| **Chip** | STM32F407ZGT6 |
| **Architecture** | ARM 32-bit (Cortex-M4) |
| **Board Type** | Mikromedia Plus for STM32F4 ARM |
| **Display Type** | Mikromedia TFT LCD |
| **Display Size** | 4.3" |
| **Resolution** | 480x272 px |
| **Graphic Controller** | SSD1963 |
| **Touch Screen** | Resistive (STMPE610) |

---

## Onboard Features and Peripherals

- Buzzer  
- USB Host  
- Temperature Sensor  
- SD Card Slot  
- RGB LED  
- RF Module Connector  
- Optical Sensor  
- ON/OFF Switch  
- MP3 Codec  
- IR Receiver  
- External DC Power Source  
- Battery Power Support  
- RTC Backup Battery  
- Accelerometer  
- TFT Display and Touch Interface  

---

## Project Description

This project includes example code for testing and initializing multiple onboard peripherals using STM32 HAL.  
Each test can be run independently to verify proper operation of hardware modules.

### Implemented Tests and Examples
- LCD Display Initialization and Test Patterns (SSD1963)
- Touchscreen Controller Initialization and Coordinate Reading (STMPE610)
- Temperature Sensor Data Reading
- RGB LED Control
- Buzzer Test
- SD Card Initialization and File Access (FatFS)
- USB Host Connection Detection
- Accelerometer Communication (I²C)

---

## Development Environment

- **IDE:** STM32CubeIDE (v1.17.0 or later)  
- **Toolchain:** arm-none-eabi-gcc  
- **Firmware Package:** STM32CubeF4 HAL  
- **Debugger/Programmer:** ST-LINK V2  

---

## Folder Structure

<img width="700" height="1060" alt="image" src="https://github.com/user-attachments/assets/d49f3d57-b5b6-4f40-941d-65848ce8170e" />
<img width="600" height="906" alt="image" src="https://github.com/user-attachments/assets/59f82e01-90e0-4034-902b-764fc4bf0304" />
<img width="700" height="245" alt="image" src="https://github.com/user-attachments/assets/0ac228ae-4c28-4dde-bc0a-f29c043afdca" />

Please refer to Mikroe's page for more details about the board : https://www.mikroe.com/mikromedia-4-stm32f4?srsltid=AfmBOor6x7smwKSUjTmyLXVr3F2tGAdiXetEdB9L3zUjx-F_5r09hRYt
