# Microprocessor Technology

This repository contains materials and laboratory exercises developed for the **Microprocessor Technology** course.  
The subject introduces the fundamentals of digital and microprocessor systems, covering both theoretical and practical aspects — from basic logic circuits to modern embedded systems and FPGA-based architectures.

---

## Course Overview

The **Microprocessor Technology** course focuses on the architecture, design, and programming of digital and microprocessor-based systems.  
Students learn how to analyze and implement digital logic circuits, configure programmable devices, and develop embedded applications in C/C++ and assembly.

### Topics covered during lectures include:
1. **TTL logic circuits** — construction, characteristics, and timing parameters.  
2. **Assembly and C/C++ programming** for selected microcontrollers in a dedicated IDE.  
3. **FPGA configuration in VHDL** — logic gates, flip-flops, counters, and more.  
4. **Flip-flops and registers** — D, JK, and T types, parallel and serial registers.  
5. **Counters, multiplexers, and decoders** — binary, BCD, and Johnson counters.  
6. **Monostable circuits and oscillators** — generators, phase-locked loops (PLL).  
7. **CMOS logic** — parameters, families, and reliability principles.  
8. **Semiconductor memories** — ROM, EEPROM, RAM, Flash, SDRAM, DDR, etc.  
9. **Testing and debugging** — JTAG protocol, boundary-scan testing.  
10. **Reconfigurable systems (FPGA)** — logic blocks, I/O, interconnects, clock distribution.  
11. **VHDL language** — hardware description, concurrent and sequential statements.  
12. **Digital-to-analog and analog-to-digital conversion** — DAC, ADC, PWM.  
13. **ARM microcontrollers** — architecture, registers, interrupts, and instruction set.  
14. **Intel i386 processor** — architecture, real and protected modes, segmentation, paging.

---

## Laboratory Exercises

The laboratory part focuses on programming and testing embedded systems using STM32 microcontrollers.  
The code examples are implemented in **STM32CubeIDE** and demonstrate basic microcontroller peripherals and interfaces.

### Available examples:

| File | Description |
|------|--------------|
| `7_segment_display.c` | Simple control of a 4-digit 7-segment display. |
| `A2D.c` | Implementation of an analog-to-digital converter (ADC). |
| `Global_interruptions.c` | Configuration and handling of global interrupts. |
| `SPI_MEMS.c` | Communication with an accelerometer via SPI interface. |
| `Serial-UART.c` | UART serial communication example. |

---

## Tools and Environment

- **IDE:** STM32CubeIDE  
- **Language:** C / C++  
- **Microcontroller:** STM32 series (ARM Cortex-M)   


