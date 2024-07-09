# AVR-pressure-multimeter
Universal multimeter for measuring absolute and differential gas pressure

## Pressure meter PCB
![pcb1](/atmega-MPX-PCB-pressure-meter.gif)

![pcb2](/winstar-LCD-PCB-template.gif)

## Atmega328p pinout
### LCD (4-bit data transfer mode)
- Pin 32: PD2 - R/W;
- Pin 31: PD1 - E;
- Pin 30: PD0 - RS;
- Pin 28: PC5 - DB5;
- Pin 27: PC4 - DB4;
- Pin 26: PC3 - DB6;
- Pin 25: PC2 - DB7;
### Pressure sensors
- Pin 24: PC1 (ADC1) - Pressure sensor 1 (NXP MPX4250AP, Absolute, 20..250kPa);
- Pin 23: PC0 (ADC0) - Pressure sensor 2 (NXP MPX4250AP, Differential, 700kPa);
- Pin 19: ADC6 - Pressure sensor 3 (Reserved);

### Buttons
- Pin 9: PD5 - BUTTON1 (Atmega internal PULLUP);
- Pin 11: PD7 - BUTTON2 NC (Reserved);
- Pin 13: PB1 - BUTTON3 NC (Reserved);

