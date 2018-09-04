# sensors
SHTC3_1: September 2018
Demo code to read and display humidity and temperature data from Sensirion SHTC3 using MSP430F5529 Launchpad.
Low-level I2C communication using registers, interrupts, and low-power modes.
Read sensor in low-power mode with clock stretching.  Display data on terminal program.
Humidity displayed with 4 significant figures; temperature 3 significant figures.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.

P3.0  SDA with 10k pullup
P3.1  SCL with 10k pullup
P3.3  TXD
P3.4  RXD

