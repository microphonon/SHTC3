# SHTC3
Demo code to read and display humidity and temperature data from Sensirion SHTC3 using MSP430F5529 Launchpad.
Low-level I2C communication using registers, interrupts, and low-power modes.
Read sensor in low-power mode with clock stretching.  Display data on terminal program.
Humidity displayed with 4 significant figures; temperature 3 significant figures.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support. The following terminals are used:

P3.0  SDA with 10k pullup
<br>P3.1  SCL with 10k pullup
<br>P3.3  UART TXD
<br>P3.4  UART RXD

Sensirion product page: https://www.sensirion.com/shtc3

Digi-Key video: https://www.youtube.com/watch?v=X7GWMmYFZ9I

