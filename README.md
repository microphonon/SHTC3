# SHTC3
Demo code to read and display temperature and humidity data from Sensirion SHTC3 using the MSP430F5529 or MSP430FR5969 Launchpad.
Low-level I2C communication using UCB0 registers, interrupts, and low-power modes.
Read sensor in low-power mode with clock stretching.  Display data on terminal program.
Humidity displayed with 4 significant figures; temperature 3 significant figures.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.
<p><b>SHTC3_F.c</b>  Code for MSP430Fx5xx/x6xx MCUs.  Launchpad terminals:
<br>P3.0  SDA with 10k pullup
<br>P3.1  SCL with 10k pullup
<br>P3.3  UART TXD
<br>P3.4  UART RXD
  
<p><b>SHTC3_FR.c</b>  Code for MSP430FR59xx MCUs. Launchpad terminals:
<br>P1.6  UCB0SDA with 10k pullup
<br>P1.7  UCB0SCL with 10k pullup
<br>P2.6  UCA1TXD
<br>P2.5  UCA1RXD

Sensirion product page: https://www.sensirion.com/shtc3

Digi-Key video: https://www.youtube.com/watch?v=X7GWMmYFZ9I

![shtc3](https://user-images.githubusercontent.com/25041061/45910425-2a8f2a80-bdc6-11e8-8203-dcc1a3046595.png)

