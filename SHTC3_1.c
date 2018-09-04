#include <msp430.h> 
#include <stdio.h>
#include <stdint.h>

/*
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

September 2018
 */
# define PERIOD 10000 //Samping period. 10000 count is approximately 1 second; maximum is 65535

void SetTimer(void);
void SetVLO(void);
void SetPins(void);
void SetUART(void);
void SetI2C(void);
void Wakeup(void);
void Sleep(void);
void Measure(void);

//Variables for I2C communication
volatile uint8_t *PTxData;   // Pointer to TX data
volatile uint8_t TXByteCtr;
volatile uint8_t *PRxData;  // Pointer to RX data
volatile uint8_t RXByteCtr;
volatile uint8_t RxBuffer[6];   // Allocate 6 bytes of RAM for data

//Variables for UART terminal display
char str[80];
volatile uint32_t H, RH, T, TC;

volatile uint8_t i, count;

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;	//Stop watchdog timer

    SetPins();
    SetVLO();
    SetTimer();
    SetUART();
    SetI2C();

    _BIS_SR(GIE); //Enable global interrupts

    while(1)
    {
    	TA0CCR0 = PERIOD; //Looping period with VLO
    	LPM3;		//Wait in low power mode
    	P4OUT |= BIT7; //Timeout. Turn on green LED on Launchpad

    	UCB0IE |= UCTXIE + UCRXIE; //Enable TX and RX I2C interrupts

    	Wakeup();	//Wakeup from sleep mode
    	Measure();	//Get raw humidity and temperature data from sensor
    	Sleep();	//Measurement done; put sensor to sleep

       	UCB0IE &= ~(UCRXIE + UCTXIE); //Disable I2C interrupts
       	//Ignore the 2 CRC bytes in *(PRxData+3) and *PRxData
    	//Process 16-bit raw humidity data
    	H = ((uint32_t)(*(PRxData+5)) << 8)|(uint32_t)*(PRxData+4);
    	//Convert humidity data with 4 significant figures
    	RH = ((H<<13) + (H<<11) - (H<<8) + (H<<4)) >> 16; //Corrected humidity without decimal point

    	//Process 16-bit raw temperature data
    	T = ((uint32_t)(*(PRxData+2)) << 8)|(uint32_t)*(PRxData+1);
    	//Convert temperature data with 3 significant figures
    	TC = (((T<<11) - (T<<8) - (T<<5) - (T<<3) - (T<<1)) >> 16) - 0x1C2; //No decimal point
    	//Convert temperature data with 2 significant figures
    	//TC = (((T<<7) + (T<<5) + (T<<4) - 0x01) >> 16) - 0x2D;

    	//Display data on terminal
    	sprintf(str,"%s %lu.%.2lu%s %lu.%.1lu%s", "Rel Humidity:", (int32_t)(RH/100),(int32_t)(RH%100),"% Temp:", (int32_t)(TC/10),(int32_t)(RH%10),"C\r\n");
    	count = sizeof str;
    	for (i=0; i < count; i++)
    		{
    	     	 while (!(UCA0IFG & UCTXIFG)); //Poll serial: USCI_A0 TX buffer ready?
    	     	 UCA0TXBUF = str[i]; //Send data 1 byte at a time
    	     }
    	P4OUT &= ~BIT7; //Turn off green LED
    	}
}

#pragma vector=TIMER0_A0_VECTOR
 __interrupt void timerfoo (void)
{
	LPM3_EXIT;
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	switch(__even_in_range(UCB0IV,12))
	  {
	  case  0: break;                           // Vector  0: No interrupts
	  case  2: break;                           // Vector  2: ALIFG
	  case  4: break;                           // Vector  4: NACKIFG
	  case  6: break;                           // Vector  6: STTIFG
	  case  8: break;                           // Vector  8: STPIF
	  case 10:
		  RXByteCtr--;                            // Decrement RX byte counter
		  if (RXByteCtr)
		  {
		 	*(PRxData + RXByteCtr) = UCB0RXBUF; // Move RX data to address PRxData
		 	if (RXByteCtr == 1)                 // Only one byte left?
		 	UCB0CTL1 |= UCTXSTP;                // Generate I2C stop condition
		  }
		  else
		  {
		 	*PRxData = UCB0RXBUF;                // Move final RX data to PRxData(0)
		 	LPM0_EXIT; 							 // Exit active CPU
		  }
		  break;
	  case 12:                                  // Vector 12: TXIFG
	    if (TXByteCtr)                          // Check TX byte counter
	    {
	      UCB0TXBUF = *PTxData++;               // Load TX buffer
	      TXByteCtr--;                          // Decrement TX byte counter
	    }
	    else
	    {
	      UCB0CTL1 |= UCTXSTP;                  // I2C stop condition
	      UCB0IFG &= ~UCTXIFG;                  // Clear USCI_B0 TX int flag
	      LPM0_EXIT; 							// Exit LPM0
	    }
	    break;
	   default: break;
	  }
}

 void SetPins(void)
  {
 	 /* Port 1
 	  * P1.0 Red LED
 	    */
 	    P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
 	    P1OUT &= ~BIT0; //LED off

 	    /* Port 2
 	    P2.1  Button on Launchpad
 		*/
 	   	P2DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

 	    /* Port 3 */
 	   	/*
 	   	 * P3.0  SDA
 	   	 * P3.1  SCL
 	   	 * P3.3	 TXD
 	   	 * P3.4  RXD
 	   	 */
 	    P3SEL |=  BIT0 + BIT1 + BIT3 + BIT4; //Set the I2C and UART lines
 	    P3DIR |= BIT2 + BIT5 + BIT6 + BIT7;

 	    /* Port 4
 	   		P4.1 -- 4.6 unused
 	   		P4.7 Green LED
 	   		*/
 	   	    P4DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
 	   	    P4OUT &= ~BIT7; //Green LED off

 	   	 /* Port 5
 	   	    P5.0 Unused
 	   	    P5.1 Unused
 	   	    P5.2--P5.5 grounded or open as per spec sheet
 	   	  */
 	   	 P5DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

 	   	/* Port 6
 	   	P6.0--6.7 unused
 	   	*/
 	   	P6DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
  }

 void SetVLO(void)
    { //Default frequency ~ 10 kHz
	 UCSCTL4 |= SELA_1;  //Set ACLK to VLO
    }

 void SetTimer(void)
     {
 	 	TA0CCTL0 |= CCIE;  //Enable timer interrupt
 	 	TA0CTL = TASSEL_1 | MC_1;  //Set Timer A to ACLK; MC_1 to count up to TA0CCR0.
     }

 void SetUART(void) //Do simple polling instead of interrupts
  {
 	 UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
 	 UCA0CTL1 |= UCSSEL_2;                     // SMCLK
 	 UCA0BR0 = 6;                              // 1MHz 9600
 	 UCA0BR1 = 0;                              // 1MHz 9600
 	 UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
 	 UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  }

 void SetI2C(void)
   {
  	 // Configure the USCI B0 module for I2C at 100 kHz
	 	 UCB0CTL1 |= UCSWRST;
  	     UCB0CTL0 |= UCMST + UCSYNC + UCMODE_3; //Set as master, synchronous, UCMODE_3 for I2C
  	     UCB0CTL1 = UCSSEL_2 + UCSWRST;  //Select SMCLK
  	     UCB0BR0 = 12; 	//Next 2 lines set SMCLK to 100 kHz
  	     UCB0BR1 = 0;
  	     UCB0I2CSA = 0x70; // SHTC3 address
  	     UCB0CTL1 &= ~UCSWRST; // Clear reset
   }

 void Sleep(void)
 {
	 const uint8_t TS[] = {0xB0,0x98};
	 UCB0CTL1 |= UCTR;  		//Set as transmitter
	 PTxData = (uint8_t *)TS; 	// TX array start address
	 TXByteCtr = 2;             // Load TX byte counter
	 UCB0CTL1 |= UCTXSTT;   	// Start condition
	 LPM0;                   	// Remain in LPM0 until all data transmitted
	 while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
 }

 void Wakeup(void)
 {
	  const uint8_t TW[] = {0x35,0x17};
	  UCB0CTL1 |= UCTR;  		// Set as transmitter
	  PTxData = (uint8_t *)TW; 	// TX array start address
	  TXByteCtr = 2;          	// Load TX byte counter
	  UCB0CTL1 |= UCTXSTT;   	// TX start condition
	  LPM0;                   	// Remain in LPM0 until all data transmitted
	  while (UCB0CTL1 & UCTXSTP); // Ensure stop condition sent
	  __delay_cycles(100); 		//Give sensor time to wakeup before measuring
 }

 void Measure(void)
 {
	 	 const uint8_t ReadSensor[] = {0x44,0xDE};  //Read sensor in low power mode with clock stretching
	     UCB0CTL1 |= UCTR;  //Set as transmitter
	     PTxData = (uint8_t *)ReadSensor;      // TX array start address
	     TXByteCtr = 2;              // Load TX byte counter
	     UCB0CTL1 |= UCTXSTT;   // Start condition
	     LPM0;                   // Remain in LPM0 until all data transmitted
	     while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition got sent
	     //Receive 6 data bytes
	     UCB0CTL1 &= ~UCTR; //Set as receiver
	     PRxData = (uint8_t *)RxBuffer;    // Start of RX buffer
	     RXByteCtr = 6;  // 2 humidity + CRC + 2 temp + CRC
	     UCB0CTL1 |= UCTXSTT; // I2C start condition
	     LPM0;
 }

