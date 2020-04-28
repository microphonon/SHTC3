#include <msp430.h> 
#include <stdio.h>
#include <stdint.h>

/*
Demo code to read and display humidity and temperature data from Sensirion SHTC3 using MSP430FR969 Launchpad.
Low-level I2C communication using registers, interrupts, and low-power modes.
Read sensor in low-power mode with clock stretching.  Display data on terminal program.
Humidity displayed with 4 significant figures; temperature 3 significant figures.
Main loop runs with timed interrupt from LPM3 and VLO clock. I2C clock 100 kHz; UART 9600 baud.
IDE with CCS 6.1.3 and nofloat printf support.

P1.6  UCB0SDA with 10k pullup
P1.7  UCB0SCL with 10k pullup
P2.5  UCA1TXD
P2.6  UCA1RXD

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
volatile uint32_t H,RH,T,TC;

volatile uint8_t i,count;

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
    	P1OUT |= BIT0; //Timeout. Turn on green LED on Launchpad

    	UCB0IE |= UCTXIE0 + UCRXIE0; //Enable TX and RX I2C interrupts

    	Wakeup();	//Wakeup from sleep mode
    	 __delay_cycles(100); //Give sensor time to wakeup before measuring
    	Measure();	//Get raw humidity and temperature data from sensor
    	Sleep();	//Measurement done; put sensor to sleep

    	UCB0IE &= ~(UCRXIE0 + UCTXIE0); //Disable I2C interrupts
    	//Ignore the 2 CRC bytes in *(PRxData+3) and *PRxData
    	//Process 16-bit raw humidity data
    	H = ((uint16_t)(*(PRxData+5)) << 8)|(uint16_t)*(PRxData+4);
    	//Convert humidity data with 4 significant figures
    	RH = ((H<<13) + (H<<11) - (H<<8) + (H<<4)) >> 16; //Corrected humidity without decimal point

    	//Process 16-bit raw temperature data
    	T = ((uint16_t)(*(PRxData+2)) << 8)|(uint16_t)*(PRxData+1);
    	//Convert temperature data with 3 significant figures
    	TC = (((T<<11) - (T<<8) - (T<<5) - (T<<3) - (T<<1)) >> 16) - 0x1C2; //No decimal point
    	//Convert temperature data with 2 significant figures
    	//TC = (((T<<7) + (T<<5) + (T<<4) - 0x01) >> 16) - 0x2D;

    	//Display data on terminal
    	sprintf(str,"%s %lu.%.1lu%s %lu.%.2lu%s", "Temperature:", (int32_t)(TC/10),(int32_t)(TC%10),
    	    			"C Rel Humidity:", (int32_t)(RH/100),(int32_t)(RH%100),"%\r\n\n");
    	count = sizeof str;
    	for (i=0; i < count; i++)
    	{
    		while (!(UCA1IFG & UCTXIFG)); //Poll serial: USCI_A0 TX buffer ready?
    		UCA1TXBUF = str[i]; //Send data 1 byte at a time
    	}
    	P1OUT &= ~BIT0; //Turn off green LED
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
	switch(__even_in_range(UCB0IV,30))
	{
            case 0: break;         // Vector 0: No interrupts
	    case 2: break;         // Vector 2: ALIFG
	    case 4: break;          // Vector 4: NACKIFG
	    case 6: break;         // Vector 6: STTIFG
	    case 8: break;         // Vector 8: STPIFG
	    case 10: break;         // Vector 10: RXIFG3
	    case 12: break;         // Vector 12: TXIFG3
	    case 14: break;         // Vector 14: RXIFG2
	    case 16: break;         // Vector 16: TXIFG2
	    case 18: break;         // Vector 18: RXIFG1
	    case 20: break;         // Vector 20: TXIFG1
	    case 22:                 // Vector 22: RXIFG0
	    	 RXByteCtr--;        // Decrement RX byte counter
	    	 if (RXByteCtr) //Execute the following of counter not zero
	    	 	 {
	    		 	 *(PRxData + RXByteCtr) = UCB0RXBUF; // Move RX data to address PRxData
	    			 if (RXByteCtr == 1)     // Only one byte left?
	    			 UCB0CTL1 |= UCTXSTP;    // Generate I2C stop condition BEFORE last read
	    	 	 }
	    	 else
	    	 	 {
	    		 	 *PRxData = UCB0RXBUF;   // Move final RX data to PRxData(0)
	    		 	 LPM0_EXIT; 			// Exit active CPU
	    	 	 }
	    	 break;
	    case 24:       		// Vector 24: TXIFG0
	    	if (TXByteCtr)      // Check TX byte counter not empty
	   	    	{
	    			UCB0TXBUF = *PTxData++; // Load TX buffer
	    			TXByteCtr--;            // Decrement TX byte counter
	   	    	}
	   	    else
	   	    	{
	   	    		UCB0CTL1 |= UCTXSTP;        // I2C stop condition
	   	    		UCB0IFG &= ~UCTXIFG0;        // Clear USCI_B0 TX int flag
	   	    		LPM0_EXIT; 					// Exit LPM0
	   	    	}
	   	    break;
	    case 26: break;        // Vector 26: BCNTIFG
	    case 28: break;         // Vector 28: clock low timeout
	    case 30: break;         // Vector 30: 9th bit
	    default: break;
	}
}

 void SetPins(void)
  {
	 PM5CTL0 &= ~LOCKLPM5; //Unlocks GPIO pins at power-up
 	 /* Port 1
 	    P1.0 Green LED
 	    P1.1 Launchpad switch
 	    P1.6 SDA I2C
 	    P1.7 SCL I2C
 	  */
 	  P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;
 	  P1SEL1 |= BIT6 + BIT7; //Setup I2C on UCB0
 	  P1OUT &= ~BIT0; //LED off

	/* Port 2
 	   P2.1  Button on Launchpad
 	   P2.5 TXD UART
 	   P2.6 RXD UART
 	*/
	P2DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT7;
 	P2SEL1 |= BIT5 + BIT6; //Setup UART on UCA1

	/* Port 3 */
 	P3DIR |=  BIT0 + BIT1 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	/* Port 4
 	P4.6 Red LED
 	*/
 	P4DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
 	P4OUT &= ~BIT6; //LED off
  }

 void SetVLO(void)
 {
	 CSCTL0 = CSKEY; //Password to unlock the clock registers
	 //Default frequency ~ 10 kHz
	 CSCTL2 |= SELA__VLOCLK;  //Set ACLK to VLO
	 CSCTL0_H = 0xFF; //Re-lock the clock registers
    }

 void SetTimer(void)
 {
	 //Enable the timer interrupt, MC_1 to count up to TA0CCR0, Timer A set to ACLK (VLO)
	 TA0CCTL0 = CCIE;
	 TA0CTL |= MC_1 + TASSEL_1;
 }

 void SetUART(void) //UCA1 module; do simple polling instead of interrupts
  {
 	 UCA1CTLW0 |= UCSWRST;
 	//Next line selects SMCLK which is DCO (default frequency: 1 MHz)
 	 UCA1CTLW0 |=  UCSSEL1; //This writes 0x80 which sets BIT7
 	 //Next two lines divide 1 MHz to get 9600 baud
 	 UCA1BRW = 0x06;
 	 UCA1MCTLW |= UCOS16 + UCBRF3 + UCBRS5;
 	 UCA1CTLW0 &= ~UCSWRST;
  }

 void SetI2C(void)
   {
  	 // Configure the eUSCI_B0 module for I2C at 100 kHz
	 UCB0CTLW0 |= UCSWRST;
	 //Select SMCLK, Master, synchronous, I2C
	 UCB0CTLW0 |=  UCSSEL__SMCLK + UCMST + UCSYNC + UCMODE_3;
	 UCB0BRW = 10; 	//Divide SMCLK by 10 to get ~100 kHz
	 UCB0I2CSA = 0x70; // SHTC3 address
	 UCB0CTLW0 &= ~UCSWRST; // Clear reset
   }

 void Sleep(void)
 {
	 const uint8_t TS[] = {0xB0,0x98};
	 UCB0CTL1 |= UCTR;  		//Set as transmitter
	 PTxData = (uint8_t *)TS;
	 TXByteCtr = 2;             // Load TX byte counter
	 UCB0CTL1 |= UCTXSTT;
	 LPM0;                   	// Remain in LPM0 until all data transmitted
	 while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
 }

 void Wakeup(void)
 {
	  const uint8_t TW[] = {0x35,0x17};
	  UCB0CTL1 |= UCTR;  		// Set as transmitter
	  PTxData = (uint8_t *)TW;
	  TXByteCtr = 2;          	// Load TX byte counter
	  UCB0CTL1 |= UCTXSTT;
	  LPM0;                   	// Remain in LPM0 until all data transmitted
	  while (UCB0CTL1 & UCTXSTP); // Ensure stop condition sent
 }

 void Measure(void)
 {
             const uint8_t ReadSensor[] = {0x44,0xDE};  //Read sensor in low power mode with clock stretching
	     UCB0CTL1 |= UCTR;  //Set as transmitter
	     PTxData = (uint8_t *)ReadSensor;
	     TXByteCtr = 2;     // Load TX byte counter
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
