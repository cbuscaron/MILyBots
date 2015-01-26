/*****************************************************
Project : MILyBots Robot Firmware
Version : 
Date    : 2008-2009
Author  : Camilo Buscaron, Luis Vega, Devin Hughes,
Comments: 


Chip 	            : ATmega128L
Clock frequency     : 8.0000 MHz
*****************************************************/
#include <avr\io.h>
#include <avr\interrupt.h>
#include <stdlib.h> 

/******************* Defines ************************/

/* Delay Function */
void delay(int times);

/* UART Defines */
#define CPU_Speed 8000000L 
#define UART0_Baud 9600L
#define UART0_RR ((CPU_Speed/(16L*UART0_Baud))-1)
#define UART1_Baud 9600L
#define UART1_RR ((CPU_Speed/(16L*UART1_Baud))-1)

/* Motor Defines */
#define ServoA OCR1A
#define ServoB OCR1B
#define Stall 750
#define MotorRange 250

/* ADC Defines */
#define ADC_FREQ  25L
#define OCR0_RR  ((CPU_Speed/(2L*(1024L)*ADC_FREQ))-1)


/************** Function Prototypes *****************/

/* PWM Functions */
void Init_PWM(void);
void setMotor(char motor, int speed); 

/* UART Functions */
void Init_UART(int UART);
void UART_Transmit(int UART, unsigned char data);
void UART_String(int UART, unsigned char data[]);

/* SWI Functions */
void Init_SWI(void);
void SWI(void);

/* ADC Functions */
void adc_init(void);
void adc_chsel(uint8_t channel);
void adc_wait(void);
void adc_start(void);
uint16_t adc_read(void);
uint16_t adc_readn(uint8_t channel, uint8_t n);
void ADC_SampleAll(uint8_t numChannels, uint8_t number);


/* IR Functions */
void Init_IRControl(void);
void IR_Enable(uint8_t IR);
void IR_Disable(uint8_t IR);
int IR_GetValue(uint8_t IR);
int right, left;

/* Bump/Switch Functions */
void Init_BumpPort(void);
char BumpPort_Pressed(unsigned char bump);

/* LED Functions */
void Init_LED(void);
void LED_Set(char character,char decimal);

/* Helper Functions */
void intToString(int number, char* result);
void Init_Timer0(void);
void HandleCommand(void);



/************** Global Variables *******************/

/* ADC Global */
int glbADC_Results[7];	// Array to hold integer ADC results
char glbADC_Str[6];	// String to hold ADC value
unsigned char glbADC_Counter = 0;	// ADC Result Index

/* Bump/Switch Port Global */
char glbBump_Pressed[7];	// Array indicating pressed bumps

/* UART Global */
unsigned char glbUART1_RXBuff[32]; // UART Receive buffer
unsigned int glbUART1_RXCounter = 0;	// UART Receive Index
int glbUART1_RXMSGCom = 0; // UART Complete message received flag

/****************** Main Function *******************/

int main(void)
{
	cli();
	
	int i,j;

	Init_PWM();	
	Init_BumpPort();
	//Init_SWI();
	adc_init();
	Init_Timer0();
	Init_UART(1);
	Init_LED();

	for(i=0;i<1000;i++)for(j=0;j<850;j++);;  /// creates a delay

	glbUART1_RXCounter = 0;

	sei();	

//	IR_Enable(1);
//	IR_Enable(2);
//	IR_Enable(3);


    /* Program behavior starts here  */
	
	
	setMotor('A', 0);
	setMotor('B', 0); 

	LED_Set('2', 'D');
	

	while(1)
	{	
		if(glbUART1_RXMSGCom == 1)
		{
			LED_Set('B', 'D');
			HandleCommand();
			
		}
		
		
		LED_Set('2', 'D');
	}

	

	return 0;

}




/******************* PWM Section **********************/

/******************************************************
Function: Init_PWM
Description: Initializes the PWM system
Accepts: Nothing
Returns: Nothing
******************************************************/
void Init_PWM(void)
{

/*	TCCR1A and TCCR1B control the waveform generated on OC1 
	Channels A,B,C. Two servos are used to drive the wheels
	(OC1A and OC1B); the other two are disconnected. PWM should
	be 50Hz with 1-2ms pulses.

	TCCR1A:
	|	7	 |	 6	  |	   5   |	4	|    3	 |	 2	  |   1	  |	  0   |
	| COM1A1 | COM1A0 | COM1B1 | COM1B0 | COM1C1 | COM1C0 | WGM11 | WGM10 |
	|   1	 |   0    |    1   |    0   |    0   |   0    |   0   |   0   |

	Above:
		PWM Phase and Frequency Correct Mode
		Clear OC pin on Up count, Set on downcount channel A and B

	TCCR1B:
	|	7   |	6   |   5   |	4	|   3   |   2  |   1  |	  0  |
	| ICNC1 | ICES1 |   -   | WGM13 | WGM12 | CS12 | CS11 | CS10 |
	|   0   |   0   |   0   |   1   |   0   |   0  |   1  |   0  |

	Above:
		PWM Phase and Frequency Correct Mode
		Clock Input Select, /8 Prescaler(TCNT 0-10000)

	ICR1: Defines the counter top value. With 8MHz clock, and
	/8 prescaler TOP = 10000.

	OCR1x: Defines the pulse duty cycle. Servos in robot are centered
	around 410us pulse length. */

	DDRB |= 0x60;

	TCCR1A = 0xA0;  // 0b1010 0000
	TCNT1 = 0;		
	TCCR1B = 0x12; /// 0b0001 0010
	ICR1 = 10000; 
	
	ServoA = Stall;	// Stop both servos in initialization
	ServoB = Stall; 

}

/*******************************************************
Function: setMotor
Description: Set the motor power value between -100 and
			 100 percent. Left motor is channel A, right 
			 motor is channel B.
Accepts: Motor channel'A' or 'B', int speed between -100
		 and 100
Returns: Nothing
******************************************************/
void setMotor(char motor, int speed)
{
	/*ServoA : Forward is shorter pulse duration*/
	if(motor =='A' || motor =='a')
	{
		if(speed >= 0)
		{
			ServoA = Stall - ((MotorRange * speed) / 100);
		}
	
		else
		{
			speed *= -1;
			ServoA = Stall + ((MotorRange * speed) / 100);
		}		
	}
	
	/*ServoB : Forward is longer pulse duration*/
	if(motor =='B' || motor =='b')
	{
		if(speed >= 0)
		{
			ServoB = Stall + ((MotorRange * speed) / 100);
		}
	
		else
		{
			speed *= -1;
			ServoB = Stall - ((MotorRange * speed) / 100);
		}		
	}
}

/*---------------- END PWM Section -------------------*/




/******************* UART Section *********************/

/*******************************************************
Function: Init_UART
Description: Initializes the selected UART to desired baud
Accepts: UART to initialize(0 or 1), Baud rate
Returns: Nothing
******************************************************/
void Init_UART(int UART)
{
	if(UART == 0)
	{
		UCSR0A |= 0x00;	// Enable normal UART transmission speed
		UCSR0B |= 0x18;	// Enable Transmit and receive
		UCSR0C |= 0x05;	// Enable 8 bit mode, 1 stop bit, no parity

		UBRR0H = (unsigned char)(UART0_RR>> 8);
		UBRR0L = (unsigned char) UART0_RR;

	}
	else if(UART == 1)
	{
		UCSR1A |= 0x00;	// Enable normal UART transmission speed
		UCSR1B |= 0x98;	// Enable Transmit and receive, turn on RX interrupt
		UCSR1C |= 0x05;	// Enable 8 bit mode, 1 stop bit, no parity

		UBRR1H = (unsigned char)(UART1_RR >> 8);
		UBRR1L = (unsigned char) UART1_RR;
	}	
}

/*******************************************************
Function: UART_Transmit
Description: Transmits a single character out of selected
			 UART
Accepts: UART(0 or 1), character
Returns: Nothing
******************************************************/
void UART_Transmit(int UART, unsigned char data)
{
	if(UART == 0)
	{
		while ( !( UCSR0A & (1<<UDRE0)) );	// Wait for empty transmit buffer
		UDR0 = data;	// Put data into buffer, sends the data
	}
	else if(UART ==1)
	{
		while ( !( UCSR1A & (1<<UDRE1)) );	// Wait for empty transmit buffer
		UDR1 = data;	// Put data into buffer, sends the data
	}
}

/*******************************************************
Function: UART_String
Description: Transmits a string using selected UART
Accepts: UART(0 or 1), string
Returns: Nothing
*******************************************************/
void UART_String(int UART, unsigned char data[])
{
	int i = 0;
	
	while(data[i] != '\0') ++i;

	unsigned char length = i + 14;

	//Build API packet

	//TODO: fix this function
	unsigned char header[] = {0x7E,0x00,length,0x10,0x00,0x00,
		0x13,0xA2,0x00,0x40,0x00,0xC4,0x2A,0x00,0x00,0x00,0x00};

	unsigned char transmit[length + 4];

	for(i = 0 ; i < 17 ; i++)
	{
		transmit[i] = header[i];
	}

	for(i = 17; i < length + 3; i++)
	{
		transmit[i] = (unsigned char)data[i-17];
	}

	unsigned int checkSum = 0;

	for(i = 3; i < length + 3 ; i++) checkSum += transmit[i];

	unsigned char byteCheck = (unsigned char)((unsigned char)0xFF & (unsigned char)checkSum);

	transmit[length + 3] = (unsigned char)((unsigned char)0xFF - byteCheck);

	for(i = 0; i < length + 4; i++)
		UART_Transmit(UART, transmit[i]);
}

/*******************************************************
Function: ISR for UART RX
Description: Handles an incoming char on UART1
Accepts: Nothing
Returns: Byte in glbUART1_RXBuff, increments counter,
		 sets complete message flag
*******************************************************/

ISR(USART1_RX_vect)
{
	glbUART1_RXBuff[glbUART1_RXCounter] = UDR1;	// Copy the received char
												// to buffer	
	
	if(glbUART1_RXBuff[glbUART1_RXCounter] == ';')
	{											// Set complete flag
		glbUART1_RXMSGCom = 1;					// if delimiter matches
	}
	
	++glbUART1_RXCounter;	// Increment the counter	*/	
}

/*--------------- END UART Section -------------------*/




/************ Software Interrupt Section **************/

/*******************************************************
Function: Init_SWI
Description: Initializes the XIRQ on INT1. By toggling
			 this output pin, user can create software
			 interrupt. Use SWI() to fire the interrupt.
Accepts: Nothing
Returns: Nothing
******************************************************/
void Init_SWI(void)
{
/*	By toggling an XIRQ configured as an output software
	driven interrupt will be possible.

	EICRA:
	|	7   |	6   |   5   |	4	|   3   |   2	|   1	|	0   |
	| ISC31 | ISC30 | ISC21 | ISC20 | ISC11 | ISC10 | ISC01 | ISC00 | 
	|   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |

	Above:
		Enables the Interrupt on the rising edge of pin INT1.
 */
 	DDRD |= 0x02;	// Pin PD1 is an output
	PORTD &= 0xFD;	// Set pin low
	EIMSK &= 0xFD;	// Ensure XIRQ interrupt is off
	EICRA |= 0x0C;	// Interrupt on rising edge
	EIFR |= 0x02;	// Ensure the interrupt flag is clear
	EIMSK |= 0x02;	// Enable the interrupt
}

/*******************************************************
Function: SWI
Description: Fires the software interrupt
Accepts: Nothing
Returns: Nothing
******************************************************/
void SWI(void)
{
	PORTD |= 0x02;	// Set the interrupt pin
}

/*******************************************************
Function: INT1_ISR
Description: Handler for the pseudo-software interrupt
Accepts: Nothing
Returns: Nothing
******************************************************/
ISR(INT1_vect)
{
	PORTD &= 0xFD;	// Clear the interrupt pin
	PORTA = ~(PORTA);
}

/*--------- END Software Interrupt Section -----------*/



/******************** ADC Section *********************/

/*******************************************************
Function: Init_ADC
Description: Initializes the ADC System
Accepts: Nothing
Returns: Nothing
*******************************************************/
void adc_init(void)
{
  /* configure ADC port (PORTF) as input */
  DDRF  = 0x00;
  PORTF = 0x00;

  ADMUX = _BV(REFS0);
  ADCSR = _BV(ADEN)|_BV(ADSC)|_BV(ADFR) | _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
}

/*******************************************************
Function: ADC_Channel
Description: Sets the current ADC mux channel(0-7)
Accepts: Channel Number
Returns: Nothing
*******************************************************/
void adc_chsel(uint8_t channel)
{
  /* select channel */
  ADMUX = (ADMUX & 0xe0) | (channel & 0x07);
}

/*******************************************************
Function: ADC_Poll
Description: Waits for AD conversion to complete
Accepts: Nothing
Returns: Nothing
*******************************************************/
void adc_wait(void)
{
  /* wait for last conversion to complete */
  while ((ADCSR & _BV(ADIF)) == 0)
    ;
}

/*******************************************************
Function: ADC_Read
Description: Retrieves value in ADC register
Accepts: Nothing
Returns: ADC integer value
*******************************************************/
uint16_t adc_read(void)
{
  return ADC;
}

/*******************************************************
Function: ADC_Start
Description: Starts a new conversion cycle
Accepts: Nothing
Returns: Nothing
*******************************************************/
void adc_start(void)
{
  /* clear conversion, start another conversion */
  ADCSR |= _BV(ADIF);
}

/*******************************************************
Function: ADC_Sample
Description: Samples selected AD channel n times and returns
			 average value
Accepts: Channel Number, number of samples
Returns: Nothing
*******************************************************/
uint16_t adc_readn(uint8_t channel, uint8_t n)
{
  uint16_t t;
  uint8_t i;

  adc_chsel(channel);
  adc_start();
  adc_wait();

  adc_start();

  /* sample selected channel n times, take the average */
  t = 0;
  for (i=0; i<n; i++) {
    adc_wait();
    t += adc_read();
    adc_start();
  }

  /* return the average of n samples */
  return t / n;
}


/*******************************************************
Function: ADC_SampleAll
Description: Samples ADC channels in order 
Accepts: number of channels to sample in order,
		 number of samples to average per channel
Returns: ADC values in an int buffer labeled glbADC_Results
*******************************************************/
void ADC_SampleAll(uint8_t numChannels, uint8_t number)
{
	unsigned char i;

	for(i=0;i<numChannels;i++)
	{		
		glbADC_Results[i] = adc_readn(i,number);
	}
}

/*----------------- END ADC Section ------------------*/




/******************** IR Functions *******************/

/*******************************************************
Function: Init_IRControl
Description: Enables user selectable IR.
Accepts: Nothing
Returns: Nothing
*******************************************************/
void Init_IRControl(void)
{
	DDRE |= 0x70;	// Set the control pins to outputs
	PORTE &= 0x8F;	// Turn off IR by default
}

/*******************************************************
Function: IR_Enable
Description: Turns on the selected IR(1,2,3). Passing any
			other valid char will turn on all three. User 
			must use function Init_ADC BEFORE using this
			function. 
Accepts: IR to turn on
Returns: Nothing
*******************************************************/
void IR_Enable(uint8_t IR)
{
	if(IR == 1)
	{
		PORTE |= 0x10;	// Turn on IR1
	}

	else if(IR == 2)
	{
		PORTE |=0x20;	// Turn on IR2
	}

	else if (IR == 3)
	{
		PORTE |=0x40;	// Turn on IR3
	}

	else
	{
		PORTE |= 0x70;	// Turn on all 3
	}
}

/*******************************************************
Function: IR_Disable
Description: Turns off the selected IR(1,2,3). Passing any
			 other valid char will turn off all three. N.B!!:
			 IR is configured to read HIGH on the analog port
			 when it is disabled.
Accepts: IR to turn on
Returns: Nothing
*******************************************************/
void IR_Disable(uint8_t IR)
{
	if(IR == 1)
	{
		PORTE &= 0xEF;	// Turn off IR1
	}

	else if(IR == 2)
	{
		PORTE &=0xDF;	// Turn off IR2
	}

	else if (IR == 3)
	{
		PORTE &=0xBF;	// Turn off IR3
	}

	else
	{
		PORTE &= 0x8F;	// Turn off all 3
	}
}

/*******************************************************
Function: IR_GetValue
Description: Samples the IR analog pin 4 times and returns
			 int value.
Accepts: IR to use(1,2,3)
Returns: Nothing
*******************************************************/
int IR_GetValue(uint8_t IR)
{
	return adc_readn(IR - 1,4);	
}
/*----------------- END IR Section ------------------*/




/*************** Bump/Switch Functions ***************/

/*******************************************************
Function: Init_BumpPort
Description: Sets the bumper/switch port to inputs.
Accepts: Nothing
Returns: Nothing
*******************************************************/
void Init_BumpPort(void)
{
	DDRC = 0; // Set PORTC to inputs
}

/*******************************************************
Function: BumpPort_Pressed
Description: Checks to see if the designated button is
			 pressed.
Accepts: Bump to check
Returns: Char value for pressed or not (1: pressed,
		 0: not pressed). If invalid port selected,
		 returns -1.
*******************************************************/
char BumpPort_Pressed(unsigned char bump)
{	
	switch(bump)
	{
		case 0:
			if((PINC & 0x01) != 0) return 0;
			else return 1;
		case 1:
			if((PINC & 0x02) != 0) return 0;
			else return 1;
		case 2:
			if((PINC & 0x04) != 0) return 0;
			else return 1;
		case 3:
			if((PINC & 0x08) != 0) return 0;
			else return 1;
		case 4:
			if((PINC & 0x10) != 0) return 0;
			else return 1;
		case 5:
			if((PINC & 0x20) != 0) return 0;
			else return 1;
		case 6:
			if((PINC & 0x40) != 0) return 0;
			else return 1;	
		case 7:
			if((PINC & 0x80) != 0) return 0;
			else return 1;		
		default:
			return -1;
	}

	return 0;
}

/*------------ END Bump/Switch Section --------------*/




/******************* LED Functions *******************/

/*******************************************************
Function: Init_LED
Description: Sets PORTA to handle LED
Accepts: Nothing
Returns: Nothing
*******************************************************/
void Init_LED(void)
{
	DDRA = 0xFF;	// Port A outputs
	PORTA = 0xFF;	// Turn off the LED
}

/*******************************************************
Function: LED_Set
Description: 
Accepts: character(0-F), decimal(D: show, other: dont);
         K=turn on a, g, d 
		 L=turn on f, e
		 M=turn on b, c
		 N=turn on f, c
		 O=turn on b, e
		 P=turn on g, e, d
		 Q=turn on a, b, g, f
		 Z=turn on g ,c, d
Returns: Nothing
*******************************************************/
void LED_Set(char character, char decimal)
{
	if (!(character == '1' || character == '4' || 	// Bit 0
		character == 'B' || character == 'b' ||
		character == 'D' || character == 'd' ||
		character == 'L' || character == 'l' ||
		character == 'M' || character == 'm' ||
		character == 'N' || character == 'n' ||
		character == 'O' || character == 'o' ||
		character == 'P' || character == 'p' ||
		character == 'Q' || character == 'q' ))
	{
		 PORTA &= 0xFE;
	}
	else PORTA |= 0x01;

	if(!(character == '5' || character == '6' || 	// Bit 1
		character == 'B' || character == 'b' ||
		character == 'C' || character == 'c' ||
		character == 'E' || character == 'e' ||
		character == 'F' || character == 'f' ||
		character == 'K' || character == 'k' ||
		character == 'L' || character == 'l' ||
		character == 'N' || character == 'n' ||
		character == 'P' || character == 'p' ||
		character == 'Q' || character == 'q' ))
	{
		PORTA &= 0xFD;
	}
	else PORTA |= 0x02;

	if(!(character == '2' ||					 	// Bit 2
		character == 'C' || character == 'c' ||
		character == 'E' || character == 'e' ||
		character == 'F' || character == 'f' ||
		character == 'K' || character == 'k' ||
		character == 'L' || character == 'l' ||
		character == 'O' || character == 'o' ||
		character == 'P' || character == 'p' ||
		character == 'Z' || character == 'z'))
	{
		PORTA &= 0xFB;
	}
	else PORTA |= 0x04;
	
	if(!(character == '1' || character == '4' || 	// Bit 3
		character == '7' || 
		character == 'A' || character == 'a' ||
		character == 'F' || character == 'f' ||
		character == 'L' || character == 'l' ||
		character == 'M' || character == 'm' ||
		character == 'N' || character == 'n' ||
		character == 'O' || character == 'o' ||
		character == 'Z' || character == 'z' ))
	{
		PORTA &= 0xF7;
	}
	else PORTA |= 0x08;

	if(!(character == '1' || character == '3' || 	// Bit 4
		character == '4' || character == '5' ||
		character == '7' || character == '9' ||
		character == 'K' || character == 'k' ||
        character == 'M' || character == 'm' ||
		character == 'N' || character == 'n' ||
        character == 'Z' || character == 'z' ))                        
	{               
		PORTA &= 0xEF;
	}                
	else PORTA |= 0x10;

	if(!(character == '1' || character == '2' || 	// Bit 5
		character == '3' || character == '7' ||
		character == 'D' || character == 'd' ||
		character == 'K' || character == 'k' ||
		character == 'M' || character == 'm' ||
		character == 'P' || character == 'p' ||
		character == 'O' || character == 'o' ||
		character == 'Q' || character == 'q' ))
	{
		PORTA &= 0xDF;
	}
	else PORTA |= 0x20;

	if(!(character == '0' || character == '1' || 	// Bit 6
		character == '7' || 
		character == 'C' || character == 'c' ||
		character == 'L' || character == 'l' ||
		character == 'M' || character == 'm' ||
		character == 'N' || character == 'n' ||
		character == 'O' || character == 'o' ))
	{
		PORTA &= 0xBF;
	}
	else PORTA |= 0x40;

	if(decimal == 'D' || decimal == 'd') 
	    PORTA &= 0x7F;	// Bit 7
	else 
	    PORTA |= 0x80;
}

/*----------------- END LED Section ------------------*/




/****************** Helper Functions ******************/

void intToString(int number, char* result)
{
    if(number == 0)
       *result = '0';
    int i = 0;
    int temp = number;
	
	int k = (number > 9999) ? 10000 : (number > 999) ? 1000 : (number > 99) ? 100 : (number > 9) ? 10 :1;
    while(number != 0)
    {
        temp = number / k;   
        *(result+(i++)) = (temp == 0) ? '0' : (temp == 1) ? '1' : (temp == 2) ? '2' : (temp == 3) ? '3' :
		      (temp == 4) ? '4' : (temp == 5) ? '5' : (temp == 6) ? '6' : (temp == 7) ? '7' :
		      (temp == 8) ? '8' : '9';
		number -= temp*k;
		k /= 10;
		
    }
	*(result+i) = '\0';
}

/*******************************************************
Function: Init_Timer0
Description: Initializes timer 0. This timer is used to
			 set the frequency of ADC checks.
Accepts: Nothing
Returns: Nothing
******************************************************/
void Init_Timer0(void)
{
	TCCR0 = 0;
	TIFR |= 0x03;
	TIMSK |= 0x02;
	TCCR0 = 0x0F;
	TCNT0 =0;
	OCR0 = OCR0_RR;
}

/*******************************************************
Function: Timer 0 OC ISR
Description: Handler for timer 0 interrupt. This is used
			 to set the ADC sampling frequency.
Accepts: Nothing
Returns: Nothing
*******************************************************/
ISR(TIMER0_COMP_vect)
{

}

/*******************************************************
Function: Handle Command
Description: Main Loop function. Executes when a new message
			 has been received over UART1.
Accepts: Nothing
Returns: Nothing
*******************************************************/
void HandleCommand(void)
{

	//Constants
	static unsigned char InvCommand[] = "Invalid Command";	
	//static unsigned char InvAdminCommand[] ="Invalid Admin Command";
	//static unsigned char InvMotorCommand[] = "Invalid Motor Command";
	static unsigned char InvIRCommand[] = "Invalid IR Command";
	static unsigned char InvLEDCommand[] = "Invalid LED Command";
	//static unsigned char replyOK[] = "OK";
	//
	unsigned char completeMessage[32];		// Get message into private
	int i;

	//char size = 0;
	//intToString(glbUART1_RXCounter, &size);
	//LED_Set(size, 'P');
	
	for(i=0;i<glbUART1_RXCounter;i++)
	{
		completeMessage[i] = glbUART1_RXBuff[i];	// buffer.
	}	

	
	glbUART1_RXMSGCom = 0;	// Reset the receive buffer for
	glbUART1_RXCounter = 0;	// new messages

	unsigned int ind = 0;

	// Handle the command
	
	if(completeMessage[ind] == (unsigned char)'M' || completeMessage[ind] == 'm')	// Motor Command
	{
		ind++;
		
		int speed100;
		int speed10;
		int speed0;
		int totalSpeed;

		speed100 =	(int)(completeMessage[ind+2] & 0x0F);
		speed10 = (int)(completeMessage[ind+3] & 0x0F);
		speed0 = (int)(completeMessage[ind+4] & 0x0F);

		totalSpeed = (speed100*100) + (speed10*10) + speed0;
		
		if(completeMessage[ind+1] == (unsigned char)'-') 
		   totalSpeed *= -1;
		
		setMotor((char)completeMessage[ind],totalSpeed);			
		
		return;

	}

	if(completeMessage[ind] == 'I' || completeMessage[ind] == 'i')
	{
		ind++;

		if(completeMessage[ind] == 'I' || completeMessage[ind] == 'i') 
		{
			IR_Enable(completeMessage[ind+1]);
		}

		else if(completeMessage[ind] == 'O' || completeMessage[ind] == 'o') 
		{
			IR_Disable(completeMessage[ind+1]);
		}

		else UART_String(1,InvIRCommand);
		return;

	}

	if(completeMessage[ind] == 'L' || completeMessage[ind] == 'l')
	{
		ind++;
		if(completeMessage[ind+1] == ';')
		{
			UART_String(1,InvLEDCommand);
		}
		else LED_Set(completeMessage[ind],completeMessage[ind+1]);
		return;
	}

	else 
	{
		UART_String(1,InvCommand);
	}
}

/*******************************************************
Function: delay
Description: creates a multiple delay of ~ 1sec*times
Accepts: int times, number that will multiply 2sec
Returns: Nothing
*******************************************************/
void delay(int times)
{
   int i,j,k;

   for(k=0; k < times; ++k)
   {
       for(i=0;i<100;i++)for(j=0;j<100;j++);;  /// creates a delay
   }
}

/*-------------- END Helper Functions ----------------*/
