/*******************************************************
Chip type               : ATmega328P
AVR Core Clock frequency: 1,000000 MHz
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <mega328p.h>

#include <alcd.h> // Alphanumeric LCD functions

#include <delay.h>
#include <stdio.h>

// global variables
unsigned char lcd_buf[2][16];

unsigned int adc1 = 0;
unsigned int adc6 = 0;

unsigned int adc1_press = 0; // MAP1 - MPX4250AP (250kPa Absolute)
unsigned int adc0_press = 0; // MAP2 - MPX...
unsigned int adc6_press = 0; // MAP3 - MPX5700DP (700kPa Differential)

unsigned int tiks = 0;
unsigned char mode_view = 0; 


// Function for selecting and polling ADC pin (leg) 0,1,2.... one-time, by channel number
int ADCsingleREAD(unsigned char adctouse) {
    unsigned int ADCval;

    ADMUX = adctouse; // Set ADC channel number to use
    //ADMUX |= (1 << REFS0); // use AVcc as the reference
    ADMUX |= (0 << REFS1)|(1 << REFS0); // AREF на Vcc 5v
    ADMUX &= ~(1 << ADLAR); // clear for 10 bit resolution
    
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 samplerate prescale
    ADCSRA |= (1 << ADEN); // Enable the ADC

	delay_us(10); // Necessary delay for stabilizing the ADC input voltage
    ADCSRA |= (1 << ADSC); // Start the ADC conversion
    while(ADCSRA & (1 << ADSC));  // this line waits for the ADC to finish 

    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval; // ADCH is read so ADC can be updated again
   
    return ADCval;
}


void main(void) {
	// Crystal Oscillator division factor: 8
	CLKPR = (1<<CLKPCE);
	CLKPR = (0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0);

	// Input/Output Ports initialization
	// Port B initialization
	// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=Out 
	DDRB = (1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (0<<DDB1) | (1<<DDB0);
	// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=P Bit0=0 
	PORTB = (0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (1<<PORTB1) | (0<<PORTB0);

	// Port C initialization
	// Function: Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=Out 
	DDRC = (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (0<<DDC1) | (1<<DDC0);
	// State: Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
	PORTC = (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

	// Port D initialization
	// Function: Bit7=Out Bit6=Out Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
	DDRD = (1<<DDD7) | (1<<DDD6) | (0<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
	// State: Bit7=0 Bit6=0 Bit5=P Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
	PORTD = (0<<PORTD7) | (0<<PORTD6) | (1<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B = (0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0 = 0x00;
	OCR0A = 0x00;
	OCR0B = 0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	ICR1H = 0x00;
	ICR1L = 0x00;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	ASSR = (0<<EXCLK) | (0<<AS2);
	TCCR2A = (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
	TCCR2B = (0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
	TCNT2 = 0x00;
	OCR2A = 0x00;
	OCR2B = 0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0 = (0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1 = (0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2 = (0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA = (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	EIMSK = (0<<INT1) | (0<<INT0);
	PCICR = (0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

	// USART initialization
	// USART disabled
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR = (1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	ADCSRB = (0<<ACME);
	// Digital input buffer on AIN0: On
	// Digital input buffer on AIN1: On
	DIDR1 = (0<<AIN0D) | (0<<AIN1D);

	// ADC initialization
	// ADC disabled
	ADCSRA = (0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

	// SPI initialization
	// SPI disabled
	SPCR = (0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

	// TWI initialization
	// TWI disabled
	TWCR = (0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

	// Alphanumeric LCD initialization
	// Connections are specified in the
	// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
	// RS - PORTD Bit 0
	// RD - PORTD Bit 2
	// EN - PORTD Bit 1
	// D4 - PORTC Bit 4
	// D5 - PORTC Bit 5
	// D6 - PORTC Bit 3
	// D7 - PORTC Bit 2
	// Characters/line: 16
	lcd_init(16);

	lcd_clear();
	// Wellcome message
	lcd_gotoxy(0,0); lcd_putsf("    PRESSURE    ");
	lcd_gotoxy(0,1); lcd_putsf("     METER      ");
	delay_ms(2000);

	// LCD test
	lcd_gotoxy(0,0); lcd_putsf("0123456789ABCDEF");
	lcd_gotoxy(0,1); lcd_putsf("0123456789ABCDEF");
	delay_ms(1000);
	lcd_clear();

	while (1) {
		  if (tiks == 0) tiks = 1; else tiks = 0; // CPU ALIVE COUNTER
						  
		  adc1 = ADCsingleREAD(1);
		  adc6 = ADCsingleREAD(6); 
		  
		  if (adc1 > 42) { // MPX4250AP 250kPa = 41..1003 DAC step range
			adc1 = (adc1-41);
		  } else {
			adc1 = 0;
		  }
		  
		  if (adc6 > 40) { // MPX5700DP 700kPa = 40..963 DAC step range  
			adc6 = (adc6-40);
		  } else {
			adc6 = 0;
		  }

		  
		  // display pages rotation
		  if ( PIND.5 == 0 ) {  
			mode_view ++;
			if (mode_view > 4) mode_view = 0;
		  }  
		  
		  
		  if (mode_view == 0) { // SHOW PRESSURE IN KPA
			//adc1_press = adc1 * 0.239; // MPX4250AP
			adc1_press = (adc1 * 0.2442) + 20; // MPX4250AP
			adc6_press = adc6 * 0.758; // MPX5700DP
			sprintf(lcd_buf[0], "A: %03u   B: %03u", adc1_press, adc0_press);
			sprintf(lcd_buf[1], "  kPa    C: %03u", adc6_press );
			  
		  } else if (mode_view == 1) { // SHOW PRESSURE IN MBAR
			adc1_press = adc1 * 2.442 + 200; // MPX4250AP
			adc6_press = adc6 * 7.58; // MPX5700DP
			sprintf(lcd_buf[0], "A: %04u B: %04u", adc1_press, adc0_press);
			sprintf(lcd_buf[1], "  mBar  C: %04u", adc6_press );  
				 
		  } else if (mode_view == 2) { // SHOW PRESSURE IN BAR 
			adc1_press = (adc1 * 0.2442) + 20; // MPX4250AP
			adc6_press = adc6 * 0.758; // MPX5700DP
			sprintf(lcd_buf[0], "A: %01u.%02u B: %01u.%02u", adc1_press/100, adc1_press%100, adc0_press/100, adc0_press%100);
			sprintf(lcd_buf[1], "  BAR   C: %01u.%02u", adc6_press/100, adc6_press%100 );        
			
		  } else if (mode_view == 3) { // SHOW PRESSURE IN PSI  
			adc1_press = (adc1 * 0.0347) + 2.9008;
			adc6_press = adc6 * 1.01; // MPX5700DP
			sprintf(lcd_buf[0], "A:%03u.%01u B:%03u.%01u", adc1_press/10, adc1_press%10, adc0_press/10, adc0_press%10);
			sprintf(lcd_buf[1], "  PSI   C:%03u.%01u", adc6_press/10, adc6_press%10 );  
				 
		  } else if (mode_view == 4) { // SHOW PRESSURE IN DAC  
			adc1_press = adc1; // MPX4250AP
			adc6_press = adc6; // MPX5700DP
			sprintf(lcd_buf[0], "A: %04u B: %04u", adc1_press, adc0_press);
			sprintf(lcd_buf[1], "  DAC   C: %04u", adc6_press );       
		  }
		  
		  
		  //lcd_clear();       
		  lcd_gotoxy(0,0); lcd_puts(lcd_buf[0]);
		  lcd_gotoxy(0,1); lcd_puts(lcd_buf[1]); 
		  lcd_gotoxy(0,1); if (tiks == 0) lcd_putchar(32); else lcd_putchar(35);
		  delay_ms(500);
	}



} // the end of main

