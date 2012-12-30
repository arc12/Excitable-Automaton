/*
 * ExcitableAutomaton1.cpp
 *
 * Created: 02/12/2012 17:08:48
 *  Author: Adam
 */ 

/* ========================
        Design Notes
   ========================
A. Device
---------
ATtiny85V - this is the low voltage version so that 2xAA cells can be used to power it

B. Fuses
--------
SELFPRGEN = [ ]
RSTDISBL = [ ]
DWEN = [ ]
SPIEN = [X]
WDTON = [ ]
EESAVE = [ ]
BODLEVEL = DISABLED
CKDIV8 = [ ]
CKOUT = [ ]
SUT_CKSEL = WDOSC_128KHZ_6CK_14CK_4MS

C. Operating Conditions
-----------------------
Main clock: 32kHz (128kHz int clock prescaled by 4). This gives an ADC clock that is outside datasheet recs but it seems OK
Timer1 clock: 7.8125Hz (main clock prescaled by 4096).

D. Jumpers
----------
jumperST = jumper pin to enable self-triggering
Pullup used- LOW input value enables behaviour. Jumper read on reset

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define nop() asm volatile("nop") 

//stereotypes
int readADC(char mux);

/* ====================
*    I/O Assignments
*  ====================*/
//ADCs use alternative assignments of Port B
const char ldr1Mux = 3;//PB3 = ADC3 : LDR input to ADCs. The order doesn't matter.
const char ldr2Mux = 2;//PB4 = ADC2
const char potMux = 1;//PB2 = ADC1 : potentiometer for trigger threshold setting. ADC connection
const char ADCD = 0x1C;//this is a mask for the ADCxD in DIDR. It must match the choice of ADC pins above

//Port B 0-2 for digital IO
const char led = PB0;//LEDS for "Excited" signalling
const char led_dir = (1<<DDB0);
const char jumperST = PB1;//jumper to enable self-triggering
const char jumperST_dir = (1<<DDB1);

/* ====================
*  Operating parameters
*  ====================*/
// time periods for the watchdog timer. The values are WDP2, WDP1 and WDP0
//		0 = 16ms
//		1 = 32ms
//		2 = 64ms
//		3 = 0.125s
//		4 = 0.25s
//		5 = 0.5s
//		6 = 1s
//		7 = 2s
//		higher values will probably stop things working!
const char t_refractory = 6;//refractory period
const char t_delay = 4;//delay between trigger event (light pulse detected) and excitation
const char t_excited = 3;//duration of the excitation (output LEDs on)

// time period for timer 1. Max value=0xFF See Design Notes above for timer clock period 
const char t_self =32;//this is the self-triggering delay count (from the end of the refractory period). 

//Jumper flags
bool selfTrigger = false;

int main(void)
{
	/* ==================
	*    Initialisation
	*  ==================*/

	//set PORTB to input with pullup enabled to read in jumper settings. low input means enable behaviour
	DDRB = 0x00;//all inputs
	PORTB = (1<<jumperST);//pullup
			
	//read in the jumper settings.
	nop();//see data sheet - "sync latch" means we need a clock cycle delay
	selfTrigger =!(0x01 & (PINB>>jumperST));
	
	//now turn the jumper pins into outputs (jumperST is used to output to a low power "I'm excitable" LED)
	PORTB = 0x00;//datasheet says we should tristate as an intermediate to changing direction with output 0
	DDRB |= jumperST_dir;
	
	//setup ADC input; this is not the default port/pin config
	ADMUX = potMux | (1<<ADLAR); // MUX set to threshold pot. VCC as ref. Left justified
	ADCSRA = (1<<ADEN); // turn ADC on and initialise. no auto-trigger, no interrupt. prescaler to div2
	DIDR0 = ADCD; //disable digital circuitry on the pins used for analog input
	
	//change data direction for LED output
	DDRB |= led_dir;
		
	//set the clock pre-scaler to give 32kHz clock
	CLKPR = 0x80;//change enable
	CLKPR = (1<<CLKPS1); //div by 4
		
	//power-down sleep is used several times with different watchdog timer periods.
	//set the sleep mode to power-down and enable sleep
	MCUCR |= (1<<SE)|(1<<SM1);
		
	// timer1 will be used for self-triggering using normal (non-PWM) mode and t_self as the output compare	
	if(selfTrigger){
		// set the prescaler to 4096 (see design notes section) and the mode.
		TCCR1 = (1<<CS13) | (1<<CS12) | (1<<CS10);//this also ensures "Normal port operation, OC0A/OC0B disconnected"
		//set the compare value to t_self
		OCR1A = t_self;
	}else{
		TCCR1 = 0x00;//disable timer
	}		
		
	//set up the watchdog timer
	//reset the counter
	__asm__ __volatile__ ("wdr");
	//enable the timer, configure for interrupt rather than reset, set to 2s
	WDTCR = (1<<WDIE) | 0x7; //but not WDE!!!!!!!!!!!!!!!!!! If WDE then reset on timeout will occur after the first interrupt - read the datasheet carefully!!!!!
	//enable interrupts at global level. The watchdog uses interrupts to wake up from power-down sleep.
	sei();
	
	/* ==================
	*      Operation
	*  ==================*/
    while(1)
    {
		//>>>>>>>>>>> Automaton is REFRACTORY <<<<<<<<<<<<<
		//LEDs must be off
		PORTB &= ~(1<<led);
		
		//set watchdog timer
		__asm__ __volatile__ ("wdr");//reset the timer
		WDTCR &= 0xF8;//clear existing prescaler period
		WDTCR |= t_refractory;// set WDP2, WDP1 and WDP0 with the timeout period
		//go into sleep mode until WDT timeout
		__asm__ __volatile__ ("sleep\n\t"::);
		
		
		//>>>>>>>>>>> Automaton is now EXCITABLE  <<<<<<<<<<<<< (or about to be...)
		//disable the watchdog timer interrupt (probably not strictly necessary since the interrupt handler would just return)
		WDTCR &=~(1<<WDIE);
			
		// prepare for self-triggering
		TCNT1 = 0;//reset the counter
		TIFR |= (1<<OCF1A);//reset the compare flag
		
		//signal "I am excitable" to a small output LED
		PORTB |= (1<<jumperST);

		//loop awaiting a trigger (external or self)
		bool adcNeutral = true;//becomes false if ADCs detect above-threshold change on an LDR channel
		int potVal=0;
		int ldr1Val=0x3FF;//max ADC value
		int ldr2Val=0x3FF;
		int ldr1Last=0;
		int ldr2Last=0;
		while(adcNeutral){
			//read in the trigger threshold. NB this is the required delta between readings
			//This is done inside the loop so that adjustments become active without requiring a trigger.
			potVal = readADC(potMux)/8;
			//store previous readings
			ldr1Last = ldr1Val;
			ldr2Last = ldr2Val;
			//read ADCs attached to LDRs
			ldr1Val = readADC(ldr1Mux);
			ldr2Val = readADC(ldr2Mux);
			//check for trigger conditions
			//ldr1 triggered		
			if(ldr1Val-potVal >= ldr1Last){
				adcNeutral = false;				
			}
			//ldr2 triggered
			if(ldr2Val-potVal >= ldr2Last){
				adcNeutral = false;
			}
			//force a trigger condition if compare flag for timer 1 is set 
			if(TIFR & (1<<OCF1A)){
				adcNeutral = false;
			}
		}
		//cancel "I am excitable" signal
		PORTB &= ~(1<<jumperST);
				
		//>>>>>>>>>>> Automaton is ACTIVATING  <<<<<<<<<<<<< (delay between trigger and EXCITED)
		__asm__ __volatile__ ("wdr");//reset the timer
		WDTCR &= 0xF8;//clear existing prescaler period
		WDTCR |= (1<<WDIE) | t_delay;// set the timeout period (and re-enable int)
		//go into sleep mode until WDT timeout
		__asm__ __volatile__ ("sleep\n\t"::);
		

		//>>>>>>>>>>> Automaton is EXCITED  <<<<<<<<<<<<<
		//LEDS go on
		PORTB |= (1<<led);
		
		//stay excited for t_excited
		__asm__ __volatile__ ("wdr");//reset the timer
		WDTCR &= 0xF8;//clear existing prescaler period
		WDTCR |= t_excited;//re-enable the watchdog timer and set WDP2, WDP1 and WDP0 with the timeout period
		//go into sleep mode until WDT timeout
		__asm__ __volatile__ ("sleep\n\t"::);		
    }
}

//read a 10bit precision value from the ADC specified by mux (see ADMUX). This is blocking single-shot.
int readADC(char mux){
	//assert the mux, VCC as ref. Left justified
	ADMUX = mux ;
	//start a conversion
	ADCSRA |= (1<<ADSC);
	//wait for end of conversion
	while (ADCSRA & (1<<ADSC));
	//compose the result. 
	int resL = ADCL; //read low first
	int resH = (int)ADCH<<8; //NB the cast to int before shifting
	int result = resH | resL;
	return result;
}

/*
*The watchdog timeout interrupt handler
*/	
ISR(WDT_vect)
{
	//doesn't actually do anything but the interrupt will have brought the MCU out of sleep
}