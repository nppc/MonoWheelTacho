//#define DEBUG_SIM
/*
MonoCycle MCU
*/
#include <EEPROM.h>
#include "U8glib.h"

//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_FAST);	// Fast I2C
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);	// Fast I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send AC, very slow

/* Currently have 3 screens
  1 - Distance screen
  2 - Speed screen
  3 - Battery screen
*/
#define TICKS_PER_METER  38.8 //Ticks per meter

#define EEPROM_ADDR 10 // address in eeprom for storing total distance

#define	HA	8	// Pin for hall sensor A
#define	HB	9	// Pin for hall sensor B
#define	HC	10	// Pin for hall sensor C

#define BUTTON_PIN  5 // Pin for Button
#define VBAT_PIN    A1 // Pin for measuring battery voltage
// divider is good as 100K/6K1 to meassure til 80V.

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))
/*
Replace
pinMode( pin, INPUT ); with pinAsInput( pin );
pinMode( pin, OUTPUT ); with pinAsOutput( pin );
pinMode( pin, INPUT_PULLUP); with pinAsInputPullUp( pin );
digitalWrite( pin, LOW ); with digitalLow( pin );
digitalWrite( pin, HIGH ); with digitalHigh( pin );
digitalRead( pin ) with digitalState( pin )

Additionally, rather than typing 
if( digitalState( pin ) == HIGH ) you can type if( isHigh( pin ) ) for clearer code clarity. 
Also use isLow( pin ) rather than digitalState( pin ) == LOW.
*/

volatile byte impuls_counter=0; // counter for hall sensor impulses
volatile byte impuls_counter_int=0; // counter for valid ticks of wheel rotation
//volatile byte state = 0; // variable for use in the interrupt
//volatile char direction = 1; // direction of the wheel rotation
//volatile byte hall_val_ptr = 99; // pointer for array
//volatile byte hall_values[]= {1,3,2,6,4,5};
//volatile unsigned int collisions = 0; // Desyncs or direction change
volatile unsigned long int_timer = 0;

unsigned long button_timer=0; // variable for storing timing values for buton pressing
unsigned long button_hold_timer=0; // variable for storing timing values for buton pressing
byte button_state = 1; // variable, storing last button state 1 means not pressed
byte button_pressed = 0; // indicates button press (once set to 1 by interrupt, need to clear in main loop)

#ifdef DEBUG_SIM
	volatile char sim_val_ptr = 0; // pointer for array
	volatile char sim_direction = 1; // direction of rotation simulation
	volatile unsigned int new_OCR = 65000;
	int analogVal = 0;
#endif

byte CurrentScreen=1;	// What screen is currently showing

//char prev_direction = 1;	// store last direction for determining a direction change

unsigned long wheel_ticks=0;
unsigned long distance_in_meters;
unsigned long total_distance_in_meters;
unsigned long distance_in_meters_after_reset;
unsigned long total_distance_in_meters_after_reset;

unsigned long speed_measure_last_millis;
unsigned long speed_measure_last_ticks;

//unsigned int counter = 0;
char tmp_string[10];
float drawSpeedVal=0.0; // in km/h
float drawSpeedPrevVal=0.1;

float drawMaxSpeedVal=0.0; // in km/h
float drawMaxSpeedPrevVal=0.1;

float drawDistVal=0.0; // in km
float drawDistPrevVal=1.0;

// Voltage divider: 
// Vout=ADC*(Vref/1024)
// R2=(Vout * R1)/(Vin-Vout)
// Vin = ((ADC/1024) * Vref) / (R1/(R1+R2))

const float vBatCompensation = 0.55; // add to the reading to get correct value
const float VdivBat = (float)61 / (float)(998 + 61); // 99.8 kOhm; 6.1 kOhm
float drawBatVoltage = 0.0;
float drawMinBatVoltage = 999.0;
int drawBatVoltagePercent = 0;
unsigned long drawBatPercentRefreshRate = 0; // Timer for refresh rate of Vbat percents on the screen

unsigned long eeprom_last_write_time;
int eeprom_writes = 0; // counter for eeprom writes

#define filterSamples   17             // filterSamples should  be an odd number, no smaller than 3
unsigned int VBAT_SmoothArray[filterSamples];   // array for holding raw ADC data 
unsigned int SPEED_SmoothArray[5];   // array for holding speed values


void draw(byte CurScrn) {
	switch (CurScrn) {
		case 1:
			drawDistance();
			break;
		case 2:
			drawSpeed();
			break;
		case 3:
			drawBattery();
			break;
	}
}

void drawDistance(void) {
   //temporarely comment out
  // graphic commands to redraw the complete screen should be placed here  
  // call procedure from base class, http://arduino.cc/en/Serial/Print
  u8g.setFont(u8g_font_helvR12); //u8g_font_fur14
  u8g.drawStr(0,13, "Distance (km)");     

  u8g.setFont(u8g_font_helvB24n); //u8g_font_timB24
  dtostrf(drawDistVal, 2, 3, tmp_string);
  u8g.drawStr(10,45, tmp_string);     

  u8g.setFont(u8g_font_unifont);
//  itoa(drawMaxSpeedVal, tmp_string, 10);
  dtostrf((float)total_distance_in_meters/1000.0, 2, 1, tmp_string);
  u8g.drawStr(0,63, "total:");     
  u8g.drawStr(53,63, tmp_string);     

/*
  u8g.setFont(u8g_font_unifont);
  itoa(eeprom_writes, tmp_string, 10);
  u8g.drawStr(5,63, tmp_string);

  u8g.setFont(u8g_font_unifont);
  itoa(state, tmp_string, 10);
  u8g.drawStr(55,63, tmp_string);

  u8g.setFont(u8g_font_unifont);
  itoa(button_state, tmp_string, 10);
  u8g.drawStr(90,63, tmp_string);

  u8g.setFont(u8g_font_unifont);
  itoa(collisions, tmp_string, 10);
  u8g.drawStr(90,40, tmp_string);
*/

}

void drawSpeed(void) {
  // graphic commands to redraw the complete screen should be placed here  
  // call procedure from base class, http://arduino.cc/en/Serial/Print
  u8g.setFont(u8g_font_helvR12); //u8g_font_fur14
  u8g.drawStr(0,13, "Speed (km/h)");     

  u8g.setFont(u8g_font_helvB24n); //u8g_font_timB24
//  itoa((int)drawSpeedVal, tmp_string, 10);
  dtostrf(drawSpeedVal, 2, 0, tmp_string);
  u8g.drawStr(25,45, tmp_string);     

  u8g.setFont(u8g_font_unifont);
//  itoa(drawMaxSpeedVal, tmp_string, 10);
  dtostrf(drawMaxSpeedVal, 2, 1, tmp_string);
  u8g.drawStr(0,63, "max:");     
  u8g.drawStr(35,63, tmp_string);     

}

void drawBattery(void) {
  // graphic commands to redraw the complete screen should be placed here  
  // call procedure from base class, http://arduino.cc/en/Serial/Print
  u8g.setFont(u8g_font_helvR12); //u8g_font_fur14
  u8g.drawStr(0,13, "Battery:");     
  dtostrf(drawBatVoltage, 2, 1, tmp_string);
  int tmpval = u8g.drawStr(65,13, tmp_string);
  u8g.drawStr(tmpval+70,13, "v");     
  u8g.setFont(u8g_font_helvB24n); //u8g_font_timB24
  itoa(drawBatVoltagePercent, tmp_string, 10);
  tmpval = u8g.drawStr(25,45, tmp_string);     
  u8g.setFont(u8g_font_helvR12); //u8g_font_fur14
  u8g.drawStr(tmpval+28,44, "%");     
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr(0,63, "min:");     
  dtostrf(drawMinBatVoltage, 2, 1, tmp_string);
  tmpval = u8g.drawStr(35,63, tmp_string);     
  u8g.drawStr(tmpval+40,63, "v");     

  //itoa((int)drawBatVoltagePercent, tmp_string, 10);


}


#ifdef DEBUG_SIM
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
	//cli();
	if(new_OCR!=0) {
		switch (sim_val_ptr) {
			case 0: // 1
				digitalHigh(HA);
	  			digitalLow(HB);
				digitalLow(HC);
				break;	
			case 1: // 3
				digitalHigh(HA);
	  			digitalHigh(HB);
				digitalLow(HC);
				break;	
			case 2: // 2
				digitalLow(HA);
	  			digitalHigh(HB);
				digitalLow(HC);
				break;	
			case 3: // 6
				digitalLow(HA);
	  			digitalHigh(HB);
				digitalHigh(HC);
				break;	
			case 4: // 4
				digitalLow(HA);
	  			digitalLow(HB);
				digitalHigh(HC);
				break;	
			case 5: // 5
				digitalHigh(HA);
	  			digitalLow(HB);
				digitalHigh(HC);
				break;	
		}
		
		if(sim_direction==1){
			sim_val_ptr++;
			if(sim_val_ptr>5){sim_val_ptr=0;}
		} else {
			sim_val_ptr--;
			if(sim_val_ptr<0){sim_val_ptr=5;}
		}
		
		OCR1A = new_OCR;	// update OCR value
	}
}
#endif
 
// Hall sensor interrupt 
/*
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
  state = digitalState(HA) + (digitalState(HB)<<1) + (digitalState(HC)<<2);
  if(hall_val_ptr>5){ // initialize pointer. search value in array
  	if(state==1 || state==3 || state==6){hall_val_ptr=state>>1;}else{hall_val_ptr=state;}
  }
  // move pointer and check for direction of rotation
  if(direction==1){
  	if(hall_val_ptr==5){hall_val_ptr=0;}else{hall_val_ptr++;}
  }else{
  	if(hall_val_ptr==0){hall_val_ptr=5;}else{hall_val_ptr--;}
  }
  // direction check
  if(state!=hall_values[hall_val_ptr]){
  	// direction is changed or desync
  	collisions++;
  	direction = -direction;
  	if(state==1 || state==3 || state==6){hall_val_ptr=state>>1;}else{hall_val_ptr=state;} // initialize pointer again
  	// TODO need to reset some counters etc maybe...
  	impuls_counter_int=0; //reset impuls cpunter to start count in other dir
  } else {
  	// direction is right
  	impuls_counter_int++;
  }
  // now lets pass counter to the main routine if its ready to receive it
  if(impuls_counter==255){
  	// main routine is ready to accept new value
  	impuls_counter=impuls_counter_int;
  	impuls_counter_int=0;
  }
}
 
*/
// simplified version (sensing only one pin)
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
  	if((millis()-int_timer)>2){
  	  impuls_counter_int++;
	  // now lets pass counter to the main routine if its ready to receive it
	  if(impuls_counter==255){
	  	// main routine is ready to accept new value
	  	impuls_counter=impuls_counter_int;
	  	impuls_counter_int=0;
	  }
  	}
  	int_timer = millis();
}
 

void setup(void) {
  // flip screen, if required
  // u8g.setRot180();
  pinAsInput(HA);
  pinAsInput(HB);
  pinAsInput(HC);
  
  pinAsInputPullUp(BUTTON_PIN);
  pinAsOutput(BUTTON_PIN+1); // next pin - ground for button
  digitalLow(BUTTON_PIN+1);
  
  pinAsOutput(13);
  digitalLow(13);

  pinAsInput(VBAT_PIN);
  
  // enable pin change interrupts on pins 8,9 and 10
  pciSetup(HA);
  //pciSetup(HB);
  //pciSetup(HC);

#ifdef DEBUG_SIM
	pinAsOutput(A0);
	pinAsOutput(A2);
	digitalLow(A0);
	digitalHigh(A2);

	  pinAsOutput(HA);
	  pinAsOutput(HB);
	  pinAsOutput(HC);
	
	cli();//stop interrupts
	
	//set timer1 interrupt at 1Hz
	  TCCR1A = 0;// set entire TCCR1A register to 0
	  TCCR1B = 0;// same for TCCR1B
	  TCNT1  = 0;//initialize counter value to 0
	  // set compare match register for 1hz increments
	  OCR1A = new_OCR; // 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	  // turn on CTC mode
	  TCCR1B |= (1 << WGM12);
	  // Set CS10 and CS12 bits for 1024 prescaler
	  TCCR1B |= (1 << CS11) | (1 << CS10);  
	  // enable timer compare interrupt
	  TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts
#endif

    // initialize ADC readings (fill digitalsmooth array)
    for(byte i=0;i<filterSamples*2;i++){
		unsigned int tmp = digitalSmooth(analogRead(VBAT_PIN), VBAT_SmoothArray);
		tmp = digitalSmooth(0, SPEED_SmoothArray);
    }

	// initialize counters and variables
	distance_in_meters_after_reset = EEPROMReadlong(EEPROM_ADDR); // read last meters travelled
	total_distance_in_meters_after_reset = EEPROMReadlong(EEPROM_ADDR+5); // read total meters travelled
	
	eeprom_last_write_time = millis(); //Initialize...
	speed_measure_last_millis=millis();	// for speed measurement, time intervals
	speed_measure_last_ticks = wheel_ticks;
}


void loop(void) {
	
	#ifdef DEBUG_SIM
		unsigned int tmpval;
		analogVal=analogRead(A1);
		analogVal=analogVal >> 2;
		analogVal=analogVal << 2;
		if (analogVal<271){
			tmpval = map(analogVal,0,270,500, 65530);
			cli();
			sim_direction = 1;
			sei();
		} else if(analogVal>299){
			tmpval = map(analogVal,300,1020,65530,500);
			cli();
			sim_direction = -1;
			sei();
		} else {
			tmpval=0; // stop ticks
		}
		
		cli();
		new_OCR = tmpval;
		sei();
	#endif
	
	cli();
	if(impuls_counter!=255){
		//new value is ready
		wheel_ticks+=impuls_counter; // count ticks for conversion
		//current_wheel_ticks+=impuls_counter; // current last distance
		impuls_counter=255; // value is read, can be overwritten
	}
	sei();
	// calculate distance
    //(float)totalwheelticks/TICKS_PER_METER // convert to meters
    // 1 wheel rotation equals 1.1176m of distance
    // 138 = 1.1176
    // x   = 1.0000
    // X   = 123.48
	distance_in_meters = distance_in_meters_after_reset + (unsigned long)((float)wheel_ticks/TICKS_PER_METER);
	total_distance_in_meters = total_distance_in_meters_after_reset + (unsigned long)((float)wheel_ticks/TICKS_PER_METER);
	
	drawDistVal = (float)distance_in_meters / 1000.0; // convert to kilometers
	
	// calculate speed
	if((speed_measure_last_millis+120)<millis()){
		//unsigned long curticks = total_wheel_ticks;
		unsigned long curmillis = millis();
		//drawSpeedVal = (3500.0 * ((float)(wheel_ticks-speed_measure_last_ticks) / TICKS_PER_METER)) / (float)(curmillis - speed_measure_last_millis); // adjust 3600 to the value, where speed is right
		drawSpeedVal = (float)digitalSmooth((unsigned int)((3500.0 * ((float)(wheel_ticks-speed_measure_last_ticks) / TICKS_PER_METER)) / (float)(curmillis - speed_measure_last_millis)*1000.0),SPEED_SmoothArray) / 1000.0; // adjust 3600 to the value, where speed is right
		speed_measure_last_ticks = wheel_ticks;
		speed_measure_last_millis = curmillis;
		
		if(drawMaxSpeedVal<drawSpeedVal){drawMaxSpeedVal=drawSpeedVal;}
	}
	
	// read voltage
	unsigned int tmpVbatADC = digitalSmooth(analogRead(VBAT_PIN), VBAT_SmoothArray);
	// temp mod
	//tmpVbatADC = tmpVbatADC / 2 + 512;
	// calculate voltage from the divider
	drawBatVoltage = (((float)tmpVbatADC/1024.0) * 5.0) / VdivBat;  // 5V - ref voltage
	drawBatVoltage += vBatCompensation;
	if (drawBatVoltage<=drawMinBatVoltage){drawMinBatVoltage = drawBatVoltage;}
	// calculate battery charge Percentage
	int VbatCell= drawBatVoltage / 16.0 * 1000.0; // im mV
	// update percents once every second (not so often)
	if((drawBatPercentRefreshRate+2000)<millis()) {	// 2 seconds refresh rate
		drawBatVoltagePercent = getChargePercent(VbatCell);
		drawBatPercentRefreshRate = millis();
	}
	
	
	//digitalHigh(13);
	  u8g.firstPage();  
	  do {
    	draw(CurrentScreen);
	  } while( u8g.nextPage() );

	//digitalLow(13);
	//delay(100);

	writeDistanceEEPROM(); // if needed, write to eeprom
	
	
	// button routine
	byte cur_button_state = digitalState(BUTTON_PIN);
	//if(button_state!=cur_button_state){button_timer = millis();} // reset timer on pin change
    if((button_timer+80)<millis() && button_pressed==0){ // 50ms - debounce
     	// software dobounce and long press detection
     	
     	if(cur_button_state==0 && button_state==1) { 
     		// button is just pressed, wait for button release...
     		button_state=0;
     		button_hold_timer = millis(); // start to count pressing time
     	} else if(cur_button_state==1 && button_state==0) {
     		// button is released after it was pressed
     		button_state=1;
     		// determine, is this short or long press
     		if((button_hold_timer+1000)<millis()){
     			button_pressed=0; // after long press reset
     		} else if(button_hold_timer != 0){
     			button_pressed=1; // short press
     		}
     	} else if(cur_button_state==0 && button_state==0 && button_hold_timer != 0){
     		// count time for long press
     		if((button_hold_timer+2000)<millis()) {
     			button_pressed=2; // long press
     			button_hold_timer = 0; // reset for not repeating the event
     		}
     	}
     	//button_state=cur_button_state;
     	button_timer = millis();
    }
	

	if(button_pressed!=0){
		if(button_pressed==1){
			// short press detected - change a screen
			CurrentScreen++;
			if(CurrentScreen>3) {CurrentScreen=1;}
		} else {
			// long press detected - clear current distance or max kilometers
			if(CurrentScreen==1){
				total_distance_in_meters_after_reset = total_distance_in_meters; // ensure correct value remains after resetting ticks value
				distance_in_meters_after_reset = 0;
				distance_in_meters = 0;
				EEPROMWritelong(EEPROM_ADDR,distance_in_meters);
				wheel_ticks = 0; // reset counter
				speed_measure_last_ticks = 0; // reset this variable for correct speed calculation after ticks reset
			}
			if(CurrentScreen==2){
				drawMaxSpeedVal = 0;
			}
		}
		button_pressed=0;
	} 
	
}

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void writeDistanceEEPROM() {
	if((int)drawSpeedVal==0 || (eeprom_last_write_time+60000)<millis()) {	// write if speed is near 0, or more than 1 min passed.
	    if((eeprom_last_write_time+60000)>millis() && ((int)drawSpeedVal>1)){return;} // exit if we have good speed and 1 minute is not passed after last write
	    unsigned long tmpval = EEPROMReadlong(EEPROM_ADDR);
	    if((distance_in_meters-tmpval)<10){return;} // do not write if value is changed for less than 10 meeters
		//digitalHigh(13);  // debug eeprom writes
		EEPROMWritelong(EEPROM_ADDR,distance_in_meters); // uncomment for real writes
		EEPROMWritelong(EEPROM_ADDR+5,total_distance_in_meters); // uncomment for real writes
		eeprom_writes++;
		eeprom_last_write_time = millis();	// update last write time 
		//digitalLow(13);  // debug eeprom writes
	}
}

void EEPROMWritelong(int address, unsigned long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }


unsigned long EEPROMReadlong(int address)
      {
      //Read the 4 bytes from the eeprom memory.
      unsigned long four = EEPROM.read(address);
      unsigned long three = EEPROM.read(address + 1);
      unsigned long two = EEPROM.read(address + 2);
      unsigned long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }


// smooth algorytm for ADC reading
unsigned int digitalSmooth(unsigned int rawIn, unsigned int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  unsigned int j, k, temp, top, bottom;
  long total;
  static int i;
  static unsigned int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total / k;    // divide by number of samples
}

// Cell voltage for every 5th of percent, eg 0%=2.900, 5%=3.000, 10%=3.200, 15%=3.300 ... 100%=4.130 
//const PROGMEM  uint16_t BatPercentTable[]  = {3270,3610,3680,3710,3730,3750,3770,3790,3800,3820,3830,3850,3870,3890,3915,3940,3970,4005,4045,4090,4130};
const PROGMEM  uint16_t BatPercentTable[]  = {2900,3080,3200,3300,3380,3425,3455,3475,3490,3500,3510,3520,3540,3560,3590,3630,3690,3760,3850,3950,4130};
int getChargePercent(int VbatCell) {
	int tmpval = pgm_read_word_near(BatPercentTable); //pgm_read_word_near(BatPercentTable + i);
	if(VbatCell<=tmpval){return 0;} // return 0 Percent if voltage is lower than min. Also it brings less hassle with approximation below...
	tmpval = pgm_read_word_near(BatPercentTable+20); // read last value in the table
	if(VbatCell>tmpval){return 101;} // return 101 Percent if voltage is higher than max
	// search for appropriate value
	int i;
	for(i=0;i<21;i++){
		 tmpval = pgm_read_word_near(BatPercentTable+i); // read value
		 if(VbatCell<=tmpval){break;} // exit
	}
	// need to find better value with approximation
	int tmpval2 = pgm_read_word_near(BatPercentTable+(i-1));
	return i*5 - (int)((float)(tmpval-VbatCell)/((float)(tmpval-tmpval2)/5.0));
	
}

