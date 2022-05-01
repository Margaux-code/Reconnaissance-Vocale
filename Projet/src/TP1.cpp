
#include <Arduino.h>
#include <avr/io.h>
byte led =5;
enum ADC_modes
{
  ADC_A0,
ADC_A1,
ADC_A2,
ADC_A3,
ADC_A4,
ADC_A5,
ADC_A6,
ADC_A7,
ADC_SENSOR,
ADC_1V1,
ADC_GND,
ADC_AREF,
ADC_VCC,
FALLING_TRIGGER,
RISING_TRIGGER,
CHANGE_TRIGGER,
ADC_AIN1,
ANALOG_MUX,
ADC_AIN0,
};

double temp;

void ADC_enable () // activer l’ADC
{
cli ();
ADCSRA |= (1 << ADEN);
sei();
}

void ADC_disable ()
{
cli ();
ADCSRA |= (1 << ADIE);
sei();
}
void setAnalogMux(ADC_modes mux){
cli();
switch(mux){
  case ADC_A0 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
  
  break;
  case ADC_A1 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1));
  ADMUX |= (1<<MUX0);
  break;
  case ADC_A2 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX2) | (1<<MUX0));
  ADMUX |= (1<<MUX1);
  break;
  case ADC_A3 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX2));
  ADMUX |= ((1<<MUX1) | (1<<MUX0));
  break;
  case ADC_A4 :
  ADMUX &=  ~((1<<MUX3)| (1<<MUX1) | (1<<MUX0));
  ADMUX |= (1<<MUX2);
  break;
  case ADC_A5 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX1));
  ADMUX |= ((1<<MUX2) | (1<<MUX0));
  break;
  case ADC_A6 :
  ADMUX &=  ~((1<<MUX3) | (1<<MUX0));
  ADMUX |= ((1<<MUX2) | (1<<MUX1));
  break;
  case ADC_A7 :
  ADMUX &=  ~((1<<MUX3) | (0<<MUX1));
  ADMUX |= ((1<<MUX2)| (0<<MUX0));
  break;
  case ADC_SENSOR :
  ADMUX &=  ~((1<<MUX2) | (1<<MUX1) | (1<<MUX0));
  ADMUX |= ((1<<MUX3));
  break;
  case ADC_GND :
  ADMUX |=  ~((0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0));
  break;
  case ADC_1V1 :
  ADMUX |=  ~((0<<MUX3) | (0<<MUX2) | (0<<MUX1) );
  ADMUX &= (0<<MUX0);
  break;
}
sei();
  
}
void ADC_setPrescaler(byte prescl){
cli();
  switch(prescl){
    case 2 : 
    ADCSRA &= ~((1<<ADPS2) | (1 << ADPS1) | (1<<ADPS0));
    break;
    case 4 : 
    ADCSRA &= ~((1<<ADPS2)  | (1<<ADPS0));
    ADCSRA |=  (1 << ADPS1);
    break;
    case 8 : 
    ADCSRA &= ~(1<<ADPS2);
    ADCSRA |=  ((1 << ADPS1) | (0<<ADPS0)) ;
    
    break;
    case 16 : 
    ADCSRA &= ~( (1 << ADPS1) | (1<<ADPS0));
    ADCSRA |= (1<<ADPS2); 
    break;
    case 32 : 
    ADCSRA &= ~((1 << ADPS1));
    ADCSRA |= ((1<<ADPS2)| (1<<ADPS0));
    break;
    case 64 : 
    ADCSRA &= ~((1<<ADPS0));
    ADCSRA |= ((1<<ADPS2) | (1<<ADPS1));
    break;
    case 128 : 
    ADCSRA |= ~((0<<ADPS2) | (0 << ADPS1) | (0<<ADPS0));
    break;
    
  }
  sei();
}

void ADC_setReference(ADC_modes ref)
{
  cli();
  switch(ref){
    case ADC_1V1 :
    ADMUX |= (1<<REFS0);
    ADMUX |=(1<<REFS1);
    break;
    case ADC_AREF :
    ADMUX &= (0<<REFS0);
    ADMUX &= (0<<REFS1);    
    break;
    case ADC_VCC :
    ADMUX |= (1<<REFS0);
    ADMUX &= (0<<REFS1); 
    break;
    
  }
  sei();
}
void leftAdjust(int ndr){
switch(ndr)
{
  case 8 :
  ADMUX |= (1<<ADLAR);
  break;
  case 10 :
  ADMUX &= (0 << ADLAR);
  break; 
  
}
}

void (*adc_isr)();
void (*acomp_isr)();
void ADC_attachInterrupt(void (*isr)()) { 
    cli();
    adc_isr = *isr;
    ADCSRA |= (1 << ADIE); 
    sei();
}

void ADC_detachInterrupt() { 
    cli();
    ADCSRA &= ~ (1 << ADIE); 
    sei();
}

void ADC_startConvert() { 
    cli();
    ADCSRA |= (1 << ADSC);
    sei();
}

unsigned int ADC_read() { 
    return ADC;
}

uint8_t ADC_read8(void) { 
    return ADCH;
}

boolean ADC_available() { 
    if (ADCSRA & (1 << ADSC)) { // if flag set - convert in process
        return false;
    }
    else { // convert complete
        return true;
    }
}

unsigned int ADC_readWhenAvailable() { 
    while (ADCSRA & (1 << ADSC));
    return ADC; 
}

uint8_t ADC_read8WhenAvailable(void) {
    while (ADCSRA & (1 << ADSC));
    return ADCH;
}

ISR(ADC_vect){ 
    (*adc_isr)();
}

void ACOMP_attachInterrupt(void (*isr)(), ADC_modes source) { 
    cli();
    acomp_isr = *isr;
    ACSR |= (1 << ACIE); // set int bit
    switch (source) {
    case FALLING_TRIGGER: // falling
        ACSR &= ~ (1 << ACIS0);
        ACSR |= (1 << ACIS1);
        break;
    case RISING_TRIGGER: // rising
        ACSR |=  ((1 << ACIS1) | (1 << ACIS0));
        break;
    case CHANGE_TRIGGER: // change (default)
        ACSR &= ~ ((1 << ACIS1) | (1 << ACIS0));
        break;
    }
    sei();
}

void ACOMP_detachInterrupt() { 
    cli();
    ACSR &= ~ (1 << ACIE); 
    sei();
}

void ACOMP_enable() { 
    cli();
    ACSR &= ~ (1 << ACD); 
    sei();
}

void ACOMP_disable() { 
    cli();
    ACSR |= (1 << ACD); // set bit "analog comp disable"
    sei();
}

boolean ACOMP_read() { // 
    return ACSR & (1 << ACO); 
}

void ACOMP_setPositiveInput(ADC_modes in) { 
    cli();
    switch (in) {
    case ADC_1V1: // 1v1
        ACSR |= (1 << ACBG); 
        break;
    case ADC_AIN0: // ADC_AIN0
        ACSR &= ~ (1 << ACBG); // disconnect 1v1
        break;
    }
    sei();
}

void ACOMP_setNegativeInput(ADC_modes in) {
    cli();
    switch (in) {
    case ADC_AIN1: // ADC_AIN1 (default) - 
        ADCSRB &= ~ (1 << ACME); // disable -in from mux
        break;
    case ANALOG_MUX: // analog MUX -
        ADCSRB |= (1 << ACME); // connetct -in to analog mux
        break;
    }
    sei();
}




ISR(ANALOG_COMP_vect){ 
    (*acomp_isr)();
}


/* an example of a simple ace operation */
void setup() {
  Serial.begin(9600);
  pinMode(led,OUTPUT);
  pinMode(A0,INPUT);
  ADC_enable(); 
  ADC_setPrescaler(32);
  //leftAdjust(8);
  ADC_setReference(ADC_1V1); 
  setAnalogMux(ADC_A0);
}

void loop() {
    //digitalWrite(led, HIGH);
  ADC_startConvert(); 
  while (!ADC_available());
  temp = ADC_read();
 //temp = analogRead(A0);
  Serial.println(temp);
    /*if(ADC_read()>= 1000)
    {
        digitalWrite(led, HIGH);  
        delay(1000);  }
        else 
        digitalWrite(led,LOW);*/

}