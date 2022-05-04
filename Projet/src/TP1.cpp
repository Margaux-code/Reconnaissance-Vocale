
#include <Arduino.h>
#include <avr/io.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <Adafruit_SSD1306.h>

byte led = 5;
int somme = 0;
int moyenne;
byte i;
int a;
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

Adafruit_SSD1306 display(-1);
#define SAMPLES 64
double vReal[SAMPLES];
double vImag[SAMPLES];
byte dB;
long maxpeak;
long les_db;
char buf[5];

arduinoFFT FFT = arduinoFFT();
double temp;

void ADC_enable() // activer l’ADC
{
    cli();
    ADCSRA |= (1 << ADEN);
    sei();
}

void ADC_disable()
{
    cli();
    ADCSRA |= (1 << ADIE);
    sei();
}
void setAnalogMux(ADC_modes mux)
{
    cli();
    switch (mux)
    {
    case ADC_A0:
        ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

        break;
    case ADC_A1:
        ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
        ADMUX |= (1 << MUX0);
        break;
    case ADC_A2:
        ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX0));
        ADMUX |= (1 << MUX1);
        break;
    case ADC_A3:
        ADMUX &= ~((1 << MUX3) | (1 << MUX2));
        ADMUX |= ((1 << MUX1) | (1 << MUX0));
        break;
    case ADC_A4:
        ADMUX &= ~((1 << MUX3) | (1 << MUX1) | (1 << MUX0));
        ADMUX |= (1 << MUX2);
        break;
    case ADC_A5:
        ADMUX &= ~((1 << MUX3) | (1 << MUX1));
        ADMUX |= ((1 << MUX2) | (1 << MUX0));
        break;
    case ADC_A6:
        ADMUX &= ~((1 << MUX3) | (1 << MUX0));
        ADMUX |= ((1 << MUX2) | (1 << MUX1));
        break;
    case ADC_A7:
        ADMUX &= ~((1 << MUX3) | (0 << MUX1));
        ADMUX |= ((1 << MUX2) | (0 << MUX0));
        break;
    case ADC_SENSOR:
        ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));
        ADMUX |= ((1 << MUX3));
        break;
    case ADC_GND:
        ADMUX |= ~((0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0));
        break;
    case ADC_1V1:
        ADMUX |= ~((0 << MUX3) | (0 << MUX2) | (0 << MUX1));
        ADMUX &= (0 << MUX0);
        break;
    }
    sei();
}
void ADC_setPrescaler(byte prescl)
{
    cli();
    switch (prescl)
    {
    case 2:
        ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
        break;
    case 4:
        ADCSRA &= ~((1 << ADPS2) | (1 << ADPS0));
        ADCSRA |= (1 << ADPS1);
        break;
    case 8:
        ADCSRA &= ~(1 << ADPS2);
        ADCSRA |= ((1 << ADPS1) | (0 << ADPS0));

        break;
    case 16:
        ADCSRA &= ~((1 << ADPS1) | (1 << ADPS0));
        ADCSRA |= (1 << ADPS2);
        break;
    case 32:
        ADCSRA &= ~((1 << ADPS1));
        ADCSRA |= ((1 << ADPS2) | (1 << ADPS0));
        break;
    case 64:
        ADCSRA &= ~((1 << ADPS0));
        ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));
        break;
    case 128:
        ADCSRA |= ~((0 << ADPS2) | (0 << ADPS1) | (0 << ADPS0));
        break;
    }
    sei();
}

void ADC_setReference(ADC_modes ref)
{
    cli();
    switch (ref)
    {
    case ADC_1V1:
        ADMUX |= (1 << REFS0);
        ADMUX |= (1 << REFS1);
        break;
    case ADC_AREF:
        ADMUX &= (0 << REFS0);
        ADMUX &= (0 << REFS1);
        break;
    case ADC_VCC:
        ADMUX |= (1 << REFS0);
        ADMUX &= (0 << REFS1);
        break;
    }
    sei();
}
void leftAdjust(int ndr)
{
    switch (ndr)
    {
    case 8:
        ADMUX |= (1 << ADLAR);
        break;
    case 10:
        ADMUX &= (0 << ADLAR);
        break;
    }
}

void (*adc_isr)();
void (*acomp_isr)();
void ADC_attachInterrupt(void (*isr)())
{
    cli();
    adc_isr = *isr;
    ADCSRA |= (1 << ADIE);
    sei();
}

void ADC_detachInterrupt()
{
    cli();
    ADCSRA &= ~(1 << ADIE);
    sei();
}

void ADC_startConvert()
{
    cli();
    ADCSRA |= (1 << ADSC);
    sei();
}

unsigned int ADC_read()
{
    return ADC;
}

uint8_t ADC_read8(void)
{
    return ADCH;
}

boolean ADC_available()
{
    if (ADCSRA & (1 << ADSC))
    { // if flag set - convert in process
        return false;
    }
    else
    { // convert complete
        return true;
    }
}

unsigned int ADC_readWhenAvailable()
{
    while (ADCSRA & (1 << ADSC))
        ;
    return ADC;
}

uint8_t ADC_read8WhenAvailable(void)
{
    while (ADCSRA & (1 << ADSC))
        ;
    return ADCH;
}

void setup()
{

    byte x = 0;
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.fillRect(0, 0, display.width() - 2, 11, WHITE);
    display.setTextColor(BLACK);

    x = 16;
    display.setCursor(x, 2);
    display.print(F("AUDIO"));
    x = 52;
    display.setCursor(x, 2);
    display.print(F("SPECTROMETER"));

    for (byte i = 0; i < SAMPLES / 2 - 1; i++)
    {
        display.drawFastHLine(i * 4 + 1, display.height() - 1, 3, WHITE);
    }
    display.setTextColor(WHITE);
    display.display();
    Serial.begin(9600);
    pinMode(led, OUTPUT);
    pinMode(A0, INPUT);
    ADC_enable();
    ADC_setPrescaler(32);
    // leftAdjust(8);
    ADC_setReference(ADC_1V1);
    setAnalogMux(ADC_A0);
}

void loop()
{
    
    // Pour la fréquence d'échantillonnage on récupère les voltages venues du micro après un filtre passe-haut et l'AOP
    for (i = 0; i < SAMPLES; i++)
    {

        ADC_startConvert();
        while (!ADC_available())
        {
            vReal[i] = ADC_read();
            temp = ADC_read();
            somme = vReal[i]; // Somme pour faire la moyenne
        }       
        vImag[i] = 0;
    }
    moyenne = somme / i;                     // Moyenne permettant de trouver les dbs.
    les_db = 37.364 * log(moyenne) - 175.75; // Calcul des décibels a partir du mappage experimental (voir excel)
    Serial.println(les_db);
    somme = 0; //Remise de la somme à zéro pour la prochaine fois 
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    display.fillRect(0, 12, display.width() - 2, display.height() - 13, BLACK);
    // Transformé de fourier et affichage 
    for (i = 0; i < SAMPLES / 2 - 1; i++)
    {
        dB = map(vReal[i], 0, 1023, 0, 52);
        display.fillRect(i * 4 + 1, abs(52 - dB) + 12, 3, dB, WHITE);
    }
    maxpeak = FFT.MajorPeak(vReal, SAMPLES, 5000);

    sprintf(buf, "%04li", les_db);
    display.setCursor(72, 16);
    display.print(F("dB:"));
    display.print(les_db);
    display.display();
}