
#include <Arduino.h>
#include <avr/io.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <Adafruit_SSD1306.h>

byte led = 6;
byte led2 = 4;
byte led3 = 5;
byte bouton = 2;
int somme = 0;

int moyenne = 0;

byte i;
int a;
double SommeRef = 0.0;
double SommeRef1 = 0.0;
double sommeC = 0.0;
double sommeC1 = 0.0;
double correlation1 = 0.0;
double correlation2 = 0.0;
double correlation3 = 0.0;
double correlation = 0.0;
double tampon = 0.0;
int pirouette = 0;
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
double vRefTrois[SAMPLES / 2] = {
    1312.01,
    1064.41,
    489.16,
    213.91,
    233.14,
    226.49,
    196.72,
    218.21,
    198.35,
    157.66,
    113.66,
    47.86,
    48.12,
    109.27,
    129.05,
    112.7,
    118.61,
    104.64,
    52.4,
    19.67,
    46.25,
    51.18,
    20.72,
    17.93,
    25.12,
    31.74,
    39.98,
    36.31,
    27.8,
    14.34,
    13.7,
    17.08,

};

double vRefDeux[SAMPLES / 2] = {
    3000.742,
    2029.16,
    1761.144,
    856.832,
    804.806,
    505.608,
    483.672,
    383.944,
    281.84,
    267.82,
    212.316,
    181.232,
    156.36,
    177.262,
    137.038,
    137.02,
    152.692,
    107.422,
    108.578,
    89.682,
    86.3,
    85.264,
    85.398,
    71.684,
    67.014,
    70.418,
    73.394,
    48.922,
    55.992,
    75.47,
    67.468,
    58.834,

};
double vRefUn[SAMPLES / 2] = {
    3291.70,
    3291.705714,
    817.495714,
    817.3171429,
    420.5242857,
    203.4528571,
    159.8771429,
    90.38,
    160.15,
    141.18,
    165.3671429,
    119.4842857,
    91.63428571,
    85.13142857,
    81.39571429,
    112.7157143,
    75.68857143,
    85.14285714,
    68.19,
    57.64714286,
    53.07714286,
    64.93714286,
    68.51857143,
    50.67,
    59.21714286,
    59.32285714,
    44.18142857,
    38.47571429,
    51.63285714,
    43.55571429,
    47.51714286,
    50.61142857,

};
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

double Calcul_correlation(byte aTester)
{
    sommeC = 0.0;
    tampon = 0.0;
    SommeRef = 0.0;
    correlation = 0.0;
    for (i = 0; i < SAMPLES / 2 - 1; i++)
    {
        switch(aTester)
        {
            case 0:
             sommeC += vReal[i] *vRefDeux[i];
            //SommeRef += pow(vRefDeux[i], 2);
            SommeRef = 18677392.00;
            break;
            case 1:
            sommeC += vReal[i] * vRefUn[i];

            SommeRef = 23433590.00;
            break;
            case -1:
            sommeC += vReal[i] * vRefTrois[i];
           // SommeRef += pow(vRefTrois[i], 2);
           SommeRef = 3493741.00;
            break;
        }

        tampon += pow(vReal[i], 2);
    }
    correlation = sommeC/ sqrt(tampon * SommeRef);
    return correlation;
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
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(bouton, INPUT_PULLUP);
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
            if (moyenne <= vReal[i])
            {
                moyenne = vReal[i];
            }
        }

        vImag[i] = 0;
    }
     Serial.println(moyenne);
    // Moyenne permettant de trouver les dbs.
    les_db = 37.400 * log(moyenne) - 175.75; // Calcul des décibels a partir du mappage experimental (voir excel)
    // Serial.println(les_db);
    somme = 0; // Remise de la somme à zéro pour la prochaine fois
    moyenne = 0;
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    display.fillRect(0, 12, display.width() - 2, display.height() - 13, BLACK);
    // Boucle pour rajouter un échantillon de son
    if (digitalRead(bouton))
    {
        for (i = 0; i < SAMPLES; i++)
        {
            Serial.println(vReal[i]);
        }
        Serial.println("\n \n \n");
    }

    correlation3 = Calcul_correlation(-1);
     correlation2 = Calcul_correlation(0);
    correlation1 = Calcul_correlation(1);
  
    // Correlation avec Un
   
   /* Serial.println(correlation1);
    Serial.println(correlation2);
    Serial.println(correlation3);
    Serial.println("\n \n \n");*/

    if (les_db >= 80 && correlation1 < 0.97 && correlation2 <0.95)
    {
        digitalWrite(led2, LOW);
        digitalWrite(led, LOW);
        digitalWrite(led3, LOW);
        delay(100);
         digitalWrite(led2, HIGH);
        digitalWrite(led, HIGH);
        digitalWrite(led3, HIGH);
        delay(500);
         digitalWrite(led2, LOW);
        digitalWrite(led, LOW);
        digitalWrite(led3, LOW);
    }
    if (correlation1 > 0.97)
    {
        digitalWrite(led, HIGH);
        digitalWrite(led2, LOW);
        digitalWrite(led3, LOW);
    }

    if (correlation2 > correlation1 && correlation2 > 0.95)
    {
        digitalWrite(led2, HIGH);
        digitalWrite(led, LOW);
        digitalWrite(led3, LOW);
    }
    if (correlation3 > 0.97)
    {
        digitalWrite(led2, LOW);
        digitalWrite(led, LOW);
        digitalWrite(led3, HIGH);
    }

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