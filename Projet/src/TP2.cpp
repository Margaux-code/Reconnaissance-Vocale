/*#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// defines for setting and clearing register bits
// defining pins for control
const int pinAnalogIn = A0;

const int bufferSize = 128;

const int minMode = 1;
const int maxMode = 7 + 500;
/
const int defaultMode = 7;

// initing display
Adafruit_SSD1306 display(128, 32, &Wire, -1);
byte buffer[bufferSize];

int mode = defaultMode;

void drawScreen(unsigned long duration) {
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  for (int v = 5; v >= 0; v --) {
    int tickY;
    if (v == 5)
      tickY = 0;
    else if (! v)
      tickY = display.height() - 1;
    else
      tickY = display.height() - display.height() * v / 5;
    for (int x = 0; x < display.width(); x += 8)
      display.drawPixel(x, tickY, WHITE);
  }
  
  for (int x = 0; x < bufferSize; x ++) {
    if (! x)
      display.drawPixel(x, buffer[x], WHITE);
    else
      display.drawLine(x - 1, buffer[x - 1], x, buffer[x], WHITE);
  }
  
  display.setCursor(0, 0);
  display.print("5V");
  display.setCursor(0, display.height() - 7);
  display.print(duration);
  display.print(" us");
  display.display();
}

void setup() {
  // defining pins for control
  pinMode(pinAnalogIn, INPUT);
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
  display.setTextSize(1);
  display.clearDisplay();
}

void loop() {
  
  unsigned long delayUs = (mode > 7) ? (mode - 7) * 20 : 0;
  unsigned long start = micros();
  for (int x = 0; x < bufferSize; x ++) {
    buffer[x] = map(analogRead(pinAnalogIn), 0, 1023, display.height() - 1, 0);
    if (delayUs)
      delayMicroseconds(delayUs);
  }
  drawScreen(micros() - start);
}*/