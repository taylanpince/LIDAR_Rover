#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
SoftwareSerial XBee(8, 7);

const char COMMAND_FORWARDS = 0x46;
const char COMMAND_BACKWARDS = 0x42;
const char COMMAND_ROTATE = 0x52;
const char COMMAND_END = 0x0D;

void setup() {
  Serial.begin(57600);
  XBee.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  } else {
    Serial.println(F("SSD1306 found"));
  }

  display.display();
  display.clearDisplay();

  delay(1000);

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Online"));
  display.println(F("Waiting..."));
  display.display();

  delay(1000);
}

char command = 0x00;
char payload[5] = {};
uint8_t payload_index = 0;

void loop() {
  while (XBee.available()) {
    char value = XBee.read();

    if (value == COMMAND_END) {
      int value = atoi((char *)payload);

      Serial.println(command);
      Serial.println(value);
      
      display.clearDisplay();
      display.setCursor(0, 0);
  
      if (command == COMMAND_FORWARDS) {
        display.println(F("Forwards:"));
      } else if (command == COMMAND_BACKWARDS) {
        display.println(F("Backwards:"));
      } else if (command == COMMAND_ROTATE) {
        display.println(F("Rotate:"));
      }
      
      display.println(value);
      display.display();

      payload_index = 0;
      command = 0x00;

      memset(payload, 0, sizeof(payload));
    } else if (payload_index == 0) {
      command = value;
      payload_index++;
    } else {
      payload[payload_index - 1] = value;
      payload_index++;
    }
  }
}
