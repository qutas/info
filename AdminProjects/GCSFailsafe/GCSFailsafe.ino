const int buttonRed = 4;
const int buttonGreen = 5;
const int ledRed = 2;
const int ledGreen = 3;

int ledStateRed = LOW;
int ledStateGreen = HIGH;
int buttonGreenState = HIGH;
int lastGreenButtonState = HIGH;
int buttonRedState = HIGH;
int lastRedButtonState = HIGH;

unsigned long lastDebounceTimeRed = 0;
unsigned long lastDebounceTimeGreen = 0;
unsigned long debounceDelay = 50;
unsigned long blinkdelay = 250;
unsigned long lastBlink = 0;

typedef enum {
  MODE_NORMAL = 0,
  MODE_FAILSAFE
} mode_t;
mode_t mode = MODE_NORMAL;

void setup() {
  pinMode(buttonGreen, INPUT_PULLUP);
  pinMode(buttonRed, INPUT_PULLUP);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);

  digitalWrite(ledGreen, ledStateGreen);
  digitalWrite(ledRed, ledStateRed);

  delay(1000);

  Serial.begin(115200);
}

void loop() {
  int readingRed = digitalRead(buttonRed);
  int readingGreen = digitalRead(buttonGreen);

  // If the switch changed, due to noise or pressing:
  if (readingRed != lastRedButtonState) {
    // reset the debouncing timer
    lastDebounceTimeRed = millis();
  }

  // If the switch changed, due to noise or pressing:
  if (readingGreen != lastGreenButtonState) {
    // reset the debouncing timer
    lastDebounceTimeGreen = millis();
  }

  if ((millis() - lastDebounceTimeRed) > debounceDelay) {
    if (readingRed != buttonRedState) {
      buttonRedState = readingRed;
      if (buttonRedState == LOW) {
        if (mode != MODE_FAILSAFE) {
          mode = MODE_FAILSAFE;
          Serial.println("Switching to failsafe mode");
        }
      }
    }
  }

  if ((millis() - lastDebounceTimeGreen) > debounceDelay) {
    if (readingGreen != buttonGreenState) {
      buttonGreenState = readingGreen;
      if (buttonGreenState == LOW) {
        if (mode != MODE_NORMAL) {
          mode = MODE_NORMAL;
          Serial.println("Switching to normal mode");
        }
      }
    }
  }

  if (mode == MODE_NORMAL) {
    ledStateRed = LOW;
    ledStateGreen = HIGH;
  } else {
    if ( (millis() - lastBlink) > blinkdelay) {
      lastBlink = millis();
      ledStateRed = !ledStateRed;
    }
    
    ledStateGreen = LOW;
    //ledStateGreen = HIGH;
  }

  // set the LED:
  digitalWrite(ledGreen, ledStateGreen);
  digitalWrite(ledRed, ledStateRed);

  lastRedButtonState = readingRed;
  lastGreenButtonState = readingGreen;
}
