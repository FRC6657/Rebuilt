#include <Keypad.h>
#include <LiquidCrystal.h>

// Keypad setup
const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// LCD setup
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

bool keyStates[16] = {false};

int getKeyIndex(char key) {
  const char allKeys[] = {'1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'};
  for (int i = 0; i < 16; i++) {
    if (allKeys[i] == key) return i;
  }
  return -1;
}

void updateLCD(String line1, String line2) {
  lcd.clear();
  delay(2);
  
  // Pad to 16 characters
  while (line1.length() < 16) line1 += " ";
  while (line2.length() < 16) line2 += " ";
  
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void setup() {
  Serial.begin(9600);
  
  lcd.begin(16, 2);
  updateLCD("FRC Controller", "Ready");
  
  delay(500);
}

void loop() {
  // Process serial input - simple line reading
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("LCD:")) {
      String content = line.substring(4);
      int separator = content.indexOf('|');
      
      if (separator >= 0) {
        updateLCD(content.substring(0, separator), 
                 content.substring(separator + 1));
      } else {
        updateLCD(content, "");
      }
    }
  }
  
  // Handle keypad
  customKeypad.getKeys();
  
  for (int i = 0; i < LIST_MAX; i++) {
    if (customKeypad.key[i].kchar) {
      char key = customKeypad.key[i].kchar;
      int idx = getKeyIndex(key);
      
      if (idx >= 0) {
        bool isPressed = (customKeypad.key[i].kstate == PRESSED || 
                         customKeypad.key[i].kstate == HOLD);
        
        if (isPressed != keyStates[idx]) {
          Serial.print(key);
          Serial.println(isPressed ? ":1" : ":0");
          keyStates[idx] = isPressed;
        }
      }
    }
  }
  
  delay(10);
}