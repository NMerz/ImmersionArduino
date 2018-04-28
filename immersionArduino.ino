#include <SimpleTimer.h>
#include <Stepper.h>
#include <Elegoo_GFX.h> // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
// D0 connects to digital pin 8 (Notice these are
// D1 connects to digital pin 9 NOT in order!)
// D2 connects to digital pin 2
// D3 connects to digital pin 3
// D4 connects to digital pin 4
// D5 connects to digital pin 5
// D6 connects to digital pin 6
// D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// Color definitions
#define ILI9341_BLACK 0x0000 /* 0, 0, 0 */
//#define ILI9341_NAVY 0x000F /* 0, 0, 128 */
#define ILI9341_DARKGREEN 0x03E0 /* 0, 128, 0 */
//#define ILI9341_DARKCYAN 0x03EF /* 0, 128, 128 */
//#define ILI9341_MAROON 0x7800 /* 128, 0, 0 */
//#define ILI9341_PURPLE 0x780F /* 128, 0, 128 */
//#define ILI9341_OLIVE 0x7BE0 /* 128, 128, 0 */
#define ILI9341_LIGHTGREY 0xC618 /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF /* 128, 128, 128 */
#define ILI9341_BLUE 0x001F /* 0, 0, 255 */
#define ILI9341_GREEN 0x07E0 /* 0, 255, 0 */
//#define ILI9341_CYAN 0x07FF /* 0, 255, 255 */
#define ILI9341_RED 0xF800 /* 255, 0, 0 */
#define ILI9341_MAGENTA 0xF81F /* 255, 0, 255 */
//#define ILI9341_YELLOW 0xFFE0 /* 255, 255, 0 */
#define ILI9341_WHITE 0xFFFF /* 255, 255, 255 */
//#define ILI9341_ORANGE 0xFD20 /* 255, 165, 0 */
//#define ILI9341_GREENYELLOW 0xAFE5 /* 173, 255, 47 */
//#define ILI9341_PINK 0xF81F

/******************* UI details */
#define BUTTON_X 40
#define BUTTON_Y 100
#define BUTTON_W 60
#define BUTTON_H 30
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2

// text box where numbers go
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_MAGENTA

#define TEXT_LEN 12
char textfield[TEXT_LEN + 1] = "";
char hometextfield[TEXT_LEN + 1] = "";
uint8_t textfield_i = 0;

#define YP A3 // must be an analog pin, use "An" notation!
#define XM A2 // must be an analog pin, use "An" notation!
#define YM 9 // can be a digital pin
#define XP 8 // can be a digital pin

//Touch For New ILI9341 TP
#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70
#define TS_MAXY 920
// We have a status line for like, is FONA working
#define STATUS_X 10
#define STATUS_Y 65


long time = 40;

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

Elegoo_GFX_Button buttons[9];

char buttonlabels[9][5] = {"Move", "Home", "Stop", "+", "-", "+10", "-10", "+2", "-2"};
uint16_t buttoncolors[9] = {ILI9341_DARKGREEN, ILI9341_DARKGREY, ILI9341_RED,
                            ILI9341_BLUE, ILI9341_BLUE,
                            ILI9341_BLUE, ILI9341_BLUE,
                            ILI9341_BLUE, ILI9341_BLUE
                           };

//break between GUI and controls

long timeToMove = 0; //total move time in seconds
long timePerMove = 800; // in milliseconds
long maxTimePerMove = 500; //in milliseconds
bool retract = false;
SimpleTimer timer;
long currentPosition = 0;
int stepsToLower = 100; //will soon not be needed, or should be changed to milliseconds to raise/lower
int moveDirection = 1;
//long maxPosition = 193*2 // old half rotation position
long maxPosition = 38600*2;
long homePosition = 0;
int rotationSpeed = 160;
long destinationPosition = 1800;
long betweenSteps = 1;
bool moving = false;
int maxSpeed = 360; //this seems to be safe but I've accidentally taken it up to 460 and it worked fine with the acceleration
Stepper actuator(3200, 49, 47);

void setup() {
  // currentPosition = analogRead(sensorPin);
  Serial.begin(9600);
  Serial.println("starting");
  actuator.setSpeed(120);
  tft.reset();

  uint16_t identifier = tft.readID();
  if (identifier == 0x0101)
  {
    identifier = 0x9341;
  }
  tft.begin(identifier);
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  int notFirstRow = 0;
  // create buttons
  for (uint8_t number = 0; number < 9; number++) { //This is messy because it was faster to reuse example and fadangle columns and rows than redo the placement from scratch
    int row = 0;
    int col = number;
    int buttonWidth = BUTTON_W;
    if (number <= 2) {
      row = 0;
    }
    else if (number > 2) {
      buttonWidth = buttonWidth * 1.5;
      row = 1;
      col -= 3;
      notFirstRow = 1;
    }
    if (number > 4) {
      row = 2;
      col -= 2;
    }
    if (number > 6){
      row = 4;
      col -=2;
    }
    buttons[number].initButton(&tft, BUTTON_X + col * (buttonWidth + BUTTON_SPACING_X) + (buttonWidth - BUTTON_W) - BUTTON_SPACING_X / 2 * notFirstRow,
                               BUTTON_Y + row * (BUTTON_H + BUTTON_SPACING_Y), // x, y, w, h, outline, fill, text
                               buttonWidth, BUTTON_H, ILI9341_WHITE, buttoncolors[number], ILI9341_WHITE,
                               buttonlabels[number], BUTTON_TEXTSIZE);
    buttons[number].drawButton();
  }

  // create 'text field'
  tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, ILI9341_WHITE);
}

//Move time should start overflowing around 25 mintues; I could perhaps extend with an unsigned long
void timedMove(long moveTime) {
  retract = true;
  /*if (moveTime < 30) {
    moveTime = 30;
    }*/
  timePerMove = 800; // in milliseconds
  moveTime -= 5;
  maxTimePerMove = (moveTime * 1000000 / (maxPosition - currentPosition) / 2) / 2;
  if (maxTimePerMove <= 350) {
    maxTimePerMove = 350;
  }
  moving = true;
  moveDirection = 1;
  destinationPosition = maxPosition;
  if (currentPosition >= (maxPosition - 5000)) {
    moveDirection = -1;
    destinationPosition = 0;
  }


  if (timePerMove < maxTimePerMove) {
    timePerMove = maxTimePerMove;
  }

  //read time input and set max rpms as well as destination position at end
}

void timerDone() {
  //digitalWrite(13, HIGH);
  Serial.print("Timer Done");
  moving = false;
}

void goHome() {
  moving = true;
  timePerMove = 800; // in milliseconds
  maxTimePerMove = 800; //in milliseconds
  if (currentPosition == homePosition*maxPosition/100) {
    moving = false;
  } else {
    destinationPosition = homePosition*maxPosition/100; //toInt apparently returns a long according to forums
    if (destinationPosition < currentPosition) {
      //retracting = true;
      moveDirection = -1;
    } else {
      moveDirection = 1;
    }
  }
}


#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop() {
  timer.run();
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //pinMode(YM, OUTPUT);
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    // scale from 0->1023 to tft.width
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = (tft.height() - map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
  }
  for (uint8_t b = 0; b < 9; b++) {
    if (buttons[b].contains(p.x, p.y)) {
      //Serial.print("Pressing: "); Serial.println(b);
      buttons[b].press(true); // tell the button it is pressed
    } else {
      buttons[b].press(false); // tell the button it is NOT pressed
    }
  }
  for (uint8_t b = 0; b < 9; b++) {
    if (buttons[b].justReleased()) {
      // Serial.print("Released: "); Serial.println(b);
      buttons[b].drawButton(); // draw normal
    }

    if (buttons[b].justPressed()) {
      buttons[b].drawButton(true); // draw invert!

      // if a numberpad button, append the relevant # to the textfield
      if (b >= 3 && b<=6) {
        if (b == 3) {
          time += 1;
        }
        else if (b == 4) {
          time -= 1;
        }
        else if (b == 5) {
          time += 10;
        }
        else if (b == 6) {
          time -= 10;
        }
        if (time < 0) {
          time = 0;
        }
        if (time >1200) {
          time = 1200;
        }
        long calctime = time;
        int counterT = sizeof(textfield) - 1;
        textfield[counterT] = 0;
        counterT--;
        Serial.println(calctime / 10);
        while (calctime / 10 > 0) {
          Serial.println(calctime);
          textfield[counterT] = calctime % 10 + '0';
          calctime = calctime / 10;
          counterT--;
        }
        textfield[counterT] = calctime % 10 + '0';
        counterT--;
        while (counterT >= 0) {
          textfield[counterT] = ' ';
          counterT--;
        }
        delay(10);
      } else if (b > 6) {
        if (b == 7) {
          homePosition += 2;
        } else if (b == 8) {
          homePosition -= 2;
        }
        if (homePosition < 0){
          homePosition = 0;
        } else if (homePosition > 100) {
          homePosition = 100;
        }
      long calchome = homePosition;
        int counterP = sizeof(hometextfield) - 1;
        hometextfield[counterP] = 0;
        counterP--;
        Serial.println(calchome / 10);
        while (calchome / 10 > 0) {
          Serial.println(calchome);
          hometextfield[counterP] = calchome % 10 + '0';
          calchome = calchome / 10;
          counterP--;
        }
        hometextfield[counterP] = calchome % 10 + '0';
        counterP--;
        while (counterP >= 0) {
          hometextfield[counterP] = ' ';
          counterP--;
        }
        delay(10);
      }
      // Home button send it home
      if (b == 1) {
        //Holder code
        timePerMove = 800; // in milliseconds
        maxTimePerMove = 800; //in milliseconds
        moving = true;
        destinationPosition = 0; //toInt apparently returns a long according to forums
        if (destinationPosition < currentPosition) {
          moveDirection = -1;
        } else {
          moveDirection = 1;
        }
      }

      // update the current time text field
      Serial.println(textfield);
      tft.setCursor(TEXT_X + 2, TEXT_Y + 10);
      tft.setTextColor(TEXT_TCOLOR, ILI9341_BLACK);
      tft.setTextSize(TEXT_TSIZE);
      tft.print(textfield);

      // update the current home text field
      Serial.println(hometextfield);
      tft.setCursor(TEXT_X + 2, BUTTON_Y + 3 * (BUTTON_H + BUTTON_SPACING_Y)-10);
      tft.setTextColor(TEXT_TCOLOR, ILI9341_BLACK);
      tft.setTextSize(TEXT_TSIZE);
      tft.print(hometextfield);

      // Stop button
      if (b == 2) {
        moving = false; // note: at speeds like 1100 sec this is only checked ever 10 sec; faster speeds aren't a problem
      } else if (b ==1) {
        goHome();
      }
      else if (b == 0) {
        timedMove(time); //broke at 12xx move command while in middle of track when dad was playing with it
      }

      //delay(100); // UI debouncing //only change during copying; I think the rest of the loop will cover this delay and I don't really want to delay the move
    }
  }
  if (Serial.available() > 0) {
    timePerMove = 800; // in milliseconds
    maxTimePerMove = 800; //in milliseconds
    maxSpeed = 360;
    rotationSpeed = 160;
    String input = Serial.readString();
    Serial.print("I received: ");
    Serial.println(input);
    moving = true;
    Serial.println(input.charAt(0));
    if (input.charAt(0) == 'p') {
      input = input.substring(1, input.length());
      bool overide = false;
      if (input.charAt(0) == 'o') {
        overide = true;
        input = input.substring(1, input.length());
      }
      Serial.println("chopped input:" + input);
      destinationPosition = input.toInt(); //toInt apparently returns a long according to forums
      //moving = false; //remove later
      Serial.print("destination position:");
      Serial.println(destinationPosition);
      if (destinationPosition < currentPosition) {
        //retracting = true;
        moveDirection = -1;
        if (overide == true) {
          currentPosition = 100000;//note after the change overide will only move it 1/10th of the way down;
          //I think this is a fine change
        }
      } else {
        moveDirection = 1;
        if (overide == true) {
          currentPosition = -100000;
        }
      }
    }
    else if (input.charAt(0) == 'z') {
      moving = false;
      currentPosition = 0;
    } else if (input.charAt(0) == 't') {
      input = input.substring(1, input.length());
      long moveTime = input.toInt(); //toInt apparently returns a long according to forums
      timedMove(moveTime);
    }
    else if (input.charAt(0) == 's') {
      moving = false;
      Serial.println(currentPosition);
    } else if (input.charAt(0) == 'h') { // value 1-100;
      moving = false;
      input = input.substring(1, input.length());
      homePosition = maxPosition/100 * input.toInt(); //toInt apparently returns a long according to forums
    } else if (input.charAt(0) == 'r') {
      goHome();
    }
    else {
      Serial.println(moving);
      timeToMove = input.toInt();
      moveDirection = 1;
      actuator.setSpeed(rotationSpeed);
      destinationPosition = 193;
      if (timeToMove < 0) {
        //retracting = true;
        timeToMove = timeToMove * -1;
        moveDirection = -1;
        destinationPosition = 0;
      }
      betweenSteps = timeToMove * 1000 / maxPosition;
      Serial.println(timeToMove);
      Serial.println(betweenSteps);
      if (timeToMove > 0) { // make sure to double check that you can't set double timers.
        //timer.deleteTimer(1); causes compile error
        timer.setTimeout(timeToMove * 1000, timerDone);
        timeToMove = 0;
      }
    }
  }
  if (moving == true) {
    //Serial.println(currentPosition);
    if (moveDirection == 1) {
      digitalWrite(47, HIGH);
    } else {
      digitalWrite(47, LOW);
    }
    for (int i = 0; i < 386; i++) {
      digitalWrite(49, HIGH); //Trigger one step forward
      delayMicroseconds(timePerMove);
      digitalWrite(49, LOW); //Pull step pin low so it can be triggered again
      delayMicroseconds(timePerMove);
    }
    currentPosition += moveDirection * 386;
    if (timePerMove > maxTimePerMove) {
      if (timePerMove > 500) {
        timePerMove -= 100;
      } else if (timePerMove > 400) {
        timePerMove -= 40;
      } else if (timePerMove > 300) {
        timePerMove -= 20;
      }
    } else {
      timePerMove = maxTimePerMove;
    }
    if (moveDirection == -1 && (currentPosition < homePosition + 150 || !(currentPosition > destinationPosition))) {
      moving = false;
    } else if (moveDirection == 1 && (currentPosition > (maxPosition - 150) || !(currentPosition < destinationPosition))) {
      if (retract == true) {
        moveDirection = -1;
        destinationPosition = homePosition*maxPosition/100;
        timePerMove = 800;
        maxTimePerMove = 800;
        delay(100);
      } else {
        moving = false;
      }
    }
  }
  // delay(betweenSteps-1);
  //Serial.println(currentPosition);
}
