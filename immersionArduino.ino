#include <SimpleTimer.h>
#include <Stepper.h>
#include <Elegoo_GFX.h> // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>
#include "SPI.h"
#include <EEPROM.h>
#include <TimerThree.h>
#include <Math.h>

#include "splashScreen.h"


// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

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

#define NOTE_A4  440
#define NOTE_CS5 554
#define NOTE_E5  659

const int buzzerPin = 47;

// Array with the notes in the melody (see pitches.h for reference)
int melody[] = {NOTE_A4, NOTE_CS5, NOTE_E5};

// Array with the note durations: a quarter note has a duration of 4, half note 2 etc.
int durations[]  = {8,8,8};

int tempo = 60; // tempo for the melody expressed in beats per minute (BPM)

/******************* UI details */
#define BUTTON_X 40
#define BUTTON_Y 100
#define BUTTON_W 60
#define BUTTON_H 30
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2

// text box where numbers go
#define TEXT_X 28
#define TEXT_Y 25
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 4
#define TEXT_TCOLOR ILI9341_MAGENTA

#define TEXT_LEN 5
char textfield[TEXT_LEN + 1] = "";
//char hometextfield[TEXT_LEN + 1] = "";
uint8_t textfield_i = 0;

#define YP A3 // must be an analog pin, use "An" notation!
#define XM A2 // must be an analog pin, use "An" notation!
#define YM 9 // can be a digital pin
#define XP 8 // can be a digital pin

//Touch For New ILI9341 TP
#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70 //70
#define TS_MAXY 920
// We have a status line for like, is FONA working
#define STATUS_X 10
#define STATUS_Y 65
// Note: You also have to connect GND, 5V/VIO and VM.
//       A connection diagram can be found in the schematics.
#define EN_PIN    31 //enable (CFG6)
#define DIR_PIN   29 //direction
#define STEP_PIN  27 //step

#define CS_PIN   49 //chip select
#define MOSI_PIN 51 //SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN 50 //SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  52 //CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)

//TMC2130 registers
#define WRITE_FLAG     (1<<7) //write flag
#define READ_FLAG      (0<<7) //read flag
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF   0x6C
#define REG_COOLCONF   0x6D
#define REG_DCCTRL     0x6E
#define REG_DRVSTATUS  0x6F

long time = 80;
double timeIncrement = 11.3;

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

const int numOfButtons = 17;

Elegoo_GFX_Button buttons[numOfButtons];

/*char buttonlabels[numOfButtons][11] = {"Dip", "Top", "Stop", "ERROR", "ERROR", "+", "-", "v", "^", "Set End"};
uint16_t buttoncolors[numOfButtons] = {ILI9341_DARKGREEN, ILI9341_DARKGREY, ILI9341_RED,
                                       ILI9341_BLUE, ILI9341_BLUE,
                                       ILI9341_BLUE, ILI9341_BLUE,
                                       ILI9341_BLUE, ILI9341_BLUE,
                                       ILI9341_DARKGREY
                                      };*/

//break between GUI and controls

char timeChar[6] = {'0', ' 1', ':', '2', '0', 0};
long elapsedTime = 0;
long timeToMove = 0; //total move time in seconds
long timePerMove = 1200; // in microseconds
long maxTimePerMove = 600; //in mircroseconds
bool retract = false;
SimpleTimer timer;
volatile long currentPosition = 0;
int stepsToLower = 100; //will soon not be needed, or should be changed to milliseconds to raise/lower
int moveDirection = 1;
//long maxPosition = 193*2 // old half rotation position
long trueMax = 38600;
long maxPosition = trueMax;
long homePosition = 0;
int rotationSpeed = 160;
long destinationPosition = 0;
long betweenSteps = 1;
int movementSubdivisions = 200;
int moveSize = maxPosition / movementSubdivisions;
bool moving = false;
bool manual = false;
bool timedMovement = false;
int maxSpeed = 360; //this seems to be safe but I've accidentally taken it up to 460 and it worked fine with the acceleration
//Stepper actuator(3200, 49, 47);

uint8_t tmc_write(uint8_t cmd, uint32_t data)
{
  uint8_t s;

  digitalWrite(CS_PIN, LOW);

  s = SPI.transfer(cmd);
  SPI.transfer((data >> 24UL) & 0xFF) & 0xFF;
  SPI.transfer((data >> 16UL) & 0xFF) & 0xFF;
  SPI.transfer((data >> 8UL) & 0xFF) & 0xFF;
  SPI.transfer((data >> 0UL) & 0xFF) & 0xFF;

  digitalWrite(CS_PIN, HIGH);

  return s;
}

void eepromWriteLong(int startByte, long value)
{
  EEPROM.write(startByte, (value >> 24UL) & 0xFF);
  EEPROM.write(startByte + 1, (value >> 16UL) & 0xFF);
  EEPROM.write(startByte + 2, (value >> 8UL) & 0xFF);
  EEPROM.write(startByte + 3, (value >> 0UL) & 0xFF);
}

uint8_t tmc_read(uint8_t cmd, uint32_t *data)
{
  uint8_t s;

  tmc_write(cmd, 0UL); //set read address

  digitalWrite(CS_PIN, LOW);

  s = SPI.transfer(cmd);
  *data  = SPI.transfer(0x00) & 0xFF;
  *data <<= 8;
  *data |= SPI.transfer(0x00) & 0xFF;
  *data <<= 8;
  *data |= SPI.transfer(0x00) & 0xFF;
  *data <<= 8;
  *data |= SPI.transfer(0x00) & 0xFF;

  digitalWrite(CS_PIN, HIGH);

  return s;
}

long eepromReadLong(int startByte)
{
  long returnValue = 0;
  returnValue += EEPROM.read(startByte) << 24UL;
  returnValue += EEPROM.read(startByte + 1) << 16UL;
  returnValue += EEPROM.read(startByte + 2) << 8UL;
  returnValue += EEPROM.read(startByte + 3) << 0UL;
  return returnValue;
}

/*void stop_ISR() {
  //if (buttons[2].isPressed() || buttons[2].justPressed()) {
  stop_move();
  timePerMove = 1200;
  tft.fillScreen(YELLOW);
  //}

}*/

void start_move(boolean forward, long interval) {
  buttons[16].drawButton();
  timePerMove = interval;
  if (forward) {
    moveDirection = 1;
    digitalWrite(29, HIGH);
  } else {
    moveDirection = -1;
    digitalWrite(29, LOW);
  }
  elapsedTime = millis();
  moving = true;
  digitalWrite(EN_PIN, LOW);
  Timer3.attachInterrupt(step_motor, interval);
}

void step_motor() {
  digitalWrite(27, HIGH); //Trigger one step forward
  digitalWrite(27, LOW); //Pull step pin low so it can be triggered again
  currentPosition += moveDirection;
}

void stop_move(bool button) {
  Timer3.detachInterrupt();
  moving = false;
  digitalWrite(EN_PIN, HIGH);
  manual = false;
  retract = false;
  buttons[10].drawButton();
  if (!button && timedMovement == true) {
    playTune(melody, durations, tempo);
  }
  timedMovement = false;
}
uint16_t norun = 0;

void setup() {
  Serial.println(norun);
  maxPosition =  eepromReadLong(0);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); //LOW or HIGH
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  digitalWrite(MOSI_PIN, LOW);
  pinMode(MISO_PIN, INPUT);
  digitalWrite(MISO_PIN, HIGH);
  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW);
  // currentPosition = analogRead(sensorPin);
  Serial.begin(9600);
  Serial.println("starting");
  //actuator.setSpeed(120);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  tmc_write(WRITE_FLAG | 0x14,      0xFFFFFUL); //TCOOLTHRS
  tmc_write(WRITE_FLAG | 0x14,      0x0); //THIGH
  tmc_write(WRITE_FLAG | REG_COOLCONF,    0x0006205UL);
  tmc_write(WRITE_FLAG | REG_GCONF,      0x00000201UL); //voltage on AIN is current reference
  tmc_write(WRITE_FLAG | REG_IHOLD_IRUN, 0x00001010UL); //IHOLD=0x10, IRUN=0x10
  tmc_write(WRITE_FLAG | REG_CHOPCONF,   0x01008008UL); //microsteps, MRES=0, TBL=1=24, TOFF=8
  digitalWrite(EN_PIN, LOW);
  Timer3.initialize();
  tft.reset();
  uint16_t identifier = tft.readID();
  if (identifier == 0x0101)
  {
    identifier = 0x9341;
  }
  tft.begin(identifier);
  tft.setRotation(2);

  tft.fillScreen(BLACK);
  uint16_t concatanator = 0;
  int h = 124 / 2, w = 126, row, col;
  int buffidx = 0;
  for (row = 222 - 2 * h; row < 222 - h; row++) { // For each scanline...
    for (col = 57; col < 183; col++) { // For each pixel...
      //color expects 16 bit int, but it is stored in bytes
      buffidx = (col - 57 + (((222 - h) - row - 1) * w)) * 2;
      concatanator = pgm_read_byte(splashScreenArray2 + buffidx);
      buffidx++;
      concatanator += pgm_read_byte(splashScreenArray2 + buffidx) << 8;//front bite is second
      tft.drawPixel(col, row, concatanator);
    } // end pixel
  }
  for (row = 222 - h; row < 222; row++) { // For each scanline...
    for (col = 57; col < 183; col++) { // For each pixel...
      //color expects 16 bit int, but it is stored in bytes
      buffidx = (col - 57 + (((222) - row - 1) * w)) * 2;
      concatanator = pgm_read_byte(splashScreenArray1 + buffidx);
      buffidx++;
      concatanator += pgm_read_byte(splashScreenArray1 + buffidx) << 8;//front bite is second
      tft.drawPixel(col, row, concatanator);
    } // end pixel
  }

  //  tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, ILI9341_WHITE);
  tft.setCursor(22, 290);
  tft.setTextColor(WHITE, ILI9341_BLACK);
  tft.setTextSize(TEXT_TSIZE - 1);
  tft.print("Calibrating");
  Serial.println("setup done");
  bool homing = true;
  uint32_t last_time = 0;
  uint32_t ms = millis();
  uint32_t data;
  uint8_t s;
  buttons[0].initButton(&tft, 79, 240, 54, 36, BLACK, BLACK, WHITE, "0", 4);
  buttons[1].initButton(&tft, 35, 114, 54, 36, BLACK, BLACK, WHITE, "1", 4);
  buttons[2].initButton(&tft, 79, 114, 54, 36, BLACK, BLACK, WHITE, "2", 4);
  buttons[3].initButton(&tft, 133, 114, 54, 36, BLACK, BLACK, WHITE, "3", 4);
  buttons[4].initButton(&tft, 35, 156, 54, 36, BLACK, BLACK, WHITE, "4", 4);
  buttons[5].initButton(&tft, 79, 156, 54, 36, BLACK, BLACK, WHITE, "5", 4);
  buttons[6].initButton(&tft, 133, 156, 54, 36, BLACK, BLACK, WHITE, "6", 4);
  buttons[7].initButton(&tft, 35, 198, 54, 36, BLACK, BLACK, WHITE, "7", 4);
  buttons[8].initButton(&tft, 79, 198, 54, 36, BLACK, BLACK, WHITE, "8", 4);
  buttons[9].initButton(&tft, 133, 198, 54, 36, BLACK, BLACK, WHITE, "9", 4);
  buttons[10].initButton(&tft, 200, 47, 62, 52, ILI9341_LIGHTGREY, ILI9341_GREEN, BLACK, "Start", 2);
  buttons[11].initButton(&tft, 200, 114, 62, 34, ILI9341_LIGHTGREY, WHITE, BLACK, "Clear", 2);
  buttons[12].initButton(&tft, 200, 157, 62, 34, ILI9341_LIGHTGREY, WHITE, BLACK, "Top", 2);
  buttons[13].initButton(&tft, 200, 200, 62, 34, ILI9341_LIGHTGREY, WHITE, BLACK, "^", 2);
  buttons[14].initButton(&tft, 200, 243, 62, 34, ILI9341_LIGHTGREY, WHITE, BLACK, "v", 2);
  buttons[15].initButton(&tft, 121, 287, 208, 34, ILI9341_LIGHTGREY, WHITE, BLACK, "Set Endpoint", 2);
  buttons[16].initButton(&tft, 200, 47, 62, 52, ILI9341_LIGHTGREY, RED, BLACK, "Stop", 2);
  while (homing) {
    ms = millis();

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(5);  //10
    if ((ms - last_time) > 100) //run every 1s
    {
      last_time = ms;
      s = tmc_read(REG_GSTAT, &data);

      if (s & 0x01) Serial.print(" reset");
      if (s & 0x02) Serial.print(" error");
      if (s & 0x04) {
        Serial.print(" sg2");
      }
      if (s & 0x08) Serial.print(" standstill");
      Serial.println(" ");

      //show REG_DRVSTATUS
      s = tmc_read(REG_DRVSTATUS, &data);
      /*Serial.print("DRVSTATUS: 0x");
      Serial.print(data, HEX);
      Serial.print("\t - ");
      Serial.print("Status: 0x");
      Serial.print(s, HEX);*/
      if (s & 0x01) Serial.print(" reset");
      if (s & 0x02) Serial.print(" error");
      if (s & 0x04) {
        Serial.print(" sg2");
        //TMC2130 outputs on (LOW active)
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(DIR_PIN, HIGH);
        delay(3000);
        homing = false;
        //digitalWrite(EN_PIN, LOW);

      }
      if (s & 0x08) Serial.print(" standstill");
      //Serial.println(" ");

    }


  }
  Serial.println("Home");
  currentPosition = -moveSize * 3;
  tmc_write(WRITE_FLAG | REG_CHOPCONF,   0x07008008UL); //microsteps, MRES=0, TBL=1=24, TOFF=8
  goHome();
  int notFirstRow = 0;
  Serial.println("screen");
  // create buttons
  drawMainScreen(true);
}

//Move time should start overflowing around 25 mintues; I could perhaps extend with an unsigned long
void timedMove(unsigned long moveTime) {
  retract = true;
  /*if (moveTime < 30) {
    moveTime = 30;
    }*/
  bool movingforward = true;
  destinationPosition = maxPosition;
  timePerMove = 1200; // in microseconds
  if ((maxPosition - currentPosition) < 10 && (maxPosition - currentPosition) > 0){
    maxPosition = currentPosition + 10;
  }
  maxTimePerMove = (moveTime * 100 / (maxPosition - currentPosition) * 10);//must be maxPosition-currentPosition > 10 || <0
  if (maxTimePerMove < 0) {
    movingforward = false;//doesn't seem to work right
    destinationPosition = 0;
    maxTimePerMove = 1000;
  }
  if (maxTimePerMove <= 600) {
    maxTimePerMove = 600; // this might be able to go a little faster, but this seems a good max, it should be about 20 sec for a full move
  }

  //homePosition = currentPosition;
  if (timePerMove < maxTimePerMove) {
    timePerMove = maxTimePerMove;
  }
  timedMovement = true;
  start_move(movingforward, timePerMove);
  //Serial.println(timePerMove);

  //read time input and set max rpms as well as destination position at end
}

void timerDone() {
  //digitalWrite(13, HIGH);
  Serial.print("Timer Done");
  stop_move(false);

}

void goHome() {
  timePerMove = 1200; // in microseconds
  maxTimePerMove = 1000; //in microseconds
  if (currentPosition == homePosition) {
  } else {
    bool movingforward = true;
    destinationPosition = homePosition;
    if (destinationPosition < currentPosition) {
      //retracting = true;
      movingforward = false;
    }
    start_move(movingforward, timePerMove);
  }
}

void moveCart(int rotations, int moveDirectionTransfer) {
  destinationPosition = currentPosition + (rotations * moveSize * moveDirectionTransfer + 1 * moveDirectionTransfer);//is the +1 legacy? It was important at one point
  if (destinationPosition > trueMax || destinationPosition < 0) {
  } else {
    if (moving == false) {
      maxTimePerMove = 1200;
      if (moveDirectionTransfer == 1) {
        start_move(true, 1200);
      } else {
        start_move(false, 1200);
      }
    }
  }
}

void setEnd() {
  maxPosition = currentPosition;
  if (maxPosition < moveSize) {
    maxPosition = moveSize;
  }
  eepromWriteLong(0, maxPosition);
}

void playTune(int notes[], int durations[], int BPM)
{
  int tuneSize = sizeof(melody) / sizeof(int);

  // iterate over the notes of the tune:
  for (int thisNote = 0; thisNote < tuneSize; thisNote++) {

    // For details on calculating the note duration using the tempo and the note type,
    // see http://bradthemad.org/guitar/tempo_explanation.php.
    // A quarter note at 60 BPM lasts exactly one second and at 120 BPM - half a second.

    int noteDuration = (int)((1000 * (60 * 4 / BPM)) / durations[thisNote] + 0.);
    tone(buzzerPin, notes[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 20% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.20;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(buzzerPin);
  }
}

void timeCharToInt() {
  time = 0;
  if (timeChar[0] != ' ') {
    time += (timeChar[0] - '0') * 600;
  }
  if (timeChar[1] != ' ') {
    time += (timeChar[1] - '0') * 60;
  }
  if (timeChar[3] != ' ') {
    time += (timeChar[3] - '0') * 10;
  }
  if (timeChar[4] != ' ') {
    time += (timeChar[4] - '0');
  }
  time *= 1000;
}

void updateTime() {
  // update the current time text field
  //Serial.println(textfield);
  tft.setCursor(TEXT_X + 2, TEXT_Y + 10);
  tft.setTextColor(BLACK, WHITE);
  tft.setTextSize(TEXT_TSIZE);
  tft.print(timeChar);
  timeCharToInt();
}

/*void timeShiftLeft() {
  time = time/1000;
  int calcMin = time / 60;
  int calcSec = time % 60;
  time = (calcMin % 10) * 600;
  time += (calcSec / 10) * 60;
  time += (calcSec % 10) * 10;
  time = time*1000;
}*/

void drawMainScreen(bool background) {
  if (background) {
    tft.fillScreen(BLACK);
  }

  for (uint8_t number = 0; number < numOfButtons - 1; number++) { //last button is stop button and I don't want to draw it in general draw time
    buttons[number].drawButton();
  }
  tft.fillRoundRect(17, 21, 144, 56, 5, ILI9341_WHITE);
  updateTime();
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void confirm(void methodToCall()) {
  tft.fillRoundRect(22, 68, 196, 206, 5, ILI9341_LIGHTGREY);
  tft.fillRoundRect(36, 221, 76, 34, 5, WHITE);
  tft.fillRoundRect(128, 221, 76, 34, 5, WHITE);
  tft.setTextColor(BLACK, ILI9341_LIGHTGREY);
  tft.setTextSize(2);
  tft.setCursor(31, 86);
  tft.print("Are you sure");
  tft.setCursor(31, 106);
  tft.print("you want to set");
  tft.setCursor(31, 126);
  tft.print("the current");
  tft.setCursor(31, 146);
  tft.print("position as the");
  tft.setCursor(31, 166);
  tft.print("lower endpoint?");
  tft.setCursor(64, 230);
  tft.setTextColor(BLACK, WHITE);
  tft.setTextSize(2);
  tft.print("OK");
  tft.setCursor(132, 230);
  tft.print("Cancel");

  while (1) {
    digitalWrite(13, HIGH);
    TSPoint o = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    o.x = map(o.x, TS_MINX, TS_MAXX, tft.width(), 0);
    o.y = (tft.height() - map(o.y, TS_MINY, TS_MAXY, tft.height(), 0));
    if (o.z > MINPRESSURE && o.z < MAXPRESSURE) {
      if (o.y >= 221 && o.y <= 255) {
        if (o.x <= 117 && o.x >= 47) {
          methodToCall();
          //tft.fillRoundRect(22, 68, 196, 206, 5, BLACK);
          drawMainScreen(true);
          return;
        } else if (o.x >= 127 && o.x <= 197) {
          //tft.fillRoundRect(22, 68, 196, 206, 5, BLACK);
          drawMainScreen(true);
          return;
        }
      }
    }
  }
}
int buttonPressed = 100;
void loop() {
  buttonPressed = 100;
  //manual = false;
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
    Serial.println(p.x);
    p.y = (tft.height() - map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
    Serial.println(p.y);
    for (uint8_t b = 0; b < numOfButtons; b++) {
      if (buttons[b].contains(p.x, p.y)) {
        Serial.println(b);
        //Serial.print("Pressing: "); Serial.println(b);
        buttonPressed = b; // tell the button it is pressed
        if (b == 16 && moving == true) {
          stop_move(true);
        } else if (moving == false) {
          if (b <= 9 && b >= 0) {
            timeChar[0] = timeChar[1];
            timeChar[1] = timeChar[3];
            timeChar[3] = timeChar[4];
            timeChar[4] = b + '0';
            updateTime();
          } else if (b == 15) {
            void (*functionHolder)() = &setEnd;
            confirm(functionHolder);
          } else if (b == 12) {
            goHome();
          } else if (b == 10) {
            timedMove(time);
          } else if (b == 13 || b == 14) {
            manual = true;
            retract = false;
          } else if (b == 11) {
            timeChar[0] = ' ';
            timeChar[1] = ' ';
            timeChar[3] = ' ';
            timeChar[4] = ' ';
            updateTime();
          }
          b = numOfButtons;
        }
      }
    }
    delay(100);

  }



  if (moving == false || manual == true) {
    if (buttonPressed == 14) {
      moveCart(1, 1);
    } else if (buttonPressed == 13) {
      moveCart(1, -1);
    }
  }
  if (moving == true) {
    if (timedMovement) {
      if (millis() - elapsedTime > 1000) {
        if (timeChar[4] > '0') {
          timeChar[4]--;
        } else if (timeChar[3] > '0') {
          timeChar[3]--;
          timeChar[4] = '9';
        } else if (timeChar[1] > '0') {
          timeChar[1]--;
          timeChar[3] = '5';
          timeChar[4] = '9';
        } else if (timeChar[0] > '0') {
          timeChar[0]--;
          timeChar[1] = '9';
          timeChar[3] = '5';
          timeChar[4] = '9';
        }
        elapsedTime = millis();
        updateTime();
      }
    }
    if (timePerMove != maxTimePerMove) {
      delay(30);
      if (timePerMove > maxTimePerMove) {
        if (timePerMove > 1200) {
          timePerMove -= 100;
        } else if (timePerMove > 1000) {
          timePerMove -= 40;
        } else if (timePerMove > 800) {
          timePerMove -= 20;
        }
      } else {
        timePerMove = maxTimePerMove;
      }
      Timer3.setPeriod(timePerMove);//check that this works and doesn't reset timer and break movement; consider an if to check if time per move = max
    }
    if (moveDirection == -1 && (currentPosition < homePosition + 1 || !(currentPosition > destinationPosition))) {
      stop_move(false);
    } else if (moveDirection == 1 && (currentPosition > (maxPosition - 1) || !(currentPosition < destinationPosition))) {
      if (retract == true) {
        destinationPosition = homePosition;
        timePerMove = 1200;
        maxTimePerMove = 1000;
        stop_move(false);
        delay(100);
        start_move(false, 1200);
      } else if (manual == true && currentPosition < (trueMax) && currentPosition < destinationPosition) {
        //avoid setting to false while the user is moving past the previously set bottom dip point
      } else {
        stop_move(false);
      }
    }
  }
}
