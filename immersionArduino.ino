#include <SimpleTimer.h>
#include <Stepper.h>
#include <Elegoo_GFX.h>    // Core graphics library
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
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

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
// the data (phone #) we store in the textfield
#define TEXT_LEN 12
char textfield[TEXT_LEN+1] = "";
uint8_t textfield_i=0;

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

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

Elegoo_GFX_Button buttons[7];

char buttonlabels[7][5] = {"Move", "Home", "Stop", "+", "-", "+10", "-10" };
uint16_t buttoncolors[7] = {ILI9341_DARKGREEN, ILI9341_DARKGREY, ILI9341_RED, 
                             ILI9341_BLUE, ILI9341_BLUE, 
                             ILI9341_BLUE, ILI9341_BLUE};

//break between GUI and controls

long timeToMove = 0; //overflows if in milliseconds
bool retracting = false;
SimpleTimer timer;
long currentPosition = 0;
int stepsToLower = 100;
int moveDirection = 1;
long maxPosition = 193*2;
int rotationSpeed = 160;
long destinationPosition = 1800;
long betweenSteps = 1;
bool moving = false;
int maxSpeed = 360; //this seems to be safe but I've accidentally taken it up to 460 and it worked fine with the acceleration
Stepper actuator(3200, 49,47);

void setup() {
  // put your setup code here, to run once:
  /*pinMode(53, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(47, OUTPUT);*/
//  currentPosition = analogRead(sensorPin); 
  Serial.begin(9600);
  Serial.println("starting");
  actuator.setSpeed(120);
  tft.reset();

  uint16_t identifier = tft.readID();
  if(identifier==0x0101)
  {     
      identifier=0x9341;
  }
  tft.begin(identifier);
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  int notFirstRow = 0;
  // create buttons
  for (uint8_t number=0; number<7; number++) { //This is messy because it was faster to reuse example and fadangle columns and rows than redo the placement from scratch
  int row = 0;
  int col = number;
  int buttonWidth = BUTTON_W;
  if (number <=2){
    row = 0;
  }
  else if (number >2){
    buttonWidth = buttonWidth *1.5;
    row = 1;
    col -= 3;
    notFirstRow = 1;
  }
  if (number >4) {
    row = 2;
    col -= 2;
  }
      buttons[number].initButton(&tft, BUTTON_X+col*(buttonWidth+BUTTON_SPACING_X)+(buttonWidth-BUTTON_W)-BUTTON_SPACING_X/2*notFirstRow, 
                 BUTTON_Y+row*(BUTTON_H+BUTTON_SPACING_Y),    // x, y, w, h, outline, fill, text
                  buttonWidth, BUTTON_H, ILI9341_WHITE, buttoncolors[number], ILI9341_WHITE,
                  buttonlabels[number], BUTTON_TEXTSIZE); 
      buttons[number].drawButton();
  }
  
  // create 'text field'
  tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, ILI9341_WHITE);
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
    p.y = (tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
   }
 for (uint8_t b=0; b<7; b++) {
    if (buttons[b].contains(p.x, p.y)) {
      //Serial.print("Pressing: "); Serial.println(b);
      buttons[b].press(true);  // tell the button it is pressed
    } else {
      buttons[b].press(false);  // tell the button it is NOT pressed
    }
  }
  for (uint8_t b=0; b<7; b++) {
    if (buttons[b].justReleased()) {
      // Serial.print("Released: "); Serial.println(b);
      buttons[b].drawButton();  // draw normal
    }
    
    if (buttons[b].justPressed()) {
        buttons[b].drawButton(true);  // draw invert!
        
        // if a numberpad button, append the relevant # to the textfield
        if (b >= 3) {
          if (b == 3){
            time += 1;
          }
          else if (b == 4) {
            time -= 1;
          }
          else if (b == 5){
            time += 10;
          }
          else if (b == 6) {
            time -= 10;
          }
          if (time < 0){
            time = 0;
          }
          /*
          if (textfield_i < TEXT_LEN) {
            textfield[textfield_i] = buttonlabels[b][0];
            textfield_i++;
	    textfield[textfield_i] = 0; // zero terminate
            
           // fona.playDTMF(buttonlabels[b][0]);
          }*/
          long calctime = time;
          int counter = sizeof(textfield)-1;
          textfield[counter] = 0;
          counter--;
          Serial.println(calctime/10);
          while (calctime/10 > 0){
            Serial.println(calctime);
            textfield[counter] = calctime%10 + '0';
            calctime = calctime/10;
            counter--;
          }
          textfield[counter] = calctime%10 + '0';
          counter--;
          while(counter>=0){
            textfield[counter] = ' ';
            counter--;
          }
        }

        // Home button send it home
        if (b == 1) {
          //TODO
        }

        // update the current text field
        Serial.println(textfield);
        tft.setCursor(TEXT_X + 2, TEXT_Y+10);
        tft.setTextColor(TEXT_TCOLOR, ILI9341_BLACK);
        tft.setTextSize(TEXT_TSIZE);
        tft.print(textfield);

        // Stop button
        if (b == 2) {
          moving = false;
        }
        // we dont really check that the text field makes sense
        // just try to call
        if (b == 0) {
          timedMove(time);
        }
        
      //delay(100); // UI debouncing //only change during copying; I think the rest of the loop will cover this delay and I don't really want to delay the move
    }
  }
  if (Serial.available() > 0) {
    maxSpeed = 360;
    rotationSpeed = 160;
    String input = Serial.readString();
    Serial.print("I received: ");
    Serial.println(input);
    moving = true; 
    Serial.println(input.charAt(0));
    if (input.charAt(0) == 'p'){
      input = input.substring(1,input.length());
      bool overide = false;
      if (input.charAt(0) == 'o'){
        overide = true;
        input = input.substring(1,input.length());
      }
      Serial.println("chopped input:" + input);
      destinationPosition = input.toInt(); //toInt apparently returns a long according to forums
      //moving = false; //remove later
      Serial.print("destination position:");
      Serial.println(destinationPosition);
      if (destinationPosition < currentPosition){
        //retracting = true;
        moveDirection = -1;
        if (overide == true){
          currentPosition = 100000;
        }
      } else{
        moveDirection = 1;
        if (overide == true){
          currentPosition = -100000;
        }
      }
    }
    else if (input.charAt(0) == 'z'){
      moving = false;
      currentPosition = 0;
    } else if (input.charAt(0) == 't'){
      input = input.substring(1,input.length());
      long moveTime = input.toInt(); //toInt apparently returns a long according to forums
      timedMove(moveTime);
    }
    else if (input.charAt(0) == 's'){
      moving = false;
      Serial.println(currentPosition);
    }
    else {
    Serial.println(moving);
    timeToMove = input.toInt();
    moveDirection = 1;
    actuator.setSpeed(rotationSpeed);
    destinationPosition = 193;
    if (timeToMove < 0){
      //retracting = true;
      timeToMove = timeToMove *-1;
      moveDirection = -1;
      destinationPosition = 0;
    }
    betweenSteps = timeToMove*1000/maxPosition;
    Serial.println(timeToMove);
    Serial.println(betweenSteps);
      if (timeToMove>0){ // make sure to double check that you can't set double timers.
        //timer.deleteTimer(1);  causes compile error
        timer.setTimeout(timeToMove*1000, timerDone);
        timeToMove = 0;
      }
    }
  }
  if (moving == true){
    if (rotationSpeed < maxSpeed) {
        rotationSpeed += 15;
      actuator.setSpeed(rotationSpeed);
      Serial.println(rotationSpeed);
    } else {
      Serial.print("current speed is ");
      Serial.println(maxSpeed);
      actuator.setSpeed(maxSpeed);
    }
    if ((moveDirection == -1 && (currentPosition < 3 || !(currentPosition > destinationPosition))) || (moveDirection == 1 && (currentPosition > maxPosition ||!(currentPosition < destinationPosition)))){
      actuator.setSpeed(rotationSpeed-100);
      moving = false;
    }
    actuator.step(moveDirection*3200);
    currentPosition += moveDirection;
   // delay(betweenSteps-1);
    //Serial.println(currentPosition);
  }

  /*if (retracting == true){
    if (currentPosition >= maxPosition-10){ //replace 1000 with max position
      Serial.println("stopping at end");
      stopMove();
    }
  }*/
}

void timerDone() {
  //digitalWrite(13, HIGH);
  Serial.print("Timer Done");
  moving = false;
}

void timedMove(long moveTime) {
  moving = true;
  moveDirection = 1;
  destinationPosition = maxPosition;
  if (currentPosition >= (maxPosition -5)){
    moveDirection = -1;
    destinationPosition = 0;
  }
  /*if (moveTime <40) { //better covered by maxSpeed
    moveTime = 40;
  }*/
  maxSpeed = (abs(destinationPosition-currentPosition)-40)*60/(moveTime-10)*3/2;
  Serial.print("max speed: ");
  Serial.println(maxSpeed);
  if (maxSpeed >360) {
    //maxSpeed = 360;
  }
  //read time input and set max rpms as well as destination position at end
}

/*
void startMove(bool up){
  // Digital write to two relay pin to move up and down
  if (up==true){
    digitalWrite(outputDirection, HIGH);//digital writes should be redone
    moveDirection = 1;
  }
  digitalWrite(outputOn, HIGH);
  moveDirection = -1;
}

void stopMove() {
  digitalWrite(outputOn, LOW);//digital writes should be redone
  digitalWrite(outputDirection, LOW);
  moving = false;
  retracting = false;
}

void retract(){
  startMove(false);
  retracting = true;
}*/
