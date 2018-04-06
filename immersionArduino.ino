#include <SimpleTimer.h>
#include <Stepper.h>

long timeToMove = 0; //overflows if in milliseconds
const int outputOn = 7;//change pins
const int outputDirection = 8;
const int sensorPin = 0;
bool retracting = false;
SimpleTimer timer;
long currentPosition = 0;
int stepsToLower = 100;
int moveDirection = 1;
long maxPosition = 193;
int rotationSpeed = 160;
long destinationPosition = 1800;
long betweenSteps = 1;
bool moving = false;
int maxSpeed = 360; //this seems to be safe but I've accidentally taken it up to 460 and it worked fine with the acceleration
Stepper actuator(3200, 8,9);

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
//  currentPosition = analogRead(sensorPin); 
  Serial.begin(9600);
  Serial.println("starting");
  actuator.setSpeed(160);

}

void loop() {
  timer.run();
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    maxSpeed = 360;
    rotationSpeed = 160;
    String input = Serial.readString();
    Serial.print("I received: ");
    Serial.println(input);
    moving = !moving; 
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
        retracting = true;
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
      int moveTime = input.toInt(); //toInt apparently returns a long according to forums
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
      retracting = true;
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
        rotationSpeed += 5;
      actuator.setSpeed(rotationSpeed);
      Serial.println(rotationSpeed);
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

void timedMove(int moveTime) {
  moving = true;
  moveDirection = 1;
  destinationPosition = 193;
  if (currentPosition >= maxPosition -5){
    moveDirection = -1;
    destinationPosition = 0;
  }
  /*if (moveTime <40) { //better covered by maxSpeed
    moveTime = 40;
  }*/
  maxSpeed = (Math.abs(destinationPosition-currentPosition)-40)*60/(moveTime-10)*3/2;
  if (maxSpeed >360) {
    maxSpeed = 360;
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