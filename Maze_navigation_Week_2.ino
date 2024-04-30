#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
LiquidCrystal lcd(19,23,5,17,16,15);
#define ROWS  4
#define COLS  3
#define I2C_SLAVE_ADDR 0x04 

int t = 1;
char keyMap[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
 
uint8_t rowPins[ROWS] = {14, 27, 26, 25}; // GIOP14, GIOP27, GIOP26, GIOP25
uint8_t colPins[COLS] = {33, 32, 18};     // GIOP33, GIOP32, GIOP18
 
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS );
MPU6050 mpu(Wire);

const int M = 35 ;
int arraySize=0;
int direction_command;
int movement_command;
int distance;
int i = 0;
char keyArray[40];


int previousAngle;
int currentAngle;
int turn_Angle;
int timer;

float distanceTravelled = 0;
int16_t encRight_count = 0;
int16_t encLeft_count = 0;

int servoAngle = 90;
int rightMotorSpeed = 0; 
int leftMotorSpeed = 0;
int ArraySize;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  lcd.begin(25,2); //seting up the columns and rows used on the lcd 
  lcd.print(" Hello world");//setup test message

  
  byte status = mpu.begin(); //setting up the MPU6050
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // gyro and accelero
  Serial.println("Done!\n");
  
}
 
void loop(){
  char key; //declaring variable 
  mpu.update();//updating values from MPu
  key= keypad.getKey(); //retrive key press 
  if (key)
  {  
    keyArray[i]=key;//key press stored at current locition in array 
    Serial.print("Key pressed:");
    Serial.println(key);//prints the key press
    lcd.clear();//clears the lcd screen of any previous inputs
    lcd.setCursor(0,0); // sets the location of the cursor to top left of the screen
    lcd.print(key);//prints key press on the lcd screen
    ArraySize++;//increments the array size by 1

    if(key == '*')
    {
      Serial.println("run now!");//serial monitor print when * is pressed, which signals that the inputs are complete 
      Serial.println(keyArray);//complete array printed 
      ArraySize--;//Array size decreased by 1 so that * is not included
      Serial.println(ArraySize);//serial monitor print of ArraySize
      runMaze(ArraySize);//runMaze function called with ArraySize passed into it
      lcd.clear(); //lcd screen cleared 
    }

    else if (key == '#'){
      i = 0; // # resets the array back to empty 
      keyArray[40]={};//array emptied
      ArraySize=0;
      Serialprintln("Array cleared");
      lcd.clear();
      lcd.print("Array cleared");
        
        
    }
    else if (( '0' < keyArray[i]) && ( keyArray[i] > '10'))
    {
      keyArray[i]=key; // array is updated with current key press
        
    }
      
    i++;// loop counter incremented by 1 
      
  }
    
  /*  else{
    Serial.println("No command given"); // no command input 
    lcd.clear();
    lcd.print("No command"); 
  }
  */
  
                    
    
}

void runMaze( int ArraySize)
{
  Serial.println("running maze");
  lcd.clear();
  lcd.print("running maze");
  // while(1){

  // }

  for (int j = 0; j < ArraySize; j++)//loops through the now smaller array 
  {
    
    if(keyArray[j] =='7')
    { // if key press at current position is 7
      Serial.print("Forwards ");
      String distanceString = "";
      char charVal = keyArray[j+1];//key press at 1 position higher than loop counter
      if (charVal == '7' || charVal == '8' || charVal == '9'){
        j++; //loop counter is incremented by 1 to avoid confusion with the set commands vs use as a multiplier
      }
      distanceString += charVal; // key press is added to string
      int distanceVal = distanceString.toInt(); // string is converted to integer and stored in new variable
      int travelDistance = distanceVal*0.1;// distance to travel is the key press multiplied by 10cm 
      Serial.print(travelDistance);//serial print of the desired distance 
      Serial.println(" cm");
      uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
      uint8_t encRight_count16_9 = Wire.read();                     // receive bits 16 to 9 of a (one byte)
      uint8_t encRight_count8_1 = Wire.read();                      // receive bits 8 to 1 of a (one byte)
      uint8_t encLeft_count16_9 = Wire.read();                      // receive bits 16 to 9 of b (one byte)
      uint8_t encLeft_count8_1 = Wire.read();                    // receive bits 8 to 1 of b (one byte)

      encRight_count = (encRight_count16_9 << 8) | (encRight_count8_1);  // combine the two bytes into a 16 bit number
      encLeft_count = (encLeft_count16_9 << 8) | (encLeft_count8_1);     // combine the two bytes into a 16 bit number
      forwards_10cm(encRight_count,encLeft_count,travelDistance);
    }

    else if (keyArray[j] == '8')
    {//if key press at current position is 8
      Serial.print("Turn 90 ");//both inputs of 8 and 9 relate to turns of either 90 or 180 degrees, in that order 
      if(keyArray[j+1] == '1')
      {// turn is to the left if the next key press in the array is 1 
        Serial.println("Left");
        turn_Angle=-90;
        rotate_90(turn_Angle);//calls the function and passes in the specified angle 
      }
      
      else if(keyArray[j+1] == '2')
      {// same as above but to the right instead 
        Serial.println("Right");
        turn_Angle=90;
        rotate_90(turn_Angle);
      }
    }

    else if(keyArray[j] == '9')
    { // same as for 8 but with 180 degrees
      Serial.print("Turn 180 ");
      if(keyArray[j+1] == '1')
      {
        Serial.println("Left");
        turn_Angle=-180;
        rotate_180(turn_Angle);
      }
      
      else if(keyArray[j+1] == '2')
      {
        Serial.println("Right");
        turn_Angle=180;
        rotate_180(turn_Angle);
      }
    }
  } 
}

void forwards_10cm(int16_t encRight_count, int16_t encLeft_count, int travelDistance)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("7");
  lcd.setCursor(0, 1);
  lcd.print("Moving forwards");
  Serial.print("Moving forwards");
  float averageCount, PPM, current_distanceTravelled, distanceTravelled;
  servoAngle=90;
  averageCount = (encRight_count + encLeft_count) / 2; //working out distance travelled 
  PPM = 24 / (M_PI * 0.059);
  distanceTravelled = (averageCount / PPM);
  while (distanceTravelled + current_distanceTravelled < travelDistance)
  {
    leftMotorSpeed=-250;
    rightMotorSpeed=-250;
    uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
    uint8_t encRight_count16_9 = Wire.read();                     // receive bits 16 to 9 of a (one byte)
    uint8_t encRight_count8_1 = Wire.read();                      // receive bits 8 to 1 of a (one byte)
    uint8_t encLeft_count16_9 = Wire.read();                      // receive bits 16 to 9 of b (one byte)
    uint8_t encLeft_count8_1 = Wire.read();                    // receive bits 8 to 1 of b (one byte)

    encRight_count = (encRight_count16_9 << 8) | encRight_count8_1;  // combine the two bytes into a 16 bit number
    encLeft_count = (encLeft_count16_9 << 8) | encLeft_count8_1;     // combine the two bytes into a 16 bit number
    averageCount = (encRight_count + encLeft_count) / 2; //working out distance travelled 
    PPM = 24 / (M_PI * 0.059);
    current_distanceTravelled = (averageCount / PPM);
  
    sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);
  }

  if (current_distanceTravelled> travelDistance){
    leftMotorSpeed=0;
    rightMotorSpeed=0;
    sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);
  }
  millis();
}


void rotate_90(int turn_Angle)
{
  timer = 0; //used for mpu data collection
  previousAngle=0;
  Serial.println("Turning 90 degrees");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("8");
  lcd.setCursor(0, 1);
  if (turn_Angle == -90)
  {
    lcd.print("Turning left");
    Serial.print("Turning left");
  }

  else if(turn_Angle == 90)
  {
    lcd.print("Turning right");
    Serial.print("Turning right");
  }
  
  if((millis()-timer)>10){
    currentAngle=mpu.getAngleX(); 
    timer=millis();
    previousAngle=currentAngle;
  }
  while(currentAngle >= (previousAngle +turn_Angle))
  {
    servoAngle=30;
    leftMotorSpeed=0;
    rightMotorSpeed=250;
    sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);
    currentAngle=mpu.getAngleX();
  }
  
  previousAngle=currentAngle;
  servoAngle=95;
  leftMotorSpeed=250;
  rightMotorSpeed=250;
  sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);
  
}

void rotate_180(int turn_Angle)
{
  timer = 0;
  previousAngle=0;
  Serial.println("right 180");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("9");
  lcd.setCursor(0,1);
  
  if (turn_Angle == -180)
  {
    lcd.print("Turning left");
    Serial.print("Turning left");
  }

  else if(turn_Angle == 180)
  {
    lcd.print("Turning right");
    Serial.print("Turning right");
  }
  if((millis()-timer)>10){
    currentAngle=mpu.getAngleX();
    previousAngle=currentAngle; 
    timer=millis();
  }
  while(currentAngle != (previousAngle+ turn_Angle))
  {
    servoAngle=150;
    leftMotorSpeed=250;
    rightMotorSpeed=0;
    sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);
    currentAngle=mpu.getAngleX();
  }
  
  
  previousAngle=currentAngle;
  servoAngle=95;
  leftMotorSpeed=250;
  rightMotorSpeed=250;
  sendCarMovementCommands (leftMotorSpeed, rightMotorSpeed, servoAngle);

}

void sendCarMovementCommands (int16_t leftMotorSpeed, int16_t rightMotorSpeed, int16_t servoAngle)  {
  Wire.beginTransmission (I2C_SLAVE_ADDR);
  Wire.write((byte) ((leftMotorSpeed & 0XFF00) >> 8)); // High byte of leftMotorSpeed 
  Wire.write((byte) (leftMotorSpeed & 0x00FF));         // Low byte of leftMotorSpeed 
  Wire.write((byte) ((rightMotorSpeed & 0xFF00) >> 8));   // High byte of rightMotorSpeed 
  Wire.write ((byte) (rightMotorSpeed & 0x00FF));           // Low byte of rightMotorSpeed 
  Wire.write((byte) ((servoAngle & 0XFF00) >> 8)); // High byte of servosteeringAngle
  Wire.write ((byte) (servoAngle & 0x00FF)); // Low byte of servosteeringAngle 
  Wire.endTransmission();// End the I2C transmission ;
}     
  

// void moveForward(int distance, int servoangle)  {
//   int16_t motorSpeed = 150; // Example speed value 
//   int duration = distance * 100; // Placeholder for duration calculation, adjust based on testing 
//   sendCarMovementCommands (motorSpeed, motorSpeed, servoAngle); delay(duration); // Delay based on distance 
//   sendCarMovementCommands (0, 0, servoAngle); // Stop the motors after moving forward
// }



