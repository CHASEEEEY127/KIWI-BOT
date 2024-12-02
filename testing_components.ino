//library inclusions
  #include <NewPing.h>//sonar

  #include <MPU9250_asukiaaa.h>//gyro

  #include <ESP32Servo.h>//servos

  #include <LiquidCrystal_I2C.h>//lcd

  #include <ps5.h>//ps5
  #include <ps5Controller.h>//ps5
  #include <ps5_int.h>//ps5

  #include <Adafruit_NeoPixel.h>//led

  #include <classes.h>//classes
  #include <Wire.h>//"CAN"

//vars and defs
  //motor definitions
    int motor1_pwm = 33;
    int motor1_FWD = 25;
    int motor1_BWD = 32;
    int motor1_angle_degrees = 0;
    float motor1_angle =motor1_angle_degrees*(M_PI/180);

    int motor2_pwm = 19;
    int motor2_FWD = 5;
    int motor2_BWD = 18;
    int motor2_angle_degrees = 120;
    float motor2_angle =motor2_angle_degrees*(M_PI/180);

    int motor3_pwm = 26;
    int motor3_FWD = 27;
    int motor3_BWD = 14;
    int motor3_angle_degrees = 240;
    float motor3_angle =motor3_angle_degrees*(M_PI/180);

  //speeds
  float motor1_Speed;
  float motor2_Speed;
  float motor3_Speed;

  int maxMotorSpeed = 204;
  //create a turret servo
    int servo_pwm=23;
    Servo turret;

  //led vars
    int led_count=8;
    Adafruit_NeoPixel indiStrip = Adafruit_NeoPixel(led_count, 0, NEO_GRB + NEO_KHZ800); 

  //gyro define
    MPU9250_asukiaaa mySensor;

  //lcd define
    LiquidCrystal_I2C lcd(0x27, 16, 2);
    int lcdScreen=0;


  float aX, aY, aZ, aSqrt,gX,gY,gZ,magX,magY,magZ,magHeading ,initialHeading,baseGyroVal;//mpu9250 vals
  float gyroTester=0;

  double heading;



  //pid vars
    double headingPID;//overall PID output
    double goalHeading;
    double error;
    double Perror;

    double headingIn=0; //gyro

    double headingP=5;    //Kp
    double headingPO;     //proportional output

    double headingI=1;    //Ki
    double headingIO=0;   //integration output
    int ITimer=0; 
    int DOI=1;//whether I needs to be integrated(1 or 0)
    double maxI=100;      //limit Iwindup
    double headingD=10;   //Kd
    double headingDO;     //derivitive output

  //controller vars
    bool r1pressedrecent;
    bool l1pressedrecent;
    bool pspressedrecent;
    



//functions
  //kiwidrive function
    void kiwiDrive(int Forwards,int Right,int Rotate,float botRot){
      Forwards*=-1;               //make positive speed forwards    
      botRot =botRot*(M_PI/180);  //convert gyro degrees to radians

      motor1_Speed = 
            (((Forwards)*sin(motor1_angle+botRot))+
            (Right)*cos(motor1_angle+botRot)+
            (Rotate));
      motor2_Speed = 
            (((Forwards)*sin(motor2_angle+botRot))+
            (Right)*cos(motor2_angle+botRot)+
            (Rotate));
      motor3_Speed = 
            (((Forwards)*sin(motor3_angle+botRot))+
            (Right)*cos(motor3_angle+botRot)+
            (Rotate));
    }

  //update gyro values
    void update_gyro(){
      mySensor.accelUpdate();
      mySensor.magUpdate();
      mySensor.gyroUpdate();
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      aSqrt = mySensor.accelSqrt();
      gX = mySensor.gyroX();
      gY = mySensor.gyroY();
      gZ = mySensor.gyroZ();
      magX = mySensor.magX();
      magY = mySensor.magY();
      magZ = mySensor.magZ();
      magHeading = mySensor.magHorizDirection();
    }




//default toggles
  bool waitForPs5 = false;
  bool doAuto = true;
  bool printConnected= false;
  bool headingCorrection=false;
  bool feildO=true;
  bool allowPIDCHanging=true;

  bool printPIDKs = true;
  bool printHeading=false;
  bool printSticks=false;
  int totalLCDScreens=3;

//setup
  void setup() { 
  //system starting
    Wire.begin();       //initiate "can" bus
    Serial.begin(9600); //communicate with computer baud rate 9600

  //gyro beginning
    mySensor.setWire(&Wire);//attach gryo
    mySensor.beginMag();
    mySensor.beginGyro();
    mySensor.beginAccel();

    update_gyro();

    baseGyroVal=0;

  //gyro calibration
    for(int i=0;i<100;i++){
      mySensor.gyroUpdate();
    
      baseGyroVal+= mySensor.gyroZ();
      delay(30);
    }
    baseGyroVal/=100;
    Serial.println(baseGyroVal);

  // servo init
    turret.attach(servo_pwm,500,2500);

  //led strip init
    indiStrip.begin();
    indiStrip.show();

  //motor pin inits
    pinMode(motor1_pwm, OUTPUT);
    pinMode(motor1_FWD, OUTPUT);
    pinMode(motor1_BWD, OUTPUT);

    pinMode(motor2_pwm, OUTPUT);
    pinMode(motor2_FWD, OUTPUT);
    pinMode(motor2_BWD, OUTPUT);

    pinMode(motor3_pwm, OUTPUT);
    pinMode(motor3_FWD, OUTPUT);
    pinMode(motor3_BWD, OUTPUT);

    digitalWrite(motor1_FWD,LOW);
    digitalWrite(motor1_BWD,LOW);
    digitalWrite(motor2_FWD,LOW);
    digitalWrite(motor2_BWD,LOW);
    digitalWrite(motor3_FWD,LOW);
    digitalWrite(motor3_BWD,LOW);
  

  //ps5 init
    ps5.begin("4C:B9:9B:A5:54:8E");

  //lcd init
    lcd.init();
    lcd.backlight(); 

    Serial.println("Ready.");
    
  }



//loop
void loop() {

  update_gyro();

  //heading reset
    if(ps5.Touchpad()){
      gyroTester=0;
      headingIn=0;
    }

  
  //integrate gyro 
    if(gZ<0){
    gyroTester+=(((gZ-baseGyroVal)*36/34)*36/38)*36/40*36/43*36/35*360/346.17*36/37*36/37;}
    else{
      gyroTester+=((((gZ-baseGyroVal)*36/34)*36/38)*360/354)*36/40*36/43*360/338*9/10*360/345.45*36/37;}

    heading= (gyroTester/12);


//toggle feild orientation and heading correction
    feildO=toggleButton(ps5.R1(),&r1pressedrecent,feildO);

    if(feildO==false){
      heading=0;
      headingCorrection=false;
    }
    else{
      headingCorrection=toggleButton(ps5.L1(),&l1pressedrecent,headingCorrection);
    }

  //watiting for ps5 controller
    while (ps5.isConnected()==false && waitForPs5==true){
      //flash LED
        indiStrip.setPixelColor(7,0,0,255);
        indiStrip.show();
          delay(500);
        indiStrip.setPixelColor(7,0,0,0);
        indiStrip.show();
          delay(500);
          
    }

  //if no heading correction
    //basic rotation drive
      if(headingCorrection==false){
        kiwiDrive(ps5.LStickY()*2,ps5.LStickX()*2,ps5.RStickX()*1.5,heading);
      }

  //if heading correction
    //heading correction
    else{
      headingIn+=ps5.RStickX()/4;//move setpoint based on controller

      kiwiDrive(ps5.LStickY()*2,ps5.LStickX()*2,pidLoop(headingIn,heading,headingP,headingI,headingD,&headingIO,false,2.0,&ITimer,100,&Perror),heading);


    }
  //Normalize motors
    if(max(max(abs(motor1_Speed),abs(motor2_Speed)),abs(motor3_Speed))>=maxMotorSpeed){
        float ratio = 256/(max(max(abs(motor1_Speed),abs(motor2_Speed)),abs(motor3_Speed)));
        
        motor1_Speed*=ratio;
        motor2_Speed*=ratio;
        motor3_Speed*=ratio;
        }
  //set motors
    //set motor directions
      if(motor1_Speed>0){
          digitalWrite(motor1_FWD,0);
          digitalWrite(motor1_BWD,1);
        }
        else{
          digitalWrite(motor1_FWD,1);
          digitalWrite(motor1_BWD,0);
        }

        if(motor2_Speed>0){
          digitalWrite(motor2_FWD,0);
          digitalWrite(motor2_BWD,1);
        }

        else{
          digitalWrite(motor2_FWD,1);
          digitalWrite(motor2_BWD,0);
        }
      

        if(motor3_Speed>0){
          digitalWrite(motor3_FWD,0);
          digitalWrite(motor3_BWD,1);
        }
        else{
          digitalWrite(motor3_FWD,1);
          digitalWrite(motor3_BWD,0);
        }
    //set motors speeds
      analogWrite(motor1_pwm,abs(motor1_Speed));
      analogWrite(motor2_pwm,abs(motor2_Speed));
      analogWrite(motor3_pwm,abs(motor3_Speed));


  //get drive direction angle
    int leftStickAngle  = atan2(ps5.LStickY(),ps5.LStickX())*180/M_PI;
    leftStickAngle-=90;
    if(leftStickAngle<0){
      leftStickAngle+=359;
    }
      leftStickAngle*=-1;
      leftStickAngle+=360;
    float leftStickMagnitude = sqrt(ps5.LStickX()*ps5.LStickX()+ps5.LStickY()*ps5.LStickY());


  //allow change of PID values
    if(allowPIDCHanging){
      
      if(ps5.Up()){
        headingP+=0.1;
      }
      if(ps5.Down()){
        headingP-=0.1;
      }

      if(ps5.Right()){
        headingD+=0.1;
      }if(ps5.Left()){
        headingD-=0.1;
      }

      if(ps5.Options()){
        headingI+=0.01;
      }if(ps5.Share()){
        headingI-=0.01;
      }
    } 
  
  //change lcd setting
  if(ps5.PSButton()){
    if(pspressedrecent==false){
      lcdScreen++;}

    pspressedrecent=true;

  }
  else{
    pspressedrecent=false;
  }
  
  //lcd printing
    lcd.clear();
    if(lcdScreen%totalLCDScreens==0){
      printPIDKs=true;
      printHeading=false;
      printSticks=false;
    }
    if(lcdScreen%totalLCDScreens==1){
      printHeading=true;
      printPIDKs=false;
      printSticks=false;
    }
    if(lcdScreen%totalLCDScreens==2){
      printHeading=false;
      printPIDKs=false;
      printSticks=true;
    }

      if(printPIDKs){
        lcd.setCursor(0,0);
        lcd.print("KP");

        lcd.setCursor(5,0);
        lcd.print("KI");

        lcd.setCursor(11,0);
        lcd.print("KD");

        lcd.setCursor(0,1);
        lcd.print(headingP,1);

        lcd.setCursor(5,1);
        lcd.print(headingI,2);

        lcd.setCursor(11,1);
        lcd.print(headingD,1);
        //(gyroTester/12)*360/340
      }

      if(printHeading){
        lcd.setCursor(0,0);
        lcd.print("0");

        lcd.setCursor(8,0);
        lcd.print("Err");

        lcd.setCursor(0,1);
        lcd.print(heading,1);

        lcd.setCursor(8,1);
        lcd.print(error,1);
      }

      if(printSticks){
        lcd.setCursor(0,0);
        lcd.print("LY");

        lcd.setCursor(4,0);
        lcd.print("LX");

        lcd.setCursor(8,0);
        lcd.print("LM");

        lcd.setCursor(12,0);
        lcd.print("RX");

        lcd.setCursor(0,1);
        lcd.print(abs(ps5.LStickY()),1);

        lcd.setCursor(4,1);
        lcd.print(abs(ps5.LStickX()),1);

        lcd.setCursor(8,1);
        lcd.print(abs(leftStickMagnitude),1);

        lcd.setCursor(12,1);
        lcd.print(abs(ps5.RStickX()),1);


      }
    
  //set leds
  /*
    if(feildO){
      indiStrip.setPixelColor(6,0,255,0);
    }
    else{indiStrip.setPixelColor(6,255,0,0);
    headingCorrection=false;}

    if(headingCorrection){
      indiStrip.setPixelColor(5,0,255,0);
    }
    else{indiStrip.setPixelColor(5,255,0,0);}
*/
    StatusLed(feildO,6,&indiStrip);
    StatusLed(headingCorrection,5,&indiStrip);

    if(ps5.isConnected()){
      indiStrip.setPixelColor(7,0,0,255);
    }

    setMotorLed(motor1_Speed,15,1,&indiStrip);
    setMotorLed(motor2_Speed,15,2,&indiStrip);
    setMotorLed(motor3_Speed,15,0,&indiStrip);


    indiStrip.show();


  delay(30);//delay loop

  
  
  

}
