#ifndef __classes_H__
#define __classes_H__
#include <Adafruit_NeoPixel.h>//led

static void setMotorLed(int Speed,int Threshhold,int ID, Adafruit_NeoPixel *strip){
    if(Speed>Threshhold){
      strip->setPixelColor(ID,0,Speed*255/260,0);
      }
    else if(Speed<-Threshhold){
      strip->setPixelColor(ID,abs(Speed*255/260),0,0);
    }
    else{
      strip->setPixelColor(ID,0,0,0);
    }

}

static void StatusLed(bool toggle, int ID, Adafruit_NeoPixel *strip){
  if(toggle){
      strip->setPixelColor(ID,0,255,0);
    }
  else{
      strip->setPixelColor(ID,255,0,0);
    }
}


static bool buttonPressOnce(bool button,bool *pressedRecent){
  bool output=false;
  if(button&&(*pressedRecent==false)){
    output=true;
    *pressedRecent=true;
    }
  if(button==false){
      *pressedRecent=false;
    }
    return output;


}
static bool toggleButton(bool button,bool *pressedRecent,bool output){
  /*if(button&&(*pressedRecent==false)){
    output=!output;
    *pressedRecent=true;
    }
    if(button==false){
      *pressedRecent=false;
    }/*
    return output;*/
    if(buttonPressOnce(button,pressedRecent)){
        output=!output;
    }
    return output;
}



static int pidLoop(double setpoint,double current,double KP,double KI,double KD, double *IO,bool loop360,double threshHold,int *ITimer,double maxI, double *PreviousError){          

          double error=(setpoint-current);
          if(loop360){
            while(error>180){
              error-=360;
            }
            while(error<-180){
              error+=360;
            }
          }
          int DOI;

          //if im not close
          //do PID
            if(abs(error)>threshHold){
              //kiwiDrive(ps5.LStickY()*2,ps5.LStickX()*2,headingPID,heading);//PID
              *ITimer=0;//reset I's cancelation timer
              DOI=1;   //ensure I is integrating
            }
        //if I am close  
          //wait 20 cycles to reset I
            else if(*ITimer<20){
              *ITimer++;
              DOI=0;  //dont integrate anymore
              //kiwiDrive(ps5.LStickY()*2,ps5.LStickX()*2,0,heading);
            }

          //stop PID  
            else{
              //kiwiDrive(ps5.LStickY()*2,ps5.LStickX()*2,0,heading);//dont rotate anymore
              *IO=0;//reset I output
              DOI=0;//stop integrateing
            }


          double PO=error*KP;//set P
          double DO=(error-*PreviousError)*KD;//set D
          *IO+= error*KI;//integreate I
          *IO=max(min(*IO,maxI),-maxI);
          int output=(PO+DO+(*IO))*DOI;//total output
          

        // setprevious values
          *PreviousError=error; //remember error fo rnext cycle

          return output;
      }






 class Motor_L928N{

  public:
    int FWD_PIN;
    int BKD_PIN;
    int SPD_PIN;
    int SPD;

    Motor_L928N(int F_PIN, int B_PIN, int S_PIN){

      int FWD_PIN=F_PIN;
      int BKD_PIN=B_PIN;
      int SPD_PIN=S_PIN;

      SPD=0;
      pinMode(F_PIN, OUTPUT);
	    pinMode(B_PIN, OUTPUT);
	    pinMode(S_PIN, OUTPUT);
      digitalWrite(F_PIN,LOW);
      digitalWrite(B_PIN,LOW);

      
      

    }

    void set(float spd){
      this->SPD=spd;

      if(spd>=0){
        digitalWrite(FWD_PIN,0);
        digitalWrite(BKD_PIN,1);
      }
      if(spd<0){
        digitalWrite(FWD_PIN,1);
        digitalWrite(BKD_PIN,0);
      }
      analogWrite(SPD_PIN,abs(spd));
      
      }

    int getSpeed(){
        return SPD;
    }
};

  class kiwiDriveTrain{
    private:
      Motor_L928N* LMot;
      Motor_L928N* MMot;
      Motor_L928N* RMot;
      int LMot_Angle;
      int MMot_Angle;
      int RMot_Angle;
      float LMOT_SPEED=0;
      float MMOT_SPEED=0;
      float RMOT_SPEED=0;
      int MaxMotorSpeed=0;


      double botRot;
    
    public:

    kiwiDriveTrain(Motor_L928N *LeftMotor,Motor_L928N *MidMotor,Motor_L928N *RightMotor, double *heading,int maxSpeed){
        *LMot =*LeftMotor;
        *MMot =*MidMotor;
        *RMot =*RightMotor;
        botRot= *heading;
        MaxMotorSpeed=maxSpeed;
    }

    void setAnglesDegrees(int left, int mid, int right){
      //0 degrees=front, 90degrees=right,180 degrees=back, 270 degrees=left
      LMot_Angle=left*(M_PI/180);
      MMot_Angle=mid*(M_PI/180);
      RMot_Angle=right*(M_PI/180);
    };

    void setGyro(float heading){
      botRot=heading;
    };

    void drive(int FWD,int RGT,int ROT,double heading){
      this->setGyro(heading);
      FWD*=-1;               //make positive speed forwards    
      botRot =botRot*(M_PI/180);  //convert gyro degrees to radians

      LMOT_SPEED=(((FWD)*sin(LMot_Angle+botRot))+
            (RGT)*cos(LMot_Angle+botRot)+
            (ROT));
      MMOT_SPEED=(((FWD)*sin(MMot_Angle+botRot))+
            (RGT)*cos(MMot_Angle+botRot)+
            (ROT));
      RMOT_SPEED=(((FWD)*sin(RMot_Angle+botRot))+
            (RGT)*cos(RMot_Angle+botRot)+
            (ROT));

      if(max(max(abs(LMOT_SPEED),abs(MMOT_SPEED)),abs(RMOT_SPEED))>=MaxMotorSpeed){
        float ratio = 256/(max(max(abs(LMOT_SPEED),abs(MMOT_SPEED)),abs(RMOT_SPEED)));
        
        LMOT_SPEED*=ratio;
        MMOT_SPEED*=ratio;
        RMOT_SPEED*=ratio;
        }

      LMot->set(LMOT_SPEED);
      MMot->set(MMOT_SPEED);
      RMot->set(RMOT_SPEED);
    };

  };




#endif