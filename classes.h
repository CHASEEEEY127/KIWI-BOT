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



static bool toggleButton(bool button,bool *pressedRecent,bool output){
  if(button&&(*pressedRecent==false)){
    output=!output;
    *pressedRecent=true;
    }
    if(button==false){
      *pressedRecent=false;
    }
    return output;
}



static int pidLoop(double setpoint,double current,double KP,double KI,double KD, double *IO,bool loop360,double threshHold,int *ITimer,double maxI, double *PreviousError, double *outputError){          

          double error=(setpoint-current);
          if(loop360){
            while(error>180){
              error-=360;
            }
            while(error<-180){
              error+=360;
            }
          }
          *outputError=error;
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






 
/*
class omniWheel{
  public:
    Motor_L928N Motor;
    int Angle_degrees;
    float Angle_rads;
    float dia;
  
    omniWheel(const Motor_L928N& motor,int angle_degrees,float diameter):
      Motor(motor),Angle_degrees(angle_degrees),dia(diameter)
      {
        this->Angle_rads=this->Angle_degrees*(M_PI/180);
      }
    int set(int speed){
      Motor.set(speed);
    }
    

};
*/



#endif