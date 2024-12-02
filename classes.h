#ifndef __classes_H__
#define __classes_H__

 class Motor_L928N{

  public:
    int FWD_PIN;
    int BKD_PIN;
    int SPD_PIN;
    int SPD;

    Motor_L928N(int forward_pin, int backward_pin, int speed_pin){
      FWD_PIN = forward_pin;
      BKD_PIN = backward_pin;
      SPD_PIN = speed_pin;
      SPD=0;
      pinMode(FWD_PIN, OUTPUT);
	    pinMode(BKD_PIN, OUTPUT);
	    pinMode(SPD_PIN, OUTPUT);
      digitalWrite(FWD_PIN,LOW);
      digitalWrite(BKD_PIN,LOW);


    }

    void set(int spd){
      this->SPD=spd;

      if(spd>=0){
        digitalWrite(FWD_PIN,0);
        digitalWrite(BKD_PIN,1);
      }
      if(spd<0){
        digitalWrite(FWD_PIN,1);
        digitalWrite(BKD_PIN,0);
      }
      //analogWrite(this->SPD_PIN,abs(spd));
      analogWrite(SPD_PIN,abs(spd));
      
      }

    int getSpeed(){
        return SPD;
    }
};
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