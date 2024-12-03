#ifndef __motor_H__
#define __motor_H__



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



#endif