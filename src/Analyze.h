#include <Arduino.h>
#define Accel_value 4 //launch_accel_value　二乗

class Analyze{
  private:
    float Accel_ave =0;
    uint8_t Accel_count =0 , Accel_num =0 ;
    int PressureAve =0 , PressureAve_old =0;
    uint8_t count_lps = 0, Pressure_count =0 , Pressure_num =0;
    
    // Pressure_count read data num   Pressure_num over num
  public:
    uint32_t start_time =0;
    uint32_t launchTime =0;
    uint8_t status = 0;  // 0: before launch  1: flight 2: landed
    bool launch_check_Accel(float Acceldata);
    bool launch_check_Pressure(int Pressuredata);
    // bool top_check_Pressure(int Pressuredata);
};


IRAM_ATTR bool Analyze::launch_check_Accel(float Acceldata){
  //in_task = 1;
  Accel_ave = Accel_ave + Acceldata;
  Accel_count++;
  
  if(Accel_count >= 20){
    Accel_count = 0;
    Accel_ave = Accel_ave / 20;
    if(Accel_ave > Accel_value){
      Accel_ave =0;
      Accel_num ++;
      if(Accel_num >=50){
        Accel_num =0;
        Accel_ave =0;
        launchTime = (micros() - 100000) - start_time;
        return true;
      }
    }
    else{
      Accel_ave =0;
      Accel_num =0;
    }
  }
  return false;
  //in_task = 0;
}

IRAM_ATTR bool Analyze::launch_check_Pressure(int Presssuredata){
  //in_task = 1;
  
  PressureAve = PressureAve + Presssuredata;
  Pressure_count++;
  if(Pressure_count >=5){
     Pressure_count =0;
     PressureAve = PressureAve / 5;
     if(PressureAve_old - PressureAve >= 10){
       Pressure_num ++;
       if(Pressure_num >= 5){
         Pressure_num =0;
         PressureAve =0;
         PressureAve_old =0;
         return true;
        }
      }
      else{
        Pressure_num = 0;
      }
      PressureAve_old = PressureAve;
      PressureAve =0;
    }
  return false;
  //in_task =0;
}