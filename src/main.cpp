#include <Arduino.h>
#include <SPICREATE.h>
#include <LPS25HB.h>
#include <ICM20948.h>
#include <S25FL512S.h>
#include <Analyze.h>
#include <Serial_COM.h>

#define SCLK  18
#define MISO1 19
#define MOSI1 23
#define RX 16
#define TX 17

#define LEDPIN 13
#define redPin 26
#define greenPin 25
#define bluePin 27
#define loggingPeriod 2
#define LPSCS 33
#define ICMCS 32
#define FLASHCS 21
#define SPIFREQ 100000
#define Max_addr 0xff //?

#define freq 5000
#define redChannel 0
#define greenChannel 1
#define blueChannel 2
#define duty 8
#define Falling_Time 12000000 //TIME (launch → landing)
#define led_Time 600000000 //TIME (lended → led_on)
#define led_ON_Time 900000000

LPS lps25;
ICM icm;
Flash flash;
SPICREATE::SPICreate SPIC1;
Analyze analyze;
SerialCom serialCom;

int tickflag =0;
int launched_icm =0,launched_lps =0;
uint32_t flight_Timebuff = 0;
uint32_t landed_start_time =0,landed_Timebuff = 0;
uint8_t in_task = 0;
uint8_t mode = 0;
uint8_t ICM_WhoAmI = 0, LPS_WhoAmI =0;


//For logging
namespace Log{
  uint32_t log_start_time =0;
  uint32_t Timestamp =0;
  int log_counter = 0;
  int lps_counter =0; //lps 25hz
  uint8_t flash_buf[256];
  uint8_t flash_rx[256];
  uint32_t flash_addr = 0x000;
  bool logging_flag = true;
}


/// For launch ///

//LED //
class LED{
  private:
    int led_counter = 0;// mode =0
    int bright =0;
  public:
    int led_on = 0;
    uint8_t R_level =0, G_level =0, B_level =0;
    void L_tika(); //mode =0
    void LED_pwm();//mode =4
};
LED led;

IRAM_ATTR void LED::L_tika(){//mode =0
  led_counter++;

  if(led_counter % 1000 == 500){
    // ledcWrite(redChannel,255);    
    // ledcWrite(blueChannel,255);
    // ledcWrite(greenChannel,255);
    digitalWrite(LEDPIN,HIGH);

  }
  if(led_counter % 1000 == 0){
    // ledcWrite(redChannel,0);    
    // ledcWrite(blueChannel,0);
    // ledcWrite(greenChannel,0);
    digitalWrite(LEDPIN,LOW);
  }
}

IRAM_ATTR void LED::LED_pwm(){
  
  for(int i=0; i<=255; i++){
    ledcWrite(redChannel,i);
    ledcWrite(greenChannel,255-i);
    ledcWrite(blueChannel,0);
    delay(10);
    }
  for(int i=0; i<=255; i++){
    ledcWrite(redChannel,255-i);
    ledcWrite(greenChannel,0);
    ledcWrite(blueChannel,i);
    delay(10);
    }
  for(int i=0; i<=255; i++){
    ledcWrite(redChannel,0);
    ledcWrite(greenChannel,i);
    ledcWrite(blueChannel,255-i);
    delay(10);
    }
}


IRAM_ATTR bool Check_sensor(){
  ICM_WhoAmI = (int)(icm.WhoAmI());
  
}

IRAM_ATTR void Reset(){
  
}

uint32_t read_addr = 0x000;
IRAM_ATTR void Read_Flash(){
  
  if(read_addr <= Log::flash_addr){
    flash.read(read_addr,Log::flash_rx);
    for(int i = 0;i < 8; i++){
        for(int j = 0;j < 32;j++){
          Serial2.print(Log::flash_rx[32*i + j]);Serial2.print("\t");
        }
        Serial2.println();
    }
    Serial2.println(read_addr);
    Serial2.println();
    read_addr += 0x100;
  }else{
    Serial2.print("all read");
    return;
  }
}

IRAM_ATTR void Logging(){
  
  uint8_t icm_data_buf[12];
  int16_t icm_data[6];
  
  //TIME//
  Log::Timestamp = micros();
  Log::Timestamp = Log::Timestamp - Log::log_start_time;
  for(uint8_t i =0; i< 4; i++){
    Log::flash_buf[32 * Log::log_counter + i] = 0xFF & (Log::Timestamp >> (8*i))%256;
  }

  //ICM
  icm.Get(icm_data);
  for(uint8_t i =0; i<6; i++){
    icm_data_buf[ i*2 + 1] = icm_data[i] %256;
    icm_data_buf[i*2] = icm_data[i] >>8;
  }
  for(uint8_t i =0; i<12; i++){
    Log::flash_buf[32*Log::log_counter + 4 + i] = icm_data_buf[i]; 
  }
  if(launched_icm ==0){// launch Check
      if(analyze.launch_check_Accel(icm.Acceldata)){
        Serial.println("Launched");
        launched_icm =1;
      }  
    }

  //LPS 25hz
  if(Log::lps_counter % 40 == 0){
    uint8_t lps_data[3];
    lps25.Get(lps_data);
    Serial.print(Log::lps_counter);
    //Serial2.println("lps");
    for(uint8_t i=0; i<3; i++){//data set
      Log::flash_buf[32*Log::log_counter + 16 + i] = lps_data[i];
    }
    if(launched_lps ==0){//launch_Check
      if(analyze.launch_check_Pressure(lps25.Plessure)){
        Serial2.println("launched");
        launched_lps = 1;
      }
    }
  }

  uint8_t LED_data[3];
  for(uint8_t i=0; i <3; i++ ){
    //LED_data[i] = 
  }

  Log::log_counter ++;
  Log::lps_counter ++;
  if(Log::log_counter >= 8){
    flash.write(Log::flash_addr,Log::flash_buf);
    Log::flash_addr += 0x100;
    Log::log_counter =0;
  }

  if(Log::flash_addr == Max_addr){//flash_max_addr
    Log::logging_flag = false;
  }

}


IRAM_ATTR void Sleep_mode(){
  serialCom.sendSerial2();
  char cmd;
  if(Serial2.available()){
    cmd = Serial2.read();
    switch (cmd)
    {
    case 'j':
      Serial.println("return");
      serialCom.stopCommand();
      break;
    case 'y':
      led.led_on = 1;
      serialCom.setCommand('y');
      break;
    case 'n':
      led.led_on = 0;
      serialCom.setCommand('n');
      break;\
    case 'p':
      Reset();
      serialCom.setCommand('p');
      mode = 1;
      break;
    case 'd':　//　
      flash.erase();
      delay(100);
      read_addr = 0x000;
      serialCom.setCommand('d');
    case 'r': // 
      serialCom.setCommand('r');
      Read_Flash();
      break;
    default:
      //Serial2.println("CMD different");
      break;
    }
  }
  if(led.led_on == 1){
    //led.L_tika();
    led.LED_pwm();
  }else{
    ledcWrite(redChannel,0);
    ledcWrite(blueChannel,0);
    ledcWrite(greenChannel,0);
    digitalWrite(LEDPIN,LOW);
  }

}

IRAM_ATTR void Wait_mode(){
  serialCom.sendSerial2();
  char cmd;
  if(Serial2.available()){
    cmd = Serial2.read();
    switch (cmd)
    {
    case 'j':
      Serial.println("return");
      serialCom.stopCommand();
      break;
    case 'l':
      Log::log_start_time = micros();
      serialCom.setCommand('l');
      mode = 2;
      break;
    case 'd':
      serialCom.setCommand('d');
      mode = 3;
      break;
    case 's':
      serialCom.setCommand('s');
      mode =0;
    default:
      break;
    }
  }
}

IRAM_ATTR void flight_mode(){
  if(Log::logging_flag){
    Logging();
  }
  
  if(analyze.status == 0){ // before launch
    Serial.print(icm.Acceldata);
    Serial.print(",/t");
    if(launched_icm == 1 || launched_lps == 1){
      if(launched_icm == 1){
        analyze.start_time = micros() - 1000000;
        Serial.println("launched_ICM");
      }
      else{
        analyze.start_time = micros() - 2000000;
        Serial.println("launched_LPS");
      }
      analyze.status = 1;
    }
  }
  else if(analyze.status == 1){// flight
    flight_Timebuff = micros();
    flight_Timebuff -= analyze.start_time;
    Serial.println(flight_Timebuff);
    led.L_tika();
    if(flight_Timebuff >= Falling_Time){//if landed
      analyze.status = 2;
      landed_start_time =micros();
      mode = 4; //
    }
  }
  serialCom.sendSerial2();
  char cmd;
  if(Serial2.available()){
    cmd = Serial2.read();
    switch (cmd)
    {
    case 'j':
      Serial.println("return");
      serialCom.stopCommand();
      break;
    case 's':
      serialCom.setCommand('s');
      mode = 0;
    default:
      break;
    }
  }
 
}

IRAM_ATTR void landed_mode(){
  if(Log::logging_flag){
    Logging();
  }
  landed_Timebuff = micros();
  landed_Timebuff -= landed_start_time;
  if(led_Time <= landed_Timebuff <= led_ON_Time){
    led.LED_pwm();
  }
  serialCom.sendSerial2();
  char cmd;
  if(Serial2.available()){
    cmd = Serial2.read();
    switch (cmd)
    {
    case 'j':
      Serial.println("return");
      serialCom.stopCommand();
      break;
    case 's':
      serialCom.setCommand('s');
      mode = 0;
    default:
      break;
    }
  }
  
}



IRAM_ATTR void MainWork(){
  
  switch (mode)
  {
  case 0:
    Sleep_mode();  //LED_practice & 
    break;
  case 1:
    Wait_mode(); //before launch (Logging) 
    break;
  case 2:
    flight_mode(); // launch →　landing
    break;
  case 3://delete_mode
    flash.erase();
    mode = 0;
  case 4:
    landed_mode();
    break;
    default:
      break;
  }
}

IRAM_ATTR void tick(){
  tickflag = 1;
}

hw_timer_t * timer = NULL;


void setup() {
  delay(100);
  Serial.begin(115200);
  delay(100);
  Serial2.begin(9600);
  while (!Serial2);
  delay(100);
  Serial.println("test");  
  SPIC1.begin(VSPI,SCLK,MISO1,MOSI1);
  delay(100);
  
  pinMode(ICMCS,OUTPUT);
  //digitalWrite(ICMCS,HIGH);
  pinMode(LPSCS,OUTPUT);
  //digitalWrite(LPSCS,HIGH);
  pinMode(25,OUTPUT);
  pinMode(LEDPIN,OUTPUT);
  delay(1000);
  ledcSetup(redChannel,freq,duty);
  ledcAttachPin(redPin,redChannel);
  pinMode(26,OUTPUT);
  ledcSetup(greenChannel,freq,duty);
  ledcAttachPin(greenPin,greenChannel);
  pinMode(27,OUTPUT);
  ledcSetup(blueChannel,freq,duty);
  ledcAttachPin(bluePin,blueChannel);
  delay(1000);
  Serial.print("begin start");

  lps25.begin(&SPIC1,LPSCS,1000000);
  delay(100);
  Serial.println("lps.begin");
  icm.begin(&SPIC1,ICMCS,1200000);
  delay(100);
  Serial.println("icm.begin");
  flash.begin(&SPIC1,FLASHCS,1000000);
  delay(100);
  Serial.println("flash.begin");
  
  setCpuFrequencyMhz(240);
  disableCore0WDT();
  
  timer = timerBegin(0,80,true);
  timerAttachInterrupt(timer, &tick, true);
  delay(1000);
  Serial.println("timer set");
  timerAlarmWrite(timer, 1000, true); 
  timerAlarmEnable(timer);
  delay(3000);
  serialCom.setup();
  serialCom.setCommand('w'); // wake up
  Serial.println("finish begin");

}

void loop(){ 

  if(tickflag >0){
    serialCom.sendSerial2();
    MainWork();

    tickflag = 0;
  }
  
  
}
  