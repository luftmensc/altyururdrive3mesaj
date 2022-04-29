#include <Arduino.h>
#include "ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "string.h"
#include "HardwareSerial.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Joy.h"

#define ARRAY_LEN 4
#define MSG_LEN ARRAY_LEN*4 + 2

static String rxData[8];
static String rxCH4, rxNH4, rxCO2, rxP, rxTemperature, rxHumidity, rxUndTemp, rxUndHmd;
int rxChar;

int incoming_byte;
int arm_incoming_byte;

String incoming_str;
String arm_incoming_str;

bool receive_cnt_flag;

bool direct_drive_flag;

bool drive_live_flag;

unsigned long last_time;

unsigned long millis_value;

String ardu_Str;

std_msgs::String millisStrig;

float torque_mode_float;

HardwareSerial DriveSerial(PC5, PC4);
HardwareSerial ArmSerial(PB11, PB10);
HardwareSerial Serial3(PC11,PC10);

std_msgs::Float64MultiArray drive_published_feedback;
std_msgs::Float64MultiArray drive_joystick_array;

float joystick_float_array[ARRAY_LEN];

void multiArrayCallback(const std_msgs::Float64MultiArray &comingMultiArray);
void marrayCallback2(const std_msgs::Float64MultiArray &comingMultiArray);

void multiArrToArr(std_msgs::Float64MultiArray command_arr, float *commands_to_send);

int mapData(float x);
String getDirection(float coming_float);
String generateString(float x);
String generateMCUmessage(float *commands_to_send, int mode_torque);

void getThrustings(String encoderwDir);
void DriveFeedbackListener(void);
void armRead(void);

void ScienceListener(void);

float my_float[8];
float my_float2[11];
float bilim_float[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
static bool receive_flag;

std_msgs::Float64 my_msgg;
std_msgs::Float64 my_msggg;
std_msgs::String All_msg;
char sensor[100];

std_msgs::String my_str;
ros::Publisher str_topic("string_topic",&my_str);
ros::Publisher ALL ("sensor_All",&All_msg);

ros::NodeHandle nh;
ros::Publisher driveFeedbackPub("drive_feedback_topic", &drive_published_feedback);
ros::Subscriber <std_msgs::Float64MultiArray> joySub("multiarray_topic", &multiArrayCallback);
ros::Subscriber <std_msgs::Float64MultiArray> marray_sub2("multiarray_topic2", &marrayCallback2);
//ros::Subscriber <std_msgs::Float64MultiArray> joySub("multiarray_topic", &multiArrayCallback);
//ros::Publisher millispub("millis_topic", &millisStrig);

ros::Publisher debugpub ("debug_topic",&my_msgg);
ros::Publisher debugpub2 ("debug_topic2",&my_msggg);

int get_abs (float my_float);
float map_data(float x);
String direction_funct(float y);
String message_creator(float z);

void setup() {
  nh.initNode();
  nh.advertise(driveFeedbackPub);
  nh.subscribe(joySub);
  nh.subscribe(marray_sub2);
  //nh.advertise(millispub);
  nh.advertise(str_topic);
  
  DriveSerial.begin(9600);
  ArmSerial.begin(9600);
  Serial3.begin(115200);

  drive_published_feedback.data=(float *)malloc(sizeof(float)*ARRAY_LEN);  
  drive_published_feedback.data_length=ARRAY_LEN;

  drive_joystick_array.data=(float *)malloc(sizeof(float)*ARRAY_LEN);  
  drive_joystick_array.data_length=ARRAY_LEN;

  last_time = millis();
}

void loop() {
  
  nh.spinOnce();
  
  delay(1);

  static bool receive_flag=false;
  rxChar=Serial3.read();
  
  nh.spinOnce();

  unsigned long current_time = millis();
  if ((!nh.connected() && !direct_drive_flag) || ((current_time-last_time) > 500)) {
        DriveSerial.println("S00000000000000000F");
        delay(1);
  }
  
  DriveFeedbackListener();
  ScienceListener();
  armRead();

  nh.spinOnce();

  ardu_Str=String(current_time-last_time);  
  millisStrig.data=ardu_Str.c_str();

  nh.spinOnce();
}

void multiArrayCallback(const std_msgs::Float64MultiArray &comingMultiArray){
  last_time = millis();
  
  drive_joystick_array.data[0]=comingMultiArray.data[1]-comingMultiArray.data[0];
  drive_joystick_array.data[1]=comingMultiArray.data[1]-comingMultiArray.data[0];
  //drive_joystick_array.data[2]=comingMultiArray.data[1]+comingMultiArray.data[0];
  //drive_joystick_array.data[3]=comingMultiArray.data[1]+comingMultiArray.data[0];
  torque_mode_float = comingMultiArray.data[2];

  multiArrToArr(drive_joystick_array, joystick_float_array);
  DriveSerial.println(generateMCUmessage(joystick_float_array, int(torque_mode_float)));
  delay(1);

  for (int i =0; i <8; i ++) 
  {
    my_float[i] = comingMultiArray.data[i];
  }

  bilim_float[0]=my_float[4]; 
  bilim_float[1]=my_float[1];

  
  bilim_float[2]=(((-1*(my_float[2]))+1)/2);
  bilim_float[3]=(((-1*(my_float[5]))+1)/2);
  

  bilim_float[4]=my_float2[4];
  bilim_float[5]=my_float2[5];

  bilim_float[6]=my_float2[6];
  bilim_float[7]=my_float2[7];
  bilim_float[8]=my_float2[3];
  bilim_float[9]=my_float2[1];

  bilim_float[10]=my_float2[0];
  bilim_float[11]=my_float2[2];


  Serial3.println("S" + message_creator(bilim_float[0])+message_creator(bilim_float[1])+(int)bilim_float[2]+(int)bilim_float[3]+(int)bilim_float[4]+(int)bilim_float[5]+(int)bilim_float[6]+(int)bilim_float[7]+(int)bilim_float[8]+(int)bilim_float[9]+(int)bilim_float[10]+(int)bilim_float[11] + "F");
  String debug_str = "S" + message_creator(bilim_float[0])+message_creator(bilim_float[1])+(int)bilim_float[2]+(int)bilim_float[3]+(int)bilim_float[4]+(int)bilim_float[5]+(int)bilim_float[6]+(int)bilim_float[7]+(int)bilim_float[8]+(int)bilim_float[9]+(int)bilim_float[10]+(int)bilim_float[11] + "F";
  char debug_ch[50];
  debug_str.toCharArray(debug_ch,50);
  my_str.data = debug_ch;
  str_topic.publish(&my_str);
}

void marrayCallback2(const std_msgs::Float64MultiArray &comingMultiArray){
  for (int i =0; i <8; i ++) 
  {
    my_float2[i] = comingMultiArray.data[i];
  }
}

void multiArrToArr(std_msgs::Float64MultiArray command_arr, float *commands_to_send){
    for (int i = 0; i < ARRAY_LEN; i++){
        commands_to_send[i]=command_arr.data[i];
    }
}

int mapData(float coming_float){
  coming_float=255*coming_float;
  if(coming_float>255){
    coming_float=255;
  }
  if(coming_float<=255){
    coming_float=coming_float;
  }
  return coming_float;
}

String getDirection(float coming_float){
    String direction;
    if(coming_float<=0){
        direction='0';
    }
    if(coming_float>0){
        direction='1';
    }
    return direction;
}

String generateString(float x){
    String processedString = String(mapData(fabs(x)));
    while(processedString.length()<3){
        processedString = "0" + processedString;
    }
    return getDirection(x) + processedString;
}

String generateMCUmessage(float *commands_to_send, int mode_torque){
  String sentString = "S";
    for(int i=0;i<ARRAY_LEN;i++){
        sentString+=generateString(commands_to_send[i]);
    }
    sentString+=String(mode_torque);
    sentString+='F';
    return sentString;
}

void getThrustings(String encoderwDir){
  String str_buffer;
  char direction_char;
  int direction;
  for(int i=0;i<ARRAY_LEN;i++){
    direction_char=encoderwDir[4*i];
    if(direction_char=='0'){
      direction=-1;
    }
    if(direction_char=='1'){
      direction=1;
    }
    for(int j=i*4;j<(i*4)+3;j++){
      str_buffer+=encoderwDir[j+1];    
    }
    drive_published_feedback.data[i]=direction*str_buffer.toFloat()/255;
    str_buffer="";
    }
}

void DriveFeedbackListener(void){
  if (DriveSerial.available() > 0){
    incoming_byte = DriveSerial.read();
    if (incoming_byte == 'A'){
      incoming_str = "";
      receive_cnt_flag = true;
      return;
    }
    else if (receive_cnt_flag = true && incoming_byte != 'B'){
      incoming_str += (char) incoming_byte;
    }
    else if (incoming_byte == 'B'){
      getThrustings(incoming_str);
      driveFeedbackPub.publish(&drive_published_feedback);
      incoming_str = "";
      receive_cnt_flag = false;
    }   
  }
}

void armRead(void){
  static unsigned long arm_message_start_time=0;
  unsigned long arm_message_control_time=millis();

  if (ArmSerial.available() > 0){
    arm_incoming_byte = ArmSerial.read();
    
    if (arm_incoming_byte == 'S'){
      arm_message_start_time=millis();
      direct_drive_flag=true;
      arm_incoming_str = "";
      receive_cnt_flag = true;
      arm_incoming_str += (char) arm_incoming_byte;
      return;
    }
    else if (receive_cnt_flag = true && arm_incoming_byte != 'F'){
      arm_incoming_str += (char) arm_incoming_byte;
    }
    else if (arm_incoming_byte == 'F'){
      arm_incoming_str += (char) arm_incoming_byte;
      DriveSerial.println(arm_incoming_str);
      delay(1);
      arm_incoming_str = "";
      receive_cnt_flag = false;
    }   
  }
  if(fabs(arm_message_control_time-arm_message_start_time)>=500){
    direct_drive_flag=false;
  }
}

int get_abs(float my_float){
  int absed;
  if(my_float<=0) {
    absed = -my_float;
  }
  else if(my_float>0) { absed = my_float;}

  return absed;
}

float map_data(float x){
  return 255*x;
}

String direction_funct(float y){
  static int direction;
  if(y<=0){
    direction =0;
  }
  else if(y >0){
    direction =1;}
  return String(direction);
}

String message_creator ( float z){
  String processed_string = String ( get_abs ( map_data (z))) ;
  while(processed_string.length()<3){
  processed_string = "0" + processed_string;
  }
  return direction_funct (z) + processed_string;
}

void ScienceListener(void){

  if(Serial3.available()>0){
    if(rxChar=='A'){rxCH4=""; receive_flag=true;}
    if(receive_flag && rxChar!='A' && rxChar!='B'){rxCH4+=(char)rxChar;}
    if(rxChar=='B'){receive_flag=false; rxData[0] = rxCH4; rxCH4="";}
  
    if(rxChar=='C'){rxNH4="";receive_flag=true;}
    if(receive_flag && rxChar!='C' && rxChar!='D'){rxNH4+=(char)rxChar;}
    if(rxChar=='D'){receive_flag=false; rxData[1] = rxNH4; rxNH4="";}
  
    if(rxChar=='E'){rxCO2=""; receive_flag=true;}
    if(receive_flag && rxChar!='E' && rxChar!='F'){rxCO2+=(char)rxChar;}
    if(rxChar=='F'){receive_flag=false; rxData[2] = rxCO2; rxCO2="";}
    
    if(rxChar=='G'){rxP="";receive_flag=true;}
    if(receive_flag && rxChar!='G' && rxChar!='H'){rxP+=(char)rxChar;}
    if(rxChar=='H'){receive_flag=false; rxData[3] = rxP; rxP="";}

    if(rxChar=='I'){rxTemperature="";receive_flag=true;}
    if(receive_flag && rxChar!='I' && rxChar!='J'){rxTemperature+=(char)rxChar;}
    if(rxChar=='J'){receive_flag=false; rxData[4] = rxTemperature; rxTemperature="";}
    
    if(rxChar=='K'){rxHumidity=""; receive_flag=true;}
    if(receive_flag && rxChar!='K' && rxChar!='L'){rxHumidity+=(char)rxChar;}
    if(rxChar=='L'){receive_flag=false; rxData[5] = rxHumidity; rxHumidity="";}

    if(rxChar=='M'){rxUndTemp=""; receive_flag=true;}
    if(receive_flag && rxChar!='M' && rxChar!='N'){rxUndTemp+=(char)rxChar;}
    if(rxChar=='N'){receive_flag=false; rxData[6] = rxUndTemp; rxUndTemp="";}

    if(rxChar=='O'){rxUndHmd=""; receive_flag=true;}
    if(receive_flag && rxChar!='O' && rxChar!='P'){rxUndHmd+=(char)rxChar;}
    if(rxChar=='P'){receive_flag=false; rxData[7] = rxUndHmd; rxUndHmd="";}
  }
}
