/* rosserial

---
$ roscore
---
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
---
$ rostopic list
---
$ rostopic pub -1 /write_pin std_msgs/UInt16 0
$ rostopic pub -1 /write_pin std_msgs/UInt16 1
...
$ rostopic pub -1 /write_pin std_msgs/UInt16 8191

$ rostopic echo /read_pin

$ rostopic pub -1 /io_sendLog_flag std_msgs/Bool "data: true"
$ rostopic pub -1 /io_sendLog_flag std_msgs/Bool "data: false"

---

Arduino UNO

 0    HardwareSerial RX
 1    HardwareSerial TX
 2    digitalWrite
 3    digitalWrite
 4    digitalWrite
 5    digitalWrite
 6    digitalWrite
 7    digitalWrite
 8    digitalWrite
 9    digitalWrite
10    digitalWrite
11    digitalWrite
12    digitalWrite
13    digitalWrite
14 A0 digitalWrite
15 A1 digitalRead
16 A2 digitalRead
17 A3 digitalRead
18 A4 digitalRead
19 A5 digitalRead

pin mapping
-> #define, output_pin[], input_pin[]

*/

// -------------------------------------------------
#include <ros.h>
#include <std_msgs/UInt8.h>   // publish    5 pin input
#include <std_msgs/UInt16.h>  // subscribe 13 pin output
#include <std_msgs/Bool.h>    // subscribe sendLog flag

ros::NodeHandle nh;
std_msgs::UInt16 msg_writePin;
std_msgs::UInt8  msg_readPin;
std_msgs::Bool   msg_sendLogFlag;

// settings -----------------------------------------
#define USE_INPUTPULLUP
//  use pinMode( pin, INPUT_PULLUP )
//  if you use pinMode( pin, INPUT ), comment out 

#define BOARD_PINS_NUM        20  // 0~19
#define OUTPUT_PIN_NUM        13
#define OUTPUT_PIN_USE_BITMASK (0b0001111111111111) // use pin:1
#define INPUT_PIN_NUM          5
#define INPUT_PIN_USE_BITMASK  (0b00011111)

// pin mapping
const int output_pin[ OUTPUT_PIN_NUM ] = 
{
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14
};
const int input_pin[ INPUT_PIN_NUM ] = 
{
  15,
  16,
  17,
  18,
  19
};

char pin_state_log[ BOARD_PINS_NUM +1 ] = // +\0
{
  "RT000000000000000000"
  // rx, tx,  0, 0, 0,...
  // 0123456...
};
char charBuf[100];

// prototype declaration
void digitalOut(int pin, int state);
int digitalIn(int pin);
void messageCbWritePin( const std_msgs::UInt16 &msg);
void messageCbSendLog( const std_msgs::Bool &msg);


#define PUB_READPIN_INTERVAL  20 // [ms]  1000[ms] / 50[Hz]
#define INFO_SENDLOG_INTERVAL 20 // [ms]  1000[ms] / 50[Hz]
unsigned long timeMs = 0;
unsigned long timeMsBuf[2] = {0};

ros::Subscriber<std_msgs::UInt16> sub_writePin("write_pin", &messageCbWritePin);
ros::Publisher                    pub_readPin ("read_pin",  &msg_readPin);
ros::Subscriber<std_msgs::Bool>   sub_sendLog ("io_sendLog_flag", &messageCbSendLog);

void setup()
{
  // pin init
  for(int i = 0; i< OUTPUT_PIN_NUM; i++)
  {
    pinMode(output_pin[ i ], OUTPUT);
    digitalOut(output_pin[ i ], LOW);
  }
  
  for(int i = 0; i< INPUT_PIN_NUM; i++)
  {
#ifdef USE_INPUTPULLUP
    pinMode(input_pin[ i ], INPUT_PULLUP);
#else
    pinMode(input_pin[ i ], INPUT);
#endif
    digitalIn(input_pin[ i ]);
  }
  
  msg_writePin.data = 0;
  msg_readPin.data  = 0;
  msg_sendLogFlag.data = false;
  
  nh.initNode();
  nh.advertise(pub_readPin);
  nh.subscribe(sub_writePin);
  nh.subscribe(sub_sendLog);
}

void loop()
{
  timeMs = millis();

  // pin read ----------------------------------------------
  if( timeMs - timeMsBuf[0] >= PUB_READPIN_INTERVAL )
  {
    msg_readPin.data = readPinState() & INPUT_PIN_USE_BITMASK;
    pub_readPin.publish(&msg_readPin);
    
    timeMsBuf[0] = timeMs;
  }
  // -------------------------------------------------------
  
  nh.spinOnce();

  // send log ----------------------------------------------
  if( msg_sendLogFlag.data )
  {
    if( timeMs - timeMsBuf[1] >= INFO_SENDLOG_INTERVAL )
    {
      nh.loginfo( pin_state_log );
      
      timeMsBuf[1] = timeMs;
    }
  }
  // -------------------------------------------------------
}

void messageCbWritePin( const std_msgs::UInt16 &msg)
{
  // update writepin state
  msg_writePin = msg;
  msg_writePin.data = msg.data & OUTPUT_PIN_USE_BITMASK;
  
  int state = 0;
  for(int i = 0; i< OUTPUT_PIN_NUM; i++)
  {
    state = (msg_writePin.data >> i) & 1;
    
    if(state == 1)
    {
      digitalOut(output_pin[ i ], HIGH );
    }else{
      digitalOut(output_pin[ i ],  LOW );
    }
  }
}
//    ex. data = 6 = 0b0110  ↓
//      i = 0, 6 >> 0 = 0b011|0|, data & 1 = 0
//      i = 1, 6 >> 1 = 0b001|1|, data & 1 = 1
//      i = 2, 6 >> 2 = 0b000|1|, data & 1 = 1
//      i = 3, 6 >> 3 = 0b000|0|, data & 1 = 0
//      ...                   ^

void messageCbSendLog( const std_msgs::Bool &msg)
{
  msg_sendLogFlag = msg;
}

// write and memory
void digitalOut(int pin, int state)
{
  digitalWrite(pin, state);
  
  if(state == HIGH)
  {
    pin_state_log[ pin ] = '1';
  }else{
    pin_state_log[ pin ] = '0';
  }
}

// read and memory
int digitalIn(int pin)
{
  int state = digitalRead(pin);

#ifdef USE_INPUTPULLUP

  if( state == LOW )
  {
    pin_state_log[ pin ] = '1';
  }else{
    pin_state_log[ pin ] = '0';
  }
  return 1 - state;

#else

  if( state == HIGH )
  {
    pin_state_log[ pin ] = '1';
  }else{
    pin_state_log[ pin ] = '0';
  }
  return state;

#endif
}

uint8_t readPinState()
{
  uint8_t readBuf = 0;  
  for(int i = 0; i < INPUT_PIN_NUM; i++)
  {
    readBuf += digitalIn(input_pin[ i ]) << i;
  }
  return readBuf;
}
