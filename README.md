# ioBoard
Use arduino board as digital I/O board. Subscribe output state, Publish input state.
## environment(test)
* Ubuntu 16.04
* ros1 kinetic
* rosserial
* Arduino Uno board

# specification
* 13 bit digital output
* 5 bit digital input

## message type

| pub/sub   | type             | node name        | value name      | data                   | default |
| --------- | ---------------- | ---------------- | --------------- | ---------------------- | ------- |
| publish   | std_msgs::UInt8  | /read_pin        | msg_readPin     | 5 bit  0b00000         | 0       |
| subscribe | std_msgs::UInt16 | /write_pin       | msg_writePin    | 13 bit 0b0000000000000 | 0       |
| subscribe | std_msgs::Bool   | /io_sendLog_flag | msg_sendLogFlag | bool                   | false   |

## pin mode and data
| pin mode      | HIGH | LOW |            |
| ------------- | ---- | --- | ---------- |
| OUTPUT        | 1    | 0   |            |
| INPUT         | 1    | 0   |            |
| INPUT(PULLUP) | 0    | 1   | <- default |

if use `INPUT` : comment out this statement.
```cpp
#define USE_INPUTPULLUP
↓
//#define USE_INPUTPULLUP
```

## pin mapping
| pin    | setting           | use                      |
| ------ | ----------------- | -----------------------: |
| D0     | HardwareSerial Rx | Rx                       |
| D1     | HardwareSerial Tx | Tx                       |
| D2     | digitalWrite      | msg_writePin lower bit 0 |
| D3     | digitalWrite      | 〃 bit 1                  |
| D4     | digitalWrite      | 〃 bit 2                  |
| D5     | digitalWrite      | 〃 bit 3                  |
| D6     | digitalWrite      | 〃 bit 4                  |
| D7     | digitalWrite      | 〃 bit 5                  |
| D8     | digitalWrite      | 〃 bit 6                  |
| D9     | digitalWrite      | 〃 bit 7                  |
| D10    | digitalWrite      | 〃 bit 8                  |
| D11    | digitalWrite      | 〃 bit 9                  |
| D12    | digitalWrite      | 〃 bit 10                 |
| D13    | digitalWrite      | 〃 bit 11                 |
| D14 A0 | digitalWrite      | 〃 bit 12                 |
| D15 A1 | digitalRead       | msg_readPin lower bit 0  |
| D16 A2 | digitalRead       | 〃 bit 1                  |
| D17 A3 | digitalRead       | 〃 bit 2                  |
| D18 A4 | digitalRead       | 〃 bit 3                  |
| D19 A5 | digitalRead       | 〃 bit 4                  |

# How to use

In Terminal,
```sh
$ roscore
```
Another terminal window, 
```sh
# rosrun rosserial_python serial_node.py [port name]
# example
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Another terminal window, show ros topic list
```sh
$ rostopic list
# /io_sendLog_flag <- 
# /read_pin        <- 
# /write_pin       <-
```
set digital output pin state
```sh
# 0b 0000000000000
$ rostopic pub -1 /write_pin std_msgs/UInt16 0
# 0b 0000000000001
$ rostopic pub -1 /write_pin std_msgs/UInt16 1
# ...
# 0b 1111111111111
$ rostopic pub -1 /write_pin std_msgs/UInt16 8191
```
echo digital input pin state
```sh
$ rostopic echo /read_pin
```
send ROS info
```sh
$ rostopic pub -1 /io_sendLog_flag std_msgs/Bool "data: true"
```
how to read info
```sh
#         time stamp         |RxTx| write bit0,1,2,3,...     | read bit 0,1,...
# [INFO] [1573301749.656628]: R T |1 1 1 0 0 0 0 0 0 0 0 0 0 |0 0 0 0 0

# ...
# [INFO] [1573301749.656628]: RT111000000000000000
# [INFO] [1573301749.677120]: RT111000000000000000
# ...
```
not send ROS info
```sh
$ rostopic pub -1 /io_sendLog_flag std_msgs/Bool "data: false"
```
