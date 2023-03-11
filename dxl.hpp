#ifndef _DXL_HPP_
#define _DXL_HPP_

#include <fcntl.h>
#include <termios.h>
#include "dynamixel_sdk.h"              // Uses Dynamixel SDK library
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>

// Control table address for AX-12W and MX-12W
#define ADDR_MX_TORQUE_ENABLE           24                  
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32

// Control table address for XL and XC model
#define ADDR_XL_TORQUE_ENABLE           64                 
#define ADDR_XL_OPERATING_MODE          11                 
#define ADDR_XL_GOAL_POSITION           116
#define ADDR_XL_PRESENT_POSITION        132
#define ADDR_XL_GOAL_VELOCITY           104
#define ADDR_XL_PRESENT_VELOCITY	    128

// Data Byte Length for AX-12W and MX-12W
#define LEN_MX_GOAL_POSITION		    2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2

// Data Byte Length for XL and XC model
#define LEN_XL_GOAL_POSITION		    4
#define LEN_XL_PRESENT_POSITION        	4
#define LEN_XL_GOAL_VELOCITY           	4
#define LEN_XL_PRESENT_VELOCITY        	4

// Protocol version for AX-12W and MX-12W
#define PROTOCOL_VERSION                1.0                 
// Protocol version for XL and XC model
//#define PROTOCOL_VERSION                2.0                 

// Default setting
#define DXL1_ID                         1		        // Dynamixel#1 ID: 1
#define DXL2_ID                         2               // Dynamixel#2 ID: 2
#define BAUDRATE                        2000000         // for AX-12W and MX-12W model
//#define BAUDRATE                        4000000       // for XL and XC model
#define DEVICENAME                      "/dev/ttyUSB0"  // Check which port is being used on your controller
                                                        // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1               // Value for enabling the torque
#define TORQUE_DISABLE                  0               // Value for disabling the torque
#define OPMODE_XL_VELOCITY              1               
#define OPMODE_XL_POSITION              3               
#define OPMODE_XL_PWM	                16              
#define ESC_ASCII_VALUE                 0x1b

class Dxl
{

private:
    int port_num;
    int group_num;
    int dxl_comm_result;             // Communication result
    uint8_t dxl_addparam_result;     // AddParam result
    uint8_t dxl_error;               // Dynamixel error
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;

public:
    Dxl(void);
    bool dxl_open(void);
    void dxl_close(void);
    bool dxl_set_velocity(int goal_velocity1, int goal_velocity2);
    void dxl_xl_open(void);
    void dxl_xl_close(void);
    //int dxl_xl_set_velocity(int goal_velocity1, int goal_velocity2);
    unsigned int vel_convert(int speed);
    static int getch(void);
    static int kbhit(void);
};

#endif //_DXL_HPP_
