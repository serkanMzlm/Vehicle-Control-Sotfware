#ifndef __REMOTE_SELECTOR_TYPE_HPP__
#define __REMOTE_SELECTOR_TYPE_HPP__

#include <string>
#include <cstdint>

#define HEADER_     27
#define FOOTHER_    71
#define HEADER_LEN  1
#define FOOTHER_LEN 1
#define PAYLOAD_LEN 4

#define KEYBOARD_W 87 
#define KEYBOARD_A 65
#define KEYBOARD_S 83
#define KEYBOARD_D 68
#define KEYBOARD_X 88

#define BAUDRATE B9600

typedef enum{

}Serial_e;

// typedef enum {
//     S_HEADER, S_JOY_X, S_JOY_Y, S_BUTTON1, 
//     S_BUTTON2, S_FOOTHER, S_ALL
// } Serial_e;

typedef struct{
    std::string file_name;
    int port;
} device_t;

typedef struct{
    int button[2];
    float joy[2];
    int state = 0;
    uint8_t curr_byte;
    uint8_t prev_byte;
    // uint8_t buffer[S_ALL];
} rx_t;

#endif