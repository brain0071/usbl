#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "seatrac_helpers.h"
#include "common/seatrac.h"
#include <serial/serial.h>


class usbl_node
{

private:
    int _beacon_ID;
    // const char *_port_name = "/dev/ttyUSB0";
    // const int _baudrate = 115200;
    std::string _usbl_port; 
    uint32_t _baud_rate;

    serial::Serial* _ser;

    // usbl protocal status machine
    SEATRAC_PROTOCOL_STATUS_E _seatrac_protocol_state;

    // position
    float _north;
    float _east;
    float _depth;

    // port status
    bool _port_status;

    // lost account
    int _lost_account;

    // if port closed, retry open
    bool retry_open();

    // get local and remote beacon response and status
    int wait_response(uint8_t cid_waited);

public:
    usbl_node(int beacon_ID);
    ~usbl_node();
    // save setting
    bool saveSetting(std::string usbl_port,uint32_t baudrate);
    // open port
    bool open_port();
    // get current remote beacon position
    float *get_pos();
    // close port
    void close_port();
    // send ping protocol and get response
    void run();
};
