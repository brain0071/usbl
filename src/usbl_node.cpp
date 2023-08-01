
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "seatrac_helpers.h"
#include "common/seatrac.h"
#include <serial/serial.h>
#include "usbl_node.h"
#include "geometry_msgs/Vector3Stamped.h"
// #include
#include <iostream>

#include <bitset> 
#include <vector>

// using namespace usbl;

usbl_node::usbl_node(int beacon)
    : _beacon_ID(beacon),
      _north(0),
      _east(0),
      _depth(0),
      _port_status(false),
      _lost_account(0),
      _seatrac_protocol_state(SEATRAC_PROTOCOL_STATUS_IDLE)
//   _ser(nullptr)
{
    _ser = new serial::Serial();
    // initial lost account\position\port_status
}

usbl_node::~usbl_node()
{
    // close port
    _ser->close();
    _ser->~Serial();
    _port_status = false;
}

// save usbl port setting
bool usbl_node::saveSetting(std::string usbl_port, uint32_t baud_rate)
{
    _usbl_port = usbl_port;
    _baud_rate = baud_rate;

    // 8 Data Bits
    // No Parity
    // 1 Start Bit
    // No handshaking / flow-control
    _ser->setPort(_usbl_port);
    _ser->setBaudrate(_baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(2);
    _ser->setTimeout(to);
    // 2 Stop Bits
    // _ser->setBytesize(8);
    _ser->setStopbits(serial::stopbits_t::stopbits_two);
    // _ser->setFlowcontrol();
    return true;
}

//
bool usbl_node::open_port()
{
    ROS_INFO("try open port....port name: %s, baudrate: %d", _usbl_port.c_str(), _baud_rate);
    if (_ser->isOpen())
    {
        ROS_INFO("Usbl port have opened, close and re-opening.");
        close_port();
    }

    try
    {
        _ser->open();
        _port_status = true;
        ROS_INFO("Usbl port open successful!");
        return true;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        // port open failed
        _port_status = false;
    }
    return false;
}

// closed port
void usbl_node::close_port()
{
    try
    {
        _ser->close();
        _port_status = false;
        ROS_INFO("Usbl port closed.");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Usbl port close failed!");
    }
}

// retry open port
bool usbl_node::retry_open()
{
    if (_ser->isOpen())
    {
        // ROS_INFO("Usbl port re-open successful!");
        // return true;
        _ser->close();
    }

    // re-open five times
    for (int i = 1; i < 5; i++)
    {
        try
        {
            _ser->open();
            _port_status = true;
            ROS_INFO("Usbl port re-open successful!");
            return true;
        }
        catch (serial::IOException &e)
        {
            // port open failed
            _port_status = false;
            // ROS_ERROR_STREAM("Unable to open port ");
        }
    }
    if (!_ser->isOpen())
    {
        ROS_ERROR_STREAM("Re-open the port has failed five times");
        return false;
    }
}

float *usbl_node::get_pos()
{
    float pos[3] = {_north, _east, _depth};
    return pos;
}

//
int usbl_node::wait_response(uint8_t cid_waited)
{

    int ret = -1;
    if (_ser->isOpen())
    {
        u_int byteCount;

        _ser->waitByteTimes(5000);

        byteCount = _ser->available();
        // printf("byteCount:%d\n", byteCount);
        if (byteCount > 0)
        {

            uint _byteCount = byteCount;
            unsigned char read_buffer[_byteCount];
            // char read_buffer[byteCount];
            size_t _bytecount;
            _bytecount = _ser->read(read_buffer, _byteCount);
            

            seatrac_message_t msg;
            seatrac_status_t status;


            for (int pos = 0; pos < _bytecount; ++pos)
            {
                // printf("pos:%d\n", pos);

                if (seatrac_parse_char(static_cast<uint8_t>(read_buffer[pos]), &msg, &status))
                {
                    // printf("command id %d", msg.cid);
                    switch (msg.cid)
                    {
                    case CID_PING_SEND:
                    {
                        msg_response_status_t resMsg;
                        seatrac_PING_SEND_response_message_decode(&msg, &resMsg);

                        if (cid_waited == CID_PING_SEND)
                        {
                            // get local beacon response
                            if (resMsg.status == CST_OK)
                            {
                                printf("Local beacon response successful.\n");
                                ret = 1;
                            }
                            else
                            {
                                // local beacon no response
                                // print resMsg status
                                printf("Usbl local beacon no response.\n");
                                ret = 0;
                                _lost_account++;
                            }
                        }
                        break;
                    }

                    case CID_PING_RESP:
                    {
                        msg_ping_resp_t resMsg;
                        seatrac_PING_RESP_message_decode(&msg, &resMsg);
                        _lost_account = 0;

                        // remote beacon response error
                        if ((resMsg.acofix.flags & ACOFIX_FLAGS_POSITION_FLT_ERROR))
                        {
                            // ROS_ERROR_STREAM("Usbl position response error.");
                            _lost_account++;
                        }
                        else if (resMsg.acofix.flags & ACOFIX_FLAGS_HAVE_POSITION_FIELDS)
                        {
                            // update position related local beacon in NED
                            _east = resMsg.acofix.position_easting / 10.0;
                            _north = resMsg.acofix.position_northing / 10.0;
                            _depth = resMsg.acofix.position_depth / 10.0;
                            printf("Usbl position update...north: %f, east: %f, depth: %f",
                                   _north, _east, _depth);
                        }
                        else
                        {
                            printf("Usbl response unknown error.\n");
                            _lost_account++;
                        }

                        if (cid_waited == CID_PING_RESP)
                        {
                            ret = 1;
                        }
                        break;
                    }

                    case CID_PING_ERROR:
                    {
                        printf("Usbl PING ERROR.\n");
                        ret = 0;
                        _lost_account++;
                        break;
                    }
                    }
                }
                // else
                // {
                    // printf("no parse results.\n");
                // }
            }

            return ret;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Port closed, retrying...");
        retry_open();
    }
}

// run usbl send ping and get position
void usbl_node::run()
{
    // wait local beacon response
    int _time_count_local_resp = 0;
    // wait remote beacon response
    int _time_count_remote_resp = 0;
    // response result
    int flag = 0;

    switch (_seatrac_protocol_state)
    {
    case SEATRAC_PROTOCOL_STATUS_IDLE:
    {

        uint8_t dest_id = _beacon_ID;
        uint8_t msg_type = MSG_REQX;

        seatrac_message_t msg;

        seatrac_PING_SEND_pack(&msg, dest_id, msg_type);
        seatrac_finalize_message(&msg);

        // write data to local beacon
        char *buf;
        memcpy(buf, msg.payload, msg.len);
 
        if (_ser->write(buf))
        {
 

            _ser->flush();
            _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_WAITING_FOR_LOCAL_RESP;
            _time_count_local_resp = 0;
        }
        else
        { 
            printf("read error\n");
        }

        break;
    }

    case SEATRAC_PROTOCOL_STATUS_WAITING_FOR_LOCAL_RESP:
    {

        flag = wait_response(CID_PING_SEND);
        // printf("test\n");
        if (flag > 0)
        {
            _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_WAITING_FOR_REMOTE_RESP;
            _time_count_remote_resp = 0;
        }
        else if (flag == 0)
        {
            //  printf("flag = 0 \n");
            // ROS_INFO("Flag is 0;");
            _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_IDLE;
        }
        else
        {
            _time_count_local_resp++;
            if (_time_count_local_resp > 40)
            {
                // local beacon response time out
                ROS_ERROR_STREAM("Usbl:Local Beacon response time out.");
                _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_IDLE;
                _lost_account++;
            }
        }
        break;
    }
    case SEATRAC_PROTOCOL_STATUS_WAITING_FOR_REMOTE_RESP:
    {

        flag = wait_response(CID_PING_RESP);

        if (flag > 0)
        {
            // ROS_INFO("Flag > 0;");
            _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_IDLE;
        }
        else if (flag == 0)
        {
            // ROS_INFO("Flag is 0;");
            _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_IDLE;
        }
        else
        {
            _time_count_remote_resp++;
            if (_time_count_remote_resp > 100)
            {
                // remote beacon response time out
                // ROS_ERROR_STREAM("Usbl: Remote Beacon response time out.");
                _seatrac_protocol_state = SEATRAC_PROTOCOL_STATUS_IDLE;
                _lost_account++;
            }
        }
        break;
    }
    }
    // naodai: sleep
    // sleep(0.5);

    if (_lost_account > 10)
    {
        printf("Usbl signal lost more than ten times\n");
        _lost_account = 0;
    }
    // sleep(0.05);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "usbl");
    ros::NodeHandle nh;
    ros::Publisher usbl_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/location/xy_depth", 1000);

    // get beacon_ID
    // int beacon_ID = nh.getParam("beacon_ID", beacon_ID);
    int beacon_ID = 9;
    ROS_INFO("Initial Remote beacon ID:%d", beacon_ID);
    // initial usbl port and baudrate
    const std::string usbl_port = "/dev/ttyUSB0";
    const uint32_t baud_rate = 115200;

    // intial usbl
    usbl_node *u = new usbl_node(beacon_ID);
    if (u->saveSetting(usbl_port, baud_rate))
    {
        ROS_INFO("Save usbl port configure");
        // open port
        u->open_port();
    }
    else
    {
        ROS_ERROR_STREAM("Usbl port configure save failed.");
    }

    // time out range 1000m
    // response time 10ms

    // ros::Rate usbl_rate(10);
    // ros::Rate pos_update_rate(10);
    // while (ros::ok())
    while(true)
    {
        // printf("running...\n");
        // run usbl
        u->run();

        // ros::spinOnce();
        // usbl_rate.sleep();
    }

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     // get pos
    //     float *cur_pos;
    //     cur_pos = u->get_pos();
    //     geometry_msgs::Vector3Stamped v;
    //     v.vector.x = cur_pos[0];
    //     v.vector.y = cur_pos[1];
    //     v.vector.z = cur_pos[2];
    //     // send usbl position
    //     usbl_pub.publish(v);
    //     ROS_INFO("Current position is %f, %f, %f", cur_pos[0], cur_pos[1], cur_pos[2]);
    //     pos_update_rate.sleep();
    // }
}
