#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <serialPort.h>
#include <ultrasonic_ros_driver/ultrarange.h>
#include <ultrasonic_ros_driver/ultrachannels.h>
#include <std_msgs/Header.h>
#include <ros/time.h>
#include <signal.h>
#include "ultrasonic.h"

using namespace SerialPort;
serialPort sp;
ros::Publisher ultra_pub;
int count = 0;

void printData(const uint8_t *data)
{
    for (int i = 0; i < 16; i++)
    {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(data[i]) << " ";
        
    }
    std::cout << std::endl;
}

bool checkData(const uint8_t *data)
{
    if (data[0] != 0xaa || data[3]!=0x08||data[6] != 0x06)
        return false;
    return true;
}

int bcd2demical(uint8_t bcd)
{
    // 16进制BCD到十进制
    int sum = 0, c = 1;
    for (int i = 1; bcd > 0; i++)
    {
        sum += (bcd % 16) * c;
        c *= 10;
        bcd /= 16;
    }
    return sum;
}

bool genMsg(ultrasonic_ros_driver::ultrachannels &msg, const uint8_t *data)
{
    if (!checkData(data))
        return false;
    // printData(data);
    msg.channels.resize(12);
    msg.header.stamp = ros::Time::now();
    if (data[7] == 0x11)
    {
        msg.channels[0].frame_id = "channel1";
        msg.channels[0].range = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);

        msg.channels[1].frame_id = "channel2";
        msg.channels[1].range = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);

        msg.channels[2].frame_id = "channel3";
        msg.channels[2].range = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);

        msg.channels[3].frame_id = "channel4";
        msg.channels[3].range = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    else if (data[7] == 0x12)
    {
        msg.channels[4].frame_id = "channel5";
        msg.channels[4].range = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);

        msg.channels[5].frame_id = "channel6";
        msg.channels[5].range = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);

        msg.channels[6].frame_id = "channel7";
        msg.channels[6].range = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);

        msg.channels[7].frame_id = "channel8";
        msg.channels[7].range = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    else
    {
        msg.channels[8].frame_id = "channel9";
        msg.channels[8].range = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);

        msg.channels[9].frame_id = "channel10";
        msg.channels[9].range = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);

        msg.channels[10].frame_id = "channel11";
        msg.channels[10].range = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);

        msg.channels[11].frame_id = "channel12";
        msg.channels[11].range = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    return true;
}



void serialRead()
{
    int nread;
    uint8_t data[96];
    ultrasonic_ros_driver::ultrachannels msg;
    bool find_sof = true;
    bool first_frame = true;
    int count = 0;

    while (ros::ok())
    {
        if (find_sof)
        {
            if ((nread = sp.readBuffer(data, 1) == 1))
            {
                if (data[0] == 0xaa)
                {
                    find_sof = false;
                }
            }
        }
        else
        {
            if (first_frame)
            { // 第一帧只读15个字节    
                uint8_t temp_data[15];
                if ((nread = sp.readBuffer(temp_data, 15)) == 15)
                {

                    data[0] = 0xaa;
                    for (int i = 0; i < 15; i++)
                    {
                        data[i + 1] = temp_data[i];
                    }
                    if (genMsg(msg, data))
                    {           
                        count++;
                        first_frame = false;
                    }
                }
            }
            else
            { // 之后读16字节
                if ((nread = sp.readBuffer(data, 16)) == 16)
                {
                    if (genMsg(msg, data))
                    {
                        count++;
                        if (count == 3)
                        {
                            ultra_pub.publish(msg);
                            ROS_INFO("publish");
                            count = 0;
                        }
                    }
                }
            }
        }
    }
}

void mySigintHandler(int sig)
{
    sp.writeBuffer(STOPCMD, 16);
    std::cout << std::endl;
    std::cout << "ultrasonic stopped" << std::endl;
    sp.ClosePort();
    std::cout << "stop ultrasonic node" << std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasonic_pub_node");
    ros::NodeHandle nh("~");
    std::string dev_port, output_topic;
    nh.param<std::string>("dev_port", dev_port, "/dev/ttyUSB0");
    nh.param<std::string>("output_topic", output_topic, "/ultra_range");
    signal(SIGINT, mySigintHandler);
    ultra_pub = nh.advertise<ultrasonic_ros_driver::ultrachannels>(output_topic, 1);
    
    
    if(!sp.OpenPort(dev_port.c_str())){
        ROS_ERROR("Failed to open port %s", dev_port.c_str());
        ros::shutdown();
    }
    sp.setup(9600, 0, 8, 1, 'N');
    sp.writeBuffer(STOPCMD, 16);
    sp.writeBuffer(STARTCMD, 16);
    serialRead();
    sp.writeBuffer(STOPCMD, 16);
    // sp.ClosePort();
    // ros::shutdown();
    return 0;
}