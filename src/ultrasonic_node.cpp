#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <serialPort.h>
#include <ultrasonic/ultrarange.h>
#include <std_msgs/Header.h>
#include <ros/time.h>
#include <signal.h>
#include "ultrasonic.h"

using namespace SerialPort;
serialPort sp;
ros::Publisher ultra_pub;
int count = 0;

bool checkData(const uint8_t *data)
{
    if (data[0] != 0xaa || data[16] != 0xaa || data[32] != 0xaa)
        return false;
    if (data[6] != 0x06 || data[22] != 0x06 || data[38] != 0x06)
        return false;
    if (data[7] != 0x11 || data[23] != 0x12 || data[39] != 0x13)
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

bool genMsg(ultrasonic::ultrarange &range, const uint8_t *data)
{
    if (!checkData(data))
        return false;
    std_msgs::Header header;
    header.frame_id = "/ultrasonic";
    header.stamp = ros::Time::now();
    header.seq = 1;
    range.header = header;
    if (data[7] == 0x11)
    {
        range.channel_1 = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);
        range.channel_2 = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);
        range.channel_3 = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);
        range.channel_4 = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    else if (data[7] == 0x12)
    {
        range.channel_5 = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);
        range.channel_6 = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);
        range.channel_7 = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);
        range.channel_8 = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    else
    {
        range.channel_9 = bcd2demical(data[8]) * 100 + bcd2demical(data[9]);
        range.channel_10 = bcd2demical(data[10]) * 100 + bcd2demical(data[11]);
        range.channel_11 = bcd2demical(data[12]) * 100 + bcd2demical(data[13]);
        range.channel_12 = bcd2demical(data[14]) * 100 + bcd2demical(data[15]);
    }
    return true;
}

void printData(const uint8_t *data)
{
    for (int i = 0; i < 48; i++)
    {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(data[i]) << " ";
        if ((i + 1) % 16 == 0)
            std::cout << std::endl;
    }
}

void serialRead()
{
    int nread;
    uint8_t data[96];
    ultrasonic::ultrarange range;
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
                    if (genMsg(range, data))
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
                    if (genMsg(range, data))
                    {
                        count++;
                        if (count == 3)
                        {
                            ultra_pub.publish(range);
                            std::cout << "publish" << std::endl;
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
    ros::NodeHandle nh("");
    signal(SIGINT, mySigintHandler);
    ultra_pub = nh.advertise<ultrasonic::ultrarange>("/ultra_range", 10);
    sp.OpenPort("/dev/ttyUSB0");
    sp.setup(9600, 0, 8, 1, 'N');
    sp.writeBuffer(STOPCMD, 16);
    sp.writeBuffer(STARTCMD, 16);
    serialRead();
    return 0;
}