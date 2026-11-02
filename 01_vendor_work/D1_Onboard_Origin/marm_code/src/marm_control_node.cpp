#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "msg/SetServoAngle_.hpp"
#include "msg/SetServoDumping_.hpp"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <time.h>

#define PubServoAngle_Topic "set_servo_angle"
#define PubArmFeedback_Topic "rt/arm_Feedback"
#define SubArmZero_Topic "arm_zero"
#define SubServoAngleControl_Topic "set_servo_angle_control"
#define SubServoCurrentAngle_Topic "current_servo_angle"

using namespace unitree::robot;
using namespace unitree::common;
using namespace rapidjson;

ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> pubArmFeedback_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::SetServoAngle_> pubServoAngle_publisher;

float servo_angle[7] = {0.0};
float servo_currentValue[7] = {0.0};

void Sleep(double ms)
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = ms * 1000;
    select(0, NULL, NULL, NULL, &delay);
}

void Sleep(int ms)
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = ms * 1000;
    select(0, NULL, NULL, NULL, &delay);
}
 
void execStateFeedback(int seq,int status)
{
    Document document;
    document.SetObject();

    Value data(kObjectType);
    Document::AllocatorType& allocator = document.GetAllocator();

    data.AddMember("exec_status", status, allocator);

    document.AddMember("seq",seq,allocator);
    document.AddMember("address",3,allocator);
    document.AddMember("funcode",2,allocator);
    document.AddMember("data", data, allocator);

    StringBuffer buffer; 
    Writer<StringBuffer> writer(buffer);

    document.Accept(writer);

    unitree_arm::msg::dds_::ArmString_ armFeedback_msg{};
    armFeedback_msg.data_() = buffer.GetString();
    pubArmFeedback_publisher->Write(armFeedback_msg,0);
}

void subArmZero_callback(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;
    std::cout << "armCommand_data:" << pm->data_() << std::endl;

    servo_currentValue[7] = {0.0};

    int seq = std::stoi(pm->data_());

    unitree_arm::msg::dds_::SetServoAngle_ data{};

    for(int i=0;i<7;i++)
    {
        data.seq_() = seq;
        data.id_() = i;
        data.angle_() = 0;
        data.delay_ms_() = 6000;

        pubServoAngle_publisher->Write(data,0);
        servo_currentValue[i] = 0.0;
    }
    execStateFeedback(seq,1);
}

void subServoAngleControl_callback(const void* msg)
{
    const unitree_arm::msg::dds_::SetServoAngle_* pm = (const unitree_arm::msg::dds_::SetServoAngle_*)msg;
    std::cout << "seq:" << pm->seq_() << ", id:" << pm->id_() << ", angle:" << pm->angle_() << ", delay_ms:" << pm->delay_ms_() << std::endl;

    int seq = pm->seq_();
    int servo_id = pm->id_();
    float servo_targetValue = pm->angle_();

    //for(int i=0;i<7;i++)
    //    servo_currentValue[i] = servo_angle[i];
    
    while(servo_currentValue[servo_id] != servo_targetValue)
    {
        if(servo_currentValue[servo_id] <= servo_targetValue)
        {
            servo_currentValue[servo_id] += 0.1;
            if(servo_currentValue[servo_id] > servo_targetValue)
                break;
        }else if(servo_currentValue[servo_id] >= servo_targetValue)
        {
            servo_currentValue[servo_id] -= 0.1;
            if(servo_currentValue[servo_id] < servo_targetValue)
                break;
        }else
            break;

        unitree_arm::msg::dds_::SetServoAngle_ data{};
        data.seq_() = seq;
        data.id_() = servo_id;
        data.angle_() = servo_currentValue[servo_id];
        data.delay_ms_() = 0;

        std::cout << "Angle: " << data.id_() << ", " << data.angle_() << std::endl;
        pubServoAngle_publisher->Write(data,0);
        Sleep(5);
    }
    execStateFeedback(seq,1);
}

void subServoCurrentAngle_callback(const void* msg)
{
    const unitree_arm::msg::dds_::PubServoInfo_* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;
    std::cout << "angle0:" << pm->servo0_data_() << ", angle1:" << pm->servo1_data_() << ", angle2:" << pm->servo2_data_() << ", angle3:" << pm->servo3_data_() << ", angle4:" << pm->servo4_data_() << ", angle5:" << pm->servo5_data_() << ", angle6:" << pm->servo6_data_() << std::endl;

    servo_angle[0] = pm->servo0_data_();
    servo_angle[1] = pm->servo1_data_();
    servo_angle[2] = pm->servo2_data_();
    servo_angle[3] = pm->servo3_data_();
    servo_angle[4] = pm->servo4_data_();
    servo_angle[5] = pm->servo5_data_();
    servo_angle[6] = pm->servo6_data_();
}

int main()
{
    ChannelFactory::Instance()->Init(0);

    pubArmFeedback_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(PubArmFeedback_Topic));
    pubArmFeedback_publisher->InitChannel();
    pubServoAngle_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::SetServoAngle_>(PubServoAngle_Topic));
    pubServoAngle_publisher->InitChannel();

    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subArmZero_subscriber(SubArmZero_Topic);
    subArmZero_subscriber.InitChannel(subArmZero_callback, 10);
    ChannelSubscriber<unitree_arm::msg::dds_::SetServoAngle_> subServoAngleControl_subscriber(SubServoAngleControl_Topic);
    subServoAngleControl_subscriber.InitChannel(subServoAngleControl_callback, 10);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subServoCurrentAngle_subscriber(SubServoCurrentAngle_Topic);
    subServoCurrentAngle_subscriber.InitChannel(subServoCurrentAngle_callback, 10);

    //char ch;
    //std::cin >> ch;
    while(true)
    {
        sleep(10);
    }

    return 0;
}