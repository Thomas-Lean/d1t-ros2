#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "msg/SetServoAngle_.hpp"
#include "msg/SetServoDumping_.hpp"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <thread>
#include <chrono>

#define PubArmFeedback_Topic "rt/arm_Feedback"
#define PubServoAngle_Topic "current_servo_angle"
#define SubServoAngle_Topic "set_servo_angle"
#define SubServoDumping_Topic "set_servo_dumping"
#define SERVO_PORT_NAME "/dev/ttyS4"

using namespace unitree::robot;
using namespace unitree::common;
using namespace fsuservo;
using namespace rapidjson;

ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> pubArmFeedback_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::PubServoInfo_> pubServoAngle_publisher;

float curAngle[7];
FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);
FSUS_Servo servo[7] = {
            FSUS_Servo(0,&protocol),
            FSUS_Servo(1,&protocol),
            FSUS_Servo(2,&protocol),
            FSUS_Servo(3,&protocol),
            FSUS_Servo(4,&protocol),
            FSUS_Servo(5,&protocol),
            FSUS_Servo(6,&protocol),
        };

class Timer_ {
    public:
        Timer_(std::function<void()> callback, int interval) : 
            callback_(callback), 
            interval_(interval), 
            running_(false) {}

        void start() {
            running_ = true;
            timer_thread_ = std::thread([this]() {
                while (running_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
                    if (callback_ != nullptr) {
                        callback_();
                    }
                }
            });
        }

        void stop() {
            running_ = false;
            if (timer_thread_.joinable()) {
                timer_thread_.join();
            }
        }

    private:
        std::function<void()> callback_;
        int interval_;
        bool running_;
        std::thread timer_thread_;
};

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

void pubServoAngle_callback()
{
    float angle;

    Document document;
    document.SetObject();

    Value data(kObjectType);
    Document::AllocatorType& allocator = document.GetAllocator();

    unitree_arm::msg::dds_::PubServoInfo_ servoInfo_msg{};
    unitree_arm::msg::dds_::ArmString_ armFeedback_msg{};

    angle = servo[0].queryRawAngle();
    curAngle[0] = angle;
    servoInfo_msg.servo0_data_() = angle;
    data.AddMember("angle0", angle, allocator);

    angle = servo[1].queryRawAngle();
    curAngle[1] = angle;
    servoInfo_msg.servo1_data_() = angle;
    data.AddMember("angle1", angle, allocator);

    angle = servo[2].queryRawAngle();
    curAngle[2] = angle;
    servoInfo_msg.servo2_data_() = angle;
    data.AddMember("angle2", angle, allocator);

    angle = servo[3].queryRawAngle();
    curAngle[3] = angle;
    servoInfo_msg.servo3_data_() = angle;
    data.AddMember("angle3", angle, allocator);

    angle = servo[4].queryRawAngle();
    curAngle[4] = angle;
    servoInfo_msg.servo4_data_() = angle;
    data.AddMember("angle4", angle, allocator);

    angle = servo[5].queryRawAngle();
    curAngle[5] = angle;
    servoInfo_msg.servo5_data_() = angle;
    data.AddMember("angle5", angle, allocator);

    angle = servo[6].queryRawAngle();
    curAngle[6] = angle;
    servoInfo_msg.servo6_data_() = angle;
    data.AddMember("angle6", angle, allocator);
    // std::cout << "{" << curAngle[0] << ", " << curAngle[1] << ", " << curAngle[2] << ", " << curAngle[3] << ", " << curAngle[4] << ", " << curAngle[5] << ", " << curAngle[6] << "}," << std::endl;

    document.AddMember("seq",10,allocator);
    document.AddMember("address",2,allocator);
    document.AddMember("funcode",1,allocator);
    document.AddMember("data", data, allocator);

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    document.Accept(writer);
    armFeedback_msg.data_() = buffer.GetString();

    pubServoAngle_publisher->Write(servoInfo_msg, 0);
    pubArmFeedback_publisher->Write(armFeedback_msg, 0);
}

void subServoAngle_callback(const void* msg)
{
    const unitree_arm::msg::dds_::SetServoAngle_* pm = (const unitree_arm::msg::dds_::SetServoAngle_*)msg;
    // std::cout << "seq:" << pm->seq_() << ", id:" << pm->id_() << ", angle:" << pm->angle_() << ", delay_ms:" << pm->delay_ms_() << std::endl;
    // std::cout << "seq:" << pm->seq_() << ", id:" << pm->id_() << std::endl;
    float maxalpha = 286.4789 / 4;
    float maxomega = 28.64789 * 4;
    float maxangle[7] = { 135,  90,  90,  135,  90,  135,  50};
    float minangle[7] = {-135, -90, -90, -135, -90, -135, -20};
    int seq = pm->seq_();
    int id = pm->id_();
    float angle = pm->angle_();
    // int delay_ms = pm->delay_ms_();
    int delay_ms = pm->delay_ms_() == 0 ? 1 : pm->delay_ms_();

    // float delta_x = abs(angle - curAngle[id]);
    // float delta_a = 2 * delta_x / (delay_ms * delay_ms);
    // float delta_t = sqrt(2 * delta_x / delta_a) * 1000;
    // std::cout << "id:" << id << "     delta_x:" << delta_x << "     delta_a:" << delta_a << "     delta_t:" << delta_t << std::endl;

    if((curAngle[id] <= 180) && (curAngle[id] >= -180))
    {
        angle = angle > maxangle[id] ? maxangle[id] : angle;
        angle = angle < minangle[id] ? minangle[id] : angle;
        float omega = abs(angle - curAngle[id]) / delay_ms * 1000;
        omega = omega > maxomega ? maxomega : omega;
        float alpha = omega / delay_ms * 1000;
        alpha = alpha > maxalpha ? maxalpha : alpha;

        // delay_ms = (int)(abs(angle) / omega);
        int tardelay_ms = (int)(sqrt(abs(angle - curAngle[id]) * 2 / alpha) * 1000);
        int testdelay_ms = (int)(((sqrt(omega * omega + abs(angle - curAngle[id]) * 2 * alpha) - omega) / alpha) * 1800);
        testdelay_ms = testdelay_ms > 0 ? testdelay_ms : 0;
        
        // std::cout << "id:" << id << " angle:" << (angle - curAngle[id]) << "     omega:" << omega << "     alpha:" << alpha << "     delay_ms:" << delay_ms << "     tardelay_ms:" << tardelay_ms << "     testdelay_ms:" << testdelay_ms << std::endl;
        // std::cout << "id:" << id << "angle:" << angle << "delay_ms:" << delay_ms << std::endl;

        // servo[id].setRawAngle(angle, tardelay_ms);
        servo[id].setRawAngle(angle, testdelay_ms);
        std::cout << "id:" << id << ", curangle:" << curAngle[id] << ", tarangle:" << angle << std::endl;
    }
    else
    {
        servo[id].setDamping(500);
    }

    //execStateFeedback(seq,1);
}

void subServoDumping_callback(const void* msg)
{
    const unitree_arm::msg::dds_::SetServoDumping_* pm = (const unitree_arm::msg::dds_::SetServoDumping_*)msg;
    // std::cout << "seq:" << pm->seq_() << ", id:" << pm->id_() << ", power:" << pm->power_() << std::endl;

    int seq = pm->seq_();
    int id = pm->id_();
    int power = pm->power_();

    servo[id].setDamping(power);

    //execStateFeedback(seq,1);
}

int main()
{
    ChannelFactory::Instance()->Init(0);

    for(int id = 0; id < 6; id++)
    {
        servo[id].clearAngle();
    }

    pubArmFeedback_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(PubArmFeedback_Topic));
    pubArmFeedback_publisher->InitChannel();
    pubServoAngle_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::PubServoInfo_>(PubServoAngle_Topic));
    pubServoAngle_publisher->InitChannel();
    
    ChannelSubscriber<unitree_arm::msg::dds_::SetServoAngle_> subServoAngle_subscriber(SubServoAngle_Topic);
    subServoAngle_subscriber.InitChannel(subServoAngle_callback, 10);
    ChannelSubscriber<unitree_arm::msg::dds_::SetServoDumping_> subServoDumping_subscriber(SubServoDumping_Topic);
    subServoDumping_subscriber.InitChannel(subServoDumping_callback, 10);

    Timer_ timer(pubServoAngle_callback, 100);
    timer.start();

    while (true)
    {
        sleep(10);
    }

    timer.stop();

    return 0;
}
