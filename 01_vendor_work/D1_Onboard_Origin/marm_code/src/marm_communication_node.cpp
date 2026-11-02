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

#define SubArmCommand_Topic "rt/arm_Command"
#define SubServoCurrentAngle_Topic "current_servo_angle"
#define PubArmFeedback_Topic "rt/arm_Feedback"
#define PubArmZero_Topic "arm_zero"
#define PubServoAngle_Topic "set_servo_angle"
#define PubServoAngleControl_Topic "set_servo_angle_control"
#define PubServoDumping_Topic "set_servo_dumping"

using namespace unitree::robot;
using namespace unitree::common;
using namespace rapidjson;

ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> pubArmFeedback_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> pubArmZero_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::SetServoAngle_> pubServoAngle_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::SetServoAngle_> pubServoAngleControl_publisher;
ChannelPublisherPtr<unitree_arm::msg::dds_::SetServoDumping_> pubServoDumping_publisher;

int enable_status = 0;
int power_status = 0;
int error_status = 0;

float servo_currentValue[7] = {0.0};

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

void recvStateFeedback(int seq,int status)
{
    Document document;
    document.SetObject();

    Value data(kObjectType);
    Document::AllocatorType& allocator = document.GetAllocator();

    data.AddMember("recv_status", status, allocator);

    document.AddMember("seq",seq,allocator);
    document.AddMember("address",3,allocator);
    document.AddMember("funcode",1,allocator);
    document.AddMember("data", data, allocator);

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    document.Accept(writer);

    unitree_arm::msg::dds_::ArmString_ armFeedback_msg{};
    armFeedback_msg.data_() = buffer.GetString();
    pubArmFeedback_publisher->Write(armFeedback_msg,0);
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

void execFeedbackCallback(int delaySeconds,int seq,int status) 
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delaySeconds));
    execStateFeedback(seq,status);
    std::cout << "Over!!!" << std::endl;
}

void subArmCommand_callback(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;
    std::cout << "armCommand_data:" << pm->data_() << std::endl;

    std::string json_str = pm->data_();

    Document document;
    document.Parse(json_str.c_str());
    std::cout << "***********************" << std::endl;

    if (!document.HasParseError()) 
    {
        std::cout << "++++++++++++++" << std::endl;
        if (document.IsObject())
        {
            std::cout << "------------" << std::endl;
            int seq = document["seq"].GetInt();
            int address = document["address"].GetInt();
            int funcode = document["funcode"].GetInt();
            
            if(address == 1 && funcode == 1)
            {
                int id = document["data"]["id"].GetInt();
                double angle = document["data"]["angle"].GetDouble();
                double delay_ms = document["data"]["delay_ms"].GetInt();
                enable_status = 1;
                
                unitree_arm::msg::dds_::SetServoAngle_ data{};

                data.seq_() = seq;
                data.id_() = id;
                data.angle_() = angle;
                data.delay_ms_() = delay_ms;

                pubServoAngle_publisher->Write(data,0);
                recvStateFeedback(seq,1);
                //execStateFeedback(seq,1);
            }else if(address == 1 && funcode == 2)
            {
                double angle[7] = {0.0};
                std::cout << "=========" << std::endl;

                angle[0] = document["data"]["angle0"].GetDouble();
                std::cout << "====0====" << std::endl;
                angle[1] = document["data"]["angle1"].GetDouble();
                std::cout << "====1====" << std::endl;
                angle[2] = document["data"]["angle2"].GetDouble();
                std::cout << "====2====" << std::endl;
                angle[3] = document["data"]["angle3"].GetDouble();
                std::cout << "====3====" << std::endl;
                angle[4] = document["data"]["angle4"].GetDouble();
                std::cout << "====4====" << std::endl;
                angle[5] = document["data"]["angle5"].GetDouble();
                std::cout << "====5====" << std::endl;
                angle[6] = document["data"]["angle6"].GetDouble();
                std::cout << "====6====" << std::endl;

                int mode = document["data"]["mode"].GetInt();
                enable_status = 1;

                int max_delay = 0;

                unitree_arm::msg::dds_::SetServoAngle_ data{};
                for(int i=0;i<7;i++)
                {
                    data.seq_() = seq;
                    data.id_() = i;
                    data.angle_() = angle[i];

                    if(mode ==0)
                    {
                        data.delay_ms_() = 40;
                        pubServoAngle_publisher->Write(data,0);
                    }
                    else if(mode ==1)
                    {
                        data.delay_ms_() = int((200*std::abs(angle[i] - servo_currentValue[i]))/3);
                        pubServoAngle_publisher->Write(data,0);

                        if(max_delay < data.delay_ms_())
                            max_delay = data.delay_ms_();
                    }
                }
                recvStateFeedback(seq,1);
                execFeedbackCallback(max_delay,seq,1);

            }else if(address == 1 && funcode == 3)
            {
                double pose_x = document["data"]["pose_x"].GetDouble();
                double pose_y = document["data"]["pose_y"].GetDouble();
                double pose_z = document["data"]["pose_z"].GetDouble();
                double roll = document["data"]["roll"].GetDouble();
                double pitch = document["data"]["pitch"].GetDouble();
                double yaw = document["data"]["yaw"].GetDouble();
                
                /*double cy = cos(yaw * 0.5);
                double sy = sin(yaw * 0.5);
                double cp = cos(pitch * 0.5);
                double sp = sin(pitch * 0.5);
                double cr = cos(roll * 0.5);
                double sr = sin(roll * 0.5);
                
                double quat_w = cy * cp * cr + sy * sp * sr;
                double quat_x = cy * cp * sr - sy * sp * cr;
                double quat_y = sy * cp * sr + cy * sp * cr;
                double quat_z = sy * cp * cr - cy * sp * sr;
                
                std::cout << "Pose: " << pose_x << ", " << pose_y << ", " << pose_z << std::endl;
                std::cout << "Pose: " << roll << ", " << pitch << ", " << yaw << std::endl;
                std::cout << "Quat: " << quat_w << ", " << quat_x << ", " << quat_y << ", " << quat_z << std::endl;*/

                recvStateFeedback(seq,1);
            }else if(address == 1 && funcode == 4)
            {
                int id = document["data"]["id"].GetInt();
                int mode = document["data"]["mode"].GetInt();

                if(mode < 1000)
                    enable_status = 0;
                else
                    enable_status = 1;
                
                unitree_arm::msg::dds_::SetServoDumping_ data{};
                data.seq_() = seq;
                data.id_() = id;
                data.power_() = mode;

                pubServoDumping_publisher->Write(data,0);
                recvStateFeedback(seq,1);
                execStateFeedback(seq,1);
            }else if(address == 1 && funcode == 5)
            {
                int mode = document["data"]["mode"].GetInt();

                if(mode < 1000)
                    enable_status = 0;
                else
                    enable_status = 1;
                
                unitree_arm::msg::dds_::SetServoDumping_ data{};
                for(int i=0;i<7;i++)
                {
                    data.seq_() = seq;
                    data.id_() = i;
                    data.power_() = mode;

                    pubServoDumping_publisher->Write(data,0);
                }

                recvStateFeedback(seq,1);
                execStateFeedback(seq,1);
            }else if(address == 1 && funcode == 6)
            {
                power_status = 1;
                int power = document["data"]["power"].GetInt();
                
                std::cout << "Power: " << power << std::endl;

                recvStateFeedback(seq,1);
                execStateFeedback(seq,1);
            }else if(address == 1 && funcode == 7)
            {
                enable_status = 1;
                unitree_arm::msg::dds_::ArmString_ data{};
                data.data_() = std::to_string(seq);
                
                pubArmZero_publisher->Write(data,0);
                recvStateFeedback(seq,1);
                execFeedbackCallback(6000,seq,1);
            }else{
                recvStateFeedback(seq,0);
                execStateFeedback(seq,0);
            }
        }else{
            recvStateFeedback(10,0);
            execStateFeedback(10,0);
        }
    }
    else
    {
        recvStateFeedback(10,0);
        std::cerr << "Failed to parse JSON" << std::endl;
    }
}

void pubArmStatue_callback()
{
    Document document;
    document.SetObject();

    Value data(kObjectType);
    Document::AllocatorType& allocator = document.GetAllocator();

    data.AddMember("enable_status", enable_status, allocator);
    data.AddMember("power_status", power_status, allocator);
    data.AddMember("error_status", error_status, allocator);

    document.AddMember("seq",10,allocator);
    document.AddMember("address",2,allocator);
    document.AddMember("funcode",3,allocator);
    document.AddMember("data", data, allocator);

    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    document.Accept(writer);

    unitree_arm::msg::dds_::ArmString_ armFeedback_msg{};
    armFeedback_msg.data_() = buffer.GetString();
    pubArmFeedback_publisher->Write(armFeedback_msg,0);
}

void subServoCurrentAngle_callback(const void* msg)
{
    const unitree_arm::msg::dds_::PubServoInfo_* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;
    //std::cout << "angle0:" << pm->servo0_data_() << ", angle1:" << pm->servo1_data_() << ", angle2:" << pm->servo2_data_() << ", angle3:" << pm->servo3_data_() << ", angle4:" << pm->servo4_data_() << ", angle5:" << pm->servo5_data_() << ", angle6:" << pm->servo6_data_() << std::endl;

    servo_currentValue[0] = pm->servo0_data_();
    servo_currentValue[1] = pm->servo1_data_();
    servo_currentValue[2] = pm->servo2_data_();
    servo_currentValue[3] = pm->servo3_data_();
    servo_currentValue[4] = pm->servo4_data_();
    servo_currentValue[5] = pm->servo5_data_();
    servo_currentValue[6] = pm->servo6_data_();
}

int main()
{
    ChannelFactory::Instance()->Init(0);

    pubArmFeedback_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(PubArmFeedback_Topic));
    pubArmFeedback_publisher->InitChannel();
    pubArmZero_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>(PubArmZero_Topic));
    pubArmZero_publisher->InitChannel();
    pubServoAngle_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::SetServoAngle_>(PubServoAngle_Topic));
    pubServoAngle_publisher->InitChannel();
    pubServoAngleControl_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::SetServoAngle_>(PubServoAngleControl_Topic));
    pubServoAngleControl_publisher->InitChannel();
    pubServoDumping_publisher.reset(new ChannelPublisher<unitree_arm::msg::dds_::SetServoDumping_>(PubServoDumping_Topic));
    pubServoDumping_publisher->InitChannel();

    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subArmCommand_subscriber(SubArmCommand_Topic);
    subArmCommand_subscriber.InitChannel(subArmCommand_callback, 10);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subServoCurrentAngle_subscriber(SubServoCurrentAngle_Topic);
    subServoCurrentAngle_subscriber.InitChannel(subServoCurrentAngle_callback, 10);

    Timer_ timer(pubArmStatue_callback, 100);
    timer.start();

    while (true)
    {
        sleep(10);
    }

    timer.stop();

    return 0;
}
