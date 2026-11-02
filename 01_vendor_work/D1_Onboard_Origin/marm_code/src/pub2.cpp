#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    //msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"angle0\":22,\"angle1\":22,\"angle2\":22,\"angle3\":22,\"angle4\":22,\"angle5\":22,\"angle6\":10}}";
    publisher.Write(msg);
 
    return 0;
}