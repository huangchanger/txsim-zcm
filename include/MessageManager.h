#pragma once
#include <list>
#include <sys/socket.h>
#include "txsim/txsim_module.h"

#include "MessageManagerBase.h"
#include "simutils.hpp"

#include "proto_msgs/basic.pb.h"
#include "proto_msgs/control.pb.h"
#include "proto_msgs/grading.pb.h"
#include "proto_msgs/header.pb.h"
#include "proto_msgs/laneMarks.pb.h"
#include "proto_msgs/location.pb.h"
#include "proto_msgs/planStatus.pb.h"
#include "proto_msgs/traffic.pb.h"
#include "proto_msgs/trajectory.pb.h"

//#define DEBUG_CARSIM
//#define USE_SLOPE

namespace txsim
{
    class MessageManager : public MessageManagerBase
    {

    public:
        MessageManager(const std::string &url);
        ~MessageManager() = default;


        // receive protobuf
        void PackNavinfo(tx_sim::StepHelper& helper);

        // send protobuf
        void SendTopicControl(tx_sim::StepHelper& helper);


    };

}; // namespace tievsim