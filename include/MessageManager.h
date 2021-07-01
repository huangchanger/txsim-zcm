#pragma once

#include <list>
#include <sys/socket.h>
#include "txsim/txsim_module.h"

#include "MessageManagerBase.h"
#include "simutils.hpp"





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
        void PackPredictedObject(tx_sim::StepHelper& helper);

        // send protobuf
        void SendTopicControl(tx_sim::StepHelper& helper);


    };

}; // namespace tievsim