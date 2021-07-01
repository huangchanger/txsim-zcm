/**
 * @file MessageManagerBase.h
 * @author leoherz_liu@163.com
 * @brief base class of message manager for tiev
 * @version 0.1
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <string>
#include <zcm/zcm-cpp.hpp>
#include <zcm/zcm.h>
#include "WGS84UTM.h"

#include "tievmsg_zcm/MsgAccessoryControlSignal.hpp"
#include "tievmsg_zcm/MsgAutonomousModeControlSignal.hpp"
#include "tievmsg_zcm/MsgCanInfoSignal.hpp"
#include "tievmsg_zcm/MsgChassisCommandSignal.hpp"
#include "tievmsg_zcm/MsgInteractionControlSignal.hpp"
#include "tievmsg_zcm/MsgNavInfoSignal.hpp"
#include "tievmsg_zcm/MsgPlannerEmergencyControlSignal.hpp"
#include "tievmsg_zcm/MsgRainDetectionSignal.hpp"
#include "tievmsg_zcm/MsgRemoteEmergencyControlSignal.hpp"
#include "tievmsg_zcm/MsgTrafficLightSignal.hpp"
#include "tievmsg_zcm/MsgTrafficSignSignal.hpp"
// #include "tievmsg_zcm/MsgTrajectorySignal.hpp"
#include "tievmsg_zcm/MsgWaterhorseSignal.hpp"
#include "tievmsg_zcm/PredictedObject.hpp"
#include "tievmsg_zcm/MsgPredictedObjectTrajectoryList.hpp"

namespace txsim
{
        class MessageManagerBase
        {
        public:
                MessageManagerBase(const std::string &url);

                MessageManagerBase(MessageManagerBase const &) = delete;

                MessageManagerBase &operator=(MessageManagerBase const &) = delete;

                virtual ~MessageManagerBase();

                // receive zcm handler
                void ChassisCommandHandler(
                    const zcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const MsgChassisCommandSignal *msg);
                void SubscribeAll();


                // send zcm
                void PublishNavinfo() const;

                void PublishNavinfoWithLock() const;

                void PublishAllAsync();

                void PublishAll() const;

        private:
                void PubLoopNavinfo(int freq);

        public:

                mutable zcm::ZCM tunnel_;

                mutable std::mutex chassisCommand_mutex_;
                mutable std::mutex navinfo_mutex_;
                mutable std::mutex predictedObject_mutex_;

                MsgChassisCommandSignal chassisCommand_;
                MsgNavInfoSignal navinfo_;
                MsgPredictedObjectTrajectoryList predictedObject_;
                

        private:
                std::vector<std::thread> pub_threads_;
                std::vector<zcm::Subscription *> sub_threads_;

                volatile bool need_stop_;

        private:
                std::string kChannelNameChassisCommand;
                std::string kChannelNameNavinfo;
                std::string kChannelNamePredictedObject;

                int kFreqNavinfo;
                int kFreqPredictedObject;

                bool kSwitchNavinfo;
                bool kSwitchPredictedObject;
        };
} // namespace tievsim
