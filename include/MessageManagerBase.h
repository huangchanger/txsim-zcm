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

                void stopSubandPub();

                // receive zcm handler
                void ChassisCommandHandler(
                    const zcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const MsgChassisCommandSignal *msg);
                void SubscribeAll();


                // send zcm
                void PublishAllAsync();

                void PublishAll() const;

                void PubLoopNavinfo(int freq);
                void PubLoopPredictedObject(int freq);
                void PubLoopTrafficLight(int freq);
                void PubLoopCaninfo(int freq);

        public:

                mutable zcm::ZCM tunnel_;

                mutable std::mutex chassisCommand_mutex_;
                mutable std::mutex navinfo_mutex_;
                mutable std::mutex predictedObject_mutex_;
                mutable std::mutex trafficLight_mutex_;
                mutable std::mutex caninfo_mutex_;

                MsgChassisCommandSignal chassisCommand_;
                MsgNavInfoSignal navinfo_;
                MsgPredictedObjectTrajectoryList predictedObject_;
                MsgTrafficLightSignal trafficLight_;
                MsgCanInfoSignal caninfo_;

                Point2d start_pt2d;
                

        private:
                std::vector<std::thread> pub_threads_;
                std::vector<zcm::Subscription *> sub_threads_;

                volatile bool need_stop_;

        private:
                std::string kChannelNameChassisCommand;
                std::string kChannelNameNavinfo;
                std::string kChannelNamePredictedObject;
                std::string kChannelNameTrafficLight;
                std::string kChannelNameCaninfo;

                int kFreqNavinfo;
                int kFreqPredictedObject;
                int kFreqTrafficLight;
                int kFreqCaninfo;

                bool kSwitchNavinfo;
                bool kSwitchPredictedObject;
                bool kSwitchTrafficLight;
                bool kSwitchCaninfo;
        };
} // namespace tievsim
