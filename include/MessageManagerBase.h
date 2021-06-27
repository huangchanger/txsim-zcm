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

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <string>
#include <zcm/zcm-cpp.hpp>

#include "zcm_msgs/icumsg/structLOCATION.hpp"


namespace txsim
{
        class MessageManagerBase
        {
                friend class Recorder;

        public:
                MessageManagerBase(
                    const std::string &url,
                    const std::string &config_file, const std::string &record_path);

                MessageManagerBase(MessageManagerBase const &) = delete;

                MessageManagerBase &operator=(MessageManagerBase const &) = delete;

                virtual ~MessageManagerBase();

                void LOCATIONHandler(
                    const zcm::ReceiveBuffer *rbuf,
                    const std::string &channel,
                    const icumsg::structLOCATION *msg);


                void PublishCaninfo() const;
                void PublishNavinfo() const;
                void PublishFusionmap() const;
                void PublishObjectlist() const;
                void PublishLanes() const;
                void PublishTrafficlight() const;
                void PublishCarsimControl() const;
                void PublishCarsimRoadContact() const;
                void PublishAll() const;

                void PublishCaninfoWithLock() const;
                void PublishNavinfoWithLock() const;
                void PublishFusionmapWithLock() const;
                void PublishObjectlistWithLock() const;
                void PublishLanesWithLock() const;
                void PublishTrafficlightWithLock() const;
                void PublishCarsimControlWithLock() const;
                void PublishCarsimRoadContactWithLock() const;
                void PublishAllAsync();

                void SubscribeAll();

        private:
                void PubLoopCaninfo(int freq);
                void PubLoopNavinfo(int freq);
                void PubLoopFusionmap(int freq);
                void PubLoopObjectlist(int freq);
                void PubLoopLanes(int freq);
                void PubLoopTrafficlight(int freq);
                void PubLoopCarsimControl(int freq);
                void PubLoopCarsimRoadContact(int freq);

        public:

                mutable zcm::ZCM tunnel_;

                mutable std::mutex location_mutex_;

                icumsg::structLOCATION location_;

        private:
                std::vector<std::thread> pub_threads_;
                std::vector<zcm::Subscription *> sub_threads_;

                volatile bool need_stop_;

        private:
                std::string kChannelNameCaninfo;
                std::string kChannelNameNavinfo;
                std::string kChannelNameFusionmap;
                std::string kChannelNameObjectlist;
                std::string kChannelNameCancontrol;
                std::string kChannelNameLanes;
                std::string kChannelNameTrafficlight;
                std::string kChannelNameCarsimControl;
                std::string kChannelNameCarsimState;
                std::string kChannelNameCarsimQuery;
                std::string kChannelNameCarsimContact;

                int kFreqCaninfo;
                int kFreqNavinfo;
                int kFreqFusionmap;
                int kFreqObjectlist;
                int kFreqLanes;
                int kFreqTrafficlight;
                int kFreqCarsimContact;
                int kFreqCarsimControl;

                bool kSwitchCaninfo;
                bool kSwitchNavinfo;
                bool kSwitchFusionmap;
                bool kSwitchObjectlist;
                bool kSwitchLanes;
                bool kSwitchTrafficlight;
                bool kSwitchCarsimContact;
                bool kSwitchCarsimControl;

                bool kRecordCaninfo;
                bool kRecordNavinfo;
                bool kRecordFusionmap;
                bool kRecordObjectlist;
                bool kRecordLanes;
                bool kRecordTrafficlight;
                bool kRecordCarsimContact;
                bool kRecordCarsimControl;
                bool kRecordCarsimState;
                bool kRecordCarsimQuery;
        };
} // namespace tievsim
