#include "MessageManagerBase.h"

using std::string;
using std::vector;

namespace txsim
{

    // tunnel_(url) is deleted for a while to test
    MessageManagerBase::MessageManagerBase(const string &url):  tunnel_(url), need_stop_(false), 
        chassisCommand_(), navinfo_(), predictedObject_(), trafficLight_(), caninfo_(),start_pt2d()
    {
        kChannelNameChassisCommand = "MsgChassisCommandSignal";
        kChannelNameNavinfo = "MsgNavInfoSignal";
        kChannelNamePredictedObject = "MsgPredictedObjectTrajectoryList";
        kChannelNameTrafficLight = "TrafficLight";
        kChannelNameCaninfo = "MsgCanInfoSignal";

        int freq = 20;
        kFreqNavinfo = freq;
        kSwitchNavinfo = 1;

        kFreqPredictedObject = freq;
        kSwitchPredictedObject = 1;

        kFreqTrafficLight = freq;
        kSwitchTrafficLight = 0;

        kFreqCaninfo = freq;
        kSwitchCaninfo = 1;
     };

    MessageManagerBase::~MessageManagerBase()
    { 
        need_stop_ = true;
        for (auto &t : pub_threads_)
        {
            if (t.joinable())
                t.join();
        }

        tunnel_.stop();

        for (auto t : sub_threads_)
        {
            tunnel_.unsubscribe(t);
        }
    }

    void MessageManagerBase::stopSubandPub()
    {
        need_stop_ = true;
        for (auto &t : pub_threads_)
        {
            if (t.joinable())
                t.join();
        }

        tunnel_.stop();

        for (auto t : sub_threads_)
        {
            tunnel_.unsubscribe(t);
        }
    }

 // receive zcm
    void MessageManagerBase::ChassisCommandHandler(const zcm::ReceiveBuffer *rbuf,const std::string &chan,const MsgChassisCommandSignal *msg)
    {
        std::lock_guard<std::mutex> lock(chassisCommand_mutex_);
        chassisCommand_.timestamp = msg->timestamp;
        chassisCommand_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
        chassisCommand_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
        chassisCommand_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
        chassisCommand_.car_gear_command = msg->car_gear_command;
    }


    void MessageManagerBase::SubscribeAll()
    {
        auto sub1 = tunnel_.subscribe(
            kChannelNameChassisCommand, &MessageManagerBase::ChassisCommandHandler, this);

        sub_threads_.push_back(sub1);

        tunnel_.start();
    }


    void MessageManagerBase::PubLoopNavinfo(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            if(true)
            {
                std::lock_guard<std::mutex> lk(navinfo_mutex_);
                tunnel_.publish(kChannelNameNavinfo, &navinfo_);
                // std::cout<<"navinfo published "<<std::endl;
            }
            std::this_thread::sleep_until(time_point);
        }
    }

    void MessageManagerBase::PubLoopCaninfo(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            if(true)
            {
                std::lock_guard<std::mutex> lk(caninfo_mutex_);
                tunnel_.publish(kChannelNameCaninfo, &caninfo_);
                // std::cout<<"caninfo published "<<std::endl;
            }
            std::this_thread::sleep_until(time_point);

        }
    }

    void MessageManagerBase::PubLoopPredictedObject(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            if(true)
            {
                std::lock_guard<std::mutex> lk(predictedObject_mutex_);
                tunnel_.publish(kChannelNamePredictedObject, &predictedObject_);
            }
            std::this_thread::sleep_until(time_point);
        }
    }
    
    void MessageManagerBase::PubLoopTrafficLight(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            if(true)
            {
                std::lock_guard<std::mutex> lk(trafficLight_mutex_);
                tunnel_.publish(kChannelNameTrafficLight, &trafficLight_);
            }
            std::this_thread::sleep_until(time_point);
        }
    }
    
    // publish automatically with a certain frequency
    void MessageManagerBase::PublishAllAsync()
    {
        pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopNavinfo, this, kFreqNavinfo));

        pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopPredictedObject, this, kFreqPredictedObject));
        
        // pub_threads_.push_back(std::thread(
        //         &MessageManagerBase::PubLoopTrafficLight, this, kFreqTrafficLight));
        
        pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopCaninfo, this, kFreqCaninfo));

        for (auto &t : pub_threads_)
        {
            t.detach();
        }
    }


} // namespace tievsim
