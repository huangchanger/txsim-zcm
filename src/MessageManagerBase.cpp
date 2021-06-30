#include "MessageManagerBase.h"


using std::string;
using std::vector;

namespace txsim
{

    MessageManagerBase::MessageManagerBase(const string &url): tunnel_(url), need_stop_(false), chassisCommand_(), navinfo_()
    {
        kChannelNameChassisCommand = "ChassisCommand";
        kChannelNameNavinfo = "Navinfo";
        kFreqNavinfo = 20;
        kSwitchNavinfo = 1;
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

 // receive zcm
    void MessageManagerBase::ChassisCommandHandler(
        const zcm::ReceiveBuffer *rbuf,
        const std::string &chan,
        const MsgChassisCommandSignal *msg)
    {
        std::lock_guard<std::mutex> lock(chassisCommand_mutex_);
        chassisCommand_.timestamp = msg->timestamp;
        chassisCommand_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
        chassisCommand_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
        chassisCommand_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
        //printf("msg manager base aimsteer: %d, aimspeed: %d\n", cancontrol_.aimsteer, cancontrol_.aimspeed);
    }


    void MessageManagerBase::SubscribeAll()
    {
        std::cout<<"subscribe all "<<std::endl;
        auto sub1 = tunnel_.subscribe(
            kChannelNameChassisCommand, &MessageManagerBase::ChassisCommandHandler, this);

        sub_threads_.push_back(sub1);

        tunnel_.start();
    }


    // publishAllAsync -> PubLoopChannelname -> PublishChannelnameWithLock -> PublishChannelname
    void MessageManagerBase::PublishNavinfo() const
    {
        std::cout<<"publish navinfo"<<std::endl;
        tunnel_.publish(kChannelNameNavinfo, &navinfo_);
        std::cout<<"navinfo zcm message published ! "<<std::endl;
        std::cout<<navinfo_.longitude<<" , "<<navinfo_.latitude<<std::endl;
    }


    void MessageManagerBase::PublishNavinfoWithLock() const
    {
        std::lock_guard<std::mutex> caninfo_lock(navinfo_mutex_);
        PublishNavinfo();
    }

    void MessageManagerBase::PubLoopNavinfo(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            PublishNavinfoWithLock();
            std::this_thread::sleep_until(time_point);
        }
    }

    // publish automatically with a certain frequency
    void MessageManagerBase::PublishAllAsync()
    {
        if (kSwitchNavinfo)
        {
            pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopNavinfo, this, kFreqNavinfo));
        }

        for (auto &t : pub_threads_)
        {
            t.detach();
        }
    }

    // publish one time
    void MessageManagerBase::PublishAll() const
    {
        if (kSwitchNavinfo)
        {
            PublishNavinfo();
        }
    }

} // namespace tievsim
