#include "MessageManager/MessageManagerBase.h"


using std::string;
using std::vector;

namespace txsim
{

    MessageManagerBase::MessageManagerBase(
        const string &url, const string &config_file,
        const string &record_path)
        : tunnel_(url), need_stop_(false),
          location_()
    { };

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

    void MessageManagerBase::LOCATIONHandler(
        const zcm::ReceiveBuffer *rbuf,
        const std::string &chan,
        const icumsg::structLOCATION *msg)
    {
        std::lock_guard<std::mutex> lock(location_mutex_);
        location_.t = msg->t;
        //printf("msg manager base aimsteer: %d, aimspeed: %d\n", cancontrol_.aimsteer, cancontrol_.aimspeed);
    }


    void MessageManagerBase::SubscribeAll()
    {
        auto sub1 = tunnel_.subscribe(
            kChannelNameCancontrol, &MessageManagerBase::LOCATIONHandler, this);

        sub_threads_.push_back(sub1);

        tunnel_.start();
    }


    void MessageManagerBase::PublishCaninfo() const
    {
        tunnel_.publish(kChannelNameCaninfo, &location_);
    }


    void MessageManagerBase::PublishCaninfoWithLock() const
    {
        std::lock_guard<std::mutex> caninfo_lock(location_mutex_);
        PublishCaninfo();
    }


    void MessageManagerBase::PublishAll() const
    {
        if (kSwitchCaninfo)
        {
            PublishCaninfo();
        }
    }

    void MessageManagerBase::PubLoopCaninfo(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            PublishCaninfoWithLock();
            std::this_thread::sleep_until(time_point);
        }
    }


    void MessageManagerBase::PublishAllAsync()
    {
        if (kSwitchCaninfo)
        {
            pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopCaninfo, this, kFreqCaninfo));
        }

        for (auto &t : pub_threads_)
        {
            t.detach();
        }
    }
} // namespace tievsim
