#include "MessageManagerBase.h"


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
            kChannelLocation, &MessageManagerBase::LOCATIONHandler, this);

        sub_threads_.push_back(sub1);

        tunnel_.start();
    }


    // publishAllAsync -> PubLoopChannelname -> PublishChannelnameWithLock -> PublishChannelname
    void MessageManagerBase::PublishLocation() const
    {
        tunnel_.publish(kChannelNameLocation, &location_);
    }


    void MessageManagerBase::PublishLocationWithLock() const
    {
        std::lock_guard<std::mutex> caninfo_lock(location_mutex_);
        PublishLocation();
    }

    void MessageManagerBase::PubLoopLocation(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);
            PublishLocationWithLock();
            std::this_thread::sleep_until(time_point);
        }
    }

    // publish automatically with a certain frequency
    void MessageManagerBase::PublishAllAsync()
    {
        if (kSwitchCaninfo)
        {
            pub_threads_.push_back(std::thread(
                &MessageManagerBase::PubLoopLocation, this, kFreqLocation));
        }

        for (auto &t : pub_threads_)
        {
            t.detach();
        }
    }

    // publish one time
    void MessageManagerBase::PublishAll() const
    {
        if (kSwitchCaninfo)
        {
            PublishLocation();
        }
    }

} // namespace tievsim
