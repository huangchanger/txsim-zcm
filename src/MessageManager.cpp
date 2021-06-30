#include "MessageManager.h"
#include "my_module.h"
#include "simutils.hpp"


using std::list;
using std::string;
using std::vector;

namespace txsim
{
    using namespace simutils;

    MessageManager::MessageManager(const string &url)
        : MessageManagerBase(url)
    { }


    void MessageManager::PackNavinfo(tx_sim::StepHelper& helper)
    {
        std::string loc_payload;
        helper.GetSubscribedMessage(tx_sim::topic::kLocation, loc_payload);
        sim_msg::Location loc;
        loc.ParseFromString(loc_payload);

        UTMCoor utmXY;
        LatLonToUTMXY(loc.position().y(), loc.position().x(),utmXY);
        std::lock_guard<std::mutex> l(navinfo_mutex_);
        navinfo_.timestamp          = loc.t();
        navinfo_.longitude          = loc.position().x();
        navinfo_.latitude           = loc.position().y();
        navinfo_.altitude           = loc.position().z();
        navinfo_.utm_x              = utmXY.x;
        navinfo_.utm_y              = utmXY.y;
        navinfo_.angle_head         = loc.rpy().z();
        navinfo_.angle_pitch        = loc.rpy().y();
        navinfo_.angle_roll         = loc.rpy().x();
        navinfo_.angular_vel_z      = loc.angular().z();
        navinfo_.speed              = sqrt(pow(loc.velocity().x(),2)+pow(loc.velocity().y(),2)+pow(loc.velocity().z(),2));
        navinfo_.velocity_east      = loc.velocity().x();
        navinfo_.velocity_north     = loc.velocity().y();
        navinfo_.curvature          = 0;
        navinfo_.RTK_status         = 0;
        navinfo_.HPOS_accuracy      = 0;
        navinfo_.is_reckoning_vaild = 0;
        navinfo_.gps_num_satellites = 0;
        navinfo_.acceleration_x     = loc.acceleration().x();
        navinfo_.acceleration_y     = loc.acceleration().y();
        std::cout<<"packNavinfo succeed: "<<navinfo_.longitude<<" , "<<navinfo_.latitude<<std::endl;
    }

    void MessageManager::SendTopicControl(tx_sim::StepHelper& helper)
    {
        sim_msg::Control cot;
        std::string cot_payload;
        cot.Clear();
        cot.set_gear_cmd(sim_msg::Control::DRIVE);
        cot.set_control_mode(sim_msg::Control::CM_AUTO_DRIVE);
        cot.set_contrl_type(sim_msg::Control::ACC_CONTROL);
        cot.mutable_acc_cmd()->set_acc(2);
        cot.mutable_acc_cmd()->set_front_wheel_angle(0);

        cot.SerializeToString(&cot_payload);
        helper.PublishMessage(tx_sim::topic::kControl, cot_payload);
        std::cout<<"kControl published\n";
    }




} // namespace tievsim