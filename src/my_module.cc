#include "my_module.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;

// since the example would process the two types of message, we need include the two corresponding protobuf headers here.
// if you need process other type of protobuf message that TADSim defines, include them too.

#include "WGS84UTM.h"

#include "basic.pb.h"
#include "control.pb.h"
#include "header.pb.h"
#include "laneMarks.pb.h"
#include "location.pb.h"
#include "planStatus.pb.h"
#include "traffic.pb.h"
#include "trajectory.pb.h"

#include <zcm/zcm-cpp.hpp>

#ifdef _WIN32
#define MODULE_API __declspec(dllexport)
#else
#define MODULE_API
#endif // _WIN32

#define SPLIT_LINE "======================================="
#define SHMEM_TOPIC "MY_SHMEM_TOPIC"


MyModule::MyModule() {}


MyModule::~MyModule() {
  std::cout << "MyModule destroyed." << std::endl;
}



// ***************************************************************************************
void MyModule::Init(tx_sim::InitHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  cout<<"start initializing ... \n";

  // zcm::ZCM tunnel_("ipc");
  //tunnel_.start();

  // txsim::MessageManager msgm("ipc");
  cout<<"msgm created"<<endl;

  // msgm.PublishAllAsync();
  // msgm.SubscribeAll();

  // get user defined initiation parameters.
  // if we defined the parameters in TADSim UI, override the default values here.
  std::string value = helper.GetParameter("max_step_count");
  if (!value.empty()) {
    max_step_count_ = std::stoi(value);
    std::cout << "Init with parameter max_step_count = " << max_step_count_ << std::endl;
  }
  value = helper.GetParameter("velocity");
  if (!value.empty()) {
    step_velocity_ = std::stod(value);
    std::cout << "Init with parameter velocity = " << step_velocity_ << std::endl;
  }

  puber_ = !helper.GetParameter("pub").empty();

  helper.Publish(tx_sim::topic::kControl);

  helper.Subscribe(tx_sim::topic::kTraffic);
  helper.Subscribe(tx_sim::topic::kLocation);
  helper.Subscribe(tx_sim::topic::kControl);
  helper.Subscribe(tx_sim::topic::kPlanStatus);    
  helper.Subscribe(tx_sim::topic::kTrajectory);   
  helper.Subscribe(tx_sim::topic::kLaneMark);   
};


// *******************************************************************************************
void MyModule::Reset(tx_sim::ResetHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;


  // first we should reset(clear) any data status before starting a new scenario.
  step_count_ = 0;
  last_timestamp_ = 0;
  start_x_ = 0, start_y_ = 0, cur_x_ = 0, cur_y_ = 0;

  tx_sim::Vector3d o = helper.map_local_origin(), d = helper.ego_destination();
  std::vector<tx_sim::Vector3d> path = helper.ego_path();
  cout<<"path.size: "<<path.size()<<endl;



  cout<<"path recorded in txt"<<endl;

};


// ***************************************************************************************
void MyModule::Step(tx_sim::StepHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;

  // msgm.PackNavinfo(helper);
  // msgm.SendTopicControl(helper);

  // 1. get current simulation timestamp.
  double time_stamp = helper.timestamp();
  std::cout << "time stamp: " << time_stamp << "\n";
  std::cout <<"changed"<<std::endl;

  // 2. get messages we subscribed.
  // helper.GetSubscribedMessage(tx_sim::topic::kLocation, payload_);
  // sim_msg::Location loc;
  // loc.ParseFromString(payload_);
  // cur_x_ = loc.position().x();
  // cur_y_ = loc.position().y();
  // std::cout << std::fixed << std::setprecision(12) << "received location: x -> " << cur_x_ << " y -> " << cur_y_ << std::endl;
  // std::cout<<"velocity: "<<loc.velocity().x()<<" , "<<loc.velocity().y()<<std::endl;
  // std::cout<<"acc: "<<loc.acceleration().x()<<" , "<<loc.acceleration().y()<<std::endl;


  // helper.GetSubscribedMessage(tx_sim::topic::kTrajectory, payload_);
  // sim_msg::Trajectory traj;
  // traj.ParseFromString(payload_);
  // std::cout<<"trajectory size: "<< traj.point_size() <<std::endl;
  

  // for(int i=0; i<traj.point_size(); ++i){
  //   const sim_msg::TrajectoryPoint &pt = traj.point(i);
  //   std::cout<<"( "<<pt.x()<<" , "<<pt.y()<<" , "<<pt.v()<<" ) "<<std::endl;
  // }

  // traj.SerializeToString(&payload_);

  // helper.GetSubscribedMessage(tx_sim::topic::kLaneMark, payload_);
  // sim_msg::LaneMarks lanemarks;
  // lanemarks.ParseFromString(payload_);
  // std::cout<<"lanemarks_left size: "<< lanemarks.left_size() <<std::endl;
  // std::cout<<"lanemarks_right size: "<< lanemarks.right_size() <<std::endl;

  // puber_ = 1;

  // if (true) {
  //   // 3. here should put the actual user algorithm, do some computing according to the subscribed messages we received.
  //   // for explanatory simplicity, it only moves a little by a constant velocity, no matter what happens.
  //   double move_distance = step_velocity_ * (time_stamp - last_timestamp_); // s = v * t
  //   // std::cout << "ego car moved " << move_distance << std::endl;
  //   double next_x = cur_x_ + move_distance, next_y = cur_y_ + move_distance;

  //   std::cout<<"position: "<<loc.position().x()<<" , "<<loc.position().y()<<" , "<<loc.position().z()<<std::endl;
  //   std::cout<<"velocity: "<<loc.velocity().x()<<" , "<<loc.velocity().y()<<" , "<<loc.velocity().z()<<std::endl;
  //   std::cout<<"angular: "<<loc.angular().x()<<" , "<<loc.angular().y()<<" , "<<loc.angular().z()<<std::endl;
  //   std::cout<<"rpy: "<<loc.rpy().x()<<" , "<<loc.rpy().y()<<" , "<<loc.rpy().z()<<std::endl;


  //   // 4. put our results into output messages and publish them.
  //   loc.mutable_position()->set_x(next_x);
  //   loc.mutable_position()->set_y(next_y);
  //   // since the message we want to publish is a protobuf message type and the PublishMessage API only accepts the std::string type,
  //   // we need serialize it to std::string using google::protobuf::MessageLite::SerializeToString method.
  //   loc.SerializeToString(&payload_);


  //   // 5. publish control parameters to drive the vehicle
  //   helper.GetSubscribedMessage(tx_sim::topic::kControl, payload_);
  //   sim_msg::Control cot_;
  //   cot_.ParseFromString(payload_);

  //   //double throt = cot_.PedalControl().throttle();
  //   // sim_msg::Control_PedalControl *pc = cot_.PedalControl();
  //   std::cout<<"output gearmode "<<cot_.gear_cmd()<<std::endl;
  //   std::cout<<"output controlmode "<<cot_.control_mode()<<std::endl;
  //   std::cout<<"output controltype "<<cot_.contrl_type()<<std::endl;
  //   std::cout<<"output pedalcontrol "<<cot_.mutable_pedal_cmd()->throttle()<<std::endl;
  //   std::cout<<"output pedalcontrol "<<cot_.mutable_pedal_cmd()->brake()<<std::endl;
  //   std::cout<<"output acccontrol "<<cot_.mutable_acc_cmd()->acc()<<std::endl;
    

  //   cot_.set_gear_cmd(sim_msg::Control::DRIVE);
  //   cot_.set_control_mode(sim_msg::Control::CM_AUTO_DRIVE);
  //   cot_.set_contrl_type(sim_msg::Control::ACC_CONTROL);
  //   // cot_.mutable_pedal_cmd()->set_throttle(50);
  //   // cot_.mutable_pedal_cmd()->set_brake(0);
  //   // cot_.mutable_pedal_cmd()->set_steer(0);
  //   cot_.mutable_acc_cmd()->set_acc(2);
  //   cot_.mutable_acc_cmd()->set_front_wheel_angle(-5);

  //   cot_.SerializeToString(&payload_);
  //   helper.PublishMessage(tx_sim::topic::kControl, payload_);
  //   std::cout<<"kControl published\n";


  //   helper.GetSubscribedMessage(tx_sim::topic::kPlanStatus, payload_);
  //   sim_msg::PlanStatus plans_;
  //   plans_.ParseFromString(payload_);
  //   std::cout<<"output expectacc "<<plans_.mutable_expect_acc()->acc()<<std::endl;

  //   plans_.mutable_expect_acc()->set_acc(2.5);
  //   plans_.SerializeToString(&payload_);


  //   last_timestamp_ = time_stamp;

  // this is just for simplicity. we should stop the scenario when we reached the destination point
  // which we received in Reset() by helper.ego_destination() method.
  step_count_++;
  if (step_count_ >= max_step_count_) {
    helper.StopScenario("we have reached our destination.");
  }
};


void MyModule::Stop(tx_sim::StopHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  std::cout << name_ << " module stopped." << std::endl;
  // here we could send out any feedback data we want the TADSim to display on front-end UI.
  helper.set_feedback("stepCounts", std::to_string(step_count_));
};


TXSIM_MODULE(MyModule)

