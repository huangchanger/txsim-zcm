#include "my_module.h"
#include "MessageManager.h"

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

#define pi 3.14159265358979

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

txsim::MessageManager msgm("ipc");

// ***************************************************************************************
void MyModule::Init(tx_sim::InitHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  cout<<"start initializing ... \n";
  cout<<"version 14"<<endl;

  msgm.PublishAllAsync();
  msgm.SubscribeAll();

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
  helper.Subscribe(tx_sim::topic::kVehicleState);
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
  tx_sim::Vector3d start_pt = path.front();
  tx_sim::Vector3d end_pt = path.back();
  UTMCoor start_utm, end_utm;
  LatLonToUTMXY(start_pt.y,start_pt.x,start_utm);
  LatLonToUTMXY(end_pt.y,end_pt.x,end_utm);

  msgm.start_pt2d.x = start_utm.x;
  msgm.start_pt2d.y = start_utm.y;

  Point2d start_interp(start_utm.x,start_utm.y);
  Point2d end_interp(end_utm.x, end_utm.y);
  Point2d end_start = end_interp - start_interp;
  std::vector<Point2d> path_interp;
  double gap = 0.2;
  double path_length = end_start.norm();
  Point2d path_gap = end_start / path_length * gap;

  for(int i=0;i<(int)(path_length/gap);++i)
  {
    path_interp.push_back(start_interp + i*path_gap);
  }
  path_interp.push_back(end_interp);

  for(int i=0;i<path_interp.size();++i)
  {
    path_interp[i] = path_interp[i] - msgm.start_pt2d;
  }
  

  ofstream outfile;
  outfile.open("/home/yxj/Projects/txsim-zcm/path.txt");
  double wgsLat1,wgsLon1,wgsLat2,wgsLon2,heading;
  Point2d pt1,pt2;
  outfile << "Id Lon Lat utmX utmY heading curvature mode SpeedMode EventMode OppositeSideMode LangeNum LaneSeq LaneWidth"<<endl;

  for(int i=0;i<path_interp.size()-1;++i)
  {
    pt1 = path_interp[i];
    pt2 = path_interp[i+1];
    if(pt2.x - pt1.x != 0)
    heading = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
    else{
      if(pt2.y - pt1.y > 0) heading = pi/2;
      if(pt2.y - pt1.y < 0) heading = -pi/2;
      if(pt2.y - pt1.y == 0) heading = 0;
    }

    outfile<<i<<" ";
    outfile.precision(14);
    outfile<<0<<" "<<0<<" "<<pt1.x<<" "<<pt1.y<<" "<<heading<<" "<<"0.00000000000000 0 0 0 0 0 0 0.00000000000000"<<endl;

  }

  outfile<<path_interp.size()-1<<" ";
  outfile.precision(14);
  pt1 = path_interp.back();
  outfile<<0<<" "<<0<<" "<<pt1.x<<" "<<pt1.y<<" "<<heading<<" "<<"0.00000000000000 0 0 0 0 0 0 0.00000000000000"<<endl;


  outfile.close();



  cout<<"path recorded in /home/yxj/Projects/txsim-zcm/path.txt"<<endl;

};


// ***************************************************************************************
void MyModule::Step(tx_sim::StepHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  std::cout<<"timestamp: "<<helper.timestamp()<<endl;

  msgm.SendTopicControl(helper);
  msgm.PackNavinfo(helper);
  msgm.PackPredictedObject(helper);
  msgm.PackCaninfo(helper);
  msgm.PackTrafficLight(helper);
  
  std::cout<<"step "<<step_count_<<" finished ! "<<std::endl;
  
  step_count_++;
  // if (step_count_ >= max_step_count_) {
  //   helper.StopScenario("we have reached our destination.");
  // }
};


void MyModule::Stop(tx_sim::StopHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  std::cout << name_ << " module stopped." << std::endl;

  msgm.stopSubandPub();

  // here we could send out any feedback data we want the TADSim to display on front-end UI.
  helper.set_feedback("stepCounts", std::to_string(step_count_));
};


TXSIM_MODULE(MyModule)

