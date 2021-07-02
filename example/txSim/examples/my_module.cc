#include "my_module.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <math.h>
#include <mutex>
#include <zcm/zcm-cpp.hpp>
using namespace std;

// since the example would process the two types of message, we need include the two corresponding protobuf headers here.
// if you need process other type of protobuf message that TADSim defines, include them too.
#include "traffic.pb.h"
#include "location.pb.h"
#include "control.pb.h"
#include "planStatus.pb.h"
#include "localLocation.pb.h"
#include "trajectory.pb.h"
#include "laneMarks.pb.h"

#include "WGS84UTM.h"

#include "tievmsg_zcm/MsgPredictedObjectTrajectoryList.hpp"
#include "tievmsg_zcm/MsgNavInfoSignal.hpp"
#include "tievmsg_zcm/MsgChassisCommandSignal.hpp"




#ifdef _WIN32
#define MODULE_API __declspec(dllexport)
#else
#define MODULE_API
#endif // _WIN32

#define SPLIT_LINE "======================================="
#define SHMEM_TOPIC "MY_SHMEM_TOPIC"

class Point2d
{
public:
    Point2d();
    Point2d(double x,double y);
    Point2d operator+(const Point2d& pt);
    friend Point2d operator*(const Point2d& pt, const double multiplier);
    friend Point2d operator*(const double multiplier, const Point2d& pt);
    double x;
    double y;
};
Point2d::Point2d():x(0),y(0){}
Point2d::Point2d(double x,double y) : x(x),y(y) {}
Point2d Point2d::operator+(const Point2d& pt)
{
    Point2d pt2;
    pt2.x = this->x + pt.x;
    pt2.y = this->y + pt.y;
    return pt2;
}
Point2d operator*(const Point2d& pt, const double multiplier)
{
    Point2d pt2;
    pt2.x = multiplier*pt.x;
    pt2.y = multiplier*pt.y;
    return pt2;
}
Point2d operator*(const double multiplier, const Point2d& pt)
{
    Point2d pt2;
    pt2.x = multiplier*pt.x;
    pt2.y = multiplier*pt.y;
    return pt2;
}

// frame counterclockwise theta(rad)
void CoorRotate(Point2d& pt, const double theta)
{
    double prex = pt.x, prey = pt.y;
    pt.x = prex * cos(theta)    + prey * sin(theta);
    pt.y = prex * (-sin(theta)) + prey * cos(theta);

}


mutex navinfo_mutex_;
MsgNavInfoSignal navinfo_;

mutex predictedObject_mutex_;
MsgPredictedObjectTrajectoryList predictedObject_;

mutex chassisCommand_mutex_;
MsgChassisCommandSignal chassisCommand_;


// !!! Attention: PackNavinfo must be called before PackPredictedObject due to the use of navinfo_ in PackPredictedObject
void PackNavinfo(tx_sim::StepHelper& helper)
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
    navinfo_.speed              = sqrt(pow(loc.velocity().x(),2)+pow(loc.velocity().y(),2));
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

void PackPredictedObject(tx_sim::StepHelper& helper)
{
    std::string traffic_payload;
    helper.GetSubscribedMessage(tx_sim::topic::kTraffic, traffic_payload);
    sim_msg::Traffic traffic;
    traffic.ParseFromString(traffic_payload);

    std::lock_guard<std::mutex> lk(predictedObject_mutex_);
    predictedObject_.time_stamp         = helper.timestamp();
    predictedObject_.data_source        = 1;
    predictedObject_.object_count       = traffic.staticobstacles_size() + traffic.dynamicobstacles_size();

    float timeHorizon = 5;
    float gap = 0.2;
    for(int i=0; i<traffic.dynamicobstacles_size(); ++i){
        PredictedObject obj;
        auto static_obj = traffic.mutable_dynamicobstacles(i);
        predictedObject_.time_stamp = static_obj->t();
        
        UTMCoor utmXY_obj;
        LatLonToUTMXY(static_obj->y(), static_obj->x(), utmXY_obj);
        // calculate relative utm_coordinate
        utmXY_obj.x -= navinfo_.utm_x;
        utmXY_obj.y -= navinfo_.utm_y;

        // coordinate rotation
        double theta = navinfo_.angle_head, obj_x, obj_y;
        Point2d objxy(utmXY_obj.x, utmXY_obj.y);
        CoorRotate(objxy, theta);

        obj.id          = static_obj->id();
        obj.type        = static_obj->type();
        obj.velocity    = 0;
        obj.accelerate  = 0;
        obj.heading     = static_obj->heading() - navinfo_.angle_head;
        obj.width       = static_obj->width();
        obj.length      = static_obj->length();
        Point2d corner_points[4];
        corner_points[0].x = 0.5 * obj.length;
        corner_points[0].y = 0.5 * obj.width;
        corner_points[1].x = -0.5 * obj.length;
        corner_points[1].y = 0.5 * obj.width;
        corner_points[2].x = 0.5 * obj.length;
        corner_points[2].y = -0.5 * obj.width;
        corner_points[3].x = -0.5 * obj.length;
        corner_points[3].y = -0.5 * obj.width;

        cout<<"initial corner_points\n";
        cout<<"corner_points: \n"<<corner_points[0].x<<" , "<<corner_points[0].y<<" | "<<corner_points[1].x<<" , "<<corner_points[1].y<<" | "
            <<corner_points[2].x<<" , "<<corner_points[2].y<<" | "<<corner_points[3].x<<" , "<<corner_points[3].y<<" | "<<endl;


        for(int i=0;i<4;++i){
            CoorRotate(corner_points[i], -obj.heading);
            corner_points[i] = corner_points[i] + objxy;
            obj.bounding_box[0][i] = corner_points[i].x;
            obj.bounding_box[1][i] = corner_points[i].y;
        }

        cout<<"after rotation corner_points\n";
        cout<<"ego heading: "<<navinfo_.angle_head<<endl;
        cout<<"obs heading: "<<static_obj->heading()<<endl;
        cout<<"rotation angle rad: "<<obj.heading<<endl;
        cout<<"corner_points: \n"<<corner_points[0].x<<" , "<<corner_points[0].y<<" | "<<corner_points[1].x<<" , "<<corner_points[1].y<<" | "
            <<corner_points[2].x<<" , "<<corner_points[2].y<<" | "<<corner_points[3].x<<" , "<<corner_points[3].y<<" | "<<endl;

        cout<<"relative coordinate: "<<objxy.x<<" , "<<objxy.y<<endl;

        obj.trajectory_point_num = 1 + timeHorizon/gap;  // predict 5s with a gap of 0.2s. 1 refers to current position
        Point2d UnitVec(cos(obj.heading), sin(obj.heading));
        for(int i=0;i<obj.trajectory_point_num;++i)
        {
            Point2d changePt = objxy +  UnitVec * obj.velocity * i*gap;
            vector<float> pt;
            pt.push_back(changePt.x);
            pt.push_back(changePt.y);
            obj.trajectory_point.push_back(pt);
        }

        predictedObject_.predicted_object.push_back(obj);
    }
    
    // for(int i=0; i<traffic.dynamicobstacles_size(); ++i){
    //     PredictedObject obj;
    //     auto obj_proto = traffic.mutable_dynamicobstacles(i);
    //     predictedObject_.time_stamp = obj_proto->t();
        
    //     UTMCoor utmXY_obj;
    //     LatLonToUTMXY(obj_proto->y(), obj_proto->x(), utmXY_obj);
    //     // calculate relative utm_coordinate
    //     utmXY_obj.x -= navinfo_.utm_x;
    //     utmXY_obj.y -= navinfo_.utm_y;

    //     // coordinate rotation
    //     double theta = navinfo_.angle_head, obj_x, obj_y;
    //     Point2d objxy(utmXY_obj.x, utmXY_obj.y);
    //     CoorRotate(objxy, theta);

    //     // Attention: global frame is east-north, vehicle frame is front-left
    //     objxy.y = -objxy.y;

    //     obj.id          = obj_proto->id();
    //     obj.type        = obj_proto->type();
    //     obj.velocity    = obj_proto->v();
    //     obj.accelerate  = obj_proto->acc();
    //     obj.heading     = obj_proto->heading() - navinfo_.angle_head;
    //     obj.width       = obj_proto->width();
    //     obj.length      = obj_proto->length();
    //     Point2d corner_points[4];
    //     corner_points[0].x = 0.5 * obj.length;
    //     corner_points[0].y = 0.5 * obj.width;
    //     corner_points[1].x = -0.5 * obj.length;
    //     corner_points[1].y = 0.5 * obj.width;
    //     corner_points[2].x = 0.5 * obj.length;
    //     corner_points[2].y = -0.5 * obj.width;
    //     corner_points[3].x = -0.5 * obj.length;
    //     corner_points[3].y = -0.5 * obj.width;

    //     for(int i=0;i<4;++i){
    //         CoorRotate(corner_points[i], -obj.heading);
    //         corner_points[i] = corner_points[i] + objxy;
    //         obj.bounding_box[0][i] = corner_points[i].x;
    //         obj.bounding_box[1][i] = corner_points[i].y;
    //     }

    //     obj.trajectory_point_num = 1 + timeHorizon/gap;  // predict 5s with a gap of 0.2s. 1 refers to current position
    //     Point2d UnitVec(cos(obj.heading), sin(obj.heading));
    //     for(int i=0;i<obj.trajectory_point_num;++i)
    //     {
    //         Point2d changePt = objxy +  UnitVec * obj.velocity * i*gap;
    //         obj.trajectory_point[0][i] = changePt.x;
    //         obj.trajectory_point[1][i] = changePt.y;
    //     }
    //     predictedObject_.predicted_object.push_back(obj);
    // }

}

void SendTopicControl(tx_sim::StepHelper& helper)
    {
        sim_msg::Control cot;
        std::string cot_payload;
        cot.Clear();
        if(chassisCommand_.car_gear_command == 0)
          cot.set_gear_cmd(sim_msg::Control::NO_CONTROL);
        if(chassisCommand_.car_gear_command == 1)
          cot.set_gear_cmd(sim_msg::Control::PARK);
        if(chassisCommand_.car_gear_command == 2)
          cot.set_gear_cmd(sim_msg::Control::REVERSE);
        if(chassisCommand_.car_gear_command == 3)
          cot.set_gear_cmd(sim_msg::Control::NEUTRAL);
        if(chassisCommand_.car_gear_command == 4)
          cot.set_gear_cmd(sim_msg::Control::DRIVE);
        cot.set_control_mode(sim_msg::Control::CM_AUTO_DRIVE);
        cot.set_contrl_type(sim_msg::Control::ACC_CONTROL);
        cot.mutable_acc_cmd()->set_acc(chassisCommand_.longitudinal_acceleration_command);
        cot.mutable_acc_cmd()->set_front_wheel_angle(chassisCommand_.steer_wheel_angle_command);

        cot.SerializeToString(&cot_payload);
        helper.PublishMessage(tx_sim::topic::kControl, cot_payload);
        std::cout<<"kControl published\n";
    }


MyModule::MyModule() {}


MyModule::~MyModule() {
  std::cout << "MyModule destroyed." << std::endl;
}


void MyModule::Init(tx_sim::InitHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;
  cout<<"start initializing ... \n";



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

    helper.Publish(tx_sim::topic::kControl);
    helper.Subscribe(tx_sim::topic::kTraffic);
    helper.Subscribe(tx_sim::topic::kLocation);
    helper.Subscribe(tx_sim::topic::kControl);
    helper.Subscribe(tx_sim::topic::kPlanStatus);    
    helper.Subscribe(tx_sim::topic::kTrajectory);   
    helper.Subscribe(tx_sim::topic::kLaneMark);   
};


void MyModule::Reset(tx_sim::ResetHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;


  // first we should reset(clear) any data status before starting a new scenario.
  step_count_ = 0;
  last_timestamp_ = 0;
  start_x_ = 0, start_y_ = 0, cur_x_ = 0, cur_y_ = 0;

  // here we could get some information(e.g. the map which the scenario runs on, suggested local origin coordinate
  // and the destination of the ego car) of the current scenario.
  tx_sim::Vector3d o = helper.map_local_origin(), d = helper.ego_destination();
  std::vector<tx_sim::Vector3d> path = helper.ego_path();
  cout<<"path.size: "<<path.size()<<endl;

  sim_msg::Location start_location;
  // get start location in reset or in first step. both works.
  start_location.ParseFromString(helper.ego_start_location());
  std::cout << "Reset scenario with parameters:\n"
    << "hadmap path: " << helper.map_file_path() << "\n"
    << "the local origin coordinate of the map: (" << o.x << ", " << o.y << ", " << o.z << ")" << "\n"
    << "ego car's destination (" << d.x << ", " << d.y << ")" << std::endl;
  // we can also get the speed limit(max) info of the ego car in current scenario.
  double speed_limit = helper.ego_speed_limit();


  ofstream outfile;
  outfile.open("/home/yxj/Projects/txsim-zcm/path.txt");
  double wgsLat1,wgsLon1,wgsLat2,wgsLon2,heading;
  UTMCoor utmXY1,utmXY2;
  outfile << "Id Lon Lat utmX utmY heading curvature mode SpeedMode EventMode OppositeSideMode LangeNum LaneSeq LaneWidth"<<endl;
  for(int i=0; i<path.size()-1; ++i){
    wgsLat1 = path[i].y;
    wgsLon1 = path[i].x;
    wgsLat2 = path[i+1].y;
    wgsLon2 = path[i+1].x;
    LatLonToUTMXY(wgsLat1,wgsLon1,utmXY1,30);
    LatLonToUTMXY(wgsLat2,wgsLon2,utmXY2,30);
    if(utmXY2.x - utmXY1.x != 0)
      heading = atan2(utmXY2.y - utmXY1.y, utmXY2.x - utmXY1.x);
    else{
      if(utmXY2.y - utmXY1.y > 0) heading = pi/2;
      if(utmXY2.y - utmXY1.y < 0) heading = -pi/2;
      if(utmXY2.y - utmXY1.y == 0) heading = 0;
    }

    outfile<<i<<" ";
    outfile.precision(14);
    outfile<<wgsLon1<<" "<<wgsLat1<<" "<<utmXY1.x<<" "<<utmXY2.y<<" "<<heading<<" "<<"0.00000000000000 0 0 0 0 0 0 0.00000000000000"<<endl;
  }
  wgsLat1 = path.back().y;
  wgsLon1 = path.back().x;
  LatLonToUTMXY(wgsLat1,wgsLon1,utmXY1,30);
  outfile<<path.size()-1<<" ";
  outfile.precision(14);
  outfile<<wgsLon1<<" "<<wgsLat1<<" "<<utmXY1.x<<" "<<utmXY2.y<<" "<<heading<<" "<<"0.00000000000000 0 0 0 0 0 0 0.00000000000000"<<endl;
  
  outfile.close();

  cout<<"path recorded in txt"<<endl;

  // we could throw an exception which must inherited from std::exception type, if there is an error.
  if (helper.map_file_path() == "/some/map/that/we/cannot/load") {
    // std::runtime_error is a subclass of the std::exception.
    throw std::runtime_error("the map " + helper.map_file_path() + " could not load.");
  }
};


void MyModule::Step(tx_sim::StepHelper& helper) {
  std::cout << SPLIT_LINE << std::endl;

  zcm::ZCM tunnel("ipc");
  cout<<"zcm created !!!"<<endl;

  PackNavinfo(helper);
  PackPredictedObject(helper);
  SendTopicControl(helper);

  // 1. get current simulation timestamp.
  double time_stamp = helper.timestamp();
  std::cout << "time stamp: " << time_stamp << "\n";


  helper.GetSubscribedMessage(tx_sim::topic::kControl, payload_);
  sim_msg::Control cot_;
  cot_.ParseFromString(payload_);

  //double throt = cot_.PedalControl().throttle();
  // sim_msg::Control_PedalControl *pc = cot_.PedalControl();
  std::cout<<"output gearmode "<<cot_.gear_cmd()<<std::endl;
  std::cout<<"output controlmode "<<cot_.control_mode()<<std::endl;
  std::cout<<"output controltype "<<cot_.contrl_type()<<std::endl;
  std::cout<<"output acc "<<cot_.mutable_acc_cmd()->acc()<<std::endl;
  std::cout<<"output steering "<<cot_.mutable_acc_cmd()->front_wheel_angle()<<std::endl;
  



  last_timestamp_ = time_stamp;


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

