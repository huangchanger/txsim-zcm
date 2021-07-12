#include "MessageManager.h"
#include "my_module.h"
#include "simutils.hpp"
#include <string>

#include "basic.pb.h"
#include "control.pb.h"
#include "header.pb.h"
#include "laneMarks.pb.h"
#include "location.pb.h"
#include "planStatus.pb.h"
#include "traffic.pb.h"
#include "trajectory.pb.h"
#include "vehState.pb.h"

#include "WGS84UTM.h"

using namespace std;

using std::list;
using std::string;
using std::vector;

namespace txsim
{
    using namespace simutils;

    MessageManager::MessageManager(const string &url)
        : MessageManagerBase(url)
    { }



// !!! Attention: PackNavinfo must be called before PackPredictedObject due to the use of navinfo_ in PackPredictedObject
void MessageManager::PackNavinfo(tx_sim::StepHelper& helper)
{
    std::string loc_payload;

    helper.GetSubscribedMessage(tx_sim::topic::kLocation, loc_payload);

    sim_msg::Location loc;
    loc.ParseFromString(loc_payload);

    UTMCoor utmXY;
    LatLonToUTMXY(loc.position().y(), loc.position().x(),utmXY);
    
    Point2d pt(utmXY.x,utmXY.y);
    pt = pt - start_pt2d;

    std::lock_guard<std::mutex> l(navinfo_mutex_);
    navinfo_.timestamp          = loc.t();
    navinfo_.longitude          = loc.position().x();
    navinfo_.latitude           = loc.position().y();
    navinfo_.altitude           = loc.position().z();
    navinfo_.utm_x              = pt.x;
    navinfo_.utm_y              = pt.y;
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

    cout<<"navinfo xy: "<<navinfo_.utm_x << " , "<<navinfo_.utm_y<<endl;
    std::cout<<"packNavinfo succeed: "<<std::endl;
}

template<typename T>
void MessageManager::pushObject(T obj_proto, string obj_type, int id)
{
    predictedObject_.time_stamp = obj_proto->t();

    PredictedObject obj;
    
    UTMCoor utmXY_obj;
    LatLonToUTMXY(obj_proto->y(), obj_proto->x(), utmXY_obj);

    //calculate utm_coordinate relative to origin
    Point2d utmXY_rel2origin(utmXY_obj.x,utmXY_obj.y);
    utmXY_rel2origin = utmXY_rel2origin - start_pt2d;
    utmXY_obj.x = utmXY_rel2origin.x;
    utmXY_obj.y = utmXY_rel2origin.y;

    //calculate utm_coordinate relative to vehicle
    // utmXY_obj.x -= navinfo_.utm_x;
    // utmXY_obj.y -= navinfo_.utm_y;

    // coordinate rotation
    double theta = navinfo_.angle_head, obj_x, obj_y;
    Point2d objxy(utmXY_obj.x, utmXY_obj.y);
    // CoorRotate(objxy, theta); //rotation for vehicle frame

    obj.id          = obj_proto->id();
    obj.type        = obj_proto->type();

    if(obj_type == "static")
    {
        obj.velocity    = 0;
        obj.accelerate  = 0;
    }
    else
    {
        obj.velocity    = obj_proto->v();
        obj.accelerate  = obj_proto->acc();
    }

    obj.heading     = obj_proto->heading();
    // obj.heading     = obj_proto->heading() - navinfo_.angle_head;
    obj.width       = obj_proto->width();
    obj.length      = obj_proto->length();
    Point2d corner_points[4];
    corner_points[0].x = 0.5 * obj.length;
    corner_points[0].y = 0.5 * obj.width;
    corner_points[1].x = -0.5 * obj.length;
    corner_points[1].y = 0.5 * obj.width;
    corner_points[2].x = 0.5 * obj.length;
    corner_points[2].y = -0.5 * obj.width;
    corner_points[3].x = -0.5 * obj.length;
    corner_points[3].y = -0.5 * obj.width;

    // cout<<"initial corner_points\n";
    // cout<<"corner_points: \n"<<corner_points[0].x<<" , "<<corner_points[0].y<<" | "<<corner_points[1].x<<" , "<<corner_points[1].y<<" | "
    //     <<corner_points[2].x<<" , "<<corner_points[2].y<<" | "<<corner_points[3].x<<" , "<<corner_points[3].y<<" | "<<endl;


    for(int i=0;i<4;++i){
        CoorRotate(corner_points[i], -obj.heading);
        corner_points[i] = corner_points[i] + objxy;
        obj.bounding_box[0][i] = corner_points[i].x;
        obj.bounding_box[1][i] = corner_points[i].y;
    }


    cout<<"after rotation corner_points\n";
    cout<<"ego heading: "<<navinfo_.angle_head<<endl;
    cout<<"obs heading: "<<obj.heading<<endl;
    cout<<"rotation angle rad: "<<obj.heading<<endl;
    cout<<"corner_points: \n"<<corner_points[0].x<<" , "<<corner_points[0].y<<" | "<<corner_points[1].x<<" , "<<corner_points[1].y<<" | "
        <<corner_points[2].x<<" , "<<corner_points[2].y<<" | "<<corner_points[3].x<<" , "<<corner_points[3].y<<" | "<<endl;

    cout<<"relative coordinate: "<<objxy.x<<" , "<<objxy.y<<endl;

    float timeHorizon = 5;
    float gap = 0.2;

    obj.trajectory_point_num = int(1 + timeHorizon/gap);  // predict 5s with a gap of 0.2s. 1 refers to current position
    Point2d UnitVec(cos(obj.heading), sin(obj.heading));

    obj.trajectory_point.resize(2);
    obj.trajectory_point[0].resize(obj.trajectory_point_num);
    obj.trajectory_point[1].resize(obj.trajectory_point_num);
    for(int i=0;i<obj.trajectory_point_num;++i)
    {
        Point2d changePt = objxy +  UnitVec * obj.velocity * i*gap;
        // obj.trajectory_point[i].resize(2);
        // obj.trajectory_point[i][0] = changePt.x;
        // obj.trajectory_point[i][1] = changePt.y;

        obj.trajectory_point[0][i] = changePt.x;
        obj.trajectory_point[1][i] = changePt.y;
        // vector<float> pt;
        // pt.resize(2);
        // pt[0] = changePt.x;
        // pt[1] = changePt.y;
        // obj.trajectory_point[i] = pt;
    }

    // predictedObject_.predicted_object.push_back(obj);
    predictedObject_.predicted_object[id] = obj;
}

void MessageManager::PackPredictedObject(tx_sim::StepHelper& helper)
{
    std::string traffic_payload;
    helper.GetSubscribedMessage(tx_sim::topic::kTraffic, traffic_payload);
    sim_msg::Traffic traffic;
    traffic.ParseFromString(traffic_payload);

    std::lock_guard<std::mutex> lk(predictedObject_mutex_);

    predictedObject_.time_stamp         = helper.timestamp();
    predictedObject_.data_source        = 1;
    predictedObject_.object_count       = 
        traffic.staticobstacles_size() + traffic.dynamicobstacles_size() + traffic.cars_size();
    
    predictedObject_.predicted_object.clear();
    predictedObject_.predicted_object.resize(predictedObject_.object_count);
    // double obj_count = traffic.staticobstacles_size() + traffic.dynamicobstacles_size();
    cout<<"static dynamic cars size: "<<traffic.staticobstacles_size()
        <<" , "<<traffic.dynamicobstacles_size()
        <<" , "<<traffic.cars_size() <<endl;

    for(int i=0; i<traffic.staticobstacles_size(); ++i){
        pushObject(traffic.mutable_staticobstacles(i), "static", i);
    }
    
    for(int i=0; i<traffic.dynamicobstacles_size(); ++i){
        pushObject(traffic.mutable_dynamicobstacles(i), "dynamic", traffic.staticobstacles_size()+i);
    }

    for(int i=0; i<traffic.cars_size(); ++i){
        pushObject(traffic.mutable_cars(i), "dynamic", traffic.dynamicobstacles_size()+traffic.staticobstacles_size()+i);
    }

    // predictedObject_.object_count = predictedObject_.predicted_object.size();

    cout<<"predicted_object size: "<<predictedObject_.predicted_object.size()<<endl;
    for(int i=0;i<predictedObject_.predicted_object.size();++i)
    {
        auto obj = predictedObject_.predicted_object[i];
        cout<<"obj "<<i<<" detected "<<endl;
        for(int j=0;j<obj.trajectory_point_num;++j)
        {
            cout<<obj.trajectory_point[0][j]<<" , "<<obj.trajectory_point[1][j]<<endl;
        }

    }
    

    cout<<"packPredictedObject succeed: "<<endl;
}

void MessageManager::SendTopicControl(tx_sim::StepHelper& helper)
    {


        sim_msg::Control cot;
        std::string cot_payload;
        // helper.GetSubscribedMessage(tx_sim::topic::kControl, cot_payload);
        // cot.ParseFromString(cot_payload);
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


        // test
        // cot.set_gear_cmd(sim_msg::Control::DRIVE);
        // cot.set_control_mode(sim_msg::Control::CM_AUTO_DRIVE);
        // cot.set_contrl_type(sim_msg::Control::ACC_CONTROL);
        // cot.mutable_acc_cmd()->set_acc(1);
        // cot.mutable_acc_cmd()->set_front_wheel_angle(0);

        cot.SerializeToString(&cot_payload);
        helper.PublishMessage(tx_sim::topic::kControl, cot_payload);
        std::cout<<"kControl published\n";
    }

void MessageManager::PackTrafficLight(tx_sim::StepHelper& helper)
{
    std::string traffic_payload;
    helper.GetSubscribedMessage(tx_sim::topic::kTraffic, traffic_payload);
    sim_msg::Traffic traffic;
    traffic.ParseFromString(traffic_payload);

    cout<<"traffic light size: "<<traffic.trafficlights_size()<<endl;
    if(traffic.trafficlights_size()>0)
    {
        auto light = traffic.mutable_trafficlights(0);
        cout<<"id: "<<light->id()<<endl;
        cout<<"coordinate: "<<light->x()<<" , "<<light->y()<<" , "<<light->z()<<endl;
        cout<<"heading: "<<light->heading()<<endl;
        cout<<"color: "<<light->color()<<endl;
    }

    std::cout<<"packTrafficLight succeed: "<<std::endl;
}

void MessageManager::PackCaninfo(tx_sim::StepHelper& helper)
{
    std::string vehstate_payload;
    helper.GetSubscribedMessage(tx_sim::topic::kVehicleState, vehstate_payload);
    sim_msg::VehicleState vehstate;
    vehstate.ParseFromString(vehstate_payload);

    caninfo_.velocity = navinfo_.speed * 3.6;
    caninfo_.yaw_rate = navinfo_.angular_vel_z;
    caninfo_.acceleration_x = navinfo_.acceleration_x;
    caninfo_.acceleration_y = navinfo_.acceleration_y;
    caninfo_.steer_wheel_angle = vehstate.mutable_chassis_state()->steeringwheelangle();


}

} // namespace tievsim