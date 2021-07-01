#include "MessageManager.h"
#include "my_module.h"
#include "simutils.hpp"

#include "basic.pb.h"
#include "control.pb.h"
#include "header.pb.h"
#include "laneMarks.pb.h"
#include "location.pb.h"
#include "planStatus.pb.h"
#include "traffic.pb.h"
#include "trajectory.pb.h"

#include "WGS84UTM.h"

using std::list;
using std::string;
using std::vector;

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

    void MessageManager::PackPredictedObject(tx_sim::StepHelper& helper)
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
        for(int i=0; i<traffic.staticobstacles_size(); ++i){
            PredictedObject obj;
            auto static_obj = traffic.mutable_staticobstacles(i);
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

            // Attention: global frame is east-north, vehicle frame is front-left
            objxy.y = -objxy.y;

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

            for(int i=0;i<4;++i){
                CoorRotate(corner_points[i], -obj.heading);
                corner_points[i] = corner_points[i] + objxy;
                obj.bounding_box[0][i] = corner_points[i].x;
                obj.bounding_box[1][i] = corner_points[i].y;
            }
            obj.trajectory_point_num = 1 + timeHorizon/gap;  // predict 5s with a gap of 0.2s. 1 refers to current position
            Point2d UnitVec(cos(obj.heading), sin(obj.heading));
            for(int i=0;i<obj.trajectory_point_num;++i)
            {
                Point2d changePt = objxy +  UnitVec * obj.velocity * i*gap;
                obj.trajectory_point[0][i] = changePt.x;
                obj.trajectory_point[1][i] = changePt.y;
            }
            predictedObject_.predicted_object.push_back(obj);
        }
        
        for(int i=0; i<traffic.dynamicobstacles_size(); ++i){
            PredictedObject obj;
            auto obj_proto = traffic.mutable_dynamicobstacles(i);
            predictedObject_.time_stamp = obj_proto->t();
            
            UTMCoor utmXY_obj;
            LatLonToUTMXY(obj_proto->y(), obj_proto->x(), utmXY_obj);
            // calculate relative utm_coordinate
            utmXY_obj.x -= navinfo_.utm_x;
            utmXY_obj.y -= navinfo_.utm_y;

            // coordinate rotation
            double theta = navinfo_.angle_head, obj_x, obj_y;
            Point2d objxy(utmXY_obj.x, utmXY_obj.y);
            CoorRotate(objxy, theta);

            // Attention: global frame is east-north, vehicle frame is front-left
            objxy.y = -objxy.y;

            obj.id          = obj_proto->id();
            obj.type        = obj_proto->type();
            obj.velocity    = obj_proto->v();
            obj.accelerate  = obj_proto->acc();
            obj.heading     = obj_proto->heading() - navinfo_.angle_head;
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

            for(int i=0;i<4;++i){
                CoorRotate(corner_points[i], -obj.heading);
                corner_points[i] = corner_points[i] + objxy;
                obj.bounding_box[0][i] = corner_points[i].x;
                obj.bounding_box[1][i] = corner_points[i].y;
            }

            obj.trajectory_point_num = 1 + timeHorizon/gap;  // predict 5s with a gap of 0.2s. 1 refers to current position
            Point2d UnitVec(cos(obj.heading), sin(obj.heading));
            for(int i=0;i<obj.trajectory_point_num;++i)
            {
                Point2d changePt = objxy +  UnitVec * obj.velocity * i*gap;
                obj.trajectory_point[0][i] = changePt.x;
                obj.trajectory_point[1][i] = changePt.y;
            }
            predictedObject_.predicted_object.push_back(obj);
        }

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