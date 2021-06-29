#pragma once
#include <list>
#include <sys/socket.h>

#include "MessageManagerBase.h"
#include "simutils.hpp"

//#define DEBUG_CARSIM
//#define USE_SLOPE

namespace tievsim
{
    class MessageManager : public MessageManagerBase
    {

    public:
        MessageManager(
            const std::string &url, int ego_id,
            const std::string &messager_config,
            const string &vehicle_config,
            const string &controller_config,
            const string &record_path);
        ~MessageManager() = default;


        // receive protobuf
        void PackLocation();
        void PackCaninfo(const RDB_OBJECT_STATE_t &ego_state);
        void PackCaninfo(const RDB_VEHICLE_SYSTEMS_t &ego_veh_sys);
        void PackNavinfo(const RDB_OBJECT_STATE_t &ego_state);
        void PackNavinfo(const RDB_VEHICLE_SYSTEMS_t &ego_veh_sys);
        void PackObjectList();
        void PackOneObject(const RDB_OBJECT_STATE_t &obj_state);
        void RasterOneObject(const RDB_OBJECT_STATE_t &obj_state);
        void PackLanes();
        void PackTrafficlight();
        void PackTimestamp(double sim_time);
        void PackCarsimControl();

        void HandleRdbEntry(const RDB_VEHICLE_SETUP_t &item);
        void HandleRdbEntry(const RDB_VEHICLE_SYSTEMS_t &item);
        void HandleRdbEntry(const RDB_OBJECT_STATE_t &item);
        void HandleRdbEntry(const RDB_CONTACT_POINT_t &item);

        void OnStartOfFrame();
        void OnEndOfFrame(int socket, double sim_time, int sim_frame);

        void ParseRdbMessage(RDB_MSG_t *msg, bool &is_image, int socket);
        void ParseRdbMessageWithLock(RDB_MSG_t *msg, bool &is_image, unsigned int socket);
        void ParseRdbMessageEntry(
            const double &sim_time, const unsigned int &sim_frame,
            RDB_MSG_ENTRY_HDR_t *entry_hdr, int socket);

        void OnStartOfFrameOdrGw();
        void OnEndOfFrameOdrGw(int socket, double sim_time, int sim_frame);

        void ParseRdbMessageOdrGw(RDB_MSG_t *msg, bool &is_image, int socket);
        void ParseRdbMessageOdrGwWithLock(RDB_MSG_t *msg, bool &is_image, unsigned int socket);
        void ParseRdbMessageEntryOdrGw(
            const double &sim_time, const unsigned int &sim_frame,
            RDB_MSG_ENTRY_HDR_t *entry_hdr, int socket);

        void SendDriverControl(int socket, double sim_time, unsigned int sim_frame);
        void SendVehState(int socket, double sim_time, unsigned int sim_frame);
        void SendRoadQuery(int socket, double sim_time, unsigned int sim_frame);

    private:
        void PullParameter(const string &messager_config, const string &vehicle_config);

        inline double steer2steerwheel(double steer)
        {
            return steer / kMaxSteer * kMaxSteerWheel;
        }
        inline double steerwheel2steer(double steerwheel)
        {
            return steerwheel / kMaxSteerWheel * kMaxSteer;
        }

        RDB_COORD_t ToVehFrame(const RDB_COORD_t &p) const;
        bool InAreaTest(double x, double y, const icumsg::BOUNDINGBOX &box) const;
        bool InVehFrameTest(const RDB_COORD_t &pos) const;

        void ResetFusionmap();
        void ResetObjectlist();
        // 参数表
    private:
        unsigned int kMyID; // ego car's ID

        float kWheelbase;
        float kMass;
        RDB_POINT_t kMassCenter;
        float kWheelRadius;
        float kMaxSteer;
        float kMaxSteerWheel;
        int kPathPointNum;
        float kPathPointTimestep;
        float kMapResolution;
        int kMapRowNum;
        int kMapColNum;
        int kMapRowCenter;
        int kMapColCenter;
        double kMapDynaMetric;
        int kLanelinePointNum;
        float kLanelinePointDistance;
        double kOffset;
        double kMaxBPF;
        int kRoadQueryId;

        GeographicLib::GeoCoords geo_origin_;

        bool kMapperMode;
        bool kAsyncMode;
        bool kCarSimMode;

        GeographicLib::GeoCoords geo_coord_;
        RDB_OBJECT_STATE_t ego_;
        TievController tiev_controller_;

        RDB_OBJECT_STATE_t ego_1st_;
        bool ego_1st_cached_;
    };

}; // namespace tievsim