#include "MessageManager.h"
#include "my_module.h"

using std::list;
using std::string;
using std::vector;

namespace tievsim
{
    using namespace simutils;

    MessageManager::MessageManager(
        const string &url, int ego_id,
        const string &messager_config,
        const string &vehicle_config,
        const string &controller_config,
        const string &record_path)
        : MessageManagerBase(url, messager_config, record_path),
          tiev_controller_(controller_config), kMyID(ego_id),
          ego_(), geo_origin_(), geo_coord_(), kMassCenter(),
          ego_1st_(), ego_1st_cached_(false)
    { }


    void MessageManager::PackLocation()
    {
        helper.GetSubscribedMessage(tx_sim::topic::kLocation, payload_);
        sim_msg::Location loc;
        loc.ParseFromString(payload_);
        cur_x_ = loc.position().x();
        cur_y_ = loc.position().y();

    }

    void MessageManager::PackCaninfo(const RDB_OBJECT_STATE_t &ego_state)
    {
        double vx = ego_state.ext.speed.x - kWheelbase * sin(ego_state.base.pos.h) * ego_state.ext.speed.h;
        double vy = ego_state.ext.speed.y + kWheelbase * cos(ego_state.base.pos.h) * ego_state.ext.speed.h;
        double vz = ego_state.ext.speed.z;
        double speed = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
        caninfo_.carspeed = int(speed * 3.6 * 100);
    }

    void MessageManager::PackCaninfo(const RDB_VEHICLE_SYSTEMS_t &ego_veh_sys)
    {
        if (!kCarSimMode)
        {
            // assumption: steering angle and steering wheel angle only have a linear relationship
            double steering = rad2deg(ego_veh_sys.steering);
            // steeringwheel orientation differs from vtd to tiev.
            caninfo_.carsteer = int(-steer2steerwheel(steering));
        }
        else
        {
            caninfo_.carsteer = int(-rad2deg(carsim_control_.steer));
            //printf("caninfo.carsteer: %d\n", caninfo_.carsteer);
        }
    }

    void MessageManager::HandleRdbEntry(const RDB_VEHICLE_SETUP_t &item)
    {
        if (item.playerId == kMyID)
        {
            kWheelbase = item.wheelBase;
        }
    }

    void MessageManager::HandleRdbEntry(const RDB_VEHICLE_SYSTEMS_t &item)
    {
        if (item.playerId == kMyID)
        {
            PackCaninfo(item);
            PackNavinfo(item);
        }
    }

    void MessageManager::HandleRdbEntry(const RDB_OBJECT_STATE_t &item)
    {
        if (item.base.id == kMyID)
        {
            PackCaninfo(item);
            PackNavinfo(item);
            ego_ = item; // cache ego car's state
            if (!ego_1st_cached_)
            {
                ego_1st_ = item;
                ego_1st_cached_ = true;
#ifdef DEBUG_CARSIM
                printf("=============== ego_object_state ==============\n");
                printf("pos: (%f, %f, %f, %f, %f, %f)\nvel: (%f, %f, %f, %f, %f, %f)\nacc: (%f, %f, %f, %f, %f, %f)\n",
                       ego_1st_.base.pos.x, ego_1st_.base.pos.y, ego_1st_.base.pos.z,
                       ego_1st_.base.pos.h, ego_1st_.base.pos.p, ego_1st_.base.pos.r,
                       ego_1st_.ext.speed.x, ego_1st_.ext.speed.y, ego_1st_.ext.speed.z,
                       ego_1st_.ext.speed.h, ego_1st_.ext.speed.p, ego_1st_.ext.speed.r,
                       ego_1st_.ext.accel.x, ego_1st_.ext.accel.y, ego_1st_.ext.accel.z,
                       ego_1st_.ext.accel.h, ego_1st_.ext.accel.p, ego_1st_.ext.accel.r);
#endif
            }
        }
        else if (InVehFrameTest(item.base.pos))
        {
            if (item.base.category == RDB_OBJECT_CATEGORY_PLAYER)
            {
                PackOneObject(item);
                RasterOneObject(item);
            }
        }
    }

    void MessageManager::HandleRdbEntry(const RDB_CONTACT_POINT_t &item)
    {
        // 11 left_front, 12 left_rear, 13 right_front, 14 right_rear
        carsim_contact_.valid = 1;
#ifdef DEBUG_CARSIM
        printf("\n================= ODRGATEWAY ==================\nitem.id: %d\n", item.id);
        printf("item.roadDataIn: (%f, %f, %f)\n", item.roadDataIn.x, item.roadDataIn.y, item.roadDataIn.z);
        printf("item.friciton: %f\n", item.friction);
#endif
        if (item.id == kRoadQueryId * 1000 + 11)
        {
            carsim_contact_.left_front.id = item.id;
            carsim_contact_.left_front.z = item.roadDataIn.z;
            carsim_contact_.left_front.friction = item.friction == 0 ? 0.9 : item.friction;
        }
        else if (item.id == kRoadQueryId * 1000 + 12)
        {
            carsim_contact_.left_rear.id = item.id;
            carsim_contact_.left_rear.z = item.roadDataIn.z;
            carsim_contact_.left_rear.friction = item.friction == 0 ? 0.9 : item.friction;
        }
        else if (item.id == kRoadQueryId * 1000 + 21)
        {
            carsim_contact_.right_front.id = item.id;
            carsim_contact_.right_front.z = item.roadDataIn.z;
            carsim_contact_.right_front.friction = item.friction == 0 ? 0.9 : item.friction;
        }
        else if (item.id == kRoadQueryId * 1000 + 22)
        {
            carsim_contact_.right_rear.id = item.id;
            carsim_contact_.right_rear.z = item.roadDataIn.z;
            carsim_contact_.right_rear.friction = item.friction == 0 ? 0.9 : item.friction;
        }
#ifdef USE_SLOPE
        else if (item.id == kRoadQueryId * 1000 + 111)
        {
            carsim_contact_.left_front.slope_x = (item.roadDataIn.z - carsim_contact_.left_front.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 112)
        {
            carsim_contact_.left_front.slope_y = (item.roadDataIn.z - carsim_contact_.left_front.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 121)
        {
            carsim_contact_.left_rear.slope_x = (item.roadDataIn.z - carsim_contact_.left_rear.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 122)
        {
            carsim_contact_.left_rear.slope_y = (item.roadDataIn.z - carsim_contact_.left_rear.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 211)
        {
            carsim_contact_.right_front.slope_x = (item.roadDataIn.z - carsim_contact_.right_front.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 212)
        {
            carsim_contact_.right_front.slope_y = (item.roadDataIn.z - carsim_contact_.right_front.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 221)
        {
            carsim_contact_.right_rear.slope_x = (item.roadDataIn.z - carsim_contact_.right_rear.z) * 100;
        }
        else if (item.id == kRoadQueryId * 1000 + 222)
        {
            carsim_contact_.right_rear.slope_y = (item.roadDataIn.z - carsim_contact_.right_rear.z) * 100;
        }
#endif
    }

    void MessageManager::OnStartOfFrame()
    {
        ResetObjectlist();
        ResetFusionmap();
    }

    void MessageManager::OnEndOfFrame(int socket, double sim_time, int sim_frame)
    {
        PackTimestamp(sim_time);
        if (!kMapperMode && kCarSimMode)
        {
            PackCarsimControl();
            if (carsim_state_.valid)
            {
                // printf("carsim_state valid\n");
                SendVehState(socket, sim_time, sim_frame);
            }
        }
        else if (!kMapperMode && !kCarSimMode)
        {
            SendDriverControl(socket, sim_time, sim_frame);
        }

        if (!kAsyncMode)
        {
            PublishAll();
        }
    }

    void MessageManager::ResetFusionmap()
    {
        fusionmap_.utmX = navinfo_.utmX;
        fusionmap_.utmY = navinfo_.utmY;
        fusionmap_.mHeading = navinfo_.mHeading;
        fusionmap_.rows = kMapRowNum;
        fusionmap_.cols = kMapColNum;
        fusionmap_.center_row = kMapRowCenter;
        fusionmap_.center_col = kMapColCenter;
        fusionmap_.resolution = kMapResolution;

        memset(fusionmap_.cells, '\0', sizeof(fusionmap_.cells));
    }

    void MessageManager::ResetObjectlist()
    {
        // printf("-----Before, object_list_.count: %d, object_list_.obj.size(): %d\n", object_list_.count, object_list_.obj.size());
        object_list_.obj.clear();
        object_list_.count = 0;
        object_list_.data_source = 1;
        // printf("-----After, object_list_.count: %d, object_list_.obj.size(): %d\n", object_list_.count, object_list_.obj.size());
    }

    void MessageManager::SendDriverControl(int socket, double sim_time, unsigned int sim_frame)
    {
        Framework::RDBHandler my_handler;

        // start a new message
        my_handler.initMsg();

        // add extended package for the object state
        RDB_DRIVER_CTRL_t *driver_ctrl = (RDB_DRIVER_CTRL_t *)my_handler.addPackage(
            sim_time, sim_frame, RDB_PKG_ID_DRIVER_CTRL, 1);

        if (!driver_ctrl)
        {
            fprintf(stderr, "sendDriverCtrl: could not create driver control\n");
            return;
        }

        driver_ctrl->playerId = kMyID;

        double aim_speed = double(cancontrol_.aimspeed);
        double aim_steer = double(cancontrol_.aimsteer); // deg, steeringwheel orientation differs from vtd to tiev.
        double veh_speed = double(caninfo_.carspeed);    // cm/h
        double veh_steer = double(caninfo_.carsteer);    // deg
        tiev_controller_.Tick(aim_speed, veh_speed, aim_steer, veh_steer);
        // printf("control ID : %d\n", driver_ctrl->playerId);
        // printf("vehicle speed: %f, vehicle steer: %f\n", veh_speed, veh_steer);
        // printf("received cancontrol: aim-speed %f, aim-steer %f\n", aim_speed, aim_steer);

        driver_ctrl->throttlePedal = tiev_controller_.throttle;
        driver_ctrl->brakePedal = tiev_controller_.brake;
        // now we use front-wheel steering to control lateral motion
        driver_ctrl->steeringTgt = steerwheel2steer(tiev_controller_.steer); // rad
        // driver_ctrl->steeringTgt = deg2rad(aim_steer);
        driver_ctrl->validityFlags =
            RDB_DRIVER_INPUT_VALIDITY_THROTTLE |
            RDB_DRIVER_INPUT_VALIDITY_BRAKE |
            RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING |
            RDB_DRIVER_INPUT_VALIDITY_ADD_ON;

        int ret_val = send(socket, (const char *)(my_handler.getMsg()), my_handler.getMsgTotalSize(), 0);

        //printf("send driver control: throttle %f, brake %f, steering %f\n", driver_ctrl->throttlePedal, driver_ctrl->brakePedal, driver_ctrl->steeringTgt);

        if (!ret_val)
            fprintf(stderr, "MessageManager::SendDriverCtrl : could not send driver control\n");
    }

    void MessageManager::SendVehState(int socket, double sim_time, unsigned int sim_frame)
    {
        Framework::RDBHandler my_handler;

        // start a new message
        my_handler.initMsg();

        // added start of frame
        RDB_START_OF_FRAME_t *sof = (RDB_START_OF_FRAME_t *)my_handler.addPackage(sim_time, sim_frame, RDB_PKG_ID_START_OF_FRAME);

        // add extended package for the object state
        RDB_OBJECT_STATE_t *vehiclestate = (RDB_OBJECT_STATE_t *)my_handler.addPackage(sim_time, sim_frame, RDB_PKG_ID_OBJECT_STATE, 1, RDB_PKG_FLAG_EXTENDED);

        if (!vehiclestate)
        {
            fprintf(stderr, "sendvehiclestate: could not create vehiclestate\n");
            return;
        }

        vehiclestate->base.id = ego_1st_.base.id;
        vehiclestate->base.category = RDB_OBJECT_CATEGORY_PLAYER;
        vehiclestate->base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
        vehiclestate->base.visMask = RDB_OBJECT_VIS_FLAG_ALL;
        strcpy(vehiclestate->base.name, ego_1st_.base.name);
        vehiclestate->base.geo = ego_1st_.base.geo;

        vehiclestate->base.pos.x = ego_1st_.base.pos.x +
                                   carsim_state_.x * cos(ego_1st_.base.pos.h) -
                                   carsim_state_.y * sin(ego_1st_.base.pos.h);
        vehiclestate->base.pos.y = ego_1st_.base.pos.y +
                                   carsim_state_.x * sin(ego_1st_.base.pos.h) +
                                   carsim_state_.y * cos(ego_1st_.base.pos.h);
        vehiclestate->base.pos.z = carsim_state_.z + ego_1st_.base.pos.z;
        vehiclestate->base.pos.h = carsim_state_.yaw + ego_1st_.base.pos.h;
        vehiclestate->base.pos.p = carsim_state_.pitch + ego_1st_.base.pos.p;
        vehiclestate->base.pos.r = carsim_state_.roll + ego_1st_.base.pos.r;
        vehiclestate->base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
        vehiclestate->base.pos.type = RDB_COORD_TYPE_INERTIAL;
        vehiclestate->base.pos.system = 0;

        vehiclestate->base.parent = ego_1st_.base.parent;
        vehiclestate->base.cfgFlags = ego_1st_.base.cfgFlags;
        vehiclestate->base.cfgModelId = ego_1st_.base.cfgModelId;

        vehiclestate->ext.speed.x = carsim_state_.vx * cos(ego_1st_.base.pos.h) - carsim_state_.vy * sin(ego_1st_.base.pos.h);
        vehiclestate->ext.speed.y = carsim_state_.vy * cos(ego_1st_.base.pos.h) + carsim_state_.vx * sin(ego_1st_.base.pos.h);
        vehiclestate->ext.speed.z = carsim_state_.vz;
        vehiclestate->ext.speed.h = carsim_state_.avy;
        vehiclestate->ext.speed.r = carsim_state_.avr;
        vehiclestate->ext.speed.p = carsim_state_.avp;
        vehiclestate->ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
        vehiclestate->ext.speed.type = RDB_COORD_TYPE_INERTIAL;
        vehiclestate->ext.speed.system = 0;

        vehiclestate->ext.accel.x = carsim_state_.dvx * cos(ego_1st_.base.pos.h) - carsim_state_.dvy * sin(ego_1st_.base.pos.h);
        vehiclestate->ext.accel.y = carsim_state_.dvy * cos(ego_1st_.base.pos.h) + carsim_state_.dvx * sin(ego_1st_.base.pos.h);
        vehiclestate->ext.accel.z = carsim_state_.dvz;
        vehiclestate->ext.accel.h = carsim_state_.davy;
        vehiclestate->ext.accel.r = carsim_state_.davr;
        vehiclestate->ext.accel.p = carsim_state_.davp;
        vehiclestate->ext.accel.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
        vehiclestate->ext.accel.type = RDB_COORD_TYPE_INERTIAL;
        vehiclestate->ext.accel.system = 0;

        vehiclestate->ext.traveledDist = carsim_state_.travaled_dist;

        // add end of frame
        RDB_END_OF_FRAME_t *eof = (RDB_END_OF_FRAME_t *)my_handler.addPackage(sim_time, sim_frame, RDB_PKG_ID_END_OF_FRAME);

        int ret_val = send(socket, (const char *)(my_handler.getMsg()), my_handler.getMsgTotalSize(), 0);

#ifdef DEBUG_CARSIM
        printf("=============== transform ego_object_state ==============\n");
        printf("pos: (%f, %f, %f, %f, %f, %f)\nvel: (%f, %f, %f, %f, %f, %f)\nacc: (%f, %f, %f, %f, %f, %f)\n",
               vehiclestate->base.pos.x, vehiclestate->base.pos.y, vehiclestate->base.pos.z,
               vehiclestate->base.pos.h, vehiclestate->base.pos.p, vehiclestate->base.pos.r,
               vehiclestate->ext.speed.x, vehiclestate->ext.speed.y, vehiclestate->ext.speed.z,
               vehiclestate->ext.speed.h, vehiclestate->ext.speed.p, vehiclestate->ext.speed.r,
               vehiclestate->ext.accel.x, vehiclestate->ext.accel.y, vehiclestate->ext.accel.z,
               vehiclestate->ext.accel.h, vehiclestate->ext.accel.p, vehiclestate->ext.accel.r);
        printf("--------------> carsim state:\n");
        printf("pos: (%f, %f, %f, %f, %f, %f)\nvel: (%f, %f, %f, %f, %f, %f)\nacc: (%f, %f, %f, %f, %f, %f)\n",
               carsim_state_.x, carsim_state_.y, carsim_state_.z,
               carsim_state_.yaw, carsim_state_.pitch, carsim_state_.roll,
               carsim_state_.vx, carsim_state_.vy, carsim_state_.vz,
               carsim_state_.avy, carsim_state_.avp, carsim_state_.avr,
               carsim_state_.dvx, carsim_state_.dvy, carsim_state_.dvz,
               carsim_state_.davy, carsim_state_.davp, carsim_state_.davr);
#endif

        if (!ret_val)
            fprintf(stderr, "sendDriverCtrl: could not send driver control\n");

        return;
    }

    void MessageManager::SendRoadQuery(int socket, double sim_time, unsigned int sim_frame)
    {
        Framework::RDBHandler my_handler;

        // start a new message
        my_handler.initMsg();

        // add extended package for the object state
        RDB_ROAD_QUERY_t *roadquery = (RDB_ROAD_QUERY_t *)my_handler.addPackage(sim_time, sim_frame, RDB_PKG_ID_ROAD_QUERY, 12);

        if (!roadquery)
        {
            fprintf(stderr, "sendRoadQuery: could not create road query\n");
            return;
        }

        double h = ego_1st_.base.pos.h;
        // setup the four wheels
        roadquery[0].id = 11 + kRoadQueryId * 1000;
        roadquery[0].x = ego_1st_.base.pos.x + carsim_query_.left_front.x * cos(h) - carsim_query_.left_front.y * sin(h);
        roadquery[0].y = ego_1st_.base.pos.y + carsim_query_.left_front.x * sin(h) + carsim_query_.left_front.y * cos(h);
        roadquery[0].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[1].id = 12 + kRoadQueryId * 1000;
        roadquery[1].x = ego_1st_.base.pos.x + carsim_query_.left_rear.x * cos(h) - carsim_query_.left_rear.y * sin(h);
        roadquery[1].y = ego_1st_.base.pos.y + carsim_query_.left_rear.x * sin(h) + carsim_query_.left_rear.y * cos(h);
        roadquery[1].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[2].id = 21 + kRoadQueryId * 1000;
        roadquery[2].x = ego_1st_.base.pos.x + carsim_query_.right_front.x * cos(h) - carsim_query_.right_front.y * sin(h);
        roadquery[2].y = ego_1st_.base.pos.y + carsim_query_.right_front.x * sin(h) + carsim_query_.right_front.y * cos(h);
        roadquery[2].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[3].id = 22 + kRoadQueryId * 1000;
        roadquery[3].x = ego_1st_.base.pos.x + carsim_query_.right_rear.x * cos(h) - carsim_query_.right_rear.y * sin(h);
        roadquery[3].y = ego_1st_.base.pos.y + carsim_query_.right_rear.x * sin(h) + carsim_query_.right_rear.y * cos(h);
        roadquery[3].flags = RDB_ROAD_QUERY_FLAG_NONE;

        // setup for height diff calculation for road slope
        roadquery[4].id = 111 + kRoadQueryId * 1000;
        roadquery[4].x = ego_1st_.base.pos.x + carsim_query_.left_front.x * cos(h) - carsim_query_.left_front.y * sin(h) + 0.01;
        roadquery[4].y = ego_1st_.base.pos.y + carsim_query_.left_front.x * sin(h) + carsim_query_.left_front.y * cos(h);
        roadquery[4].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[5].id = 121 + kRoadQueryId * 1000;
        roadquery[5].x = ego_1st_.base.pos.x + carsim_query_.left_front.x * cos(h) - carsim_query_.left_front.y * sin(h);
        roadquery[5].y = ego_1st_.base.pos.y + carsim_query_.left_front.x * sin(h) + carsim_query_.left_front.y * cos(h) + 0.01;
        roadquery[5].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[6].id = 211 + kRoadQueryId * 1000;
        roadquery[6].x = ego_1st_.base.pos.x + carsim_query_.left_rear.x * cos(h) - carsim_query_.left_rear.y * sin(h) + 0.01;
        roadquery[6].y = ego_1st_.base.pos.y + carsim_query_.left_rear.x * sin(h) + carsim_query_.left_rear.y * cos(h);
        roadquery[6].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[7].id = 221 + kRoadQueryId * 1000;
        roadquery[7].x = ego_1st_.base.pos.x + carsim_query_.left_rear.x * cos(h) - carsim_query_.left_rear.y * sin(h);
        roadquery[7].y = ego_1st_.base.pos.y + carsim_query_.left_rear.x * sin(h) + carsim_query_.left_rear.y * cos(h) + 0.01;
        roadquery[7].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[8].id = 112 + kRoadQueryId * 1000;
        roadquery[8].x = ego_1st_.base.pos.x + carsim_query_.right_front.x * cos(h) - carsim_query_.right_front.y * sin(h) + 0.01;
        roadquery[8].y = ego_1st_.base.pos.y + carsim_query_.right_front.x * sin(h) + carsim_query_.right_front.y * cos(h);
        roadquery[8].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[9].id = 122 + kRoadQueryId * 1000;
        roadquery[9].x = ego_1st_.base.pos.x + carsim_query_.right_front.x * cos(h) - carsim_query_.right_front.y * sin(h);
        roadquery[9].y = ego_1st_.base.pos.y + carsim_query_.right_front.x * sin(h) + carsim_query_.right_front.y * cos(h) + 0.01;
        roadquery[9].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[10].id = 212 + kRoadQueryId * 1000;
        roadquery[10].x = ego_1st_.base.pos.x + carsim_query_.right_rear.x * cos(h) - carsim_query_.right_rear.y * sin(h) + 0.01;
        roadquery[10].y = ego_1st_.base.pos.y + carsim_query_.right_rear.x * sin(h) + carsim_query_.right_rear.y * cos(h);
        roadquery[10].flags = RDB_ROAD_QUERY_FLAG_NONE;

        roadquery[11].id = 222 + kRoadQueryId * 1000;
        roadquery[11].x = ego_1st_.base.pos.x + carsim_query_.right_rear.x * cos(h) - carsim_query_.right_rear.y * sin(h);
        roadquery[11].y = ego_1st_.base.pos.y + carsim_query_.right_rear.x * sin(h) + carsim_query_.right_rear.y * cos(h) + 0.01;
        roadquery[11].flags = RDB_ROAD_QUERY_FLAG_NONE;

        // add the end-of-frame boundary
        my_handler.addPackage(sim_time, sim_frame, RDB_PKG_ID_END_OF_FRAME, 0);

        // fprintf(stderr, "sendRoadQuery: sending queries, total size = %d\n", my_handler.getMsgTotalSize());

        int ret_val = send(socket, (const char *)(my_handler.getMsg()), my_handler.getMsgTotalSize(), 0);
        if (!ret_val)
            fprintf(stderr, "sendRoadQuery: could not send queries\n");
    }

    void MessageManager::ParseRdbMessage(RDB_MSG_t *msg, bool &is_image, int socket)
    {
        if (!msg)
            return;

        if (!msg->hdr.dataSize)
            return;

        RDB_MSG_ENTRY_HDR_t *entry = (RDB_MSG_ENTRY_HDR_t *)(((char *)msg) + msg->hdr.headerSize);
        uint32_t remain_bytes = msg->hdr.dataSize;

        while (remain_bytes)
        {
            ParseRdbMessageEntry(msg->hdr.simTime, msg->hdr.frameNo, entry, socket);

            is_image |= (entry->pkgId == RDB_PKG_ID_IMAGE);

            remain_bytes -= (entry->headerSize + entry->dataSize);

            if (remain_bytes)
                entry = (RDB_MSG_ENTRY_HDR_t *)((((char *)entry) + entry->headerSize + entry->dataSize));
        }
    }

    void MessageManager::ParseRdbMessageWithLock(RDB_MSG_t *msg, bool &is_image, unsigned int socket)
    {
        std::lock_guard<std::mutex> lock_caninfo(caninfo_mutex_);
        std::lock_guard<std::mutex> lock_navinfo(navinfo_mutex_);
        std::lock_guard<std::mutex> lock_objectlist(objectlist_mutex_);
        std::lock_guard<std::mutex> lock_fusionmap(fusionmap_mutex_);

        ParseRdbMessage(msg, is_image, socket);
    }

    void MessageManager::ParseRdbMessageEntry(
        const double &sim_time, const unsigned int &sim_frame,
        RDB_MSG_ENTRY_HDR_t *entry_hdr, int socket)
    {
        if (!entry_hdr)
            return;

        if (entry_hdr->pkgId == RDB_PKG_ID_START_OF_FRAME)
        {
            OnStartOfFrame();
            return;
        }
        else if (entry_hdr->pkgId == RDB_PKG_ID_END_OF_FRAME)
        {
            OnEndOfFrame(socket, sim_time, sim_frame);
            return;
        }

        int no_elements = entry_hdr->elementSize ? (entry_hdr->dataSize / entry_hdr->elementSize) : 0;

        unsigned char ident = 6;
        char *data_ptr = (char *)entry_hdr;
        data_ptr += entry_hdr->headerSize;

        while (no_elements--) // skip the SOF and EOF
        {
            switch (entry_hdr->pkgId)
            {
            case RDB_PKG_ID_VEHICLE_SETUP:
                HandleRdbEntry(*((RDB_VEHICLE_SETUP_t *)data_ptr));
                break;
            case RDB_PKG_ID_VEHICLE_SYSTEMS:
                HandleRdbEntry(*((RDB_VEHICLE_SYSTEMS_t *)data_ptr));
                break;
            case RDB_PKG_ID_OBJECT_STATE:
                HandleRdbEntry(*((RDB_OBJECT_STATE_t *)data_ptr));
                break;
            // case RDB_PKG_ID_CONTACT_POINT:
            //     HandleRdbEntry(*((RDB_CONTACT_POINT_t *)data_ptr));
            default:
                break;
            }
            data_ptr += entry_hdr->elementSize;
        }
    }

    void MessageManager::OnStartOfFrameOdrGw()
    {
        ;
    }

    void MessageManager::OnEndOfFrameOdrGw(int socket, double sim_time, int sim_frame)
    {
        ;
    }

    void MessageManager::ParseRdbMessageOdrGw(RDB_MSG_t *msg, bool &is_image, int socket)
    {
        if (!msg)
            return;

        if (!msg->hdr.dataSize)
            return;

        RDB_MSG_ENTRY_HDR_t *entry = (RDB_MSG_ENTRY_HDR_t *)(((char *)msg) + msg->hdr.headerSize);
        uint32_t remain_bytes = msg->hdr.dataSize;

        while (remain_bytes)
        {
            ParseRdbMessageEntryOdrGw(msg->hdr.simTime, msg->hdr.frameNo, entry, socket);

            is_image |= (entry->pkgId == RDB_PKG_ID_IMAGE);

            remain_bytes -= (entry->headerSize + entry->dataSize);

            if (remain_bytes)
                entry = (RDB_MSG_ENTRY_HDR_t *)((((char *)entry) + entry->headerSize + entry->dataSize));
        }
    }

    void MessageManager::ParseRdbMessageOdrGwWithLock(RDB_MSG_t *msg, bool &is_image, unsigned int socket)
    {
        //std::lock_guard<std::mutex> lock1(carsim_query_mutex_);
        //std::lock_guard<std::mutex> lock2(carsim_contact_mutex_);
        ParseRdbMessageOdrGw(msg, is_image, socket);
    }

    void MessageManager::ParseRdbMessageEntryOdrGw(
        const double &sim_time, const unsigned int &sim_frame,
        RDB_MSG_ENTRY_HDR_t *entry_hdr, int socket)
    {
        if (!entry_hdr)
            return;

        if (entry_hdr->pkgId == RDB_PKG_ID_START_OF_FRAME)
        {
            OnStartOfFrameOdrGw();
            return;
        }
        else if (entry_hdr->pkgId == RDB_PKG_ID_END_OF_FRAME)
        {
            OnEndOfFrameOdrGw(socket, sim_time, sim_frame);
            return;
        }

        int no_elements = entry_hdr->elementSize ? (entry_hdr->dataSize / entry_hdr->elementSize) : 0;

        unsigned char ident = 6;
        char *data_ptr = (char *)entry_hdr;
        data_ptr += entry_hdr->headerSize;

        while (no_elements--) // skip the SOF and EOF
        {
            switch (entry_hdr->pkgId)
            {
            case RDB_PKG_ID_CONTACT_POINT:
                HandleRdbEntry(*((RDB_CONTACT_POINT_t *)data_ptr));
            default:
                break;
            }
            data_ptr += entry_hdr->elementSize;
        }
    }

    RDB_COORD_t MessageManager::ToVehFrame(const RDB_COORD_t &p) const
    {
        // TODO: now we don't consider the Z direction.
        double offset = kWheelbase - kOffset;
        double heading = ego_.base.pos.h;
        double xr = p.x - ego_.base.pos.x;
        double yr = p.y - ego_.base.pos.y;
        double tarx = sin(heading) * xr - cos(heading) * yr;
        double tary = cos(heading) * xr + sin(heading) * yr - offset;
        RDB_COORD_t res;
        res.x = tarx;
        res.y = tary;
        return res;
    }

    bool MessageManager::InAreaTest(double x, double y, const icumsg::BOUNDINGBOX &box) const
    {
        // only need to calculate the z dimension of vectors' cross products.
        // if all cross products have the same symbol, the point is located within the area.
        double z1 = (box.p2.x - box.p1.x) * (y - box.p1.y) - (box.p2.y - box.p1.y) * (x - box.p1.x);
        bool isNeg = z1 < 0;
        double z2 = (box.p3.x - box.p2.x) * (y - box.p2.y) - (box.p3.y - box.p2.y) * (x - box.p2.x);
        if ((isNeg && z2 >= 0) || (!isNeg && z2 < 0))
            return false; // once result's symbol is different from previous ones, return false.
        double z3 = (box.p4.x - box.p3.x) * (y - box.p3.y) - (box.p4.y - box.p3.y) * (x - box.p3.x);
        if ((isNeg && z3 >= 0) || (!isNeg && z3 < 0))
            return false;
        double z4 = (box.p1.x - box.p4.x) * (y - box.p4.y) - (box.p1.y - box.p4.y) * (x - box.p4.x);
        if ((isNeg && z4 >= 0) || (!isNeg && z4 < 0))
            return false;
        return true;
    }

    bool MessageManager::InVehFrameTest(const RDB_COORD_t &pos) const
    {
        double z = pos.z - ego_.base.pos.z;
        auto pos_vf = ToVehFrame(pos);
        return (pos_vf.y < kMapResolution * kMapRowCenter) &&
               (pos_vf.y > -(kMapResolution * (kMapRowNum - kMapRowCenter))) &&
               (abs(pos_vf.x) < kMapResolution * kMapColCenter) &&
               (z < 3);
    }
} // namespace tievsim