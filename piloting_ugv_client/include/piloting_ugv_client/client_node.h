#pragma once

#include <ros/ros.h>

#include <piloting_ugv_client/mavsdk_include.h>
#include <piloting_ugv_client/parameters.h>

// msgs
#include <piloting_ugv_client/AlarmStatus.h>
#include <piloting_ugv_client/TextStatus.h>

// srvs
#include <piloting_ugv_client/SetUploadAlarm.h>
#include <piloting_ugv_client/SetUploadChecklist.h>
#include <piloting_ugv_client/SetUploadHLAction.h>
#include <piloting_ugv_client/SetUploadWaypointList.h>
#include <piloting_ugv_client/UpdateSeqWaypointItem.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace piloting_ugv_client {
class ClientNode {
public:
    ClientNode();
    ~ClientNode();

    bool init();

private:
    void initAlarm(std::shared_ptr<mavsdk::System>& target_system);
    void initCommand(std::shared_ptr<mavsdk::System>& target_system);
    void initChecklist(std::shared_ptr<mavsdk::System>& target_system);
    void initHLAction(std::shared_ptr<mavsdk::System>& target_system);
    void initInspection(std::shared_ptr<mavsdk::System>& target_system);
    void initTelemetry(std::shared_ptr<mavsdk::System>& target_system);

    void alarmStatusCb(const AlarmStatus::ConstPtr& msg);
    void telemetryCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const geometry_msgs::TwistStamped::ConstPtr& velocity_msg);
    void textStatusCb(const TextStatus::ConstPtr& msg);
    
    // clang-format off
    bool setUploadAlarmCb(SetUploadAlarm::Request& request,
                          SetUploadAlarm::Response& response);
    bool setUploadChecklistCb(SetUploadChecklist::Request& request,
                              SetUploadChecklist::Response& response);
    bool setUploadHLActionCb(SetUploadHLAction::Request& request,
                             SetUploadHLAction::Response& response);
    bool setUploadWaypointListCb(SetUploadWaypointList::Request& request,
                                 SetUploadWaypointList::Response& response);
    bool updateCurrentWaypointItemCb(UpdateSeqWaypointItem::Request& request,
                                     UpdateSeqWaypointItem::Response& response);
    bool updateReachedWaypointItemCb(UpdateSeqWaypointItem::Request& request,
                                     UpdateSeqWaypointItem::Response& response);
    // clang-format on

    ros::NodeHandle _nh;
    Parameters _params;

    std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
    std::shared_ptr<mavsdk::TelemetryRoboticVehicle> _telemetry;
    std::shared_ptr<mavsdk::InspectionRoboticVehicle> _inspection;
    std::shared_ptr<mavsdk::AlarmRoboticVehicle> _alarm;
    std::shared_ptr<mavsdk::ChecklistRoboticVehicle> _checklist;
    std::shared_ptr<mavsdk::CommandRoboticVehicle> _command;
    std::shared_ptr<mavsdk::HLActionRoboticVehicle> _hl_action;

    // ROS Services
    ros::ServiceServer _set_upload_alarm_srv;
    ros::ServiceServer _set_upload_checklist_srv;
    ros::ServiceServer _set_upload_hl_action_srv;
    ros::ServiceServer _set_upload_waypoint_list_srv;
    ros::ServiceServer _update_current_waypoint_item_srv;
    ros::ServiceServer _update_reached_waypoint_item_srv;

    // ROS Publishers
    ros::Publisher _received_inspection_set_current_pub;

    // ROS Subscribers
    ros::Subscriber _alarm_status_sub;
    ros::Subscriber _text_status_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> _pose_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> _vel_sub;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> _sync;
};
} // namespace piloting_ugv_client
