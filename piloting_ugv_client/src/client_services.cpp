#include <future>

#include <piloting_ugv_client/client_node.h>

namespace piloting_ugv_client {

bool ClientNode::setUploadAlarmCb(
    piloting_ugv_client::SetUploadAlarm::Request& request, piloting_ugv_client::SetUploadAlarm::Response&)
{
    mavsdk::AlarmBase::AlarmList alarm_list;
    for (auto alarm : request.alarms) {
        mavsdk::AlarmBase::AlarmItem alarm_item;
        alarm_item.index       = alarm.index;
        alarm_item.name        = alarm.name;
        alarm_item.description = alarm.description;
        alarm_list.items.push_back(alarm_item);
    }

    _alarm->set_upload_alarm(alarm_list);

    return true;
}

bool ClientNode::setUploadChecklistCb(
    piloting_ugv_client::SetUploadChecklist::Request& request, piloting_ugv_client::SetUploadChecklist::Response&)
{
    mavsdk::ChecklistBase::Checklist checklist_list;
    for (auto checklist : request.checklist) {
        mavsdk::ChecklistBase::ChecklistItem checklist_item;
        checklist_item.index       = checklist.index;
        checklist_item.name        = checklist.name;
        checklist_item.description = checklist.description;
        checklist_list.items.push_back(checklist_item);
    }

    _checklist->set_upload_checklist(checklist_list);

    return true;
}

bool ClientNode::setUploadHLActionCb(
    piloting_ugv_client::SetUploadHLAction::Request& request, piloting_ugv_client::SetUploadHLAction::Response&)
{
    mavsdk::HLActionBase::HLActionList hl_action_list;
    for (auto hl_action : request.hl_actions) {
        mavsdk::HLActionBase::HLActionItem hl_action_item;
        hl_action_item.index       = hl_action.index;
        hl_action_item.command     = hl_action.command;
        hl_action_item.name        = hl_action.name;
        hl_action_item.description = hl_action.description;
        hl_action_list.items.push_back(hl_action_item);
    }

    _hl_action->set_upload_hl_action(hl_action_list);

    return true;
}

bool ClientNode::setUploadWaypointListCb(
    piloting_ugv_client::SetUploadWaypointList::Request& request, piloting_ugv_client::SetUploadWaypointList::Response&)
{
    mavsdk::InspectionBase::WaypointList waypoint_list;

    for (auto waypoint_item : request.waypoint_list.items) {
        mavsdk::InspectionBase::WaypointItem waypoint_item_base;
        waypoint_item_base.task_id      = waypoint_item.task_id;
        waypoint_item_base.command      = waypoint_item.command;
        waypoint_item_base.autocontinue = waypoint_item.autocontinue;
        waypoint_item_base.param1       = waypoint_item.param1;
        waypoint_item_base.param2       = waypoint_item.param2;
        waypoint_item_base.param3       = waypoint_item.param3;
        waypoint_item_base.param4       = waypoint_item.param4;
        waypoint_item_base.x            = waypoint_item.x;
        waypoint_item_base.y            = waypoint_item.y;
        waypoint_item_base.z            = waypoint_item.z;

        waypoint_list.items.push_back(waypoint_item_base);
    }

    _inspection->set_upload_inspection(waypoint_list);

    return true;
}

bool ClientNode::updateCurrentWaypointItemCb(
    piloting_ugv_client::UpdateSeqWaypointItem::Request& request, piloting_ugv_client::UpdateSeqWaypointItem::Response&)
{
    uint16_t item_seq = request.item_seq;
    _inspection->update_current_inspection_item(item_seq);

    return true;
}

bool ClientNode::updateReachedWaypointItemCb(
    piloting_ugv_client::UpdateSeqWaypointItem::Request& request, piloting_ugv_client::UpdateSeqWaypointItem::Response&)
{
    uint16_t item_seq = request.item_seq;
    _inspection->update_reached_inspection_item(item_seq);

    return true;
}

} // namespace piloting_ugv_client
