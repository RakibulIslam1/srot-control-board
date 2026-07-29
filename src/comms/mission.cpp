// =============================================================================
//  comms/mission — implementation
// =============================================================================

#include "comms/mission.h"
#include "config.h"

namespace mission {

static mavlink_mission_item_int_t s_items[MAX_ITEMS];
static uint16_t s_count = 0;

// Upload session state.
static bool     s_uploading = false;
static uint16_t s_expected = 0;   // total items being uploaded
static uint16_t s_next_req = 0;   // next seq we expect

uint16_t count() { return s_count; }

bool getItem(uint16_t seq, mavlink_mission_item_int_t& out) {
    if (seq >= s_count) return false;
    out = s_items[seq];
    return true;
}

static void sendRequest(uint16_t seq) {
    mavlink_message_t m;
    mavlink_msg_mission_request_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, seq, MAV_MISSION_TYPE_MISSION);
    mav::tx(m);
}

static void sendAck(uint8_t type) {
    mavlink_message_t m;
    mavlink_msg_mission_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, type, MAV_MISSION_TYPE_MISSION, 0);
    mav::tx(m);
}

static void sendCount() {
    mavlink_message_t m;
    mavlink_msg_mission_count_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, s_count, MAV_MISSION_TYPE_MISSION, 0);
    mav::tx(m);
}

static void sendItem(uint16_t seq) {
    if (seq >= s_count) { sendAck(MAV_MISSION_ERROR); return; }
    mavlink_message_t m;
    const mavlink_mission_item_int_t& it = s_items[seq];
    mavlink_msg_mission_item_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, it.seq, it.frame, it.command,
        it.current, it.autocontinue, it.param1, it.param2, it.param3, it.param4,
        it.x, it.y, it.z, MAV_MISSION_TYPE_MISSION);
    mav::tx(m);
}

bool handle(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t mc;
            mavlink_msg_mission_count_decode(&msg, &mc);
            if (mc.mission_type != MAV_MISSION_TYPE_MISSION) { sendAck(MAV_MISSION_UNSUPPORTED); return true; }
            if (mc.count > MAX_ITEMS) { sendAck(MAV_MISSION_NO_SPACE); return true; }
            s_uploading = true;
            s_expected = mc.count;
            s_next_req = 0;
            s_count = 0;
            if (s_expected == 0) { s_uploading = false; sendAck(MAV_MISSION_ACCEPTED); }
            else                 sendRequest(0);
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            mavlink_mission_item_int_t it;
            mavlink_msg_mission_item_int_decode(&msg, &it);
            if (!s_uploading) return true;
            if (it.seq != s_next_req) { sendRequest(s_next_req); return true; }  // re-request
            s_items[it.seq] = it;
            s_next_req++;
            if (s_next_req >= s_expected) {
                s_count = s_expected;
                s_uploading = false;
                sendAck(MAV_MISSION_ACCEPTED);
            } else {
                sendRequest(s_next_req);
            }
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            sendCount();
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
            mavlink_mission_request_int_t rq;
            mavlink_msg_mission_request_int_decode(&msg, &rq);
            sendItem(rq.seq);
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST: {   // legacy download request
            mavlink_mission_request_t rq;
            mavlink_msg_mission_request_decode(&msg, &rq);
            sendItem(rq.seq);
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            s_count = 0;
            s_uploading = false;
            sendAck(MAV_MISSION_ACCEPTED);
            return true;
        }
        case MAVLINK_MSG_ID_MISSION_ACK:
            return true;   // download-complete ack from GCS — nothing to do
        default:
            return false;
    }
}

}  // namespace mission
