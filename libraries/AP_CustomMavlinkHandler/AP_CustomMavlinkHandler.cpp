#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE == 1

#include "AP_CustomMavlinkHandler.h"

#include <stdio.h>
#include <string.h>

void AP_CustomMavlinkHandler::init(void) 
{
    g_custom_storage.init();
}

void AP_CustomMavlinkHandler::handle_custom_message(mavlink_channel_t chan, const mavlink_message_t& msg) 
{
    if (msg.msgid != CUSTOM_MSG_ID)
        return;

    // Manual message decoding
    uuid_update_t packet;
    memcpy(&packet, msg.payload64, sizeof(packet));
    packet.value[MAX_AB_PARAM_SIZE - 1] = '\0';  // Ensure null termination

    switch (packet.param) {
    case AIRBOUND_PARAMETER_PARAM_ID_UUID:
        switch (packet.action) {
        case AIRBOUND_PARAMETER_ACTION_GET: {
            char uuid[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.get_uuid(uuid, sizeof(uuid))) {
                result = AIRBOUND_PARAMETER_RESULT_OK;
                gcs().send_text(MAV_SEVERITY_INFO, "AB:UUID:%s", uuid);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to fetch UUID");
            }
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char*)uuid, result);
            break;
        }
        case AIRBOUND_PARAMETER_ACTION_SET: {
            char uuid[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.set_uuid(packet.value)) {
                gcs().send_text(MAV_SEVERITY_INFO, "AB:UUID updated");
                result = AIRBOUND_PARAMETER_RESULT_OK;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to update UUID");
            }
            g_custom_storage.get_uuid(uuid, sizeof(uuid));
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char*)uuid, result);
            break;
        }
        default:
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char*)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        break;

    // The password is write-only over MAVLink
    case AIRBOUND_PARAMETER_PARAM_ID_PASS:
        switch (packet.action) {
        case AIRBOUND_PARAMETER_ACTION_SET: {
            char pass[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.set_password(packet.value)) {
                gcs().send_text(MAV_SEVERITY_INFO, "AB:Updated password");
                result = AIRBOUND_PARAMETER_RESULT_OK;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING,"AB:Failed to update password");
            }
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char*)pass, result);
            break;
        }
        default: {
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char*)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        }
        break;

    case AIRBOUND_PARAMETER_PARAM_ID_CRAFT_ID:
        switch (packet.action) {
        case AIRBOUND_PARAMETER_ACTION_GET: {
            char craft_id[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.get_craft_id(craft_id, sizeof(craft_id))) {
                result = AIRBOUND_PARAMETER_RESULT_OK;
                gcs().send_text(MAV_SEVERITY_INFO, "AB:CRAFT_ID:%s", craft_id);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to fetch craft ID");
            }
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_CRAFT_ID, (const char*)craft_id, result);
            break;
        }
        case AIRBOUND_PARAMETER_ACTION_SET: {
            char craft_id[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.set_craft_id(packet.value)) {
                gcs().send_text(MAV_SEVERITY_INFO, "AB:Craft ID updated");
                result = AIRBOUND_PARAMETER_RESULT_OK;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to update craft ID");
            }
            g_custom_storage.get_craft_id(craft_id, sizeof(craft_id));
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_CRAFT_ID, (const char*)craft_id, result);
            break;
        }
        default: {
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_CRAFT_ID, (const char*)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        }
        break;

    case AIRBOUND_PARAMETER_PARAM_ID_UIN:
        switch (packet.action) {
        case AIRBOUND_PARAMETER_ACTION_GET: {
            char uin[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.get_uin(uin, sizeof(uin))) {
                result = AIRBOUND_PARAMETER_RESULT_OK;
                gcs().send_text(MAV_SEVERITY_INFO, "AB:UIN:%s", uin);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to fetch UIN");
            }
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UIN, (const char*)uin, result);
            break;
        }
        case AIRBOUND_PARAMETER_ACTION_SET: {
            char uin[MAX_AB_PARAM_SIZE] = {0};
            AIRBOUND_PARAMETER_RESULT result = AIRBOUND_PARAMETER_RESULT_FAILED;
            if (g_custom_storage.set_uin(packet.value)) {
                gcs().send_text(MAV_SEVERITY_INFO, "AB:UIN updated");
                result = AIRBOUND_PARAMETER_RESULT_OK;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "AB:Failed to update UIN");
            }
            g_custom_storage.get_uin(uin, sizeof(uin));
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UIN, (const char*)uin, result);
            break;
        }
        default: {
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UIN, (const char*)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        }
        break;

    default: {
        char buf[MAX_AB_PARAM_SIZE] = {0};
        mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char*)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
        break;
    }
    }
}
#endif