#if defined(AP_ENABLE_CUSTOM_STORAGE) && AP_ENABLE_CUSTOM_STORAGE==1

#include "AP_CustomMavlinkHandler.h"
#include <string.h>
#include <stdio.h>

void AP_CustomMavlinkHandler::init(void){
    g_custom_storage.init();
}
void AP_CustomMavlinkHandler::handle_custom_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (msg.msgid != CUSTOM_MSG_ID)
        return;

    // Manual message decoding
    uuid_update_t packet;
    memcpy(&packet, msg.payload64, sizeof(packet));
    packet.value[MAX_AB_PARAM_SIZE - 1] = '\0'; // Ensure null termination

    switch (packet.param)
    {
    case AIRBOUND_PARAMETER_PARAM_ID_UUID:
        switch (packet.action)
        {
        case AIRBOUND_PARAMETER_ACTION_GET: // read
        {
            char uuid[MAX_AB_PARAM_SIZE] = {0};
            g_custom_storage.get_uuid(uuid, sizeof(uuid));
            gcs().send_text(MAV_SEVERITY_INFO, "uuid:%s", uuid);
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char *)uuid, AIRBOUND_PARAMETER_RESULT_OK);
            break;
        }
        case AIRBOUND_PARAMETER_ACTION_SET: // write
        {
            char uuid[MAX_AB_PARAM_SIZE] = {0};
            g_custom_storage.set_uuid(packet.value);
            g_custom_storage.get_uuid(uuid, sizeof(uuid));
            gcs().send_text(MAV_SEVERITY_INFO, "uuid updated : %s", uuid);
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char *)uuid, AIRBOUND_PARAMETER_RESULT_OK);

            break;
        }
        default:
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_UUID, (const char *)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        break; 

    case AIRBOUND_PARAMETER_PARAM_ID_PASS:
        switch (packet.action)
        {
        case AIRBOUND_PARAMETER_ACTION_GET: // read
        {
            char pass[MAX_AB_PARAM_SIZE] = {0};
            g_custom_storage.get_password(pass, sizeof(pass));
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char *)pass, AIRBOUND_PARAMETER_RESULT_OK);
            break;
        }
        case AIRBOUND_PARAMETER_ACTION_SET: // write
        {
            char pass[MAX_AB_PARAM_SIZE] = {0};
            g_custom_storage.set_password(packet.value);
            g_custom_storage.get_password(pass, sizeof(pass));
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char *)pass, AIRBOUND_PARAMETER_RESULT_OK);
            break;
        }
        default:
        {
            char buf[MAX_AB_PARAM_SIZE] = {0};
            mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char *)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
            break;
        }
        }
        break;

    default:
    {
        char buf[MAX_AB_PARAM_SIZE] = {0};
        mavlink_msg_airbound_parameter_status_send(chan, AIRBOUND_PARAMETER_PARAM_ID_PASS, (const char *)buf, AIRBOUND_PARAMETER_RESULT_UNSUPPORTED);
        break;
    }
    }
}
#endif