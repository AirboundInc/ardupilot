#pragma once
#include <GCS_MAVLink/GCS.h>
#include "AP_CustomStorage/AP_CustomStorage.h"

class AP_CustomMavlinkHandler
{
public:
    static void handle_custom_message(mavlink_channel_t chan, const mavlink_message_t &msg);

// Manually define our message structure
#pragma pack(push, 1)
    typedef struct
    {
        uint8_t param;
        uint8_t action;
        char value[37];
    } uuid_update_t;
#pragma pack(pop)
    static const uint16_t CUSTOM_MSG_ID = 15222;
};