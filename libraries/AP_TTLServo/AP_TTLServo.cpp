/*
   Inspired by work done here
   https://github.com/tridge/ardupilot/tree/pr-robotis-servo from tridge

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   Implementation of communication protocol for controlling TTL Servos, using
   Robotis Dynamixel 1.0 protocol (https://emanual.robotis.com/docs/en/dxl/protocol1/)
   or FeetechRC protocol (similar to Dynamixel 1.0 protocol with minor differences
   which aren't important for this code)
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_TTLServo.h"

#if AP_FEETECHSERVO_ENABLED

#if NUM_SERVO_CHANNELS

extern const AP_HAL::HAL& hal;

// Common protocol reserved IDs
#define BROADCAST_ID       0xFE
#define MAX_ID             0xFD

// Common protocol commands
#define INST_PING          0x01
#define INST_READ          0x02
#define INST_WRITE         0x03
#define INST_REG_WRITE     0x04
#define INST_REG_ACTION    0x05
#define INST_SYNC_WRITE    0x83

// Specific Feetech protocol commands
#define INST_SYNC_READ     0x82

// Specific Robotis Dynamixel 1.0 protocol commands
#define INST_FACTORY_RESET 0x06
#define INST_REBOOT        0x08
#define INST_BULK_READ     0x92

// Protocol Packet offsets
#define PKT_HEADER0        0
#define PKT_HEADER1        1
#define PKT_ID             2
#define PKT_LENGTH         3
#define PKT_INSTRUCTION    4
#define PKT_ERROR          4
#define PKT_PARAMETER0     5

// Register offset for goal position
#define GOAL_POSITION_REG 0x2A

// Register offset for running speed
#define RUNNING_SPEED_REG 0x2E

// Define the desired running speed
#define RUNNING_SPEED 2500

// How many times to broadcast messages to configure the servos
#define CONFIGURE_SERVO_COUNT 1

// How many times should ping messages be sent to detect servos
#define DETECT_SERVO_COUNT 1



const AP_Param::GroupInfo AP_TTLServo::var_info[] = {

    // @Param: DET_EN
    // @DisplayName: TTL servo auto-detection
    // @Description: Enables or disables the auto-detection of the connected servo IDs. When servo bitmask isn't used or when auto-detection of the IDs of the connected servos is desired, enable this option. If disabled, set SERVO_TTL_ID_BM
    // @Values: 0:Auto-detection disabled, 1:Auto-detection enabled
    // @User: Advanced
    AP_GROUPINFO("DET_EN", 1, AP_TTLServo, servo_auto_det_en, 1),

    // @Param: POSMIN
    // @DisplayName: TTL servo min position
    // @Description: Minimum position of servo at its minimum value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMIN", 2, AP_TTLServo, pos_min, 0),

    // @Param: POSMAX
    // @DisplayName: TTL servo max position
    // @Description: Maximum position of servo at its maximum value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMAX", 3, AP_TTLServo, pos_max, 4095),

    // @Param: DESSPD
    // @DisplayName: Servo desired running speed
    // @Description: Value of the desired running speed of the servo. Value and units are servo dependent, see servo datasheet
    // @Range: 0 65535
    // @User: Standard
    AP_GROUPINFO("DESSPD", 4, AP_TTLServo, servo_des_run_speed, RUNNING_SPEED),

    // @Param: ID_BM
    // @DisplayName: Servo IDs bitmask
    // @Description: Bitmask of the servo IDs connected. Enable the servo in the corresponding servo_channel slot. Servo ID 0 corresponds to servo1_channel
    // @Bitmask: 0:ID 0, 1:ID 1, 2:ID 2, 3:ID 3, 4:ID 4, 5:ID 5, 6:ID 6, 7:ID 7, 8:ID 8, 9:ID 9, 10:ID 10, 11:ID 11, 12:ID 12, 13:ID 13, 14:ID 14, 15:ID 15, 16:ID 16, 17:ID 17, 18:ID 18, 19:ID 19, 20:ID 20, 21:ID 21, 22:ID 22, 23:ID 23, 24:ID 24, 25:ID 25, 26:ID 26, 27:ID 27, 28:ID 28, 29:ID 29, 30:ID 30, 31:ID 31
    // @User: Advanced
    AP_GROUPINFO("ID_BM", 5, AP_TTLServo, servo_id_mask, 0),

    AP_GROUPEND
};

// Constructor
AP_TTLServo::AP_TTLServo(void)
{
    // Set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

// Calculate communication protocol CRC (same as Robotis Dynamixel 1.0 protocol CRC)
uint8_t AP_TTLServo::calculate_crc(uint8_t *tx_packet, uint8_t len)
{
    uint8_t checkSum, i;

    for (i = PKT_ID, checkSum = 0; i < len; i++) {
        checkSum += tx_packet[i];
    }

    return(~checkSum);

}

// Use a broadcast to set the speed of all servos.
// Without speed configuration, servos will not run!
void AP_TTLServo::configure_servos(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Configuring servo");
    send_command(BROADCAST_ID, RUNNING_SPEED_REG, servo_des_run_speed, 2);
}

// Use a broadcast ping to find attached servos
void AP_TTLServo::detect_servos(void)
{
    struct packet {
        uint8_t id = BROADCAST_ID;       //#1 Packet is a broadcast
        uint8_t length = 2;              //#2 Packet Length equals number of Parameters (0) + 2
        uint8_t instruction = INST_PING; //#3 Instruction is a Ping
    } tx_packet;

    send_packet((const uint8_t *) &tx_packet, tx_packet.length);
}

// Init the serial port
void AP_TTLServo::init(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TTLServo: Initializing");
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TTLServo, 0);
    if (port) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TTLServo: Found Serial Port");
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_TTLServo, 0);
        us_per_byte = 10 * 1e6 / baudrate;
        us_gap = 4 * 1e6 / baudrate;
    }
}

// Process received Packet from servo
void AP_TTLServo::process_packet(const RESPONSE_TYPE& response,const uint8_t *packet, uint8_t length)
{
    if(length < 6)
    {
        return;
    }
    
    uint8_t id = packet[PKT_ID];

    // Discard servos beyond the maximum permissible number of servo channels
    if (id < 1 || id > NUM_SERVO_CHANNELS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TTLServo: Invalid Servo id:%d",id);
        for(int i = 0;i<length;i++)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Packet:%x",packet[i]);
        }
        return;
    }

    // If the servo wasn't previously identified, mark its existence on the network
    uint32_t id_mask = (1U<<(id-1));
    if (!(id_mask & servo_id_mask)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TTLServo: ID %u identified\n",id);
        servo_id_mask.set_and_save_ifchanged(servo_id_mask+id_mask);
        servo_count++;
    }

    switch (response)
    {
        case RESPONSE_TYPE::CURRENT_POSITION:
        {
            if(length != 8)
            {
#if TTLSERVO_DEBUG_LEVEL > 0
                _debug.bad_response_count++;
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: Invalid Position read response:%d",length);
#endif
#if TTLSERVO_DEBUG_LEVEL > 1

                for(int i = 0;i<length;i++)
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Packet:%x",packet[i]);
                }
#endif
            }
            else
            {
                uint8_t low = packet[5];
                uint8_t high = packet[6];
                uint16_t raw = (high<<8)|low;
                int8_t direction = (raw & 0x8000)==0?1:-1;
                uint16_t raw_magnitude = (raw & 0x7FFF);
                float position = direction * raw_magnitude * 0.087;
                int8_t i = id - 1;
                telem_data[i].angle = position;
                telem_data[i].last_response_ms = AP_HAL::millis();
#if TTLSERVO_DEBUG_LEVEL > 0
                if(!is_equal(telem_data[i].angle, position))
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo:Curr Position:%0.2f",position);

                }
                _debug.read_position_response_count++;
#endif
            }
            break;
        }
        case RESPONSE_TYPE::POSITION_COMMAND:
        {
            if(length != 6)
            {
#if TTLSERVO_DEBUG_LEVEL > 0
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: Invalid Position command response:%d",length);
                _debug.bad_response_count++;
#endif
#if TTLSERVO_DEBUG_LEVEL > 1
                for(int i = 0;i<length;i++)
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Packet:%x",packet[i]);
                }
#endif
            }
            else
            {
                uint8_t error_status = packet[4];
                if(error_status != 0)
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: Command Error: %d",error_status);
                }
                int8_t i = id - 1;
                telem_data[i].error_flags = error_status;
                telem_data[i].last_response_ms = AP_HAL::millis();
#if TTLSERVO_DEBUG_LEVEL > 0
                _debug.position_command_response_count++;
#endif
            }
            break;
        }
        case RESPONSE_TYPE::PING:
        {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TTLServo:Recieved ping response");
            break;
        }
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"TTLServo: Invalid response");
            break;
    }

}

// Read the bytes received from responses
void AP_TTLServo::read_bytes(const RESPONSE_TYPE& response)
{
    /*Each read must start the buffer afresh. 
    This ensures the command - response remains clean. 
    */
    uint32_t n = port->available();
    
    // If no bytes received or return in order to wait for the required number of bytes
    if (n == 0) {
        return;
    }

    if(n > rxbytes.get_size())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"TTLServo: RX Buffer insufficient");
        return;
    }

    for (uint8_t i = 0; i < n; i++) {
        uint8_t byte = port->read();
        rxbytes.write(&byte,1);
    }

    //Discard bad leading bytes
    uint8_t discard_count = 0;
    for(uint8_t i =0; i< rxbytes.available()-1;i++)
    {
        if((rxbytes.peek(i) == 0xFF && rxbytes.peek(i+1)== 0xFF))
        {
           rxbytes.advance(discard_count);
           break; 
        } 
        else if(i == rxbytes.available()-2)
        {
            //Clear buffer if no Header bytes found
            rxbytes.clear();
            break;
        }
        else
        {
            discard_count++;
        }
    }

    //Minimum size of a response is 6
    if(rxbytes.available()< 6)
    {
        return;
    }

    const uint8_t total_response_length = rxbytes.peek(PKT_LENGTH)+4;
    if(total_response_length > rxbytes.available())
    {
 
        return;
    }

    if(total_response_length> rxbytes.get_size())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"TTLServo: Response length exceeds max recieve buffer size");
        return;
    }

    //Minimun size of response is 6
    if(total_response_length < 6)
    {
        return;
    }

    //Verify CRC of response
    uint8_t response_packet[total_response_length];
    //Get response packet
    uint8_t ret = rxbytes.read(response_packet,total_response_length);
    if(ret != total_response_length)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: RX buffer read fail");
    }
    const uint8_t CRC = response_packet[total_response_length-1];
    const uint8_t calculated_crc = calculate_crc(response_packet, total_response_length - 1);
    
    if(CRC == calculated_crc)
    {
        process_packet(response,response_packet,total_response_length);
    }
    
}

void AP_TTLServo::send_position_read_command()
{
    uint8_t id = 1; 
    uint8_t reg_address = 0x38;
    uint8_t len = 2;
    send_read_register_instruction(id,reg_address,len);
#if TTLSERVO_DEBUG_LEVEL > 0    
    _debug.read_position_count++;
#endif
}

void AP_TTLServo::send_read_baudrate_command()
{
    uint8_t id = 1; 
    uint8_t reg_address = 0x06;
    uint8_t len = 1;
    send_read_register_instruction(id,reg_address,len);
}

void AP_TTLServo::send_read_voltage_command()
{
    uint8_t id = 1; 
    uint8_t reg_address = 0x3E;
    uint8_t len = 1;
    send_read_register_instruction(id,reg_address,len);
}

void AP_TTLServo::send_read_register_instruction(uint8_t id, uint8_t reg,uint8_t readlen)
{
    struct packet {
        uint8_t id;                       //#1 Servo ID
        uint8_t length;                   //#2 length field of feetech protocol.
        uint8_t instruction = INST_READ; //#3 Instruction read
        uint8_t reg;                      //#4 First parameter is the register address
        uint8_t readlength;             //#5
    } tx_packet;

    tx_packet.id = id;
    //lengthfield byte + instruction byte + Read address byte + Readlength byte
    tx_packet.length = 4;
    tx_packet.reg = reg;
    tx_packet.readlength = readlen;

    send_packet((const uint8_t *) &tx_packet, tx_packet.length);
}

// Send a command to the servos, changing a register value
// Relook at this function. Supports only write function with parameter length = 2
void AP_TTLServo::send_command(uint8_t id, uint8_t reg, uint16_t value, uint8_t len)
{
    struct packet {
        uint8_t id;                       //#1 Servo ID
        uint8_t length;                   //#2 Packet length 
        uint8_t instruction = INST_WRITE; //#3 Instruction is a Write
        uint8_t reg;                      //#4 First parameter is the register
        uint16_t value;           //#5 Following parameters is the value
    } tx_packet;
    
    tx_packet.id = id;
    // Packet length equals number of Parameters
    // (length byte + instruction + register byte + parameter 1 + parameter)
    tx_packet.length = 3 + len;
    tx_packet.reg = reg;
    tx_packet.value = value;
    

    send_packet((const uint8_t *) &tx_packet, tx_packet.length);
}

// Send a communication Packet
void AP_TTLServo::send_packet(const uint8_t *packet, uint8_t len)
{
    // Calculate total Packet length
    //Length field + 1 (ID field). Excludes Header bytes and CRC byte
    uint8_t total_packet_length = len + 1;
    uint8_t crc = 0;
    uint8_t tx_packet;
    uint8_t packet_header[2];
    
    // Send header
    packet_header[PKT_HEADER0] = 0xFF;
    packet_header[PKT_HEADER1] = 0xFF;
    port->write(packet_header, 2);
    // hal.scheduler->delay_microseconds(us_per_byte*2);
    
    // Send remaining Packet
    while (total_packet_length) {
        tx_packet = *packet;
        if (port->write(tx_packet) == 1) {
            total_packet_length--;
            // Calculate CRC
            crc += tx_packet;
            packet++;
            // hal.scheduler->delay_microseconds(us_per_byte);
        } else {
            // Communication error
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"TTLServo: comm error");
            // hal.scheduler->delay_microseconds(100);
            return;
        }
    }
    // Finally, transmit the CRC
    port->write(~crc);
    // hal.scheduler->delay_microseconds(us_per_byte*total_packet_length+3*us_per_byte + us_gap);
}

void AP_TTLServo::set_pwm()
{
        // Loop through all servo channels
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {

        // If this channel doesn't correspond to a servo ID, skip it
        if (((1U << i) & servo_id_mask) == 0) {
            continue;
        }

        SRV_Channel *c = SRV_Channels::srv_channel(i);

        if (c == nullptr) {
            continue;
        }

        // Calculate the desired goal position, converting the channel values
        // to the servo values
        const uint16_t pwm = c->get_output_pwm();
        const uint16_t min = c->get_output_min();
        const uint16_t max = c->get_output_max();
        float v = float(pwm - min) / (max - min);
        uint16_t goalPosition = (uint16_t)(pos_min) + (uint16_t)(v * (pos_max - pos_min));

        // Send the goal position to the servo
        uint8_t id = i+1;
        send_command(id, GOAL_POSITION_REG, goalPosition, 2);
#if TTLSERVO_DEBUG_LEVEL > 0
                _debug.position_command_count++;
#endif
    }
}

#if TTLSERVO_DEBUG_LEVEL > 0
void AP_TTLServo::print_debug()
{
    uint32_t now = AP_HAL::millis();
    if(now - _debug.last_gcs_announce_time > 5000)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: looptime:%lu",deltat); 
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: PositionCommandCount:%d",_debug.position_command_count); 
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: positioncommandresponsecount:%d",_debug.position_command_response_count);  
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: badresponsecount:%d",_debug.bad_response_count); 
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: ReadPosCount:%d",_debug.read_position_count); 
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo: ReadPosCountResponse:%d",_debug.read_position_response_count); 
        _debug.last_gcs_announce_time = AP_HAL::millis();
    }

}
#endif



void AP_TTLServo::update()
{
    // Initialize the serial port
    if (!initialised) {
        init();
        if (servo_auto_det_en) {
            servo_id_mask.set_and_save(0);
        }
        initialised = true;
        last_send_us = AP_HAL::micros();
        delay_time_us = 7*1e6;
        return;
    }

    // If it wasn't possible to initialize serial port
    if (port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TTLServo: No port found");
        return;
    }

    // If auto-detection of servo IDs is enabled, we need send a Ping Packet in
    // order to receive servo IDs and check the data received to determine those
    // IDs
    if (servo_auto_det_en && !auto_detect_complete) {

        // Read any data that may have been received
        read_bytes(servo_response);

        // Waiting for last send to complete
        if (last_send_us != 0 && AP_HAL::micros() - last_send_us < delay_time_us) {
            return;
        }

        // Send a Ping Packet
        if (detection_count < DETECT_SERVO_COUNT) {
            detection_count++;
            detect_servos();
            servo_response = RESPONSE_TYPE::PING;
            last_send_us = AP_HAL::micros();
            delay_time_us = 200 * us_per_byte;
            return;
        }

        auto_detect_complete = true;
    }

    
    // If any servo wasn't detected, return
    if (auto_detect_complete && servo_id_mask == 0 ) {
        if (!_gcs_announce.empty_servo_bus)
        {    
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo:Empty Servo bus");
            _gcs_announce.empty_servo_bus = true;
        }
        return;
    }
    
    
    switch(servo_comm_state){
        case COMM_STATE::IDLE:
            if(servo_id_mask>0)
            {
                servo_comm_state = COMM_STATE::COMMAND_POSITION;
            }
            break;
        
        case COMM_STATE::COMMAND_POSITION:
            set_pwm();
            servo_comm_state = COMM_STATE::GET_COMMAND_POSITION_RESPONSE;
            last_send_us = AP_HAL::micros();
            servo_response = RESPONSE_TYPE::POSITION_COMMAND;
            FALLTHROUGH;
        
        case COMM_STATE::GET_COMMAND_POSITION_RESPONSE:
            read_bytes(servo_response);
            if(AP_HAL::micros() - last_send_us > 800)
            {
                servo_comm_state = COMM_STATE::READ_CURRENT_POSITION;
                rxbytes.clear();
            }
            break;

        case COMM_STATE::READ_CURRENT_POSITION:
            send_position_read_command();
            servo_comm_state = COMM_STATE::GET_CURRENT_POSITION_RESPONSE;
            last_send_us = AP_HAL::micros();
            servo_response = RESPONSE_TYPE::CURRENT_POSITION; 
            FALLTHROUGH;
        
        case COMM_STATE::GET_CURRENT_POSITION_RESPONSE:
        {
            read_bytes(servo_response);
            if(AP_HAL::micros() - last_send_us > 800)
            {
                servo_comm_state = COMM_STATE::COMMAND_POSITION;
                rxbytes.clear();
            }
            break;
        }
        default:
            break;
    }

    update_telem();
#if TTLSERVO_DEBUG_LEVEL > 0
    print_debug();
#endif    
    deltat = AP_HAL::millis() - last_update_time;
    last_update_time = AP_HAL::millis();

    // if(AP_HAL::millis()-last_gcs_announce_t > 5000)
    // {
    //     GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"TTLServo:Looptime:%lu",deltat);
    //     last_gcs_announce_t = AP_HAL::millis();
    // }
}

void AP_TTLServo::update_telem()
{
    // Report telem data
    AP_Servo_Telem *servo_telem = AP_Servo_Telem::get_singleton();
    if (servo_telem != nullptr) {
        const uint32_t now_ms = AP_HAL::millis();
        
        for (uint8_t i=0; i<ARRAY_SIZE(telem_data); i++) {


            if ((telem_data[i].last_response_ms == 0) || ((now_ms - telem_data[i].last_response_ms) > 5000)) {
                // Never seen telem, or not had a response for more than 5 seconds
                continue;
            }

            const AP_Servo_Telem::TelemetryData data {
                .command_position = 0,
                .measured_position = telem_data[i].angle,
                .voltage = 0,
                .current = 0,
                .status_flags = telem_data[i].error_flags,
                .present_types = AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION |
                                AP_Servo_Telem::TelemetryData::Types::STATUS,
            };

            servo_telem->update_telem_data(i, data);
        }
    }

}

#endif //NUM_SERVO_CHANNELS

#endif //AP_FEETECHSERVO_ENABLED