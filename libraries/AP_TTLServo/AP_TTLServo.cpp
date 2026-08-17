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
#define RESPONSE_PKT_FIXED_SIZE 4

// Register offset for goal position
#define GOAL_POSITION_REG 0x2A

// Register offset for running speed
#define RUNNING_SPEED_REG 0x2E

// Define the desired running speed
#define RUNNING_SPEED 2500

//Register address for Angle position state
#define ANGLE_POSITION_REG 0x38

// How many times to broadcast messages to configure the servos
#define CONFIGURE_SERVO_COUNT 4

// How many times should ping messages be sent to detect servos
#define DETECT_SERVO_COUNT 4

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
    
    comm_state = COMM_STATE::AWAITING_PING_RESPONSE;
    
    // Give plenty of time to receive replies from all servos
    last_send_us = AP_HAL::micros();
    delay_time_us += 1000 * us_per_byte;
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
void AP_TTLServo::process_packet(const uint8_t *packet, uint8_t length)
{

    switch(comm_state){
        case COMM_STATE::AWAITING_PING_RESPONSE:
        {
                uint8_t id = packet[PKT_ID];
                
                // Discard servos beyond the maximum permissible number of servo channels
                if (id > NUM_SERVO_CHANNELS) {
                        return;
                }  
   
                uint32_t id_mask = (1U<<(id));
                if (!(id_mask & servo_id_mask)) {
                        servo_states[id].present= true;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TTLServo: ID %u identified\n",id);
                        servo_id_mask.set_and_save_ifchanged(servo_id_mask+id_mask);
                }
                break;
        }
        case COMM_STATE::AWAITING_POS_RESPONSE:
        {
            uint8_t id = packet[PKT_ID];
            //IDs must not be higher than 4
            if(id> 4)
            {   
                return;
            }

            //If servoid was not pinged before
            if (servo_states[id].present == false)
            {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Pinging first time");
                return;
            }

            //Invalid position packet length
            if(packet[PKT_LENGTH]!=4)
            {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Wrong payload:%d %d", packet[PKT_LENGTH],length);
                return;
            }

            uint8_t low_byte = packet[5];
            uint8_t high_byte = packet[6];
            //Postion bytes: two bytes with bit 15 assigned to direction
            uint16_t direction = (high_byte>>7) & (1U)? -1: 1;
            uint16_t position = ((high_byte&0x7F)<<8)|low_byte;
            double angle_position = direction*position*0.087;
            servo_states[id].position = angle_position;
            last_pos_updated_time = AP_HAL::millis();
            break;
        }
        default: 
            bad_message_count++;
            // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Total len: %d",length); 
            // for(int i = 0; i<length;i++)
            // {
            //     GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"Recieved bytes %u: %u",i,packet[i]);
            
            // }
            // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TTLServo: Unexpected state:%d",bad_message_count);
            break;
    }
    
}

// Read the bytes received from responses
void AP_TTLServo::read_bytes(void)
{
    uint32_t n = port->available();
    // If no bytes received or received less than the required to decode an
    // instruction, return in order to wait for the required number of bytes
    if (n == 0 && pktbuf_ofs < PKT_INSTRUCTION) {
        return;
    }

    // Read from serial the maximum number of bytes that would fill the buffer
    if (n > sizeof(pktbuf) - pktbuf_ofs) {
        n = sizeof(pktbuf) - pktbuf_ofs;
    }
    for (uint8_t i = 0; i < n; i++) {
        pktbuf[pktbuf_ofs++] = port->read();
    }

    // Discard bad leading data. This should be rare
    while (pktbuf_ofs >= 2 && (pktbuf[0] != 0xFF || pktbuf[1] != 0xFF)) {
        memmove(pktbuf, &pktbuf[1], pktbuf_ofs-1);
        pktbuf_ofs--;
    }

    // If enough data hasn't been received, return
    if (pktbuf_ofs < 5) {
        return;
    }

    // Check if enough data has been received according to the Packet.
    // If it hasn't been received, return and wait for the rest of the Packet
    const uint8_t total_packet_length = pktbuf[PKT_LENGTH] + RESPONSE_PKT_FIXED_SIZE;
    if (total_packet_length > sizeof(pktbuf)) {
        pktbuf_ofs = 0;
        return;
    }

    if (pktbuf_ofs < total_packet_length) {
        return;
    }

    // Compare the Packet's CRC with the received Packet data. If it is equal,
    // the Packet has been received without data errors. Otherwise, just discard
    // the received Packet (had errors)
    const uint8_t crc = pktbuf[total_packet_length - 1];
    const uint8_t calc_crc = calculate_crc(pktbuf, total_packet_length - 1);
    if (calc_crc == crc) {
      process_packet(pktbuf, total_packet_length);
    }

    // Removed the processed Packet data from the buffer
    memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
    pktbuf_ofs -= total_packet_length;
}

// Send a command to the servos, changing a register value
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
    // Packet length equals number of Parameters (one of the params is the 
    // desired register + length of value) + 2
    tx_packet.length = 3 + len;
    tx_packet.reg = reg;
    tx_packet.value = value;
    

    send_packet((const uint8_t *) &tx_packet, tx_packet.length);
}

// Send a communication Packet
void AP_TTLServo::send_packet(const uint8_t *packet, uint8_t len)
{
    // Calculate total Packet length
    uint8_t total_packet_length = len + 1;
    uint8_t crc = 0;
    uint8_t tx_packet;
    uint8_t packet_header[2];
    
    // Send header
    packet_header[PKT_HEADER0] = 0xFF;
    packet_header[PKT_HEADER1] = 0xFF;
    port->write(packet_header, 2);
    hal.scheduler->delay_microseconds(us_per_byte*2);
    
    // Send remaining Packet
    while (total_packet_length) {
        tx_packet = *packet;
        if (port->write(tx_packet) == 1) {
            total_packet_length--;
            // Calculate CRC
            crc += tx_packet;
            packet++;
            hal.scheduler->delay_microseconds(us_per_byte);
        } else {
            // Communication error
            hal.scheduler->delay_microseconds(100);
            return;
        }
    }
    // Finally, transmit the CRC
    port->write(~crc);
    hal.scheduler->delay_microseconds(us_per_byte + us_gap);
    delay_time_us += (total_packet_length + 1) * us_per_byte + us_gap;
}

void AP_TTLServo::read_register(uint8_t servo_id, uint8_t reg_address, uint16_t num_of_bytes_to_read)
{
    struct packet {
        uint8_t id;                       //#1 Servo ID
        uint8_t length;                   //#2 Packet length 
        uint8_t instruction = INST_READ; //#3 Instruction is a Write
        uint8_t reg;                      //#4 First parameter is the register
        uint16_t value;           //#5 Following parameters is the value
    } tx_packet;

    tx_packet.id = servo_id;
    // Packet length equals number of Parameters (one of the params is the 
    // desired register + length of value) + 2ght
    tx_packet.length = 3+2;//ID(1)+Lengthfield(1)+READ_DATA(1)+REG_ADDRESS(1)+NUMBEROFBYTES(1)
    tx_packet.reg = reg_address;
    tx_packet.value = num_of_bytes_to_read;

    send_packet((const uint8_t *) &tx_packet, tx_packet.length);
}

void AP_TTLServo::read_angle_position(uint8_t servoid)
{
    read_register(servoid, ANGLE_POSITION_REG, 2);
    last_read_pos_cmd_time = AP_HAL::millis();
}

void AP_TTLServo::update()
{
    // Initialize the serial port
    if (!initialised) {
        memset(servo_position, 0xFF, sizeof(servo_position));
        init();
        if (servo_auto_det_en) {
            servo_id_mask.set_and_save(0);
        }
        initialised = true;
        comm_state = COMM_STATE::IDLE;
        last_send_us = AP_HAL::micros();
        return;
    }

    // If it wasn't possible to initialize serial port
    if (port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "TTLServo: No port found");
        return;
    }

    uint32_t now = AP_HAL::micros();

    // Read any data that may have been received
    read_bytes();


    // If auto-detection of servo IDs is enabled, we need send a Ping Packet in
    // order to receive servo IDs and check the data received to determine those
    // IDs
    if (servo_auto_det_en) {

        // Waiting for last send to complete
        if (last_send_us != 0 && now - last_send_us < delay_time_us) {
            return;
        }

        // Send a Ping Packet
        if (detection_count < DETECT_SERVO_COUNT) {
            detection_count++;
            detect_servos();
        }

        // If any servo wasn't detected, return
        if (servo_id_mask == 0) {
            return;
        }
    }

    // Configure the servos with the required values so they can work - sent by
    // broadcast Packet
    if (configured_servos < CONFIGURE_SERVO_COUNT) {
        configured_servos++;
        last_send_us = now;
        configure_servos();
        return;
    }

    last_send_us = now;
    delay_time_us = 0;

    switch(comm_state){
       case COMM_STATE::COMMAND_POSITION:
            servos_output();
            comm_state = COMM_STATE::READ_POSITION;
            break;
       case COMM_STATE::READ_POSITION:
            read_angle_position(3);
            comm_state = COMM_STATE::AWAITING_POS_RESPONSE;
            break;
        case COMM_STATE::AWAITING_PING_RESPONSE:
        case COMM_STATE::AWAITING_POS_RESPONSE:
            comm_state = COMM_STATE::COMMAND_POSITION;
            break;
        default:
            break;
    }

    if(!is_equal(prev_angle_position, servo_states[3].position))
    {
        prev_angle_position = servo_states[3].position;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Angle position: %f",prev_angle_position);
    }


}

void AP_TTLServo::servos_output(void)
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
        
        // Don't send goal position if it is equal to previous
        if (servo_position[i] == goalPosition) {
            continue;
        } else {
            servo_position[i] = goalPosition;
        }
   
        // Send the goal position to the servo
        send_command(i, GOAL_POSITION_REG, goalPosition, 2);
    }
}

#endif //NUM_SERVO_CHANNELS