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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#ifndef AP_FEETECHSERVO_ENABLED
#define AP_FEETECHSERVO_ENABLED 1
#endif

#define TTLSERVO_DEBUG_LEVEL 1

#if AP_FEETECHSERVO_ENABLED
class AP_TTLServo {
  public:

    AP_TTLServo();

    // Do not allow copies
    CLASS_NO_COPY(AP_TTLServo);

    static const struct AP_Param::GroupInfo var_info[];

    void update();

  private:

    AP_HAL::UARTDriver *port;
    uint32_t baudrate;
    uint32_t us_per_byte;
    uint32_t us_gap;
    uint32_t read_command_sent_time_us;

    enum class COMM_STATE
    {
      IDLE = 0,
      COMMAND_POSITION,
      GET_COMMAND_POSITION_RESPONSE,
      READ_CURRENT_POSITION,
      GET_CURRENT_POSITION_RESPONSE,
    };

    enum class RESPONSE_TYPE
    {
      PING,
      POSITION_COMMAND,
      CURRENT_POSITION,
    }servo_response;


    uint8_t calculate_crc(uint8_t *txpacket, uint8_t len);
    void configure_servos(void);
    void detect_servos(void);
    void init(void);
    void process_packet(RESPONSE_TYPE response,const uint8_t *packet, uint8_t length);
    void read_bytes(RESPONSE_TYPE response);

    void send_command(uint8_t id, uint8_t reg, uint16_t value, uint8_t len);
    void send_packet(const uint8_t *packet, uint8_t len);
    void send_read_register_instruction(uint8_t id, uint8_t reg,uint8_t readlen);
  
    void send_position_read_command();
    void send_read_baudrate_command();
    void send_read_voltage_command();

    //Servo
    void set_pwm();

    COMM_STATE servo_comm_state=COMM_STATE::IDLE;
    
    struct gcs_announce{
      bool empty_servo_bus = false;
    }_gcs_announce;

    struct Servo_state
    {
      float angular_position_deg;
      uint32_t last_position_update_ms;
    };

    Servo_state servo_state[4];

    bool initialised;
    bool auto_detect_complete;

    // Used for the auto-detection of the servo IDs connected
    uint8_t detection_count;
    int8_t configured_servos;
    int8_t ping_count;

    // Received data buffer
    uint8_t pktbuf[64];
    uint8_t pktbuf_ofs;

    // Keep track of the data sent required time
    uint32_t last_send_us;
    uint32_t delay_time_us;
    
    // Keep track of the servo positions
    uint16_t servo_position[32];

    // PARAMETERS
    // Servo position limits
    AP_Int16 pos_min;
    AP_Int16 pos_max;

    // Enable servo auto-detection
    AP_Int8 servo_auto_det_en;

    // Servo desired running speed
    AP_Int16 servo_des_run_speed;

    // Servo desired running speed register address
    AP_Int8 servo_des_run_speed_reg;

    // Servo ID mask
    AP_Int32 servo_id_mask;

    // Servo goal position register adress
    AP_Int8 servo_goal_pos_reg;

#if TTLSERVO_DEBUG_LEVEL > 0
    struct debug{
      uint16_t position_command_count;
      uint16_t position_command_response_count;
      uint16_t read_position_count;
      uint16_t read_position_response_count;
      uint16_t bad_response_count;
      bool operator!=(const debug& other) const{return position_command_count != other.position_command_count 
                                                ||position_command_response_count != other.position_command_response_count 
                                                || bad_response_count != other.bad_response_count
                                                || read_position_count != other.read_position_count
                                                || read_position_response_count != other.read_position_response_count;}
      uint32_t last_gcs_announce_time;
    };

    debug _debug;
    debug _prev_debug; 
    void print_debug();

#endif

};

#endif //AP_FEETECHSERVO_ENABLED