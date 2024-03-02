#include <Arduino.h>
#include <mav.h>


AsyncUDP udp;

const int port = 14550;
IPAddress ip;

mavlink_message_t mvl_tx_message; //A special MAVLink message data structure. 
mavlink_message_t mvl_rx_message;
mavlink_status_t mvl_rx_status;
const uint8_t mvl_compid = 1; //Component ID and System ID identify us to QGroundControl
const uint8_t mvl_sysid = 1;
const uint8_t mvl_chan = MAVLINK_COMM_1;  //MAVLink channel 1 appears to be required at least for Blue Robotics QGC

uint32_t t_last_hb = 0;
const uint32_t sys_stat_interval = 100; //10 system status messages per second
uint32_t t_last_sys_stat =0;
int16_t sys_stat_count = 0;
uint8_t mvl_armed = 0;
uint8_t mvl_packet_received = 0;



uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
uint8_t component_id = 240;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
uint8_t system_type = MAV_TYPE_FREE_BALLOON;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t system_mode = 64;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
uint32_t custom_mode = 0;     // Usually set to 0
uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter

int16_t heading = 0;      // Geographical heading angle in degrees
float lat = 0;   // GPS latitude in degrees (example: 47.123456)
float lon = 0;   // GPS longitude in degrees
float alt = 0;        // Relative flight altitude in m
float groundspeed = 0; // Groundspeed in m/s
float airspeed = 0.0;    // Airspeed in m/s
float climbrate = 0.0;    // Climb rate in m/s, currently not working

// GPS parameters
int16_t gps_sats = 0;    // Number of visible GPS satellites
int32_t gps_alt = 0.0;  // GPS altitude (Altitude above MSL)
float gps_hdop = 100.0;     // GPS HDOP
uint8_t fixType = 3;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix



void init_mav(){
    


    udp.onPacket([](AsyncUDPPacket packet)
    {
      parsePacket(packet);
    });
}

void parsePacket(AsyncUDPPacket packet)
{
    Serial.println(packet.remoteIP().toString());

}

void send_mavlink(float lat, float lon, float alt, float sats){

    gps_alt = alt;
    command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);
}

void mav_fast_loop(){

    
    //mvl_packet_received = mavlink_parse_char(mvl_chan,rxbyte, &mvl_rx_message, &mvl_rx_status);

}

void mav_slow_loop(){
    //heart_beat();
    send_fake_telem();
    command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);
    command_parameters(system_id, component_id);
    //recieved_message_handler();
}




void MVL_Transmit_Message(mavlink_message_t* mvl_msg_ptr)
{
  uint8_t tx_byte_buffer[512]={0}; //A byte buffer that will be sent from the serial port.
  uint16_t tx_buflen = mavlink_msg_to_send_buffer(tx_byte_buffer,mvl_msg_ptr);

  //Serial.write(tx_byte_buffer,tx_buflen);
  udp.broadcastTo(tx_byte_buffer,tx_buflen, port);
}



void command_parameters(int8_t system_id, uint8_t component_id) {

    // Initialize the required buffers
    mavlink_message_t msg;
    // Pack the message
    mavlink_msg_param_value_pack(system_id, component_id, &msg, "RC_SPEED", 50, 1, 1, 0);
    MVL_Transmit_Message(&msg);

}

void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {

    int16_t velx = 0; //x speed
    int16_t vely = 0; //y speed
    int16_t velz = 0; //z speed

    // Initialize the required buffers
    mavlink_message_t msg;
    // Pack the message
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat * 10000000.0, lon * 10000000.0, gps_alt * 1000.0, alt * 1000.0, velx, vely, velz, heading);
MVL_Transmit_Message(&msg);
}

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

    // Initialize the required buffers
    mavlink_message_t msg;
    // Pack the message
    mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat * 10000000.0, lon * 10000000.0, alt * 1000.0, gps_hdop * 100.0, 65535, groundspeed, 65535, gps_sats, 0, 0, 0, 0, 0);
    MVL_Transmit_Message(&msg);

}

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

    // Initialize the required buffers
    mavlink_message_t msg;
    // Pack the message
    mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
    MVL_Transmit_Message(&msg);
}


void heart_beat(){
   //#0 HEARTBEAT https://mavlink.io/en/messages/common.html#HEARTBEAT
    mavlink_heartbeat_t mvl_hb; //struct with user fields: uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status;
    mvl_hb.type = MAV_TYPE_FREE_BALLOON; //My vehicle is an underwater ROV. Change as appropriate. See: https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L69
    mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC; //See https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L40
    mvl_hb.system_status = MAV_STATE_ACTIVE;
    if (mvl_armed) 
    {
      mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
    }
    else 
    { 
      mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
    }
    mvl_hb.base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; //I always use CUSTOM_MODE_ENABLED
    mvl_hb.custom_mode=0xABBA; //custom mode, can be anything, I guess
    mavlink_msg_heartbeat_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_hb);
    MVL_Transmit_Message(&mvl_tx_message);
 
}

void send_fake_telem(){
    //We'll make up some fake periodic data to feed to the QGroundControl widget
    float pfreq = 0.2; //slowly varying
    float phaserad = 2*PI*pfreq*millis()/1000.0;
    
    //We'll send a varying voltage signal that ramps from 10000 to 16,000 mV over 60 sys_stat_intervals... 10-16V on the QGC widget
    int16_t angle_as_mV = sys_stat_count*100+10000;
    sys_stat_count+=1;
    sys_stat_count%=60;
    //I often use the mavlink_<message name>_pack_chan() functions that 
    //accept each field as an argument instead of the mavlink_<message name>_encode() that
    //accepts a struct. They should save some memory to skip the extra
    //message struct, but I think setting each field by name in a demo code is easier to follow.
    
    mavlink_sys_status_t mvl_sys_stat; //#1 SYS_STATUS https://mavlink.io/en/messages/common.html#SYS_STATUS
    mvl_sys_stat.onboard_control_sensors_present = 0; //To set these, consult https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR
    mvl_sys_stat.onboard_control_sensors_enabled = 0;
    mvl_sys_stat.onboard_control_sensors_health = 0;
    mvl_sys_stat.load = 0;
    mvl_sys_stat.voltage_battery = angle_as_mV; //the only non-trivial telemetry we're sending, shows up several places in QGC
    mvl_sys_stat.current_battery = -1;
    mvl_sys_stat.battery_remaining = -1;
    mvl_sys_stat.drop_rate_comm = 0;
    mvl_sys_stat.errors_comm = 0;
    mvl_sys_stat.errors_count1 = 0;
    mvl_sys_stat.errors_count2 = 0;
    mvl_sys_stat.errors_count3 = 0;
    mvl_sys_stat.errors_count4 = 0;
    
    //We'll also send an attitude quaternion to display something in the QGC
    //roll/pitch widget and in the compass.
    //The code below results in a gentle spherical rocking in the QGC roll/pitch widget
    //and a continuous rotation of the displayed heading.
    
    float maxang = 0.0873; // about five degrees
    float roll = maxang*sin(phaserad);
    float pitch = maxang*cos(phaserad);
    float yaw = phaserad;
    
    //Quaternion conversion taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
  
    mavlink_attitude_quaternion_t mvl_att_quat; //#31 ATTITUDE_QUATERNION https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION
    mvl_att_quat.time_boot_ms = millis();
    mvl_att_quat.q1 = cy * cr * cp + sy * sr * sp; //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    mvl_att_quat.q2 = cy * sr * cp - sy * cr * sp;
    mvl_att_quat.q3 = cy * cr * sp + sy * sr * cp;
    mvl_att_quat.q4 = sy * cr * cp - cy * sr * sp;
    
    mvl_att_quat.rollspeed = 2*PI*pfreq*maxang*cos(phaserad); //d/dt A*sin(2*pi*f*t) = 2*pi*f*A*cos(2*pi*f*t)
    mvl_att_quat.pitchspeed = -2*PI*pfreq*maxang*sin(phaserad);
    mvl_att_quat.yawspeed = 2*PI*pfreq; 
    
    mavlink_msg_sys_status_encode_chan(mvl_sysid,mvl_compid,mvl_chan, &mvl_tx_message,&mvl_sys_stat);
    MVL_Transmit_Message(&mvl_tx_message);
    mavlink_msg_attitude_quaternion_encode_chan(mvl_sysid,mvl_compid,mvl_chan, &mvl_tx_message,&mvl_att_quat);
    MVL_Transmit_Message(&mvl_tx_message);
  }

void MVL_Handle_Command_Long(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_command_long_t mvl_cmd;
  mavlink_msg_command_long_decode(mvl_msg_ptr,&mvl_cmd);
  switch (mvl_cmd.command)
  {
    case (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES):
    {
      if (1==mvl_cmd.param1)
      {
        mavlink_autopilot_version_t mvl_apv; https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
        mvl_apv.flight_sw_version = 2;
        mvl_apv.middleware_sw_version = 1;
        mvl_apv.board_version = 1;
        mvl_apv.vendor_id = 10101;
        mvl_apv.product_id = 20202;
        mvl_apv.uid = 0;
        mvl_apv.capabilities = 0; //See: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY
        mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET; //Just as an example, code does not support! https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
        mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
        mavlink_msg_autopilot_version_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                                  &mvl_tx_message,&mvl_apv);
        MVL_Transmit_Message(&mvl_tx_message);
      }
      break;
    }//end handling of autopilot capabilities request
    case (MAV_CMD_COMPONENT_ARM_DISARM):
    {
      if (1==mvl_cmd.param1)
      {
        mvl_armed = 1;
      }
      else
      {
        mvl_armed = 0;
      }
      //Acknowledge the arm/disarm command.
      mavlink_command_ack_t mvl_ack; //https://mavlink.io/en/messages/common.html#COMMAND_ACK
      mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM; //https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
      mvl_ack.result = MAV_RESULT_ACCEPTED; //https://mavlink.io/en/messages/common.html#MAV_RESULT
      mavlink_msg_command_ack_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                          &mvl_tx_message,&mvl_ack);
      //skipped setting several fields here, with unknown consequences.
      MVL_Transmit_Message(&mvl_tx_message);
      break;
    }//end handling of arm/disarm command
  }//end switch/case
}//end MVL_Handle_Command_Long()

void MVL_Handle_Mission_Request_List(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_mission_count_t mvl_mc;
  mvl_mc.target_system = mvl_sysid;
  mvl_mc.target_component = mvl_compid;
  mvl_mc.count = 0;
  mavlink_msg_mission_count_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                        &mvl_tx_message,&mvl_mc);
  MVL_Transmit_Message(&mvl_tx_message);
}

void MVL_Handle_Param_Request_List(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_param_value_t mvl_param;
  
  mvl_param.param_id[0] = 'a'; //a parameter ID string, less than 16 characters.
  mvl_param.param_id[1] = '_';
  mvl_param.param_id[2] = 'p';
  mvl_param.param_id[3] = 'a';
  mvl_param.param_id[4] = 'r';
  mvl_param.param_id[5] = 'm';
  mvl_param.param_id[6] = 0; //null terminated
  mvl_param.param_value = 123.456; //the parameter value as a float
  mvl_param.param_type = MAV_PARAM_TYPE_REAL32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
  mvl_param.param_count = 1; //We have just one parameter to send. 
  mvl_param.param_index = 0; 
  mavlink_msg_param_value_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_param);
  MVL_Transmit_Message(&mvl_tx_message);
  
}

void recieved_message_handler(){
    if ((mvl_packet_received) && (255==mvl_rx_message.sysid)) 
    {
        mvl_packet_received = 0; //reset the "packet received" flag
        switch (mvl_rx_message.msgid)
        {
        case MAVLINK_MSG_ID_MANUAL_CONTROL: //#69 https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        {
            //MVL_Handle_Manual_Control(&mvl_rx_message);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21 https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
        {
            MVL_Handle_Param_Request_List(&mvl_rx_message);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: //#76 https://mavlink.io/en/messages/common.html#COMMAND_LONG
        {
            MVL_Handle_Command_Long(&mvl_rx_message);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //#43 https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST
        {
            MVL_Handle_Mission_Request_List(&mvl_rx_message);
            break;
        }
        
        }
    }    
}   