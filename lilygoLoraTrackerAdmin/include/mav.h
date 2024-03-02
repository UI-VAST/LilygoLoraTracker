#include <common/mavlink.h>
#include <AsyncUDP.h>

void init_mav();
void send_mavlink(float lat, float lon, float alt, float sats);
void parsePacket(AsyncUDPPacket packet);
void mav_fast_loop();
void mav_slow_loop();

void MVL_Transmit_Message(mavlink_message_t* mvl_msg_ptr);
void command_parameters(int8_t system_id, uint8_t component_id);
void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading);
void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats);
void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state);
void heart_beat();
void send_fake_telem();
void MVL_Handle_Command_Long(mavlink_message_t* mvl_msg_ptr);
void MVL_Handle_Mission_Request_List(mavlink_message_t* mvl_msg_ptr);
void MVL_Handle_Param_Request_List(mavlink_message_t* mvl_msg_ptr);
void recieved_message_handler();

