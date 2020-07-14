#include <C:\Users\drozd\Documents\Arduino\libraries\mavlink\standard\mavlink.h> //need to find latest c++ mavlink library

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

void setup() {
  // MAVLink interface start
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("MAVLink starting.");
}

void loop() {

  // MAVLink
  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  //uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {
    // Record last HB update
    previousMillisMAVLink = currentMillisMAVLink;

    //Mav_Request_Data();
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      // Request streams from Pixhawk
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }

  }

  // Check reception buffer
  comm_receive();
}


void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // To be setup according to the needed information to be requested from the Pixhawk
//  const int  maxStreams = 1;
//  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_POSITION};
//  const uint16_t MAVRates[maxStreams] = {0x02};
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {/*MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1,*/ MAV_CMD_CONDITION_CHANGE_ALT};
  const uint16_t MAVRates[maxStreams] = {/*0x02,0x02,*/0x05};

  for (int i = 0; i < maxStreams; i++) {
      mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
//    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//    Serial.write(buf, len);
//    mavlink_msg_request_data_stream_pack(6, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
  }
}

void comm_receive() {
      mavlink_message_t msg;
      mavlink_status_t status;
      while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      Serial.print("DEBUG msgid:"); Serial.println(msg.msgid);
      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            //Serial.println(attitude.roll);
          }
          break;

        
       default:
          break;
      }
    }
  }
} 
