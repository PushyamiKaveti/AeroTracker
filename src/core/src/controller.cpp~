#include "ros/ros.h"
#include "mavlink/v1.0/common/mavlink.h"
#include <core/Camera_msgs.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <ros_mavlink/Mavlink.h>

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>


using namespace std;

// Settings
int sysid = 1;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 1;

//Global Variables

ros::Publisher controller_mav_pub;
ros::Subscriber controller_sensor_sub;
ros::Subscriber controller_mav_sub;
ros::Subscriber command_sub;
ros::Publisher  command_pub;
int bs_mode;
float altitude;
int roll_max=1881;
int roll_min = 1046;
int roll_stable=1400;
int pitch_min=1052;
int pitch_stable=1400;
int pitch_max=1886;
int current_roll;
int current_pitch;
int current_throttle;
int current_yaw;


//function definitions
void launch_loiter_land();
void initial_setup();
mavlink_message_t create_rc_channel_override_msg(int,int,int,int,int,int,int,int);
void publish_ros_message(mavlink_message_t&);

class Pid{
	float gain_p;
	float gain_i;
	float gain_d;
    float old_error;
    float integral_error;
public:
	Pid();
	Pid(float,float,float);
	void set_gain_p(float);
	void set_gain_i(float);
	void set_gain_d(float);
	float get_gain_p();
	float get_gain_i();
	float get_gain_d();
	float get_old_error();
	void set_old_error(float);
	float get_output(float);
};

Pid::Pid(){
	gain_d=0.0;
	gain_i=0.0;
	gain_p=0.0;
	integral_error=0.0;
	old_error=0.0;
}

Pid::Pid(float p, float i, float d){
	gain_d=d;
	gain_i=i;
	gain_p=p;
	integral_error=0.0;
	old_error=0.0;
}
void Pid::set_gain_p(float p){
	gain_p =p;
}

void Pid::set_gain_i(float i){
	gain_i =i;
}

void Pid::set_gain_d(float d){
	gain_d =d;
}

float Pid::get_gain_p(){
	return gain_p;
}

float Pid::get_gain_i(){
	return gain_i ;
}

float Pid::get_gain_d(){
	return gain_d ;
}


float Pid::get_old_error() {
	return old_error;
}

void Pid::set_old_error(float e) {
	old_error = e;
}


float Pid::get_output(float error){
	float delta_time = 0.5;
	float delta_error, output_signal;
	delta_error=error-old_error;
	integral_error+=error;
	output_signal=gain_p*error + (gain_i/delta_time)*integral_error + (gain_d/delta_time)*delta_error;
	old_error = error;
    return output_signal;
}

void sensor_callback(const core::Camera_msgs &cam_track_msg){

     static bool swtch=true;
     Pid roll_pid = Pid();
     Pid pitch_pid = Pid();
     roll_pid.set_gain_p(0.04);
     pitch_pid.set_gain_p(0.04);
     float roll_out,pitch_out;
     float roll_error = (float)cam_track_msg.centroid_x-cam_track_msg.center_x;
     float pitch_error = (float)cam_track_msg.center_y-cam_track_msg.centroid_y;
     if(swtch){
		 if(roll_error>=abs(20)){

			 roll_out= roll_pid.get_output(roll_error);
			 roll_out+=current_roll;
			 if(roll_out>=roll_max || roll_out<=roll_min)
				 roll_out=current_roll;

		  }
		 else{
			 roll_out = roll_stable;
		 }
		 pitch_out=pitch_stable;
		 swtch=false;
     }
     else{
		 if(pitch_error>=abs(20)){
			pitch_out = pitch_pid.get_output(pitch_error);
			pitch_out+=current_pitch;
			if(pitch_out>=pitch_max || pitch_out<=pitch_min)
					 pitch_out=current_pitch;
		 }
		 else{
			 pitch_out=pitch_stable;
		 }
		 roll_out=roll_stable;
		 swtch=true;

     }

     printf("roll_error %f roll out: %f \n pitch error %f pitch_out: %f \n ",roll_error,roll_out,pitch_error,pitch_out);
     mavlink_message_t rc_msg = create_rc_channel_override_msg(roll_out,pitch_out,current_throttle,current_yaw,1000,1000,1000,1000);
     publish_ros_message(rc_msg);
}

void mavlink_callback(const ros_mavlink::Mavlink &mavlink_ros_msg){

	mavlink_message_t message;
	static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

	message.msgid = mavlink_ros_msg.msgid;
	message.compid=mavlink_ros_msg.compid;
	message.sysid = mavlink_ros_msg.sysid;
	message.seq = mavlink_ros_msg.seq;
	message.len = mavlink_ros_msg.len;
	message.magic = mavlink_ros_msg.magic;
	message.checksum = mavlink_ros_msg.checksum;

    //printf("controller call back. message id : %d \n ",mavlink_ros_msg.msgid);

	  //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
	//copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), message.payload64);
	for (int i = 0; i < sizeof(mavlink_ros_msg.payload64); i++)
	        {
	          message.payload64[i]=mavlink_ros_msg.payload64[i];

	        }

	//mavlink_finalize_message_chan(&message, mavlink_ros_msg.sysid, mavlink_ros_msg.compid, MAVLINK_COMM_0,mavlink_ros_msg.len, mavlink_crcs[message.msgid]);

	static unsigned int scaled_imu_receive_counter = 0;
	static unsigned int stupid_counter = 0;
	switch (message.msgid)
				{
					case MAVLINK_MSG_ID_HIGHRES_IMU:
					{
						if (scaled_imu_receive_counter % 50 == 0)
						{
							mavlink_highres_imu_t imu;
							mavlink_msg_highres_imu_decode(&message, &imu);

							printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
							printf("\t time: %llu\n", imu.time_usec);
							printf("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
							printf("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
							printf("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
							printf("\t baro: \t %f (mBar)\n", imu.abs_pressure);
							printf("\t altitude: \t %f (m)\n", imu.pressure_alt);
							printf("\t temperature: \t %f C\n", imu.temperature);
							printf("\n");
						}
						scaled_imu_receive_counter++;
					}
					break;

					case MAVLINK_MSG_ID_HEARTBEAT:
					{

						//printf(" Heart Beat ! ! !System ID: %d  Component ID: %d \n ", message.sysid, message.compid);
						stupid_counter++;
						mavlink_heartbeat_t hrt;
						mavlink_msg_heartbeat_decode(&message,&hrt);
						if(stupid_counter == 5){
							initial_setup();
						}
						bs_mode=hrt.base_mode;
						//printf(" Heart Beat  base : %d cus mode : %d \n ", hrt.base_mode,hrt.custom_mode);
					}
					    break;
					case MAVLINK_MSG_ID_RAW_IMU:
					{
						mavlink_raw_imu_t raw_imu;
						mavlink_msg_raw_imu_decode(&message,&raw_imu);
						//printf("\t acc  %d  %d  %d (m/s^2)\n", raw_imu.xacc, raw_imu.yacc, raw_imu.zacc);
						//printf("\t gyro %d  %d  %d (rad/s)\n", raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro);
					}
					break;

					case MAVLINK_MSG_ID_PARAM_VALUE:
					{
						mavlink_param_value_t par_val;
						mavlink_msg_param_value_decode(&message,&par_val);
						printf("param id %s  param value  %f \n", par_val.param_id,par_val.param_value);
					}
					break;

					case MAVLINK_MSG_ID_VFR_HUD:
					{
						mavlink_vfr_hud_t vfr;
						mavlink_msg_vfr_hud_decode(&message,&vfr);
						altitude = vfr.alt;
						//printf("altitude: %f  throttle %d \n",vfr.alt,vfr.throttle);
					}
					break;

					case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
					{

						mavlink_rc_channels_raw_t rc_raw;
						mavlink_msg_rc_channels_raw_decode(&message,&rc_raw);
						current_roll=rc_raw.chan1_raw;
						current_pitch=rc_raw.chan2_raw;
						current_throttle=rc_raw.chan3_raw;
						//printf("channel 1: %d  2: %d 3: %d 4: %d \n", rc_raw.chan1_raw,rc_raw.chan2_raw,rc_raw.chan3_raw,rc_raw.chan4_raw);
						scaled_imu_receive_counter++;

					}
					break;

					case MAVLINK_MSG_ID_COMMAND_ACK:
					{
						mavlink_command_ack_t cmd_ack;
						mavlink_msg_command_ack_decode(&message,&cmd_ack);
						//printf("cmd id : %d  result : %d \n",cmd_ack.command,cmd_ack.result);
					}
					break;

					case MAVLINK_MSG_ID_GPS_RAW_INT:
					{
						if (scaled_imu_receive_counter % 50 == 0)
												{
						mavlink_gps_raw_int_t gps;
						mavlink_msg_gps_raw_int_decode(&message,&gps);
						//printf("GPS LAT: %d  LON: %d  VEL: %d HDOP: %d \n",gps.lat,gps.lon,gps.vel, gps.eph);
						}
						scaled_imu_receive_counter++;

					}
					break;


					case MAVLINK_MSG_ID_STATUSTEXT:
					{
						mavlink_statustext_t stat_txt;
						mavlink_msg_statustext_decode(&message,&stat_txt);
						//printf("severity : %d  text : %s \n",stat_txt.severity,stat_txt.text);
					}
					break;
					default:
						//printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
	                    break;
				}

}

void command_callback(const std_msgs::Int16 &command_msg){

    if(command_msg.data == 1){
    	initial_setup();

    }

}
void publish_ros_message(mavlink_message_t &message){

	ros_mavlink::Mavlink mavlink_msg;
    mavlink_msg.len = message.len;
    mavlink_msg.seq = message.seq;
    mavlink_msg.sysid = message.sysid;
    mavlink_msg.compid = message.compid;
    mavlink_msg.msgid = message.msgid;
    mavlink_msg.checksum = message.checksum;
    mavlink_msg.magic = message.magic;
    for (int i = 0; i < sizeof(message.payload64); i++)
      {
        (mavlink_msg.payload64).push_back(message.payload64[i]);
      }
    //printf("check %d\n",message.msgid);
      controller_mav_pub.publish(mavlink_msg);
}

mavlink_message_t create_request_params_msg(int sysid, int compid){

	mavlink_message_t message;
	mavlink_param_request_list_t msg_struct;
	msg_struct.target_component=compid;
	msg_struct.target_system=sysid;
	mavlink_msg_param_request_list_encode(sysid,compid,&message,&msg_struct);
	return message;
}

mavlink_message_t create_request_data_stream_msg(int sysid, int compid){

	mavlink_message_t message;

	mavlink_request_data_stream_t  msg_struct;
	msg_struct.req_message_rate=10;
	msg_struct.start_stop=1;
	msg_struct.target_component=compid;
	msg_struct.target_system=sysid;
	msg_struct.req_stream_id=MAV_DATA_STREAM_ALL;
	mavlink_msg_request_data_stream_encode(sysid,compid,&message,&msg_struct);
	return message;
}

mavlink_message_t create_rc_channel_override_msg(int chan1, int chan2, int chan3, int chan4, int chan5, int chan6, int chan7, int chan8){
    mavlink_message_t message;
	mavlink_rc_channels_override_t msg_struct;
	msg_struct.chan1_raw=chan1;
	msg_struct.chan2_raw=chan2;
	msg_struct.chan4_raw=chan4;
	msg_struct.chan5_raw=chan5;
	msg_struct.chan6_raw=chan6;
	msg_struct.chan7_raw=chan7;
	msg_struct.chan8_raw=chan8;
	msg_struct.chan3_raw=chan3;
	msg_struct.target_system=sysid;
	msg_struct.target_component=MAV_COMP_ID_ALL;
	mavlink_msg_rc_channels_override_encode(255,MAV_COMP_ID_ALL,&message,&msg_struct);
	return message;
}

mavlink_message_t create_arm_disarm_msg(float par1, int conf){
    mavlink_message_t message;
	mavlink_command_long_t msg_struct;
	msg_struct.command= MAV_CMD_COMPONENT_ARM_DISARM;
	msg_struct.param1=par1;
	msg_struct.target_component= MAV_COMP_ID_SYSTEM_CONTROL;
	msg_struct.target_system=sysid;
	msg_struct.confirmation=conf;
	mavlink_msg_command_long_encode(sysid,MAV_COMP_ID_SYSTEM_CONTROL,&message,&msg_struct);
	return message;

}
mavlink_message_t create_set_param_msg(float value){

	mavlink_message_t message;
	mavlink_param_set_t msg_struct;
	string str= "ARMING_CHECK";
	msg_struct.target_system=sysid;
	msg_struct.target_component=MAV_COMP_ID_ALL;
	strcpy(msg_struct.param_id,"ARMING_CHECK") ;
	msg_struct.param_value=value;
	mavlink_msg_param_set_encode(sysid,MAV_COMP_ID_ALL,&message,&msg_struct);
	return message;
}

mavlink_message_t create_set_mode_msg(int base_mode,int cust_mode){

	mavlink_message_t message;
	mavlink_set_mode_t msg_struct;
	msg_struct.base_mode=base_mode;
	msg_struct.custom_mode=cust_mode;
	msg_struct.target_system=sysid;
	mavlink_msg_set_mode_encode(sysid,compid,&message,&msg_struct);
	return message;
}

void initial_setup(){
	//requesting the parameter list from APM
		//mavlink_message_t msg=create_request_params_msg(sysid,compid);
		//publish_ros_message(msg);

	    mavlink_message_t set_param = create_set_param_msg(0.0);
	    publish_ros_message(set_param);

		mavlink_message_t message= create_request_data_stream_msg(sysid,compid);
		publish_ros_message(message);


		mavlink_message_t rc1_msg=create_rc_channel_override_msg(1000,1000,1100,1000,1000,0,0,1000);
		publish_ros_message(rc1_msg);
		mavlink_message_t mode_msg = create_set_mode_msg(81,5);
		publish_ros_message(mode_msg);

		usleep(100000);

	     mavlink_message_t arm_msg = create_arm_disarm_msg(0.0,0);
		 publish_ros_message(arm_msg);

        //printf("launch loiter land");
		//launch_loiter_land();

}

// Test function to raise the quadcopter toa a particluar height, change mode to loiter
// and let it hover for 5 secs and the land. This function tests the loiter mode, land mode
// and sensor readinngs from the APM
void launch_loiter_land(){

        int counter = 1000;
        printf("WAITING TO ARM\n bs_mode %d",bs_mode);
//        while(bs_mode<=81){
//        	usleep(500000);
//        	mavlink_message_t arm_msg = create_arm_disarm_msg(1,0);
//        	publish_ros_message(arm_msg);
//        	printf("bs_mode : %d\n",bs_mode);
//
//        }
        //mavlink_message_t msg = create_set_mode_msg(81,0);
        //publish_ros_message(msg);

        printf("RADIO SIGNALS\n");
        while(counter<1200){
        	counter+=20;
        	mavlink_message_t rc_msg = create_rc_channel_override_msg(1000,1000,counter,1000,1000,0,0,1000);
        	publish_ros_message(rc_msg);
        	printf("throttle %d \n",counter);
        	usleep(2000000);

        }

        mavlink_message_t arm_msg = create_arm_disarm_msg(0,0);
        publish_ros_message(arm_msg);


}

int main(int argc, char **argv){

	ros::init(argc, argv, "controller");
	ros::NodeHandle controller_nh;
	controller_mav_pub= controller_nh.advertise<ros_mavlink::Mavlink>("mavlink/to",1000);

	controller_sensor_sub = controller_nh.subscribe("tracking_inputs",1,sensor_callback);
    controller_mav_sub = controller_nh.subscribe("mavlink/from",1000,mavlink_callback);

    ros::NodeHandle nh;
    command_pub = nh.advertise<std_msgs::Int16>("command",10);
    command_sub = nh.subscribe("command",10,command_callback);

    ros::spin();
}
