// /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2015, University of Pisa
// *  Copyright (c) 2010, ISR University of Coimbra.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the ISR University of Coimbra nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: Gonçalo Cabrita on 05/10/2010
// * Author: Alessandro Settimi 2015
// * Author: Mirko Ferrati 2015
// * Author: Matthew Bohl 2015
// *********************************************************************/

// #define NODE_VERSION 2.01

// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>              // odom
// #include <geometry_msgs/Twist.h>            // cmd_vel
// #include <std_msgs/String.h>
// #include <irobotcreate2/Battery.h>      // battery
// #include <irobotcreate2/Bumper.h>       // bumper
// #include <irobotcreate2/Buttons.h>      // buttons
// #include <irobotcreate2/RoombaIR.h>     // ir_bumper cliff
// #include <irobotcreate2/IRCharacter.h>  // ir_character
// #include <irobotcreate2/WheelDrop.h>    // wheel_drop
// #include <irobotcreate2/Leds.h>         // leds
// #include <irobotcreate2/DigitLeds.h>    // digit_leds
// #include <irobotcreate2/Song.h>         // song
// #include <irobotcreate2/PlaySong.h>     // play_song

// #include "irobotcreate2/OpenInterface.h"

// #include <string>

// std::string port;
// std::string mode;
// irobot::OpenInterface * roomba;
// bool pollSensors = true;

// std::string prefixTopic(std::string prefix, char * name)
// {
//     std::string topic_name = prefix;
//     topic_name.append(name);
    
//     return topic_name;
// }

// void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
// {
//     roomba->drive(cmd_vel->linear.x, cmd_vel->angular.z);
// }

// void cmdModeReceived(const std_msgs::String::ConstPtr& cmd_)
// {
//     std::string cmd = cmd_->data.c_str();

//     if(cmd=="exit") return;
//     else if(cmd=="start")
//     {
//         roomba->Start();
//     }
//     else if(cmd=="stop")
//     {
//         roomba->Stop();
//     }
//     else if(cmd=="reset")
//     {
//         roomba->Reset();
//     }
//     else if(cmd=="powerdown")
//     {
//         roomba->powerDown();
//     }
//     else if(cmd=="safe")
//     {
//         roomba->Safe();
//     }
//     else if(cmd=="full")
//     {
//         roomba->Full();
//     } 
//     else if(cmd=="dock")
//     {
//         roomba->goDock();
//     } 
// }

// void ledsReceived(const irobotcreate2::Leds::ConstPtr& leds)
// {
//     roomba->setLeds(leds->warning, leds->dock, leds->spot, leds->dirt_detect, leds->clean_color, leds->clean_intensity);
// }

// void digitLedsReceived(const irobotcreate2::DigitLeds::ConstPtr& leds)
// {
//     if(leds->digits.size()!=4) return;

//     roomba->setDigitLeds(leds->digits[3], leds->digits[2], leds->digits[1], leds->digits[0]);
// }

// void songReceived(const irobotcreate2::Song::ConstPtr& song)
// {
//     unsigned char notes[song->notes.size()];
//     unsigned char lengths[song->notes.size()];

//     for(int i=0 ; i<song->notes.size() ; i++)
//     {
//         notes[i] = song->notes[i].note;
//         lengths[i] = song->notes[i].length;
//     }

//         ROS_INFO("Recieved Song");
//         // Pause sesor requests so song can be loaded
//         ROS_INFO("Pausing sensor collection to load song");
//         pollSensors = true;
//     roomba->setSong(song->song_number, song->notes.size(), notes, lengths);
//         pollSensors = false;
//         ROS_INFO("Song Loaded, sensing re-enabled");
// }

// void playSongReceived(const irobotcreate2::PlaySong::ConstPtr& song)
// {
//         ROS_INFO("Playing Song %d",song->song_number);
//     roomba->playSong(song->song_number);
// }

// void loadSongs() {
//     irobotcreate2::Song song;
//     irobotcreate2::Note note;

//         // Load 5 songs, each consisting of N beeps
//         for (int i = 1; i <= 4; i++) {

//         song.notes.clear();
//         song.song_number = i;

//         for(int j = 0; j < i; j++) {
//             // play a note
//             note.note = 64;
//             note.length = 24;
//             song.notes.push_back(note);

//                         // play a beat
//             note.note = 0;
//             note.length = 24;
//             song.notes.push_back(note);
//         }

//         unsigned char notes[song.notes.size()];
//         unsigned char lengths[song.notes.size()];

//         ROS_INFO("Loading Default Songs");

//         for(int k=0 ; k<song.notes.size() ; k++)
//         {
//             notes[k] = song.notes[k].note;
//             lengths[k] = song.notes[k].length;
//         }

//         roomba->setSong(song.song_number, song.notes.size(), notes, lengths);
//         ROS_INFO("Default Songs Loaded");
//     }
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "roomba560_node");

//     ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
    
//     double last_x, last_y, last_yaw;
//     double vel_x, vel_y, vel_yaw;
//     double dt;
//     float last_charge = 0.0;
//     int time_remaining = -1;
    
//     ros::NodeHandle n;
//     ros::NodeHandle pn("~");
    
//     pn.param<std::string>("port", port, "/dev/ttyUSB0");
//     pn.param<std::string>("mode", mode, "safe");
    
//     std::string base_frame_id;
//     std::string odom_frame_id;
//     pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
//     pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    
//     roomba = new irobot::OpenInterface(port.c_str());

//     ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
//     ros::Publisher battery_pub = n.advertise<irobotcreate2::Battery>("/battery", 50);
//     ros::Publisher bumper_pub = n.advertise<irobotcreate2::Bumper>("/bumper", 50);
//     ros::Publisher buttons_pub = n.advertise<irobotcreate2::Buttons>("/buttons", 50);
//     ros::Publisher cliff_pub = n.advertise<irobotcreate2::RoombaIR>("/cliff", 50);
//     ros::Publisher irbumper_pub = n.advertise<irobotcreate2::RoombaIR>("/ir_bumper", 50);
//     ros::Publisher irchar_pub = n.advertise<irobotcreate2::IRCharacter>("/ir_character", 50);
//     ros::Publisher wheeldrop_pub = n.advertise<irobotcreate2::WheelDrop>("/wheel_drop", 50);

//     tf::TransformBroadcaster tf_broadcaster;
    
//     ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);
//     ros::Subscriber leds_sub  = n.subscribe<irobotcreate2::Leds>("/leds", 1, ledsReceived);
//     ros::Subscriber digitleds_sub  = n.subscribe<irobotcreate2::DigitLeds>("/digit_leds", 1, digitLedsReceived);
//     ros::Subscriber song_sub  = n.subscribe<irobotcreate2::Song>("/song", 1, songReceived);
//     ros::Subscriber playsong_sub  = n.subscribe<irobotcreate2::PlaySong>("/play_song", 1, playSongReceived);
//     ros::Subscriber mode_sub  = n.subscribe<std_msgs::String>("/mode", 1, cmdModeReceived);
    
//     irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
//     roomba->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);

//     if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");

//             // Send base mode
//         if(mode=="exit") exit;
//         else if(mode=="start")
//         {
//             roomba->Start();
//         }
//         else if(mode=="stop")
//         {
//             roomba->Stop();
//         }
//         else if(mode=="reset")
//         {
//             roomba->Reset();
//         }
//         else if(mode=="powerdown")
//         {
//             roomba->powerDown();
//         }
//         else if(mode=="safe")
//         {
//             roomba->Safe();
//         }
//         else if(mode=="full")
//         {
//             roomba->Full();
//         }
//         else if(mode=="dock")
//         {
//             roomba->goDock();
//         } 

//     else
//     {
//         ROS_FATAL("Could not connect to Roomba.");
//         ROS_BREAK();
//     }
    
//     ros::Time current_time, last_time;
//     current_time = ros::Time::now();
//     last_time = ros::Time::now();

//     bool first_loop=true;

//     ros::Rate r(10.0);
//     while(n.ok())
//     {
//         current_time = ros::Time::now();
        
//         last_x = roomba->odometry_x_;
//         last_y = roomba->odometry_y_;
//         last_yaw = roomba->odometry_yaw_;
        
//         if( pollSensors && roomba->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
//         else roomba->calculateOdometry();
        
//         dt = (current_time - last_time).toSec();
//         vel_x = (roomba->odometry_x_ - last_x)/dt;
//         vel_y = (roomba->odometry_y_ - last_y)/dt;
//         vel_yaw = (roomba->odometry_yaw_ - last_yaw)/dt;
        
//         // ******************************************************************************************
//         //first, we'll publish the transforms over tf
//         geometry_msgs::TransformStamped odom_trans;
//         odom_trans.header.stamp = current_time;
//         odom_trans.header.frame_id = odom_frame_id;
//         odom_trans.child_frame_id = base_frame_id;
//         odom_trans.transform.translation.x = roomba->odometry_x_;
//         odom_trans.transform.translation.y = roomba->odometry_y_;
//         odom_trans.transform.translation.z = 0.0;
//         odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
//         tf_broadcaster.sendTransform(odom_trans);
        
//         //TODO: Finish brodcasting the tf for all the ir sensors on the Roomba
//         /*geometry_msgs::TransformStamped cliff_left_trans;
//         cliff_left_trans.header.stamp = current_time;
//         cliff_left_trans.header.frame_id = "base_link";
//         cliff_left_trans.child_frame_id = "base_cliff_left";
//         cliff_left_trans.transform.translation.x = 0.0;
//         cliff_left_trans.transform.translation.y = 0.0;
//         cliff_left_trans.transform.translation.z = 0.0;
//         cliff_left_trans.transform.rotation = ;
//         tf_broadcaster.sendTransform(cliff_left_trans); */

//         // ******************************************************************************************
//         //next, we'll publish the odometry message over ROS
//         nav_msgs::Odometry odom;
//         odom.header.stamp = current_time;
//         odom.header.frame_id = odom_frame_id;
        
//         //set the position
//         odom.pose.pose.position.x = roomba->odometry_x_;
//         odom.pose.pose.position.y = roomba->odometry_y_;
//         odom.pose.pose.position.z = 0.0;
//         odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(roomba->odometry_yaw_);
        
//         //set the velocity
//         odom.child_frame_id = base_frame_id;
//         odom.twist.twist.linear.x = vel_x;
//         odom.twist.twist.linear.y = vel_y;
//         odom.twist.twist.angular.z = vel_yaw;
        
//         //publish the message
//         odom_pub.publish(odom);

//         // ******************************************************************************************
//         //publish battery
//         irobotcreate2::Battery battery;
//         battery.header.stamp = current_time;
//         battery.power_cord = roomba->power_cord_;
//         battery.dock = roomba->dock_;
//         battery.level = 100.0*(roomba->charge_/roomba->capacity_);
//         if(last_charge > roomba->charge_) time_remaining = (int)(battery.level/((last_charge-roomba->charge_)/roomba->capacity_)/dt)/60;
//         last_charge = roomba->charge_;
//         battery.time_remaining = time_remaining;
//         battery_pub.publish(battery);
    
//         // ******************************************************************************************   
//         //publish bumpers
//         irobotcreate2::Bumper bumper;
//         bumper.left.header.stamp = current_time;
//         bumper.left.state = roomba->bumper_[LEFT];
//         bumper.right.header.stamp = current_time;
//         bumper.right.state = roomba->bumper_[RIGHT];
//         bumper_pub.publish(bumper);
    
//         // ******************************************************************************************   
//         //publish buttons
//         irobotcreate2::Buttons buttons;
//         buttons.header.stamp = current_time;
//         buttons.clean = roomba->buttons_[BUTTON_CLEAN];
//         buttons.spot = roomba->buttons_[BUTTON_SPOT];
//         buttons.dock = roomba->buttons_[BUTTON_DOCK];
//         buttons.day = roomba->buttons_[BUTTON_DAY];
//         buttons.hour = roomba->buttons_[BUTTON_HOUR];
//         buttons.minute = roomba->buttons_[BUTTON_MINUTE];
//         buttons.schedule = roomba->buttons_[BUTTON_SCHEDULE];
//         buttons.clock = roomba->buttons_[BUTTON_CLOCK];
//         buttons_pub.publish(buttons);

//         // ******************************************************************************************
//         //publish cliff
//         irobotcreate2::RoombaIR cliff;
//         cliff.header.stamp = current_time;

//         cliff.header.frame_id = "base_cliff_left";
//         cliff.state = roomba->cliff_[LEFT];
//         cliff.signal = roomba->cliff_signal_[LEFT];
//         cliff_pub.publish(cliff);

//         cliff.header.frame_id = "base_cliff_front_left";
//         cliff.state = roomba->cliff_[FRONT_LEFT];
//         cliff.signal = roomba->cliff_signal_[FRONT_LEFT];
//         cliff_pub.publish(cliff);

//         cliff.header.frame_id = "base_cliff_front_right";
//         cliff.state = roomba->cliff_[FRONT_RIGHT];
//         cliff.signal = roomba->cliff_signal_[FRONT_RIGHT];
//         cliff_pub.publish(cliff);

//         cliff.header.frame_id = "base_cliff_right";
//         cliff.state = roomba->cliff_[RIGHT];
//         cliff.signal = roomba->cliff_signal_[RIGHT];
//         cliff_pub.publish(cliff);

//         // ******************************************************************************************
//         //publish irbumper
//         irobotcreate2::RoombaIR irbumper;
//         irbumper.header.stamp = current_time;

//         irbumper.header.frame_id = "base_irbumper_left";
//         irbumper.state = roomba->ir_bumper_[LEFT];
//         irbumper.signal = roomba->ir_bumper_signal_[LEFT];
//         irbumper_pub.publish(irbumper);

//         irbumper.header.frame_id = "base_irbumper_front_left";
//         irbumper.state = roomba->ir_bumper_[FRONT_LEFT];
//         irbumper.signal = roomba->ir_bumper_signal_[FRONT_LEFT];
//         irbumper_pub.publish(irbumper);

//         irbumper.header.frame_id = "base_irbumper_center_left";
//         irbumper.state = roomba->ir_bumper_[CENTER_LEFT];
//         irbumper.signal = roomba->ir_bumper_signal_[CENTER_LEFT];
//         irbumper_pub.publish(irbumper);

//         irbumper.header.frame_id = "base_irbumper_center_right";
//         irbumper.state = roomba->ir_bumper_[CENTER_RIGHT];
//         irbumper.signal = roomba->ir_bumper_signal_[CENTER_RIGHT];
//         irbumper_pub.publish(irbumper);

//         irbumper.header.frame_id = "base_irbumper_front_right";
//         irbumper.state = roomba->ir_bumper_[FRONT_RIGHT];
//         irbumper.signal = roomba->ir_bumper_signal_[FRONT_RIGHT];
//         irbumper_pub.publish(irbumper);

//         irbumper.header.frame_id = "base_irbumper_right";
//         irbumper.state = roomba->ir_bumper_[RIGHT];
//         irbumper.signal = roomba->ir_bumper_signal_[RIGHT];
//         irbumper_pub.publish(irbumper);

//         // ******************************************************************************************
//         //publish irchar
//         irobotcreate2::IRCharacter irchar;
//         irchar.header.stamp = current_time;
//         irchar.omni = roomba->ir_char_[OMNI];
//         irchar.left = roomba->ir_char_[LEFT];
//         irchar.right = roomba->ir_char_[RIGHT];
//         irchar_pub.publish(irchar);

//         // ******************************************************************************************
//         //publish wheeldrop
//         irobotcreate2::WheelDrop wheeldrop;
//         wheeldrop.left.header.stamp = current_time;
//         wheeldrop.left.state = roomba->wheel_drop_[LEFT];
//         wheeldrop.right.header.stamp = current_time;
//         wheeldrop.right.state = roomba->wheel_drop_[RIGHT];
//         wheeldrop_pub.publish(wheeldrop);
        
//         ros::spinOnce();
//         r.sleep();
        
//         if(first_loop) {roomba->startOI(true); first_loop=false; loadSongs();}
//     }
    
//     roomba->powerDown();
//     roomba->closeSerialPort();
// }

// // EOF
