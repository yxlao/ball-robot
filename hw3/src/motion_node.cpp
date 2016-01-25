#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv_apps/FBackFlowConfig.h>
#include <opencv_apps/FlowArrayStamped.h>

#include <sensor_msgs/image_encodings.h>
#include <actionlib/server/simple_action_server.h>

#include "VideoModes.h"
#include <hw3/ChangeVideoModeAction.h>
using namespace hw3;

class MotionNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  actionlib::SimpleActionServer<ChangeVideoModeAction> as_; 
  std::string action_name_;
  cv::BackgroundSubtractor *pMOG2;


  // create messages that are used to publish feedback/result to ActionClient
  ChangeVideoModeResult result_;
  ChangeVideoModeGoal currentState_;
  cv::Mat prevgray, gray, flow, cflow, fgMaskMOG2, prevFrame, currFrame, frameDiff, thresholdFrame, diff_frame;
  bool first;
  
public:
  MotionNode(std::string name) :
    as_(nh_, name, boost::bind(&MotionNode::executeCB, this, _1), false),
    action_name_(name),
    it_(nh_)
  {
    as_.start();
    currentState_.video_mode_id = MODE_RAW_VIDEO;
    currentState_.visualization_mode_id = VISUALIZATION_OFF;
    ROS_INFO("motion_node ready");
    // do we need a callback here?
    image_pub_= it_.advertise("/motion_node/image",1);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &MotionNode::frameCallback, this);
    pMOG2 = new cv::BackgroundSubtractorMOG2(10,16,false);
    first = true;
  }

  ~MotionNode(void)
  {
  }

  void frameCallback(const sensor_msgs::ImageConstPtr& msg) {

    // Image to return
    sensor_msgs::Image::Ptr out_msg;
    cv::Mat element;
    cv::Mat img_smooth, img_color, img_gray, img_result, img_edge_color, img_edge_gray_x, img_edge_gray_y; 
    cv::Mat img_contour, drawing;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Rect> boundRect (4100);
    std::vector<std::vector<cv::Point> > contours_poly (4100);

    // Should this be seclectable?
    int erosion_type = cv::MORPH_RECT;
    int erosion_size = 4;
    int iterations = 2;

    try {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
 
      switch(currentState_.video_mode_id) {
        case MODE_RAW_VIDEO: 
          frame.copyTo(img_result);
          break;
 
        case MODE_MOG2:
          pMOG2->operator()(frame, fgMaskMOG2);
          cv::cvtColor(fgMaskMOG2, img_result, CV_GRAY2RGB);
          //fgMaskMOG2.copyTo(img_result);
          break;

        case MODE_FARNEBACK:
          // Messages

          if ( frame.channels() > 1 ) {
            cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY );
          } else {
            frame.copyTo(gray);
          }
          if( prevgray.data ) {
            cv::calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
            cv::cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
            /*
            int step = 16;
            cv::Scalar color = cv::Scalar(0, 255, 0);
            for(int y = 0; y < cflow.rows; y += step)
              for(int x = 0; x < cflow.cols; x += step) {
                const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
                cv::line(cflow, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
                cv::circle(cflow, cv::Point(x,y), 2, color, -1);

                opencv_apps::Flow flow_msg;
                opencv_apps::Point2D point_msg;
                opencv_apps::Point2D velocity_msg;
                point_msg.x = x;
                point_msg.y = y;
                velocity_msg.x = fxy.x;
                velocity_msg.y = fxy.y;
                flow_msg.point = point_msg;
                flow_msg.velocity = velocity_msg;
              }
            */
          }

          std::swap(prevgray, gray);

          // convert to something we can publish
          cflow.copyTo(img_result);
          break;

        default:
          ROS_INFO("Default hit in video mode selection, resetting.");
          currentState_.video_mode_id = MODE_RAW_VIDEO;
          frame.copyTo(img_result);
          break; 
      }

      // Apply any visualizations to the output image
      switch(currentState_.visualization_mode_id) {
        case VISUALIZATION_OFF:
          // Dont have to do anything here...
          first = true;
          break;

        case VISUALIZATION_BOUNDING_BOX:
          // Play with image to get somethign we can create boxes around..
          cv::absdiff(prevFrame, frame, diff_frame);
          cv::blur(diff_frame, img_smooth, cv::Size(3,3), cv::Point(-1,-1));
	  element = cv::getStructuringElement(erosion_type, cv::Size(2*erosion_size+1, 2*erosion_size+1), cv::Point(erosion_size, erosion_size) );
          cv::morphologyEx(img_smooth, img_smooth, cv::MORPH_ERODE, element);

          cv::cvtColor( img_smooth, img_gray, cv::COLOR_BGR2GRAY );
          //cv::cvtColor( diff_frame, img_gray, cv::COLOR_BGR2GRAY );

          /*
          cv::Sobel(img_gray, img_edge_gray_x, CV_16S, 1, 0, 3);
          cv::convertScaleAbs(img_gray, img_edge_gray_x);

          cv::Sobel(img_gray, img_edge_gray_y, CV_16S, 0, 1, 3);
          cv::convertScaleAbs(img_gray, img_edge_gray_y);

          cv::addWeighted(img_edge_gray_x, 0.5, img_edge_gray_y, 0.5, 0, img_gray);

          */

          //cv::Canny(img_gray, img_gray, 1, 2, 3);

         cv::threshold(img_gray, img_gray, 1, 255, CV_THRESH_BINARY);
         //cv::threshold(diff_frame, diff_frame, 44, 255, CV_THRESH_BINARY);
       
          cv::cvtColor( img_gray, img_color, cv::COLOR_GRAY2BGR );

          //cv::add(img_result, img_color, img_result);
          //img_color.copyTo(img_result);

	  cv::findContours( img_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );
	  //cv::findContours( diff_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

	  for( int i = 0; i< contours.size(); i++ )
	     {
               //ROS_INFO("%d", i);
               cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
               boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
	     }

	  drawing = cv::Mat::zeros( img_gray.size(), CV_8UC3 );

	  for( int i = 0; i< contours.size(); i++ ) {
	       cv::Scalar color = cv::Scalar( 107,222, 46 );
               //cv::drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
               if(boundRect[i].area() > 20000) {
                 cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
               }
          }

          cv::add(drawing, img_result, img_result);
          break;

        default:
          ROS_INFO("Default hit in visualization mode selection, resetting.");
          currentState_.visualization_mode_id = VISUALIZATION_OFF;
          break;
      }         

      frame.copyTo(prevFrame);
      // Publish Message
      out_msg = cv_bridge::CvImage(msg->header, msg->encoding, img_result).toImageMsg();
      image_pub_.publish(out_msg);
    } catch (cv::Exception &e) {
      ROS_INFO("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void executeCB(const ChangeVideoModeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Executing, changing Video Mode to %d, Visualization to %d.", goal->video_mode_id, goal->visualization_mode_id);

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    } else {
      // Change configuration which will cause frameCallback to behave differently
      ROS_INFO("Configuration updated");
    }
    
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();

    if (success)
    {
      result_.video_mode_changed = 1;
      result_.visualization_mode_changed = 1;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
      
      currentState_.video_mode_id = goal->video_mode_id;
      currentState_.visualization_mode_id = goal->visualization_mode_id;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_node");
  MotionNode motionNode(ros::this_node::getName());
  ros::spin();
  return 0;
}
