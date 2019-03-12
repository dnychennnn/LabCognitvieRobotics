#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){
  
  int count_valid = 0;  
  // Publish the data.
  ROS_INFO("+++++++++++++++++++++START+++++++++++++++++++++++++++ %s", "data");
  for (uint j=0; j < input->data.size()/4; j++){
            int step = input->point_step/8;
            
            uint x = input->data[j*step + 0];
            uint y = input->data[j*step + 1];
            uint z = input->data[j*step + 2];
            uint intensity = input->data[j*step + 3];
            if(intensity!=0){
              ROS_INFO("x:%u, y:%u, z:%u, intensity:%u", x, y, z, intensity);
              count_valid++;
            }
  }
  ROS_INFO("+++++++++++++++++++++SUMMARY++++++++++++++++++++++++ %d", count_valid);

  ROS_INFO("POINTS LENGTH: %u", input->row_step/4);
  ROS_INFO("VALID: %d", count_valid);
  ROS_INFO("---");
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/mmWaveDataHdl/RScan", 1, cloud_callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
