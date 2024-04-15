#include "ros/ros.h"  //ros header file includes necessary headers for interacting with ros
#include "std_msgs/String.h"//string header file used in ros for interacting with string data
#include <iostream>// standard c++ header for input output stream

void subscriberCallback(const std_msgs::String::ConstPtr& msg)  
//a callback function that gets executed when a message is recieved on the subscribed topic(here topic2)
{
  ROS_INFO("Received: [%s]", msg->data.c_str());   
   // we print out the received message in place of the format specifier using ros_info 
}

int main(int argc, char **argv) //main function
{
  ros::init(argc, argv, "publisher_node"); //initialises the ros node with the name publisher node
  ros::NodeHandle nh; //creating a ros node handle object used to communicate with rosmaster


//publisher subscriber initialization

  // initializes and creates Publisher for topic1
  ros::Publisher pub_topic1 = nh.advertise<std_msgs::String>("topic1", 10); //publishes messages of type std_msgs::String

  // initializes and creates Subscriber for topic2
  ros::Subscriber sub_topic2 = nh.subscribe("topic2", 10, subscriberCallback); //It will invoke the subscriberCallback function whenever a message is received.

  ros::Rate loop_rate(1); // Publish at 1Hz

  while (ros::ok()) //the loop runs as long as ros is operational
  {

// we prompt the user to input a message and then we publish it to topic1

    std_msgs::String msg;
    std::cout << "Enter a message to publish to topic1: ";
    std::getline(std::cin, msg.data);
    pub_topic1.publish(msg); //publishes the msg entered by the user to topic1
    ros::spinOnce();// Allows the subscriber callback to be executed if a message is received
    loop_rate.sleep(); //sleep for 1sec to maintain publishing rate
  }

  return 0;
}

