#include "ros/ros.h" //ros header file to interact with ros
#include "std_msgs/String.h" //ros message type for passing string
#include <iostream>


//callback that gets executed when message is received on the subscribed topic1
void subscriberCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received: [%s]", msg->data.c_str()); //we print the message in place of format specifier using rosinfo 


//initialise nodehandle and publisher

  //create a nodehandle object used to interact with ros
  static ros::NodeHandle nh;  
  static ros::Publisher pub = nh.advertise<std_msgs::String>("topic2", 10);
//we prompt the user to input a message of type string and publish it to topic2, 10 specifies the size of publishers message queue

//creating a message object of type string
  std_msgs::String new_msg;
  std::cout << "Enter a message to publish to topic2: ";
  std::getline(std::cin, new_msg.data); //stores message inputted by user in new_msg

  pub.publish(new_msg);//publishes the message to topic2
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_node"); //initialise ros node with name subscriber node
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("topic1", 10, subscriberCallback); 
  //creates a subscriber to subscribe to topic1 and specify subscribercallback to handle incoming messages

  ros::spin(); 
  //it creates a sort of looping event which makes node ready to process incoming msgs

  return 0;
}
