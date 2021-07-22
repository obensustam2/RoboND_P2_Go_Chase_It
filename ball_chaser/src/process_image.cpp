#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_target");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int ballX = -1;
    int imgSize = img.height * img.width;
    int r_pix = 0;
    int g_pix = 0;
    int b_pix = 0;

    for(int i = 0; i < imgSize; ++i)    //Go through image data which stored sequinatlly in array. Size of array is (imgSize x 3 channel)
    {
        r_pix = img.data[i*3];  //Read RGB pixels from image data array
        g_pix = img.data[i*3 + 1];
        b_pix = img.data[i*3 + 2];

        if((r_pix == white_pixel) && (g_pix == white_pixel) && (b_pix == white_pixel)) //If each channel R,G and B equal to 255 white color set X position of the ball
        {
            ballX = i % img.width; //Set X position of the ball
            ROS_INFO_STREAM("White pixel X = " << ballX);
            break;
        }
    }
    
    //Check X position of the ball and send command to service
    if(ballX == -1) //waiting for ball there are no ball in camera view
        drive_robot(0.0, 0.0);
    else if((ballX > -1) && (ballX < img.width / 3)) //ball is on the left part of camera view 
        drive_robot(0.0, 0.1);    
    else if(ballX < (int)(img.width * 2.0/3.0)) //ball is in the  middle of camera view
        drive_robot(0.2, 0.0);
    else    //ball in the right part ob view
        drive_robot(0.0, -0.1);
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}