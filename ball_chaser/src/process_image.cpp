#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request new velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the drive_to_target service and pass the request velocities
    if (!client.call(srv)) {
      ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    float lin_x = 0.0;
    float ang_z = 0.0;

    int red_pixel, green_pixel, blue_pixel, column_position = 0;

    int left_barrier = static_cast<int>(img.step / 3);
    int right_barrier = img.step - left_barrier;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < img.height * img.step; i += 3) {
      red_pixel = img.data[i];
      green_pixel = img.data[i+1];
      blue_pixel = img.data[i+2];

      if (red_pixel == 255 && blue_pixel == 255 && green_pixel == 255) {
        lin_x = 0.1;
        column_position = i % img.step;

        if (column_position <= left_barrier) {
          ang_z = 0.1;
        } else if (column_position >= right_barrier) {
          ang_z = -0.1;
        }
        break;
      }
    }
    return drive_robot(lin_x, ang_z);
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