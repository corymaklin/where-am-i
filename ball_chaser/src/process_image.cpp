#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // The size is step * rows as opposed to height * width
    // because each pixel is composed of 1 byte (e.g. grayscale) or
    // 3 bytes (e.g. RGB) where every byte represents numbers 0 to 255

    for (int i = 0; i < img.height * img.step; i += 3)
    {
        // Scenario 1: the entire ball (or a portion) is strictly on the left side of the image
        // Scenario 2: the entire ball (or a portion) is strictly on the right side of the image
        // Scenario 3: the majority of ball is in the middle of the image
        // Scenario 4: the majority of ball is is on the right side of the image
        // Scenario 5: the majority of ball is is on the left side of the image

        // In scenarios 3, 4, 5 the first pixel we see is going to be aligned with the center of the ball because we're going from top to bottom
        // In scenarios 1 and 2, the first pixel we see will tell us which side the ball is on
        // Therefore, we don't require any complicated logic (e.g. sum pixels and use majority)

        int red = img.data[i];
        int green = img.data[i+1];
        int blue = img.data[i+2];

        if (red == 255 && green == 255 && blue == 255)
        {
            auto column = i % img.step;

            if (column < img.step / 3)
            {
                // Turn left
                drive_robot(0.5, 1.0);
            }
            else if (column <= 2 * img.step / 3 && column >= img.step / 3)
            {
                // Go straight
                drive_robot(0.5, 0.0);
            }
            else
            {
                // Turn right
                drive_robot(0.5, -1.0);
            }

            return;
        }
    }

    // If we do not see the ball stop
    drive_robot(0.0, 0.0);
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
