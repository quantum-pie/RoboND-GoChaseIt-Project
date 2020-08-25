#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
double linear_vel = 0;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
    ROS_INFO_STREAM("Driving robot towards the ball");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
    unsigned char white_pixel = 255;

    unsigned img_height = img.height;
    unsigned img_width = img.width;
    auto left_threshold = img_width / 3.0;
    auto right_threshold = left_threshold * 2.0;

    float ball_centroid = 0;
    int white_count = 0;
    for (unsigned i = 0; i < img_height; ++i) {
        for (unsigned j = 0; j < img_width; ++j) {
            auto r_value = img.data[i * img.step + 3*j];
            auto g_value = img.data[i * img.step + 3*j + 1];
            auto b_value = img.data[i * img.step + 3*j + 2];
            bool is_white = (r_value == white_pixel) and (g_value == white_pixel) and (b_value == white_pixel);
            if (is_white) {
                ball_centroid += j;
                white_count++;
            }
        }
    }

    if (white_count) {
        ball_centroid /= white_count;
        int left_condition = ball_centroid < left_threshold;
        int right_condition = ball_centroid > right_threshold;
        auto angular_vel = 1.0 * (left_condition - right_condition);
        linear_vel = std::min(0.5, linear_vel + 0.01);  
        drive_robot(linear_vel, angular_vel);
    } else {
        linear_vel = 0;
        drive_robot(0, 0);
    }
}

int main(int argc, char** argv) {
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
