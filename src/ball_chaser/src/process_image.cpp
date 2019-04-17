#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "ball_chaser/HorizontalLocation.h"

#include <string>

double GetRelativeHorizontalPosition(int column, int num_row_pixels)
{
    int midpoint = num_row_pixels / 2;
    // Correction to get symmetric values for even number of pixels\
    // in a row. In this case, the result can't be 0, but +-k/midpoint,
    // for k = 1, ..., midpoint
    if(num_row_pixels % 2 == 0 and column > num_row_pixels)
        ++column;
    return static_cast<double>(column - midpoint)/static_cast<double>(midpoint);

}

class ImageProcessor
{
    ros::Publisher location_publisher_;

    void ProcessImageCallback(const sensor_msgs::Image& img);

public:
    ImageProcessor() {}
    // Run the bot driver
    void Run();


};

void ImageProcessor::Run()
{
    ros::NodeHandle nodeh;

    // Subscribe to /camera/rgb/image_raw topic to read the image data
    ros::Subscriber sub_image = nodeh.subscribe("/camera/rgb/image_raw", 10,
                                                &ImageProcessor::ProcessImageCallback,
                                                this);

    // Inform ROS master that we will be publishing a message of type
    // ball_chaser::HorizontalLocation on the /ball_chaser/ball_hor_loc topic
    // with a publishing queue size of 10
    location_publisher_ = nodeh.advertise<ball_chaser::HorizontalLocation>("/ball_chaser/ball_hor_loc", 10);

    // Handle ROS communication events
    ros::spin();
}

// TODO WRITE THIS
void ImageProcessor::ProcessImageCallback(const sensor_msgs::Image& img)
{

    // Create a motor_command object of type geometry_msgs::Twist
    ball_chaser::HorizontalLocation hloc;

    uint8_t saturated_color_component{255};
    bool is_there_white_pixel{false};
    double rel_pos_white_pixel{0.0};
    int red_byte_offset = 0;
    int green_byte_offset = 1;
    int blue_byte_offset = 2;
    for(int row = 0; row < img.height; ++row)
    {
        for(int column = 0; column < img.step; column+=3)
        {
            int index_raw_data = row * img.step + column;
            if ( img.data[index_raw_data + red_byte_offset] == saturated_color_component
              && img.data[index_raw_data + green_byte_offset] == saturated_color_component
              && img.data[index_raw_data + blue_byte_offset] == saturated_color_component)
            {
                is_there_white_pixel = true;
                rel_pos_white_pixel = GetRelativeHorizontalPosition(column, img.step);
                break;
            }
        }
    }

    // In the case there is no white pixel, horizontal_relative_position
    // will be ignored, so we can fill it anyway.
    hloc.contains_object = is_there_white_pixel;
    hloc.horizontal_relative_position = rel_pos_white_pixel;

    // Publish angles to drive the robot
    location_publisher_.publish(hloc);
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Initialize image processor
    ImageProcessor processor;

    // Run processor
    processor.Run();

    return 0;
}