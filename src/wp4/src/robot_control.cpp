#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
using JointVector = Eigen::Matrix<double, 5, 1>;



struct Cube {
    string color;
    Point2f center;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_ = std::make_shared<rclcpp::Node>("software_node");
    auto surros_control = std::make_shared<robot::SurrosControl>(node_);

    // Initialize the SurrosControl instance
    surros_control->initialize();

    Mat img = imread("/home/programmer/ros2_ws/src/wp4/test_image.jpeg");
    
    Size original_size = img.size();
    resize(img, img, original_size/3);
    imshow("Test", img);

    cvtColor(img, img, cv::COLOR_BGR2HSV);

    Mat img_blue;
    inRange(img, Scalar(100, 80, 80), Scalar(130, 255, 255), img_blue);

    Mat img_green;
    inRange(img, Scalar(45, 80, 80), Scalar(75, 255, 255), img_green);

    Mat img_red;
    inRange(img, Scalar(0, 80, 80), Scalar(10, 255, 255), img_red);

    Mat img_red_2;
    inRange(img, Scalar(170, 80, 80), Scalar(179, 255, 255), img_red_2);

    img_red = img_red+img_red_2;

    cvtColor(img, img, cv::COLOR_HSV2BGR);
    
    //Extract the contours so that
    vector<vector<Point> > contours_blue;
    vector<Vec4i> hierarchy_blue;
    findContours(img_blue, contours_blue, hierarchy_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(img, contours_blue, -1, cv::Scalar(250, 0, 0), 2, 8, hierarchy_blue);

    //Extract the contours so that
    vector<vector<Point> > contours_green;
    vector<Vec4i> hierarchy_green;
    findContours(img_green, contours_green, hierarchy_green, RETR_TREE, CHAIN_APPROX_SIMPLE);
    drawContours(img, contours_green, -1, cv::Scalar(0, 250, 0), 2, 8, hierarchy_green);

    //Extract the contours so that
    vector<vector<Point> > contours_red;
    vector<Vec4i> hierarchy_red;
    findContours(img_red, contours_red, hierarchy_red, RETR_TREE, CHAIN_APPROX_SIMPLE);
    drawContours(img, contours_red, -1, cv::Scalar(0, 0, 250), 2, 8, hierarchy_red);

    imshow("Test Blue",img_blue);
    imshow("Test Green",img_green);
    imshow("Test Red",img_red);

    imshow("Test Contours", img);


    waitKey(0);    

    return 0;
}
