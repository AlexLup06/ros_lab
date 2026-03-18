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
    float rotation = 0.0;

    // definition of an == operator to compare the cubes found in workspace with the cubes in the current cube vector
    bool operator==(const Cube& other) const {
        return color == other.color && std::abs(center.x-other.center.x)<5 && std::abs(center.y-other.center.y)<5;
    }
};

//Initialization of global variables
const Size img_size = Size(638,675);
const Size rect_size = Size(50, 50);
const Point2f base_link = Point2f(230,508); // base link coordinates in the worspace image
double tower_height = 0.0;
vector<Cube> current_cubes;

vector<vector<Point>> findColorContours(const Mat& img_bgr, const string color="red"){
    // define the hsv threshold values for each possible color
    Scalar hsv_bottom;
    Scalar hsv_top;

    if(color=="red"){
        hsv_bottom = Scalar(0, 150, 80);
        hsv_top = Scalar(10, 255, 255);
    }
    else if(color=="green"){
        hsv_bottom = Scalar(45, 80, 80);
        hsv_top = Scalar(75, 255, 255);
    }
    else if(color=="blue"){
        hsv_bottom = Scalar(100, 80, 80);
        hsv_top = Scalar(130, 255, 255);
    }
    else{
        std::cout << "An invalid color was given.";
        return vector<vector<Point>>(); // return empty contor vector
    }

    // convert the image to hsv for thresholding
    Mat img_hsv;
    cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);

    // threshold the hsv image to find components of the given color
    Mat img_color_thresh;
    inRange(img_hsv, hsv_bottom, hsv_top, img_color_thresh);

    // red has two hsv value spans => the second needs to be added
    if(color=="red"){
        Mat second_red_thresh;
        inRange(img_hsv, Scalar(170, 150, 80), Scalar(179, 255, 255), second_red_thresh);
        img_color_thresh += second_red_thresh;
    }

    // extract the contors from the color-filtered image
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(img_color_thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    return contours;
}

vector<Cube> findColoredCubes(const Mat& img_bgr, const string color="red"){
    // initialize the resulting Cube vector
    vector<Cube> cubes;

    // get the contours corresponding to the color
    vector<vector<Point>> contours = findColorContours(img_bgr, color);

    // find contours that belong to a cube
    for(vector<Point> contour: contours){
        // check if the contour area is great enough to count as a cube
        double area = contourArea(contour);

        if(area > 500){
            // create a cube at the center of the contour
            Cube cube;
            cube.color = color;
            Rect cube_rect = boundingRect(contour);
            cube.center = Point2f(cube_rect.x+cube_rect.width/2, cube_rect.y+cube_rect.height/2);
            // add the cube to the result vector
            cubes.push_back(cube);
        }
    }
    return cubes;
}

void addCubeToWorkspace(Mat& img, Cube& cube){
    Scalar bgr;
    if(cube.color=="red"){
        bgr = Scalar(0,0,255);
    }
    else if(cube.color=="green"){
        bgr = Scalar(0,255,0);
    }
    else if(cube.color=="blue"){
        bgr = Scalar(255,0,0);
    }

    //creating the rectangular shape (cube) in the image of the workspace
    RotatedRect rect = RotatedRect(cube.center,rect_size, cube.rotation);
    Point2f vertices2f[4];
    rect.points(vertices2f);
    Point vertices[4];
    for(int i = 0; i < 4; i++){
        vertices[i] = vertices2f[i];
    }
    fillConvexPoly(img, vertices, 4, bgr);
}

void drawCurrentWorkspace(Mat& img){
    // load and size the image of the workspace
    Mat img_empty_workspace = imread("/home/programmer/ros2_ws/src/wp4/workspace.png");
    resize(img_empty_workspace, img, img_size);

    //add all cubes in the current workspace image
    for(Cube cube: current_cubes){
        addCubeToWorkspace(img, cube);
    }
}

Point2f getRealWorldCubeCenter(const Cube& cube){
    //converting the image pixel coordinates to real world xy coordinates
    float center_x = (cube.center.x-base_link.x)*0.2/435;
    float center_y = (base_link.y-cube.center.y)*0.2/435;
    return Point2f(center_x, center_y);
}

Point2f getWSCoordinates(const Eigen::Vector2d& point){
    //Converting the real world xy coordinates to image pixel coordinates
    float center_x = point(0)*435/0.2+base_link.x;
    float center_y = base_link.y-point(1)*435/0.2;
    return Point2f(center_x, center_y);
}

// create a cube with the chosen color and add it to the current cubes
void createCube(string color="red", const Eigen::Vector2d& real_world_xy = Eigen::Vector2d::Zero(), double rotation = 0.0, bool random = false){
    Cube cube;
    cube.color = color;
    if(random){
        int center_x = std::rand() % img_size.width;
        int center_y = std::rand() % img_size.height;
        cube.center = Point2f(center_x, center_y);
        cube.rotation = std::rand() % 100;
    }
    else{
        cube.center = getWSCoordinates(real_world_xy);
        cube.rotation = rotation;
    }
    current_cubes.push_back(cube);
}

// picking up or putting down at the position a_xy
void pickUpOrPutDown(Eigen::Vector2d& a_xy, 
    robot::SurrosControl& robot,
    bool upMode = true,
    double hover_dist = 0.06,
    int open_percentage = 10,
    int close_percentage = 80,
    const std::chrono::seconds duration = std::chrono::seconds(5)){

    // define the gripper percentage according to the Mode
    double gripper_percentage = open_percentage;
    if(upMode){
        gripper_percentage = close_percentage;
    }        

    // create 3d position for a
    Eigen::Vector3d a(a_xy(0)+0.015, a_xy(1), 0.0); //offset on x-axis to account for block width

    // add the height of the tower for putting the block on it
    if(!upMode){
        a(2) = tower_height;
    }

    //adding more hovering distance for the first block to avoid collision with other blocks when moving to the platform
    if(tower_height <= 0.025){
        hover_dist += 0.01;
    }

    // create position the hover above the targets
    Eigen::Vector3d a_above(a(0),a(1),a(2)+hover_dist);

    // orientation slightly downwards
    double psi = 1.2;
    double phi = 0.0;

    //updating the orientation for the last cube to achieve a reachable pose
    if(tower_height + 0.05 >= 0.09 ){
        psi = 0.8;
    }

    // move the gripper to hover above A
    robot.setEndeffectorPose(a_above, psi, phi, rclcpp::Duration(duration));
    // add buffer time
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // open the gripper when picking up a block
    if(upMode){
        // open the gripper fully
        robot.setGripperJoint(open_percentage);
        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // lowering the gripper
    robot.setEndeffectorPose(a, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));
    // add buffer time
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // when picking up a block close the gripper, when putting down a block open the gripper
    robot.setGripperJoint(gripper_percentage);
    // add buffer time
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // move the gripper to hover above A to avoid knocking over blocks when motion continues
    robot.setEndeffectorPose(a_above, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));
    // add buffer time
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // close the gripper
    robot.setGripperJoint(close_percentage);
    // add buffer time
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ------------------- Initialization ------------------------
    auto node_ = std::make_shared<rclcpp::Node>("software_node");
    robot::SurrosControl robot(node_);

    // Initialize the SurrosControl instance
    robot.initialize();

    // ------------------- Create Workspace ----------------------
    // initialize the position of the platform in xy coordinates
    Eigen::Vector2d platform(0.15, 0.05);

    // initialize image to store workspace
    Mat img;

    // populate the workspace
    createCube("red", Eigen::Vector2d(0.0, 0.2));
    createCube("green", Eigen::Vector2d(0.08, 0.18), 27);
    createCube("blue", Eigen::Vector2d(-0.08, 0.18), -25);

    drawCurrentWorkspace(img);
    imshow("Workspace", img);

    // find the cubes in the sequence
    vector<string> color_sequence;
    color_sequence.push_back("red");
    color_sequence.push_back("green");
    color_sequence.push_back("blue");

    //------------------------- Pick And Place -------------------------------
    //for the chosen color sequence find the cubes and execute stacking loop
    for(string color: color_sequence){
        vector<Cube> cubes = findColoredCubes(img, color);
        for(Cube cube: cubes){
            //transfore cube center from image pixel coordinates to real world xy coordinates
            Point2f world_center = getRealWorldCubeCenter(cube);
            Eigen::Vector2d center_xy(world_center.x, world_center.y);
            // pick up the cube
            pickUpOrPutDown(center_xy, robot);
            // put the cube on the tower
            pickUpOrPutDown(platform, robot, false, 0.05);
            // add height to the tower
            tower_height += 0.035;

            // remove the cube from current_cubes
            current_cubes.erase(find(current_cubes.begin(), current_cubes.end(), cube));

            // add the cube to the platform in the simulated workspace
            cube.center = getWSCoordinates(platform);
            //update current cubes vector
            current_cubes.push_back(cube);
            
            //draw the updated workspace
            drawCurrentWorkspace(img);
            imshow(color, img);
        }
            
    }

    waitKey(0);    

    return 0;
}
