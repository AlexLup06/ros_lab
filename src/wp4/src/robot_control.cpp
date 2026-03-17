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

const Size img_size = Size(638,675);
const Size rect_size = Size(50, 50);
const Point2f base_link = Point2f(230,508);

struct Cube {
    string color;
    Point2f center;
    float rotation = 0.0;
};

vector<vector<Point>> findColorContours(const Mat& img_bgr, const string color="red"){
    // define the hsv values for each possible color
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
        //std::cout << "Contour Area: \n" << area << "\n";
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

void addCubeToWorkspace(Mat& img, Cube& cube, bool remove=false){
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

    if(remove){
        bgr = Scalar(255,255,255);
    }

    RotatedRect rect = RotatedRect(cube.center,rect_size, cube.rotation);
    Point2f vertices2f[4];
    rect.points(vertices2f);
    Point vertices[4];
    for(int i = 0; i < 4; i++){
        vertices[i] = vertices2f[i];
    }
    fillConvexPoly(img, vertices, 4, bgr);
}

Point2f getRealWorldCubeCenter(const Cube& cube){
    float center_x = (cube.center.x-base_link.x)*0.2/435;
    float center_y = (base_link.y-cube.center.y)*0.2/435;
    return Point2f(center_x, center_y);
}

// create a cube with the chosen color and a random center position
Cube createRandomCube(string color="red"){
    Cube cube;
    cube.color = color;
    int center_x = std::rand() % img_size.width;
    int center_y = std::rand() % img_size.height;
    cube.center = Point2f(center_x, center_y);
    cube.rotation = std::rand() % 100;
    return cube;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_ = std::make_shared<rclcpp::Node>("software_node");
    auto surros_control = std::make_shared<robot::SurrosControl>(node_);

    // Initialize the SurrosControl instance
    surros_control->initialize();

    // load and size the image of the workspace
    Mat img = imread("/home/programmer/ros2_ws/src/wp4/workspace.png");
    resize(img, img, img_size);

    //Cube blue_cube = createRandomCube("blue");
    //addCubeToWorkspace(img, blue_cube);

    // add the cubes to the workspace
    Cube red_cube;
    red_cube.color = "red";
    red_cube.center = Point2f(230, 73);
    red_cube.rotation = 0;
    addCubeToWorkspace(img, red_cube);

    Cube green_cube;
    green_cube.color = "green";
    green_cube.center = Point2f(357, 249);
    green_cube.rotation = 27;
    addCubeToWorkspace(img, green_cube);

    Cube blue_cube;
    blue_cube.color = "blue";
    blue_cube.center = Point2f(105, 245);
    blue_cube.rotation = -25;
    addCubeToWorkspace(img, blue_cube);

    // find the cubes
    vector<Cube> red_cubes = findColoredCubes(img, "red");

    for (Cube red_cube: red_cubes){
        drawMarker(img, red_cube.center, Scalar(0,0,0));
        std::cout << "Red Cube \n";
        std::cout << "Cube Center: \n" << red_cube.center << "\n";
        std::cout << "Cube Center Real World: \n" << getRealWorldCubeCenter(red_cube) << "\n";
    }

    vector<Cube> green_cubes = findColoredCubes(img, "green");

    for (Cube green_cube: green_cubes){
        drawMarker(img, green_cube.center, Scalar(0,0,0));
        std::cout << "Green Cube \n";
        std::cout << "Cube Center: \n" << green_cube.center << "\n";
        std::cout << "Cube Center Real World: \n" << getRealWorldCubeCenter(green_cube) << "\n";
    }

    vector<Cube> blue_cubes = findColoredCubes(img, "blue");

    for (Cube blue_cube: blue_cubes){
        drawMarker(img, blue_cube.center, Scalar(0,0,0));
        std::cout << "Blue Cube \n";
        std::cout << "Cube Center: \n" << blue_cube.center << "\n";
        std::cout << "Cube Center Real World: \n" << getRealWorldCubeCenter(blue_cube) << "\n";
    }

    drawMarker(img, base_link, Scalar(0,0,0));

    imshow("Test", img);


    waitKey(0);    

    return 0;
}
