#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)    //mouse choose control point
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

//Example
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)    //Algebraic Formula
{
    //4 control points
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)   
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1) return control_points[0];    //Only one points,algorithm end. And this point is in Bazier when t

    std::vector<cv::Point2f> temp_control_points;

    for(int i = 0; i < (control_points.size()-1); i++)
    {
        temp_control_points.push_back(t * control_points[i] + (1-t) * control_points[i+1]);   //t:(1-t)
        //temp_control_points.push_back((1-t) * control_points[i] + t * control_points[i+1]);   //(1-t):t
    }

    //return cv::Point2f();   //The Point coordinate in Bezier Curve
    return recursive_bezier(temp_control_points,t);   //Recursive
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.  

    for(double t = 0.0; t <= 1.0; t+=0.001)   //De Casteljau Algorithem
    {
        auto point = recursive_bezier(control_points,t);   //Recurisve,and finally get the point in Bazier when t
        //img.at<cv::Vec3b>(int y,int x) means Color Brightness in Coordinate (x,y)
        //img.at<cv::Vec3b>(int y,int x)[] means RGB aisel
        window.at<cv::Vec3b>(point.y,point.x)[1] = 255;   //require to be green,that RGB[1] is Green;
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
