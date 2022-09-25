#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)   //Translate center to origin    
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << -1, 0, 0, -eye_pos[0], 0, -1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)    //模型绕Z轴旋转
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f Zrotate;
    Zrotate << std::cos(rotation_angle/180.0*acos(-1)),-std::sin(rotation_angle/180.0*acos(-1)),0,0,
    std::sin(rotation_angle/180.0*acos(-1)),std::cos(rotation_angle/180.0*acos(-1)),0,0,0,0,1,0,0,0,0,1;

    model = Zrotate * model;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis,float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f E = Eigen::Matrix3f::Identity();  //I
    Eigen::Matrix3f temp;
    angle=angle/180*MY_PI;
    Eigen::Matrix3f N;  //Rodrigue Matrix
    N << 0,-axis[2],axis[1],axis[2],0,-axis[0],-axis[1],axis[0],0;
    temp = std::cos(angle) * E + (1-std::cos(angle)) * axis * axis.adjoint() + std::sin(angle) * N;
    model << temp(0,0),temp(0,1),temp(0,2),0,temp(1,0),temp(1,1),temp(1,2),0,temp(2,0),temp(2,1),temp(2,2),0,0,0,0,1;
    
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // zNear=zNear*-1;
    // zFar=zFar*-1;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f m;
    m << zNear,0,0,0,0,zNear,0,0,0,0,zNear+zFar,-zNear*zFar,0,0,1,0;   //persp->ortho

    float halfangle = eye_fov/2*MY_PI/180;
    float top = std::tan(halfangle)*zNear;
    float buttom = -top;
    float right = top*aspect_ratio;
    float left = -right;

    Eigen::Matrix4f n,p;

    n << 2/(right-left),0,0,0,0,2/(top-buttom),0,0,0,0,2/(zNear-zFar),0,0,0,0,1;       //S
    p << 1,0,0,-(right+left)/2,0,1,0,-(top+buttom)/2,0,0,1,-(zNear+zFar)/2,0,0,0,1;    //T

    projection = n*p*m;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);  //width and length

    Eigen::Vector3f eye_pos = {0, 0, 5};  //

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};    //

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};  //

    //Eigen::Vector3f axis1 = {1,0,1}; //rotation example

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        //r.set_model(get_rotation(axis1,angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_rotation(axis1,angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
