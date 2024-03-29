#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 
    1, 0, 0, -eye_pos[0], 
    0, 1, 0, -eye_pos[1],
    0, 0, 1,-eye_pos[2],
    0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float arc = rotation_angle/180 * MY_PI;
    Eigen::Matrix4f rot;
    rot << 
        std::cos(arc), - std::sin(arc), 0 , 0,
        std::sin(arc), std::cos(arc), 0 , 0,
        0, 0, 1 , 0,
        0, 0, 0 , 1;
    model = rot * model;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    float l,r,t,b,n,f;
    float arc = eye_fov / 180 * MY_PI;
    t = abs(zNear) * std::tan(arc/2);
    b = -t;
    r = t * aspect_ratio;
    l = -r;
    n = zNear;
    f = zFar;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    
    Eigen::Matrix4f orthongraphic;
    Eigen::Matrix4f zoom,move;
    zoom << 
        2/(r - l), 0, 0 , 0,
        0, 2/(t - b), 0 , 0,
        0, 0, 2/(n - f) , 0,
        0, 0, 0 , 1;
    move << 
        1, 0, 0 , -(l + r)/2,
        0, 1, 0 , -(t + b)/2,
        0, 0, 1 , -(n + f)/2,
        0, 0, 0 , 1;
    orthongraphic = zoom * move;

    Eigen::Matrix4f perspctive;
    perspctive << 
        n, 0, 0 , 0,
        0, n, 0 , 0,
        0, 0, n + f , - n * f,
        0, 0, 1 , 0;
        
    projection = perspctive * projection;
    projection = orthongraphic * projection;
    return projection;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    // set commond line parameters
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);// reset the output file name
        }
        else
            return 0;
    }
    // the size of window
    rst::rasterizer r(700, 700);
    // the position of eyes(camera)
    Eigen::Vector3f eye_pos = {0, 0, 5};
    // the position for each vertex of the triangle
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // the id for each vertex
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};
    

    //return struct pos_buf_id to store the id 0
    auto pos_id = r.load_positions(pos);
    //return struct rst::ind_buf_id, id is provided by the user, 
    //ind_buf is a map and will store {id,ind}
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        // eye_pos is a Vector3f, the position of camera
        // get_view_matrix(eye_pos) returns a Matrix4f for remove to origin
        r.set_view(get_view_matrix(eye_pos));
        // get_projection_matrix four paremeter eye_fov, aspect_ratio, zNear, zFar
        // 这里修改成了在Z的负半轴
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // std::vector<Eigen::Vector3f> my_image(700*700);
        // for(float i = 0; i < 700 * 700; i++)
        //     my_image[i] << i/700, 1 - i/700,i/700;
        // cv::Mat image(700, 700, CV_32FC3, my_image.data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // get the current status
        std::cout << "angle: " << angle << "     ";
        std::cout << "frame count: " << frame_count++ << '\n';

        // change the angle by commond line
        if (key == 'a') {
            angle += 5;
        }
        else if (key == 'd') {
            angle -= 5;
        }
    }

    return 0;
}