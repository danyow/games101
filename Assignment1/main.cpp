#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0,           1;
    view = translate * view;

    return view;
}


Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    // 04-00:16:12
    Matrix4f model = Matrix4f::Identity();
    Matrix3f R, I = Matrix3f::Identity();

    float rad = MY_PI / 180.0f * angle;

    auto n = axis.normalized();

    Matrix3f N = Matrix3f::Identity();
    N <<      0, -n.z(),  n.y(),
          n.z(),      0, -n.x(),
         -n.y(),  n.x(),      0;

    R = I * cos(rad) + (1 - cos(rad)) * n * n.transpose() + sin(rad) * N;

    model.block<3, 3>(0, 0) = R;

    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float rad = MY_PI / 180.0f * rotation_angle;

    // 04-00:09:13  
    // Create the model matrix for rotating the triangle around the X axis.
    // model << 1,        0,         0, 0,
    //          0, cos(rad), -sin(rad), 0,
    //          0, sin(rad),  cos(rad), 0,
    //          0,        0,         0, 1;

    // Create the model matrix for rotating the triangle around the Y axis.
    // model << cos(rad), 0, sin(rad), 0,
    //                 0, 1,        0, 0,
    //         -sin(rad), 0, cos(rad), 0,
    //                 0, 0,        0, 1;


    // Create the model matrix for rotating the triangle around the Z axis.
    model << cos(rad), -sin(rad), 0, 0,
             sin(rad),  cos(rad), 0, 0,
                    0,         0, 1, 0,
                    0,         0, 0, 1;

    return model;
    // TEST: Bonus
    // return get_rotation(Vector3f(0, 0, 1), rotation_angle);
    // return get_rotation(Vector3f(0, 1, 0), rotation_angle);
    // return get_rotation(Vector3f(1, 0, 0), rotation_angle);
    // return get_rotation(Vector3f(1, 1, 1), rotation_angle);
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection, orthographic, translate, scale, viewport = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.


    /*    
           top      ???tef = eye_fov * 0.5f
           /|       t = tan(???tef) * abs(n) // not top
          t |
         /| |
        / | |
        e-n-f
    */


    // Z = -Z
    float z = -1;
    float n = zNear * z;
    float f = zFar * z;
    // 05-00:12:12
    float tef = eye_fov * 0.5f;
    float t = tan(MY_PI / 180.0f * tef) * abs(n); 
    float r = t * aspect_ratio; // right
    float l = -r;
    float b = -t;
    // float w = r - l;
    // float h = t - b;

    // // 05-00:22:11
    // viewport << w * 0.5f,        0, 0, w * 0.5f,
    //                    0, h * 0.5f, 0, h * 0.5f,
    //                    0,        0, 1,        0,
    //                    0,        0, 0,        1;

    // 04-00:49:02
    // ?????? orthographic = translate + scale
    translate << 1, 0, 0, - (r + l) * 0.5f,
                 0, 1, 0, - (t + b) * 0.5f,
                 0, 0, 1, - (n + f) * 0.5f,
                 0, 0, 0,                1;

    scale << 2 / (r - l),           0,           0, 0,
                       0, 2 / (t - b),           0, 0,
                       0,           0, 2 / (n - f), 0,
                       0,           0,           0, 1;
    orthographic = scale * translate;

    // 04-01:15:08
    projection << n,  0,      0,         0,
                  0,  n,      0,         0,
                  0,  0,  n + f,  -(n * f),
                  0,  0,      1,         0;
                  

    projection = orthographic * projection;

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

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

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

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::cout << "Key: " << key << '\n';

        if (key == 1048673) {
            angle += 10;
        }
        else if (key == 1048676) {
            angle -= 10;
        }
    }

    return 0;
}
