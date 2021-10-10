#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
const int CONTROL_POINTS_COUNT = 4;
const int STEP = 4;
const int POINT_COUNT = 10000;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

std::vector<cv::Point2f> recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    auto length = control_points.size();
    std::vector<cv::Point2f> finds = std::vector<cv::Point2f>();
    for (size_t index = 0; index < length - 1; index++)
    {
        finds.push_back((1.0f - t) * control_points[index] + t * control_points[index + 1]);
    }
    
    if (finds.size() == 1)
    {
        return finds;
    } 
    
    return recursive_bezier(finds, t);

}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    // auto &p_0 = points[0];
    // auto &p_1 = points[1];
    // auto &p_2 = points[2];
    // auto &p_3 = points[3];

    // for (double t = 0.0; t <= 1.0; t += 0.001) 
    // {
    //     auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
    //              3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

    //     // Changed Color
    //     window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    // }

    auto step = 1.0 / POINT_COUNT;
    for (double t = 0.0; t <= 1.0; t += step)
    {
        for (auto &&point : recursive_bezier(control_points, t))
        {
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        }   
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    auto step = 1.0 / POINT_COUNT;
    for (double t = 0.0; t <= 1.0; t += step)
    {
        for (auto &&point : recursive_bezier(control_points, t))
        {
        
            cv::Point2f pix_center_point(0.5, 0.5);
            pix_center_point += point;

            // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

            // draw red at point w/t anti-aliasing
            int min_x = std::max(0, (int)floor(point.x));
            int max_x = std::min(window.cols-1, (int)ceil(point.x));
            int min_y = std::max(0, (int)floor(point.y));
            int max_y = std::min(window.rows-1, (int)ceil(point.y));

            static float pow_antialiasing = 0.5;

            window.at<cv::Vec3b>(min_y, min_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
            window.at<cv::Vec3b>(max_y, min_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
            window.at<cv::Vec3b>(min_y, max_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
            window.at<cv::Vec3b>(max_y, max_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
        }   
    }
}

void step_bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    auto step = 1.0 / POINT_COUNT;
    for (double t = 0.0; t <= 1.0; t += step)
    {

        std::vector<cv::Point2f> steps = std::vector<cv::Point2f>();
        auto loop = (int)ceil(control_points.size() / STEP);
        for (size_t i = 0; i < loop; i++)
        {
            for (size_t j = 0; j < STEP; j++)
            {
                auto index = i * STEP + j;
                if (index <= control_points.size() - 1)
                {
                    steps.push_back(control_points[index]);
                }
            }
            for (auto &&point : recursive_bezier(steps, t))
            {
                cv::Point2f pix_center_point(0.5, 0.5);
                pix_center_point += point;

                // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

                // draw red at point w/t anti-aliasing
                int min_x = std::max(0, (int)floor(point.x));
                int max_x = std::min(window.cols-1, (int)ceil(point.x));
                int min_y = std::max(0, (int)floor(point.y));
                int max_y = std::min(window.rows-1, (int)ceil(point.y));

                static float pow_antialiasing = 0.5;

                window.at<cv::Vec3b>(min_y, min_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
                window.at<cv::Vec3b>(max_y, min_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (min_x-point.x)*(min_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
                window.at<cv::Vec3b>(min_y, max_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (min_y-point.y)*(min_y-point.y))), pow_antialiasing) );
                window.at<cv::Vec3b>(max_y, max_x)[1] += (uint) ( 255 * pow((1.0f - sqrt( (max_x-point.x)*(max_x-point.x) + (max_y-point.y)*(max_y-point.y))), pow_antialiasing) );
            }   
            steps.clear();
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    // window.

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() > STEP - 1) 
        {
            // naive_bezier(control_points, window);
            step_bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            // key = cv::waitKey(0);

            // return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
