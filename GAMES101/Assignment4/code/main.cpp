#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255; //显示是红色
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int len = control_points.size();
    if (len == 1)
        return control_points[0];

    std::vector<cv::Point2f> lerp_control_points(len - 1, cv::Point2f(0.0, 0.0));
    for (int i = 0; i < len - 1; ++i)
    {
        lerp_control_points[i] = t * control_points[i] + (1 - t) * control_points[i + 1];
    }
    
    return recursive_bezier(lerp_control_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t) ;

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; //显示是绿色

        float xDelta = point.x - std:: floor(point.x);
        float yDelta = point.y - std:: floor(point.y);

        int  xDir = xDelta < 0.5f ? -1 : 1;
        int yDir = yDelta < 0.5f ? -1 : 1;

        cv::Point2f p0 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p1 = cv::Point2f(std::floor(point.x + xDir * 1.0f) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p2 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y + yDir * 1.0f) + 0.5f);
        cv::Point2f p3 = cv::Point2f(std::floor(point.x + xDir * 1.0f) + 0.5f, std::floor(point.y + yDir * 1.0) + 0.5f);

        std::vector<cv::Point2f> pvec;
        pvec.push_back(p1);
        pvec.push_back(p2);
        pvec.push_back(p3);

        float d1 = std::sqrt(std::pow(p0.x - point.x, 2) + std::pow(p0.y - point.y, 2));

        for (auto& p: pvec)
        {


            float dp = std::sqrt(std::pow(p.x - point.x, 2) + std::pow(p.y - point.y, 2));
            float weight = d1 / dp;
            float colorG = window.at<cv::Vec3b>(p.y, p.x)[1];
            colorG = std::fmin(colorG, weight * 255.0);
            window.at<cv::Vec3b>(p.y, p.x)[1] = (int)colorG;
        }
 
               
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
            // naive_bezier(control_points, window);
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
