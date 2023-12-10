//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {

        if(u<0) u=0;
        if(v<0) v=0;
        if(u>1) u=1;
        if(v>1) v=1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {

        if(u<0) u=0;
        if(v<0) v=0;
        if(u>1) u=1;
        if(v>1) v=1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float ul = floor(u_img);
        float uh = ceil(u_img);
        float vl = floor(v_img);
        float vh = ceil(v_img);
        float s = (u_img - ul) / (uh - ul);
        float t = (v_img - vl) / (vh - vl);

        // 双线性插值
        auto color_o = image_data.at<cv::Vec3b>(v_img, u_img);
        auto color_00 = image_data.at<cv::Vec3b>(vl, ul);//color的序号按照u,v的顺序来，大值对应h，小值对应l
        auto color_01 = image_data.at<cv::Vec3b>(vh, ul);
        auto color_10 = image_data.at<cv::Vec3b>(vl, uh);
        auto color_11 = image_data.at<cv::Vec3b>(vh, uh);
        auto color_lerp1 = (1 - s) * color_00 + s * color_10;
        auto color_lerp2 = (1 - s) * color_01 + s * color_11;

        auto color = (1 - t) * color_lerp1 + t * color_lerp2 ;

            

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
