// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f AP(x - _v[0].x(), y - _v[0].y());
    Eigen::Vector2f AB(_v[1].x() - _v[0].x(), _v[1].y()- _v[0].y());
    Eigen::Vector2f BP(x - _v[1].x(), y - _v[1].y());
    Eigen::Vector2f BC(_v[2].x()-_v[1].x(), _v[2].y()-_v[1].y());
    Eigen::Vector2f CP(x - _v[2].x(), y - _v[2].y());
    Eigen::Vector2f CA(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y());

    auto P_AB = AP.x() * AB.y() - AB.x() * AP.y();
    auto P_BC = BP.x() * BC.y() - BC.x() * BP.y();
    auto P_CA = CP.x() * CA.y() - CA.x() * CP.y();

    if ((P_AB >0 && P_BC >0 && P_CA >0) || (P_AB < 0 && P_BC < 0 && P_CA < 0)) {
        return true;
    }
    else {
        return false;
    }
}  

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            //std::cout << " before change:" << vert.z() << "  ";
            vert.z() = -vert.z() * f1 + f2;
            //std::cout << "after change:" << vert.z() << std::endl;
        }
        //std::cout << std::endl;

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // TODO : Find out the bounding box of current triangle.
    int bounding_box_left_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    int bounding_box_right_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    int bounding_box_bottom_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    int bounding_box_top_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    /*without MSAA*/
    
    // iterate through the pixel and find if the current pixel is inside the triangle
    /*
    for (int x = bounding_box_left_x; x <= bounding_box_right_x; x++) {
        for (int y = bounding_box_bottom_y; y <= bounding_box_top_y; y++) {
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                // If so, use the following code to get the interpolated z value.   
                auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated <  depth_buf[get_index(x, y)]) {
                    depth_buf[get_index(x, y)] = z_interpolated;
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    set_pixel(Eigen::Vector3f(x, y, z_interpolated),  t.getColor());
                }
            }
        }
    }
    */

   
   /*with MSAA*/
    // iterate through the pixel and find if the current pixel is inside the triangle
    
    int inNumber;
    float x_sample;
    float y_sample;
    int sample_index[4];

    std::vector<Eigen::Vector3f> pixelvec;

    float rd[4][2] ={
                    {0.25, 0.25}, 
                    {0.25, 0.75},
                    {0.75, 0.25},
                    {0.75, 0.75}};
    for (int x = bounding_box_left_x; x <= bounding_box_right_x; x++) {
        for (int y = bounding_box_bottom_y; y <= bounding_box_top_y; y++) {
            inNumber = 0;
            for (int i = 0; i < 4; i++){
                    x_sample = x + rd[i][0];
                    y_sample = y + rd[i][1];
                    sample_index[i] =  get_sample_index(int(2 * x_sample), int(2 * y_sample));
                    if (insideTriangle(x_sample, y_sample, t.v)) {
                    // If so, use the following code to get the interpolated z value.   
                    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    //z_interpolated *= w_reciprocal;

                        
                        //std::tuple <float, float, float> sample_Barycentric= computeBarycentric2D(x_sample, y_sample, t.v);
                        //float sample_alpha, sample_beta, sample_gamma;
                        //std::tie(sample_alpha, sample_beta, sample_gamma) = sample_Barycentric;
                        auto[sample_alpha, sample_beta, sample_gamma] = computeBarycentric2D(x_sample, y_sample, t.v);
                        float sample_w_reciprocal = 1.0/(sample_alpha / v[0].w() + sample_beta / v[1].w() + sample_gamma / v[2].w());
                        float sample_z_interpolated = sample_alpha * v[0].z() / v[0].w() + sample_beta * v[1].z() / v[1].w() + sample_gamma * v[2].z() / v[2].w();
                        sample_z_interpolated *= sample_w_reciprocal;
                        //depth_buf[ind] = 0;
                        //depth_buf[get_index(x, y)] = 0;
                        if (sample_z_interpolated <  sample_depth_buf[sample_index[i]]) {
                            sample_depth_buf[sample_index[i]] = sample_z_interpolated;
                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                            sample_frame_buf[sample_index[i]] = t.getColor();
                            inNumber = inNumber + 1;
                        //std::cout << pixelColor<<std::endl;
                        }
                    }    
            }
            if (inNumber > 0){
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                Eigen::Vector3f pixelColor;
                pixelColor <<  (sample_frame_buf[sample_index[0]] + sample_frame_buf[sample_index[1]]
                                                + sample_frame_buf[sample_index[2]] + sample_frame_buf[sample_index[3]])/ 4;
                set_pixel(Eigen::Vector3f(x, y, 0),   pixelColor);
            }
        }
    } 
    
}



void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        //MSAA sample_frame_buf
        std::fill(sample_frame_buf.begin(), sample_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //MSAA sample_depth_buf
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    
    //MSAA
    sample_frame_buf.resize(4 * w * h);
    sample_depth_buf.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_sample_index(int x, int y)
{
    return (2 * height -1 -y) * 2 * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    //auto ind = (height-1-point.y())*width + point.x();
    //frame_buf[ind] = color;
    // old index: auto ind = point.y() + point.x() * width;
    int j = round(point.y());
    int i = round(point.x());
    if (j < 0 || j > height - 1)
        return;
    if (i < 0 || i > width - 1)
        return;
    // auto ind = (height - 1 - round(point.y())) * width + round(point.x());
    auto ind = (height - 1 - j) * width + i;
    frame_buf[ind] = color;
}

// clang-format on