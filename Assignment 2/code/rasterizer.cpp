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


// static bool insideTriangle(int x, int y, const Vector3f* _v)
// 实现Supersampling，需要判断小数
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Eigen::Vector3f p0{x,y,1};
    Eigen::Vector3f p1p2 = _v[1] - _v[0];
    Eigen::Vector3f p2p3 = _v[2] - _v[1];
    Eigen::Vector3f p3p1 = _v[0] - _v[2];
    Eigen::Vector3f p1p0 = p0 - _v[0];
    Eigen::Vector3f p2p0 = p0 - _v[1];
    Eigen::Vector3f p3p0 = p0 - _v[2];
    float z1 = p1p2.cross(p1p0).z();
    float z2 = p2p3.cross(p2p0).z();
    float z3 = p3p1.cross(p3p0).z();
    return (z1 > 0 && z2 > 0 && z3 > 0 || z1 < 0 && z2 < 0 && z3 < 0);
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

    // float f1 = (50 - 0.1) / 2.0;
    // float f2 = (50 + 0.1) / 2.0;
    float f1 = (50 - 0.1) / 2.0;
    float f2 = - (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        // v 是经过视角变换后三角形三个顶点的坐标
        //Homogeneous division
        for (auto& vec : v) {
            //vec /= vec.w(); 这个地方源代码有问题
            // 不应该改变vec.w
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);//将x从[-1,1]映射到[0,width]
            vert.y() = 0.5*height*(vert.y()+1.0);//将y从[-1,1]映射到[0,height]
            vert.z() = vert.z() * f1 + f2;
            //f2修改前将Z从[-1,1]映射到[1,50]
            //修改后将Z从[-1,1]映射到[-1,-50]
        }
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
    int x_min = this->width,x_max = 0,y_min = this->height,y_max = 0;
    for(const auto& item : t.v){
        if(item.x() > x_max) x_max = item.x();
        if(item.x() < x_min) x_min = item.x();
        if(item.y() > y_max) y_max = item.y();
        if(item.y() < y_min) y_min = item.y();
    }
    
    // iterate through the pixel and find if the current pixel is inside the triangle
    // sub_point用来遍历4个小点
    // Supersampling
    bool Super = false;
    std::vector<std::pair<float,float>>sub_point{{0.25,0.25},{0.25,0.75},{0.75,0.25},{0.75,0.75}};
    for(int x = x_min; x < x_max + 1; x++){
        for(int y = y_min; y < y_max + 1; y++){
            if(!Super){//不抗锯齿
                if(insideTriangle(x, y, t.v)){
                    // If so, use the following code to get the interpolated z value.
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    if(z_interpolated > depth_buf[get_index(x,y)]){
                        depth_buf[get_index(x,y)] = z_interpolated;
                        set_pixel(Vector3f(x,y,1),t.getColor());
                    }
                }
            }
            else{//抗锯齿
                float x_,y_;
                Vector3f color = {0,0,0};
                for(int k = 0; k < 4; k++){
                    x_ = x + sub_point[k].first;
                    y_ = y + sub_point[k].second;
                    if(insideTriangle(x_,y_,t.v)){
                        // If so, use the following code to get the interpolated z value.
                        auto[alpha, beta, gamma] = computeBarycentric2D(x_, y_, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        if(z_interpolated > depth_sample_buf[get_index(x,y) * 4 + k]){
                            depth_sample_buf[get_index(x,y) * 4 + k] = z_interpolated;
                            sample_buf[get_index(x,y) * 4 + k] = t.getColor()/4;
                        }
                    }
                    color += sample_buf[get_index(x,y) * 4 + k];
                }
                set_pixel(Vector3f(x,y,1),color);
            }
            // color_weight == 0不管
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
        std::fill(sample_buf.begin(), sample_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        // std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //这里修改为最小值
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
        std::fill(depth_sample_buf.begin(), depth_sample_buf.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    // fram_buf的操作，sample_buf都做一遍
    sample_buf.resize(w * h * 4);
    depth_buf.resize(w * h);
    depth_sample_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on