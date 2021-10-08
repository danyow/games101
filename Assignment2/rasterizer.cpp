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
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //  _v0, v1, v2 in counter clockwise order_
    /*  c----------------
        |\              |
        | \             |
        |  \            |
        | p \           |
        |    b          |
        |   /           |
        |  /            |
        | /             |
        |/              |
        a----------------
    */
   Vector3f p(x, y, 0);

   const Vector3f& a = _v[0];
   const Vector3f& b = _v[1];
   const Vector3f& c = _v[2];

    Vector3f c_ap_ab = (p - a).cross(b - a);
    Vector3f c_ac_ab = (c - a).cross(b - a);

    Vector3f c_bp_bc = (p - b).cross(c - b);
    Vector3f c_ba_bc = (a - b).cross(c - b);

    Vector3f c_cp_ca = (p - c).cross(a - c);
    Vector3f c_cb_ca = (b - c).cross(a - c);

    /*    
        from: ![caspar](http://games-cn.org/forums/topic/zuoye2sanjiaoxingdibianshangchuxianleyuanjiao/)
        问题出在insideTriangle使用dot判断错误，假如
            auto times_0 = (px – a).cross(b – a)
            ap.cross(ab)
        做叉乘那么对应的dot应该是和
            (c – a).cross(b – a)
            ac.cross(ab)
        进行点乘，并且结果大于等于0；
        表示这个点在这个夹角内，同理再计算其余两条边全部大于等于0则点在三角形内。
        不过也可以不用dot来判断，直接取time_012三个值同时小于0或者同时大于0则点在三角形内。
    */
    
    return c_ap_ab.dot(c_ac_ab) >= 0    // 
        && c_bp_bc.dot(c_ba_bc) >= 0    //
        && c_cp_ca.dot(c_cb_ca) >= 0    // 
        ;
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
            vert.z() = vert.z() * f1 + f2;
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
    // TODO : Find out the bounding box of current triangle.

    auto v = t.toVector4();
    float max_x, max_y, min_x, min_y = 0; // Bounding Box
    for (size_t index = 0; index < 3; index++)
    {
        float x = t.v[index].x();
        float y = t.v[index].y();

        if (index == 0)
        {
            min_x = max_x = x;
            min_y = max_y = y;
            continue;
        } 
        
        min_x = min_x > x ? x : min_x;
        min_y = min_y > y ? y : min_y;

        max_x = max_x < x ? x : max_x;
        max_y = max_y < y ? y : max_y;
    }
    
    /*  
          --------b   a: (min_x, min_y)
          |       |   b: (max_x, max_y)
          |       |
        y |       |
        ^ |       |   
        | a--------
        o---> x
    */

    
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++)
        {
            /*
                1: p[-----a-----] = 1/2
                2: p[--a--|--b--] = 1/4 3/4
                3: p[-a-|-b-|-c-] = 1/6 3/6 5/6
            */
            float k = 1.0f / (MSAA * 2.0f);
            // std::cout  << k << std::endl;
            int count = 0;
            for (size_t i = 0; i < MSAA; i++)
            {
                for (size_t j = 0; j < MSAA; j++)
                {
                    float x_pos = x + (2 * i + 1) * k;
                    float y_pos = y + (2 * j + 1) * k;

                    if (!insideTriangle(x_pos, y_pos, t.v))
                    {
                        continue;
                    }
                    
                    // inside
                    // If so, use the following code to get the interpolated z value.
                    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    //z_interpolated *= w_reciprocal;
                    auto[alpha, beta, gamma] = computeBarycentric2D(x_pos, y_pos, t.v);
                    // w 倒数
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    // z 插值
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                
                    /*
                        ## Index:

                        x + (y) * width
                        0, 1, 2, 3, 4,  
                        5, 6, 7, 8, 9; 

                        (x * msaa + i) + j * (width * msaa) + y * (width * msaa * msaa)
                        | 0, 1|  2, 3|  4, 5|  6, 7|  8, 9| 
                        |10,11| 12,13| 14,15| 16,17| 18,19|
                        |20,21| 22,23| 24,25| 26,27| 28,29|
                        |30,31| 32,33| 34,35| 36,37| 38,39|
                    */
                    // int index = get_index(x, y); // (height-1-y)*width + x;
                    int index = (x * MSAA + i) + j * (width * MSAA) + y * (width * MSAA * MSAA);

                    if (-z_interpolated >= depth_buf[index]) 
                    {
                        continue;
                    }
                    
                    color_buf[index] = t.getColor();
                    depth_buf[index] = -z_interpolated;
                    count ++;
                }
            }

            if (count > 0)
            {
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                Vector3f final_color = Vector3f(0, 0, 0);
                for (size_t i = 0; i < MSAA; i++)
                {
                    for (size_t j = 0; j < MSAA; j++)
                    {
                        int index = (x * MSAA + i) + j * (width * MSAA) + y * (width * MSAA * MSAA);
                        final_color += (color_buf[index] / (float)(MSAA * MSAA));
                    }
                }
                set_pixel(Vector3f(x, y, 0), final_color);
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
        std::fill(color_buf.begin(), color_buf.end(), Eigen::Vector3f{0, 0, 0}); // Bonus
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    color_buf.resize(w * h * MSAA * MSAA);
    depth_buf.resize(w * h * MSAA * MSAA);
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