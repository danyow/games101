#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection, orthographic, translate, scale, viewport = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.


    /*    
           top      ∠tef = eye_fov * 0.5f
           /|       t = tan(∠tef) * abs(n) // not top
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
    // 正交 orthographic = translate + scale
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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        // This Api ???
        return_color = payload.texture ->getColor(payload.tex_coords(0), payload.tex_coords(1));
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        // diffuse reflection
        auto L = light.position;
        auto P = point;
        auto v = (eye_pos - P).normalized();
        auto n = normal;
        auto PL = L - P;
        auto l = PL.normalized();

        auto I = light.intensity;
        auto r2 = PL.dot(PL); 

        auto cos_lpn = n.dot(l);
        auto Ld = kd.cwiseProduct(I) / r2 * std::max(0.0f, cos_lpn);

        result_color += Ld;

        // specular highlights
        auto h = (v + l).normalized();
        auto cos_nph = n.dot(h);
        auto Ls = ks.cwiseProduct(I) / r2 * std::pow(std::max(0.0f, cos_nph), p);

        result_color += Ls;

        // ambient lighting
        auto Ia = amb_light_intensity;
        auto La = ka.cwiseProduct(Ia);

        result_color += La;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}}; //Teapot: auto l1 = light{{20, 20, -20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};


    // 08-00:12:40
    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        
        // diffuse reflection
        /*
              L
               \
                l   n  -v-      Ld = kd(I/r2)max(0, n*l)
                 \  |  /        v unuse
                  \ | /
            _______\|/________
                    P
        */
        // 07-00:58:00
        auto L = light.position;
        auto P = point;
        auto v = (eye_pos - P).normalized();
        auto n = normal;
        auto PL = L - P;
        auto l = PL.normalized();

        // 07-00:57:00
        auto I = light.intensity;
        auto r2 = PL.dot(PL); //PL.sqrt();

        auto cos_lpn = n.dot(l);
        // auto Ld = kd * (I / r2) * std::max(0.0f, cos_lpn);
        auto Ld = kd.cwiseProduct(I) / r2 * std::max(0.0f, cos_lpn);

        result_color += Ld;

        // specular highlights
        // 08-00:07:00
        /*
              L
               \         R
                l   n   /  v     
                 \  |  /  /      R
                  \ | /  /
            _______\|/  /________
                     P
        */

        // 08-00:07:22
        /*
              L                 h = bisector(v, l)
               \                        v + l
                l   n h   v       =  ------------
                 \  | |  /           || v + l ||
                  \ | | /
            _______\| |/________  Ls = ks(I/r2)max(0, cos(nph))p
                     P               = ks(I/r2)max(0, n*h)p
        */
        auto h = (v + l).normalized();
        auto cos_nph = n.dot(h);
        // auto Ls = ks * (I / r2) * std::pow(std::max(0.0f, cos_nph), p);
        auto Ls = ks.cwiseProduct(I) / r2 * std::pow(std::max(0.0f, cos_nph), p);

        result_color += Ls;

        // ambient lighting
        // 08-00:15:05
        // La = ka * Ia
        auto Ia = amb_light_intensity;
        // auto La = ka * Ia;
        auto La = ka.cwiseProduct(Ia);

        result_color += La;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}}; //Teapot: auto l1 = light{{20, 20, -20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    

    Eigen::Vector3f return_color = Vector3f(0, 0, 0);

    if (payload.texture)
    {
        return_color = payload.texture -> getColor(payload.tex_coords(0), payload.tex_coords(1));
    }
    

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    auto P = payload.view_pos;
    auto n = normal;
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    Vector3f t;
    t << 
        x * y / sqrt(x * x + z * z ),
        sqrt(x * x + z * z),
        z * y / sqrt(x * x + z * z );
    auto b = n.cross(t);

    Matrix3f TBN;
    TBN << t, b, n;

    auto u = payload.tex_coords(0);
    auto v = payload.tex_coords(1);
    auto w = payload.texture -> width;
    auto h = payload.texture -> height;
    auto dpp = payload.texture -> getColor(u, v).norm();
    auto dup = payload.texture -> getColor(u + 1.0f / w, v).norm();
    auto dvp = payload.texture -> getColor(u, v + 1.0f / h).norm();
    auto du = kh * kn * (dup - dpp);
    auto dv = kh * kn * (dvp - dpp);
    auto ln = Vector3f(-du, -dv, 1);

    P = P + kn * n * dpp;
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

        for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        // diffuse reflection
        auto L = light.position;
        // auto P = point;
        auto v = (eye_pos - P).normalized();
        auto n = normal;
        auto PL = L - P;
        auto l = PL.normalized();

        auto I = light.intensity;
        auto r2 = PL.dot(PL); 

        auto cos_lpn = n.dot(l);
        auto Ld = kd.cwiseProduct(I) / r2 * std::max(0.0f, cos_lpn);

        result_color += Ld;

        // specular highlights
        auto h = (v + l).normalized();
        auto cos_nph = n.dot(h);
        auto Ls = ks.cwiseProduct(I) / r2 * std::pow(std::max(0.0f, cos_nph), p);

        result_color += Ls;

        // ambient lighting
        auto Ia = amb_light_intensity;
        auto La = ka.cwiseProduct(Ia);

        result_color += La;
    }
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}}; //Teapot: auto l1 = light{{20, 20, -20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    Eigen::Vector3f return_color = Vector3f(0, 0, 0);

    if (payload.texture)
    {
        return_color = payload.texture -> getColor(payload.tex_coords(0), payload.tex_coords(1));
    }
    

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    // 10-00::

    /*
     n     t
      \   /|
       \ / | dp
        b--b.x+1
        |
        |
        n  
    */


    // https://www.cnblogs.com/yaukey/archive/2013/06/04/normalmap_tangentspace_tbn.html
    // https://www.cnblogs.com/yaukey/archive/2013/06/04/bumpmap_normalmap_tangentspace_tbn_ii.html
    // https://zhuanlan.zhihu.com/p/144357517


    /*
        n = N.normalize()
        b = B.normalize()
        t = T.normalize()
        e2 = P2 - P0;
        e1 = P1 - P0;

        N                  T                    pi - pj = (ui - uj)t + (vi-vj)b
         \               p1t                    pi - pj = (pit - pjt)t + (pib - pjb)b
          \              /   
           \           p2t    P2_______P1       e2, e1 uv value:
            \          /        ╲     ╱             (t2, b2) = (u2 - u0, v2 - v0)
             \        /        e2╲   ╱e1            (t1, b1) = (u1 - u0, v1 - v0)
              \     p0t           ╲ ╱           -->
               \    /              P0               e2 = t2 * t + b2 * b
                \  /                                e1 = t1 * t + b1 * b
                 \/                             -->
                  ------p2b----p0b---p1b--B         | e1 |   | t1 b1 |  | t |
                                                    |    | = |       |  |   |
                                                    | e2 |   | t2 b2 |  | b |
                                                -->
                                                | x1 y1 z1 |   | t1 b1 |  | xt yt zt |
                                                |          | = |       |  |          |
                                                | x2 y2 z2 |   | t2 b2 |  | xb yb zb |


    */

    /*
        FAQ:
            (1) bump mapping 部分的 h(u,v)=texture_color(u,v).norm, 其中 u,v 是 tex_coords, w,h 是 texture 的宽度与高度
            (2) rasterizer.cpp 中 v = t.toVector4()
            (3) get_projection_matrix 中的 eye_fov 应该被转化为弧度制
            (4) bump 与 displacement 中修改后的 normal 仍需要 normalize
            (5) 可能用到的 eigen 方法：norm(), normalized(), cwiseProduct()
            (6) 实现 h(u+1/w,v) 的时候要写成 h(u+1.0/w,v)
            (7) 正规的凹凸纹理应该是只有一维参量的灰度图，而本课程为了框架使用的简便性而使用了一张 RGB 图作为凹凸纹理的贴图，因此需要指定一种规则将彩色投影到灰度，而我只是「恰好」选择了 norm 而已。为了确保你们的结果与我一致，我才要求你们都使用 norm 作为计算方法。
            (8) bump mapping & displacement mapping 的计算的推导日后将会在光线追踪部分详细介绍，目前请按照注释实现。
    */
    auto n = normal;
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    // 切线空间 abut t???
    // https://zhuanlan.zhihu.com/p/139593847
    // https://zhuanlan.zhihu.com/p/370927083
    // http://games-cn.org/forums/topic/zuoye3-bump-mappingzhongtbndet-gongshizenmetuidaode/
    // auto t = Vector3f(
    //     x * y / sqrt(x * x + z * z ),
    //     sqrt(x * x + z * z),
    //     z * y / sqrt(x * x + z * z )
    // );
    Vector3f t;
    t << 
        x * y / sqrt(x * x + z * z ),
        sqrt(x * x + z * z),
        z * y / sqrt(x * x + z * z );
    auto b = n.cross(t);

    Matrix3f TBN;
    TBN << t, b, n;

    auto u = payload.tex_coords(0);
    auto v = payload.tex_coords(1);
    auto w = payload.texture -> width;
    auto h = payload.texture -> height;
    auto dpp = payload.texture -> getColor(u, v).norm();
    auto dup = payload.texture -> getColor(u + 1.0f / w, v).norm();
    auto dvp = payload.texture -> getColor(u, v + 1.0f / h).norm();
    auto du = kh * kn * (dup - dpp);
    auto dv = kh * kn * (dvp - dpp);
    auto ln = Vector3f(-du, -dv, 1);

    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10}; // Teapot: Eigen::Vector3f eye_pos = {0,3,20};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
