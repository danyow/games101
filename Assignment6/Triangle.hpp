#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>

bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1,
                          const Vector3f &v2, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v) {
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det == 0 || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}

class Triangle : public Object {
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    Material *m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr)
            : v0(_v0), v1(_v1), v2(_v2), m(_m) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
    }

    bool intersect(const Ray &ray) override;

    bool intersect(const Ray &ray, float &tnear,
                   uint32_t &index) const override;

    Intersection getIntersection(Ray ray) override;

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }

    Vector3f evalDiffuseColor(const Vector2f &) const override;

    Bounds3 getBounds() override;
};

class MeshTriangle : public Object {
public:
    MeshTriangle(const std::string &filename) {
        objl::Loader loader;
        loader.LoadFile(filename);

        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;
            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z) *
                            60.f;
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            auto new_mat =
                    new Material(MaterialType::DIFFUSE_AND_GLOSSY,
                                 Vector3f(0.5, 0.5, 0.5), Vector3f(0, 0, 0));
            new_mat->Kd = 0.6;
            new_mat->Ks = 0.0;
            new_mat->specularExponent = 0;

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], new_mat);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri: triangles)
            ptrs.push_back(&tri);

        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray &ray) { return true; }

    bool intersect(const Ray &ray, float &tnear, uint32_t &index) const {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
                                     u, v) &&
                t < tnear) {
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const {
        float scale = 5;
        float pattern =
                (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray) {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel *bvh;

    Material *m;
};

inline bool Triangle::intersect(const Ray &ray) { return true; }

inline bool Triangle::intersect(const Ray &ray, float &tnear,
                                uint32_t &index) const {
    return false;
}

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray) {
    Intersection inter;

    if (dotProduct(ray.direction, normal) > 0)
        return inter;
    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;

    /*
     *  这就是Moller-Trumbore算法的计算过程。一共得到三个变量：三角形的两个重心坐标u，v，还有距离t_tmp。
     *
     *  前面所有return都返回了默认初始化的inter，找到Intersection类的默认构造函数，可以发现inter.happened=false，刚好和失败return的条件（光线与三角形未相交）一致。
     *
     *  最后的返回值仍然是inter，把算法的成功条件写出来，在里面修改inter的成员值即可。第二步完成。
     * */

    // TODO find ray triangle intersection


    // 13-00:49:30
    //[1] 直线与三角形相交Moller Trumbore算法推导：https://www.blurredcode.com/2020/04/%E7%9B%B4%E7%BA%BF%E4%B8%8E%E4%B8%89%E8%A7%92%E5%BD%A2%E7%9B%B8%E4%BA%A4moller-trumbore%E7%AE%97%E6%B3%95%E6%8E%A8%E5%AF%BC/
    //[2] 克莱姆法则：https://zh.wikipedia.org/wiki/%E5%85%8B%E8%90%8A%E5%A7%86%E6%B3%95%E5%89%87
    //[3] Möller–Trumbore 算法：https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    //[4] 行列式的几何意义：https://www.cnblogs.com/andyjee/p/3491487.html
    /*
     *  O + tD = (1-b1-b2)P0 + b1P1 + b2P2
     * */
//    auto P0 = v0;
//    auto P1 = v1;
//    auto P2 = v2;
//    auto O = orig;
//    auto D = dir;
//
//    auto E1 = P1 - P0;
//    auto E2 = P2 - P0;
//    auto S = O - P0;
//    auto S1 = crossProduct(D, E2);
//    auto S2 = crossProduct(S, E1);
//
//    auto ES1 = dotProduct(S1, E1);
//
//    float t = dotProduct(S2, E2) / ES1;
//    float b1 = dotProduct(S1, S) / ES1;
//    float b2 = dotProduct(S2, D) / ES1;
//
//    tnear = t;
//    u = b1;
//    v = b2;
//    return (t > 0) && (b1 > 0) && (b2 > 0) && (1 - b1 - b2 > 0);
    // `happened` 表示光与三角形是否出现了碰撞，这项为false时，整个框架不会再考虑剩余的所有元素；
    inter.happened = (t_tmp > 0) && (u > 0) && (v > 0) && (1 - u - v > 0);
    // `coords` 表示光首次打到三角形的坐标，可以用Ray类中重载的“( )”来计算；
    inter.coords = ray(t_tmp);
    // `normal` 就是三角形的 `normal`；
    inter.normal = this->normal;
    // `distance` 表示光从origin点出发到抵达coords的距离；
    inter.distance = t_tmp;
    // `obj` 应该是表示后边BVH树中叶子节点中所包含的 `primitive`，所以这里直接把这个函数所属的 `Triangle` 实例丢进去应该是正确的；
    inter.obj = this;
    // `m` 指三角形的材质。
    inter.m = m;

    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f &) const {
    return Vector3f(0.5, 0.5, 0.5);
}
