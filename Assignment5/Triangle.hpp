#pragma once

#include "Object.hpp"

#include <cstring>


/*
 * v0, v1, v2 是三角形的三个顶点，orig 是光线的起点，dir 是光线单位化的方向向量。tnear,
 * u, v 是你需要使用我们课上推导的Moller-Trumbore 算法来更新的参数。
 */
bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v) {
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.

    // 13-00:49:30
    //[1] 直线与三角形相交Moller Trumbore算法推导：https://www.blurredcode.com/2020/04/%E7%9B%B4%E7%BA%BF%E4%B8%8E%E4%B8%89%E8%A7%92%E5%BD%A2%E7%9B%B8%E4%BA%A4moller-trumbore%E7%AE%97%E6%B3%95%E6%8E%A8%E5%AF%BC/
    //[2] 克莱姆法则：https://zh.wikipedia.org/wiki/%E5%85%8B%E8%90%8A%E5%A7%86%E6%B3%95%E5%89%87
    //[3] Möller–Trumbore 算法：https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    //[4] 行列式的几何意义：https://www.cnblogs.com/andyjee/p/3491487.html
    /*
     *  O + tD = (1-b1-b2)P0 + b1P1 + b2P2
     * */
    auto P0 = v0;
    auto P1 = v1;
    auto P2 = v2;
    auto O = orig;
    auto D = dir;

    auto E1 = P1 - P0;
    auto E2 = P2 - P0;
    auto S = O - P0;
    auto S1 = crossProduct(D, E2);
    auto S2 = crossProduct(S, E1);

    auto ES1 = dotProduct(S1, E1);

    float t = dotProduct(S2, E2) / ES1;
    float b1 = dotProduct(S1, S) / ES1;
    float b2 = dotProduct(S2, D) / ES1;

    tnear = t;
    u = b1;
    v = b2;
    return (t > 0) && (b1 > 0) && (b2 > 0) && (1 - b1 - b2 > 0);
//    return false;
}

class MeshTriangle : public Object {
public:
    MeshTriangle(const Vector3f *verts, const uint32_t *vertsIndex, const uint32_t &numTris, const Vector2f *st) {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f &orig, const Vector3f &dir, float &tnear, uint32_t &index,
                   Vector2f &uv) const override {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear) {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void
    getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &index, const Vector2f &uv, Vector3f &N,
                         Vector2f &st) const override {
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

    Vector3f evalDiffuseColor(const Vector2f &st) const override {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
