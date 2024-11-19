#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <cmath>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    
    struct Vec3f
    {
        float x, y, z;

        float operator[](int idx) const {
            if (idx == 0) return x;
            else if (idx == 1) return y;
            else return z;
        }
        float& operator[](int idx) {
            if (idx == 0) return x;
            else if (idx == 1) return y;
            else return z;
        }
    };

    struct Ray{
        Vec3f dir;
        Vec3f origin;
        int depth;
    };
    struct HitRecord{
        Vec3f intersection_p;
        int m;
        Vec3f normal;
    };

    struct Vec3i
    {
        int x, y, z;

        int operator[](int idx) const {
            if (idx == 0) return x;
            else if (idx == 1) return y;
            else return z;
        }
        int& operator[](int idx) {
            if (idx == 0) return x;
            else if (idx == 1) return y;
            else return z;
        }
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror = false;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
        Vec3f normal;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face faces;
        
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    enum PrimitiveType {
        TRIANGLE,
        SPHERE
    };

    struct Primitive {
        PrimitiveType type;
        int material_id;
        Vec3f bbox_min;
        Vec3f bbox_max;

        // Triangle data
        Vec3f v0, v1, v2;
        Vec3f normal;

        // Sphere data
        Vec3f center;
        float radius;
    };

    struct BVHNode {
        Vec3f bbox_min; // Minimum corner of the bounding box
        Vec3f bbox_max; // Maximum corner of the bounding box
        BVHNode* left = nullptr;    // Pointer to the left child
        BVHNode* right = nullptr;   // Pointer to the right child
        std::vector<int> primitive_indices; // Indices of primitives
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        std::vector<Primitive> primitives;
        std::vector<int> primitive_indices;
        BVHNode* bvh_root = nullptr; // Initialize to nullptr


        //Functions
        void loadFromXml(const std::string &filepath);
        void buildBVH();
    
    };
    
float inline dot(parser::Vec3f vec1, parser::Vec3f vec2){
    return vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z;
}
parser::Vec3f inline cross(parser::Vec3f vec1, parser::Vec3f vec2){
    parser::Vec3f vector{
    vec1.y*vec2.z-vec1.z*vec2.y,
    vec1.z*vec2.x-vec1.x*vec2.z,
    vec1.x*vec2.y-vec1.y*vec2.x};
    return vector;
}
parser::Vec3f inline vecScalar(parser::Vec3f vec1,  float t){
    parser::Vec3f vector{
        vec1.x*t,
        vec1.y*t,
        vec1.z*t
    };
    return vector;
}
float inline len( parser::Vec3f vec1){
    return sqrt(vec1.x*vec1.x+vec1.y*vec1.y+vec1.z*vec1.z);
 };
parser::Vec3f inline normalize( parser::Vec3f vec1){
    float length = parser::len(vec1);
    if(length>0){return Vec3f{vec1.x/length,vec1.y/length,vec1.z/length};}
    else return vec1;
     
}
parser::Vec3f inline vectorPlusVector(parser::Vec3f vec1, parser::Vec3f vec2){
    return parser::Vec3f{vec1.x+vec2.x,vec1.y+vec2.y,vec1.z+vec2.z};
}
parser::Vec3f inline vectorMinusVector(parser::Vec3f vec1,  parser::Vec3f vec2){
    return parser::Vec3f{vec1.x-vec2.x,vec1.y-vec2.y,vec1.z-vec2.z};
}
parser::Vec3f inline componentwiseProduct(parser::Vec3f vec1,  parser::Vec3f vec2){
    return parser::Vec3f{vec1.x*vec2.x,vec1.y*vec2.y,vec1.z*vec2.z};
}
parser::Vec3i inline clamp(parser::Vec3f vec1){
    parser::Vec3i veci;
    veci.x = vec1.x>255?255:(int)vec1.x;
    veci.y = vec1.y>255?255:(int)vec1.y;
    veci.z = vec1.z>255?255:(int)vec1.z;
    return veci;
}
parser::Vec3f inline negate(Vec3f vec1){
    return Vec3f{-vec1.x,-vec1.y,-vec1.z};
}

}

#endif
