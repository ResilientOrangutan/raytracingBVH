#include "parser.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <algorithm>


using namespace parser;

BVHNode* buildBVHRecursive(std::vector<int>& indices, int start, int end, const std::vector<Primitive>& primitives);
void computeBoundingBox(const std::vector<Primitive>& primitives, int start, int end, Vec3f& bbox_min, Vec3f& bbox_max);
float computeSurfaceArea(const Vec3f& min, const Vec3f& max);

void parser::Scene::loadFromXml(const std::string &filepath)
{
    tinyxml2::XMLDocument file;
    std::stringstream stream;

    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }

    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }

    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0 0 0" << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;

    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> shadow_ray_epsilon;

    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0" << std::endl;
    }
    stream >> max_recursion_depth;

    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        auto child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Gaze");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Up");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearPlane");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearDistance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageResolution");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageName");
        stream << child->GetText() << std::endl;

        stream >> camera.position.x >> camera.position.y >> camera.position.z;
        stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
        stream >> camera.up.x >> camera.up.y >> camera.up.z;
        stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
        stream >> camera.near_distance;
        stream >> camera.image_width >> camera.image_height;
        stream >> camera.image_name;

        cameras.push_back(camera);
        element = element->NextSiblingElement("Camera");
    }

    //Get Lights
    element = root->FirstChildElement("Lights");
    auto child = element->FirstChildElement("AmbientLight");
    stream << child->GetText() << std::endl;
    stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    element = element->FirstChildElement("PointLight");
    PointLight point_light;
    while (element)
    {
        child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Intensity");
        stream << child->GetText() << std::endl;

        stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
        stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

        point_lights.push_back(point_light);
        element = element->NextSiblingElement("PointLight");
    }

    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {
        material.is_mirror = (element->Attribute("type", "mirror") != NULL);

        child = element->FirstChildElement("AmbientReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("DiffuseReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("SpecularReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("MirrorReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("PhongExponent");
        stream << child->GetText() << std::endl;

        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        stream >> material.specular.x >> material.specular.y >> material.specular.z;
        stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        stream >> material.phong_exponent;
        
      

        materials.push_back(material);
        element = element->NextSiblingElement("Material");
    }

    //Get VertexData
    element = root->FirstChildElement("VertexData");
    stream << element->GetText() << std::endl;
    parser::Vec3f vertex;
    while (!(stream >> vertex.x).eof())
    {
        stream >> vertex.y >> vertex.z;
        vertex_data.push_back(vertex);
    }
    stream.clear();

    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    while (element)
    {
        Mesh mesh;
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> mesh.material_id;

        child = element->FirstChildElement("Faces");
        stream << child->GetText() << std::endl;
        Face face;
        while (!(stream >> face.v0_id).eof())
        {
            stream >> face.v1_id >> face.v2_id;
            face.normal = normalize(cross(
                vectorMinusVector(vertex_data[face.v1_id - 1], vertex_data[face.v0_id - 1]),
                vectorMinusVector(vertex_data[face.v2_id - 1], vertex_data[face.v0_id - 1])
            ));
            mesh.faces.push_back(face);

            // Create a Primitive for this face
            Primitive prim;
            prim.type = TRIANGLE;
            prim.material_id = mesh.material_id;
            prim.v0 = vertex_data[face.v0_id - 1];
            prim.v1 = vertex_data[face.v1_id - 1];
            prim.v2 = vertex_data[face.v2_id - 1];
            prim.normal = face.normal;

            // Compute bounding box
            prim.bbox_min = Vec3f{
                std::min({prim.v0.x, prim.v1.x, prim.v2.x}),
                std::min({prim.v0.y, prim.v1.y, prim.v2.y}),
                std::min({prim.v0.z, prim.v1.z, prim.v2.z})
            };
            prim.bbox_max = Vec3f{
                std::max({prim.v0.x, prim.v1.x, prim.v2.x}),
                std::max({prim.v0.y, prim.v1.y, prim.v2.y}),
                std::max({prim.v0.z, prim.v1.z, prim.v2.z})
            };

            primitives.push_back(prim);
        }
        stream.clear();

        meshes.push_back(mesh);
        element = element->NextSiblingElement("Mesh");
    }

    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    while (element)
    {
        Primitive prim;
        prim.type = TRIANGLE;
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> triangle.material_id;

        child = element->FirstChildElement("Indices");
        stream << child->GetText() << std::endl;
        stream >> triangle.faces.v0_id >> triangle.faces.v1_id >> triangle.faces.v2_id;
        triangle.faces.normal = normalize(cross(vectorMinusVector(vertex_data[triangle.faces.v1_id-1],vertex_data[triangle.faces.v0_id-1]),
                                                              vectorMinusVector(vertex_data[triangle.faces.v2_id-1],vertex_data[triangle.faces.v0_id-1])));
        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");

        prim.material_id = triangle.material_id;
        prim.v0 = vertex_data[triangle.faces.v0_id - 1];
        prim.v1 = vertex_data[triangle.faces.v1_id - 1];
        prim.v2 = vertex_data[triangle.faces.v2_id - 1];
        prim.normal = triangle.faces.normal;

        // Compute bounding box
        prim.bbox_min = Vec3f{
            std::min({prim.v0.x, prim.v1.x, prim.v2.x}),
            std::min({prim.v0.y, prim.v1.y, prim.v2.y}),
            std::min({prim.v0.z, prim.v1.z, prim.v2.z})
        };
        prim.bbox_max = Vec3f{
            std::max({prim.v0.x, prim.v1.x, prim.v2.x}),
            std::max({prim.v0.y, prim.v1.y, prim.v2.y}),
            std::max({prim.v0.z, prim.v1.z, prim.v2.z})
        };

        primitives.push_back(prim);
    }

    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        Primitive prim;
        prim.type = SPHERE;

        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> sphere.material_id;

        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> sphere.center_vertex_id;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> sphere.radius;

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");

        prim.material_id = sphere.material_id;
        prim.center = vertex_data[sphere.center_vertex_id - 1];
        prim.radius = sphere.radius;

        prim.bbox_min = Vec3f{
            prim.center.x - prim.radius,
            prim.center.y - prim.radius,
            prim.center.z - prim.radius
        };
        prim.bbox_max = Vec3f{
            prim.center.x + prim.radius,
            prim.center.y + prim.radius,
            prim.center.z + prim.radius
        };

        primitives.push_back(prim);
    }
    
}

void parser::Scene::buildBVH() {
    // Initialize the primitive_indices array
    primitive_indices.resize(primitives.size());
    for (int i = 0; i < primitives.size(); ++i) {
        primitive_indices[i] = i;
    }

    bvh_root = buildBVHRecursive(primitive_indices, 0, primitive_indices.size(), primitives);
}

BVHNode* buildBVHRecursive(std::vector<int>& indices, int start, int end, const std::vector<Primitive>& primitives) {
    BVHNode* node = new BVHNode();

    // Compute the bounding box for the primitives in [start, end)
    Vec3f bbox_min = Vec3f{ INFINITY, INFINITY, INFINITY };
    Vec3f bbox_max = Vec3f{ -INFINITY, -INFINITY, -INFINITY };

    for (int i = start; i < end; ++i) {
        const Primitive& prim = primitives[indices[i]];
        bbox_min.x = std::min(bbox_min.x, prim.bbox_min.x);
        bbox_min.y = std::min(bbox_min.y, prim.bbox_min.y);
        bbox_min.z = std::min(bbox_min.z, prim.bbox_min.z);

        bbox_max.x = std::max(bbox_max.x, prim.bbox_max.x);
        bbox_max.y = std::max(bbox_max.y, prim.bbox_max.y);
        bbox_max.z = std::max(bbox_max.z, prim.bbox_max.z);
    }

    node->bbox_min = bbox_min;
    node->bbox_max = bbox_max;

    int num_primitives = end - start;

    if (num_primitives <= 4) { // Leaf node
        for (int i = start; i < end; ++i) {
            node->primitive_indices.push_back(indices[i]);
        }
        node->left = node->right = nullptr;
    } else {
        // Compute the extent
        float extent[3] = {
            bbox_max.x - bbox_min.x,
            bbox_max.y - bbox_min.y,
            bbox_max.z - bbox_min.z
        };

        // Choose the axis to split
        int axis = 0;
        if (extent[1] > extent[0]) axis = 1;
        if (extent[2] > extent[axis]) axis = 2;

        // Partition indices along the chosen axis
        int mid = start + num_primitives / 2;

        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [&](int a_idx, int b_idx) {
                const Primitive& a = primitives[a_idx];
                const Primitive& b = primitives[b_idx];
                float a_centroid = (a.bbox_min[axis] + a.bbox_max[axis]) * 0.5f;
                float b_centroid = (b.bbox_min[axis] + b.bbox_max[axis]) * 0.5f;
                return a_centroid < b_centroid;
            });

        node->left = buildBVHRecursive(indices, start, mid, primitives);
        node->right = buildBVHRecursive(indices, mid, end, primitives);
    }

    return node;
}


