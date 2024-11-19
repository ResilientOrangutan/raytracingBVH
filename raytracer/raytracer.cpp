#include <iostream>
#include <thread>
#include <mutex>

#include "parser.h"
#include "ppm.h"


using namespace parser;


typedef unsigned char RGB[3];

parser::Vec3f computeColor(const Ray& r, const Scene& scene);
parser::Vec3f applyShading(const Ray& r, const HitRecord& hitRecord, const Scene& scene);
bool closestHit(Ray r, Scene &scene, HitRecord &h);
float inline discriminant(Vec3f v1,Vec3f v2,Vec3f v3);
bool isThereHit(const Ray& r, const Scene& scene);
bool intersectPrimitive(const Ray& ray, const Primitive& prim, const Scene& scene, float& t, Vec3f& normal);
bool rayIntersectsAABB(const Ray& ray, const Vec3f& min, const Vec3f& max, float& tmin, float& tmax) ;


int main(int argc, char* argv[])
{   
    // Sample usage for reading an XML scene file
    

    parser::Scene scene;
    scene.loadFromXml(argv[1]);
    scene.buildBVH();
    

    

    for(const auto& currentCamera : scene.cameras){
        Vec3f cameraGaze = currentCamera.gaze;
        Vec3f cameraPosition = currentCamera.position;
        Vec3f cameraUp = currentCamera.up;
        Vec3f cameraW = parser::negate(cameraGaze);
        Vec3f cameraU = normalize(cross(cameraUp, cameraW));
        Vec3f cameraV = cross(cameraW, cameraU);

        float planeL = currentCamera.near_plane.x;
        float planeR = currentCamera.near_plane.y;
        float planeB = currentCamera.near_plane.z;
        float planeT = currentCamera.near_plane.w;
        float planeDistance = currentCamera.near_distance;

        int width = currentCamera.image_width;
        int height = currentCamera.image_height;

        Vec3f m = parser::vectorPlusVector(cameraPosition, parser::vecScalar(cameraGaze, planeDistance));
        Vec3f lu = parser::vecScalar(cameraU, planeL);
        Vec3f tv = parser::vecScalar(cameraV, planeT);

        unsigned char* image = new unsigned char[width * height * 3];
        int i = 0;

        float pixelWidth = (planeR - planeL) / width;
        float pixelHeight = (planeT - planeB) / height;
        Vec3f q = parser::vectorPlusVector(m, parser::vectorPlusVector(lu, tv));

                // Determine the number of hardware threads available
        unsigned int numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) numThreads = 4; // Default to 4 if hardware_concurrency returns 0

        std::vector<std::thread> threads;
        std::mutex mutex;

        // Function to render a portion of the image
        auto renderSection = [&](int startY, int endY) {
            for (int y = startY; y < endY; ++y) {
                float sy = (y + 0.5f) * pixelHeight;
                Vec3f s_v = parser::vecScalar(cameraV, sy);
                for (int x = 0; x < width; ++x) {
                    float sx = (x + 0.5f) * pixelWidth;
                    Vec3f s_u = parser::vecScalar(cameraU, sx);

                    Vec3f s = parser::vectorPlusVector(q, parser::vectorMinusVector(s_u, s_v));
                    Vec3f viewDir = normalize(parser::vectorMinusVector(s, cameraPosition));
                    Ray viewRay{ viewDir, cameraPosition, 0 };

                    Vec3i currentPixelColor = parser::clamp(computeColor(viewRay, scene));

                    int index = (y * width + x) * 3;
                    // No need for mutex here because each thread writes to a unique part of the image array
                    image[index] = currentPixelColor.x;
                    image[index + 1] = currentPixelColor.y;
                    image[index + 2] = currentPixelColor.z;
                }
            }
        };

        // Split the image into sections for each thread
        int rowsPerThread = height / numThreads;
        int remainingRows = height % numThreads;
        int currentY = 0;

        for (unsigned int i = 0; i < numThreads; ++i) {
            int startY = currentY;
            int endY = startY + rowsPerThread;
            if (i == numThreads - 1) {
                endY += remainingRows; // Add remaining rows to the last thread
            }
            threads.emplace_back(renderSection, startY, endY);
            currentY = endY;
        }

        // Wait for all threads to finish
        for (auto& thread : threads) {
            thread.join();
        }

        write_ppm(currentCamera.image_name.c_str(), image, width, height);
        delete[] image;
    }
    
    return 0;
    }

float inline discriminant(Vec3f v1,Vec3f v2,Vec3f v3){
    return v1.x*(v2.y*v3.z-v3.y*v2.z) + 
           v1.y*(v3.x*v2.z-v2.x*v3.z)+ 
           v1.z*(v2.x*v3.y-v2.y*v3.x);
}


bool closestHit(const Ray& ray, const Scene& scene, HitRecord& h) {
    bool hit = false;
    float tmin = INFINITY;

    std::vector<const BVHNode*> stack;
    stack.push_back(scene.bvh_root);

    while (!stack.empty()) {
        const BVHNode* node = stack.back();
        stack.pop_back();

        float tmin_node, tmax_node;
        if (rayIntersectsAABB(ray, node->bbox_min, node->bbox_max, tmin_node, tmax_node)) {
            if (!node->left && !node->right) { // Leaf node
                for (int idx : node->primitive_indices) {
                    const Primitive& prim = scene.primitives[idx];
                    float t = tmin;
                    Vec3f normal;
                    if (intersectPrimitive(ray, prim, scene, t, normal)) {
                        if (t < tmin) {
                            tmin = t;
                            h.intersection_p = vectorPlusVector(ray.origin, vecScalar(ray.dir, t));
                            h.normal = normal;
                            h.m = prim.material_id;
                            hit = true;
                        }
                    }
                }
            } else {
                if (node->left) stack.push_back(node->left);
                if (node->right) stack.push_back(node->right);
            }
        }
    }
    return hit;
}

bool isThereHit(const Ray& ray, const Scene& scene, float maxDistance) {
    float tmin = scene.shadow_ray_epsilon;
    float tmax = maxDistance;

    std::vector<const BVHNode*> stack;
    stack.push_back(scene.bvh_root);

    while (!stack.empty()) {
        const BVHNode* node = stack.back();
        stack.pop_back();

        float tmin_node, tmax_node;
        if (rayIntersectsAABB(ray, node->bbox_min, node->bbox_max, tmin_node, tmax_node)) {
            if (!node->left && !node->right) { // Leaf node
                for (int idx : node->primitive_indices) {
                    const Primitive& prim = scene.primitives[idx];
                    float t = tmax;
                    Vec3f normal;
                    if (intersectPrimitive(ray, prim, scene, t, normal)) {
                        if (t < tmax) {
                            return true; // Intersection within maxDistance
                        }
                    }
                }
            } else {
                if (node->left) stack.push_back(node->left);
                if (node->right) stack.push_back(node->right);
            }
        }
    }
    return false;
}

parser::Vec3f computeColor(const Ray& r, const Scene& scene) {
    if (r.depth > scene.max_recursion_depth) {
        return Vec3f{ 0.0f, 0.0f, 0.0f };
    }
    HitRecord h;
    if (closestHit(r, scene, h)) {
        return applyShading(r, h, scene);
    } 
    else if(r.depth==0) {
        // Set background color for missed rays
        return Vec3f{
            static_cast<float>(scene.background_color.x),
            static_cast<float>(scene.background_color.y),
            static_cast<float>(scene.background_color.z)
        };
    }
    return Vec3f{ 0.0f, 0.0f, 0.0f };
}

parser::Vec3f applyShading(const Ray& r, const HitRecord& hitRecord, const Scene& scene){
    const Material& material = scene.materials[hitRecord.m - 1];
    Vec3f hitPoint = hitRecord.intersection_p;
    Vec3f normal = hitRecord.normal;
    Vec3f viewDir = parser::negate(r.dir);

    Vec3f color = componentwiseProduct(material.ambient, scene.ambient_light);

    if (material.is_mirror && r.depth < scene.max_recursion_depth) {
        Vec3f reflectedDir = normalize(parser::vectorMinusVector(r.dir, parser::vecScalar(normal, 2 * dot(r.dir, normal))));
        Vec3f reflectedOrigin = parser::vectorPlusVector(hitPoint, parser::vecScalar(normal, scene.shadow_ray_epsilon));
        Ray reflectedRay{ reflectedDir, reflectedOrigin, r.depth + 1 };
        Vec3f reflectedColor = componentwiseProduct(material.mirror, computeColor(reflectedRay, scene));
        color = parser::vectorPlusVector(color, reflectedColor);
    }

     for (const auto& light : scene.point_lights) {
        Vec3f lightDir = parser::vectorMinusVector(light.position, hitPoint);
        float distanceSquared = parser::dot(lightDir, lightDir);
         float distanceToLight = sqrt(distanceSquared);
         lightDir = vecScalar(lightDir, 1.0f / distanceToLight);

        // Shadow ray
        Ray shadowRay{ lightDir, parser::vectorPlusVector(hitPoint, parser::vecScalar(normal, scene.shadow_ray_epsilon)), 0 };

		if (!isThereHit(shadowRay, scene,  distanceToLight)) {
            // Diffuse component
            float cosTheta = dot(normal, lightDir);
            if (cosTheta > 0.0f) {
                Vec3f diffuse = parser::vecScalar(componentwiseProduct(material.diffuse, light.intensity), cosTheta / distanceSquared);
                color = parser::vectorPlusVector(color, diffuse);
            }

            // Specular component
            Vec3f halfVector = normalize(parser::vectorPlusVector(lightDir, viewDir));
            float cosAlpha = dot(normal, halfVector);
            if (cosAlpha > 0.0f) {
                float spec = pow(cosAlpha, material.phong_exponent);
                Vec3f specular = parser::vecScalar(componentwiseProduct(material.specular, light.intensity), spec / distanceSquared);
                color = parser::vectorPlusVector(color, specular);
            }
        }
    }

    return color;
}



bool rayIntersectsAABB(const Ray& ray, const Vec3f& min, const Vec3f& max, float& tmin, float& tmax) {
    tmin = -INFINITY;
    tmax = INFINITY;

    for (int i = 0; i < 3; ++i) {
        if (ray.dir[i] != 0.0f) {
            float invD = 1.0f / ray.dir[i];
            float t0 = (min[i] - ray.origin[i]) * invD;
            float t1 = (max[i] - ray.origin[i]) * invD;

            if (invD < 0.0f) std::swap(t0, t1);

            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;

            if (tmax < tmin)
                return false;
        } else {
            // Ray is parallel to the slab. No hit if origin not within slab
            if (ray.origin[i] < min[i] || ray.origin[i] > max[i])
                return false;
        }
    }
    return true;
}

bool intersectPrimitive(const Ray& ray, const Primitive& prim, const Scene& scene, float& t, Vec3f& normal) {

    if (prim.type == SPHERE) {
        // Sphere intersection
        Vec3f oC = vectorMinusVector(ray.origin, prim.center);
        float radius = prim.radius;

        float a = dot(ray.dir, ray.dir);
        float b = 2.0f * dot(ray.dir, oC);
        float c = dot(oC, oC) - radius * radius;
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
            return false;

        float sqrtDisc = sqrt(discriminant);
        float inv2a = 0.5f / a;

        float t1 = (-b - sqrtDisc) * inv2a;
        float t2 = (-b + sqrtDisc) * inv2a;

        float shadowRayEpsilon = scene.shadow_ray_epsilon;

        if (t1 > shadowRayEpsilon && t1 < t) {
            t = t1;
            Vec3f hitPoint = vectorPlusVector(ray.origin, vecScalar(ray.dir, t));
            normal = normalize(vectorMinusVector(hitPoint, prim.center));
            return true;
        }
        if (t2 > shadowRayEpsilon && t2 < t) {
            t = t2;
            Vec3f hitPoint = vectorPlusVector(ray.origin, vecScalar(ray.dir, t));
            normal = normalize(vectorMinusVector(hitPoint, prim.center));
            return true;
        }
        return false;
    } else if (prim.type == TRIANGLE) {
        // Triangle intersection
        Vec3f vertex0 = prim.v0;
        Vec3f vertex1 = prim.v1;
        Vec3f vertex2 = prim.v2;

        Vec3f edge1 = vectorMinusVector(vertex1, vertex0);
        Vec3f edge2 = vectorMinusVector(vertex2, vertex0);
        Vec3f pvec = cross(ray.dir, edge2);
        float det = dot(edge1, pvec);
	float backFaceCullingCos = dot(ray.dir, prim.normal);
	
	
        
        
        double EPSILON = 1e-8;
        if (fabs(det) < EPSILON)
            return false;

        float invDet = 1.0f / det;
        Vec3f tvec = vectorMinusVector(ray.origin, vertex0);
        float u = dot(tvec, pvec) * invDet;
        if (u < 0.0f || u > 1.0f)
            return false;

        Vec3f qvec = cross(tvec, edge1);
        float v = dot(ray.dir, qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f)
            return false;

        float t_candidate = dot(edge2, qvec) * invDet;
        float shadowRayEpsilon = scene.shadow_ray_epsilon;
        if (t_candidate > shadowRayEpsilon && t_candidate < t) {
            t = t_candidate;
            normal = normalize(cross(edge1, edge2)); // Compute normal here
            return true;
        }
        return false;
    }
    return false;
}
