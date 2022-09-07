//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}



Vector3f Scene::shade(const Intersection& inter, Vector3f wo) const
{
    if (inter.m->hasEmission())
    {
        return inter.emit;
    }

    Vector3f dir{0.0,0.0,0.0},indir{0.0,0.0,0.0};
    Vector3f p = inter.coords;
    Material *m = inter.m;
    Vector3f N = inter.normal; // normal

    switch (m->getType()) {
        case DIFFUSE:
        {
            Intersection light_inter;
            float pdf_light;
            sampleLight(light_inter,pdf_light);
            Vector3f ws = (light_inter.coords - p).normalized();
            Intersection light = intersect(Ray(p,ws));
            // if light and light_inter is the same obj
            // there is no other between point and light
            // == operator is implented by myself in Vector.hpp
            if(light.coords == light_inter.coords){
                float dis2 = light.distance * light.distance;
                Vector3f NN = light.normal;

                dir = light.emit * m->eval(wo,ws,N)
                * std::max(0.f,dotProduct(N,ws))
                * std::max(0.f,dotProduct(-ws,NN))
                /dis2/pdf_light;
            }
            
            float my_RussianRoulette = get_random_float();
            if(my_RussianRoulette < RussianRoulette){
                Vector3f wi = m->sample(wo,N);
                float pdf = m->pdf(wo,wi,N);
                Intersection obj = intersect(Ray(p,wi));
                if(pdf > EPSILON && obj.happened && !obj.m->hasEmission()){
                    indir += shade(obj,-wi) * m->eval(wo,wi,N)
                            * std::max(0.f,dotProduct(N,wi)) 
                            / pdf / RussianRoulette;;
                }
            }
        }
    }
    return dir + indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // there is no use for depth for path tracing
    // the ray will be stoped by RussianRoulette instead of depth

    Intersection inter = intersect(ray);
    Vector3f hitColor{0};
    if(inter.happened) {
        hitColor = shade(inter,-ray.direction);
    }
    return hitColor;
}