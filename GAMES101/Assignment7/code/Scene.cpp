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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection p = intersect(ray);
    if (!p.happened)
        return Vector3f(0.0);
    
    if (p.m->hasEmission())
        return p.m->getEmission(); 
    
    Vector3f L_dir(0.0, 0.0, 0.0);
    Vector3f L_indir(0.0, 0.0, 0.0);

    Intersection x;
    float pdf_light = 0.0;
    sampleLight(x, pdf_light);

    Vector3f vec_pTox = x.coords - p.coords;
    Vector3f ws = vec_pTox.normalized();
    float dist_pTox2 = dotProduct(vec_pTox, vec_pTox);
    Vector3f emit = x.m->getEmission();
    Vector3f N = p.normal;
    Vector3f NN = x.normal;
    Vector3f wo = ray.direction;
    Ray ray_pTox(p.coords, ws);
    Intersection interRay_pTox = intersect(ray_pTox);
    
    if (interRay_pTox.distance  + 0.01 > vec_pTox.norm())
    {
        L_dir = emit * p.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dist_pTox2 / pdf_light;
    }
    
    if (get_random_float() <= RussianRoulette)
    {
        Vector3f wi = p.m->sample(wo, N);
        Ray rayWi(p.coords, wi);
        Intersection q =intersect(rayWi);
        if (q.happened && !q.m->hasEmission())
            L_indir = castRay(rayWi, depth + 1) * p.m->eval(wo, wi, N) * dotProduct(wi, N) / p.m->pdf(wo, wi, N) / RussianRoulette;
    }
    return L_dir + L_indir;

}