//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const    //calculate the ray's Intersection with the scene 
{
    return this->bvh->Intersect(ray);    //scene->BVH->Intersection
}

void Scene::sampleLight(Intersection &pos, float &pdf) const    //Uniformly sample the light
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
Vector3f Scene::castRay(const Ray &ray, int depth) const    //scene.castRay(Ray(eye_pos,dir),0)/spp
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir,L_indir;   //Initialization

    //judge whether the ray hit
    Intersection obj_inter = intersect(ray);  
    if(!obj_inter.happened) return L_dir;

    //hit the light
    if(obj_inter.m->hasEmission()) return obj_inter.m->getEmission();  

    //obj_inter's Attributes
    Vector3f N = obj_inter.normal.normalized();
    Vector3f p = obj_inter.coords;
    Material* m = obj_inter.m;
    Vector3f wo = ray.direction;    //pixel to obj_inter

    //Uniformly simple the light source and calculate the pdf of sample
    float pdf_L = 1.0f;
    Intersection light_inter;
    sampleLight(light_inter,pdf_L);

    //light_inter's Attributes
    Vector3f x = light_inter.coords;
    Vector3f ws = (x-p).normalized();    //objTolight
    Vector3f NN = light_inter.normal.normalized();   
    Vector3f emit = light_inter.emit;    //direction
    float d = (x-p).norm();   //distance

    //judge whether the ray is blocked in the middle and calculate L_dir
    Ray objTo_light(p,ws);   //shoot a ray from object to light
    float d2 = intersect(objTo_light).distance;
    if(d2-d > -0.001)   //-EPSILON
    {
        Vector3f dir_eval = m->eval(wo,ws,N);
        L_dir = emit * dir_eval * dotProduct(N,ws) * dotProduct(NN,-ws) / std::pow(d,2) / pdf_L ;
    }

    //Test RR and calculate L_indir
    float RR_P = get_random_float();    //const float operate const float
    if(RR_P < RussianRoulette)      
    {
        Vector3f wi = m->sample(wo,N).normalized();   //from object to other object
        Ray objTo_otherobj(p,wi);   //shoot a ray from object to other object
        Intersection otherobj_inter = intersect(objTo_otherobj);     //Intersection
        if(otherobj_inter.happened && !otherobj_inter.m->hasEmission())   //hit and the other object doesn't have emission
        {
            Vector3f indir_eval = m->eval(wo,wi,N);
            float pdf_otherobj = m->pdf(wo,wi,N);
            L_indir = castRay(objTo_otherobj,depth+1) * indir_eval * dotProduct(wi,N) / pdf_otherobj / RussianRoulette ;    //p
        }
    }
    return L_dir + L_indir;
}