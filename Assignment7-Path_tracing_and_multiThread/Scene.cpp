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
    // TO DO Implement Path Tracing Algorithm here

    //find intersection
    Intersection obj_intersection = this->intersect(ray);
    if(!obj_intersection.happened)
        return Vector3f(0.0,0.0,0.0);
    
    //light directly shoot eye
    if(obj_intersection.m->hasEmission()){
        if(0 == depth){
            return obj_intersection.m->getEmission();
        }
        else{
            return Vector3f(0.0,0.0,0.0);
        }
    }

    auto format = [](Vector3f& a){
        if(a.x < 0) a.x = 0.0;
        if(a.y < 0) a.y = 0.0;
        if(a.z < 0) a.z = 0.0;
    };

    //light directly shoot object   
    Vector3f L0  = Vector3f(0.0,0.0,0.0);

    Intersection lightInter;
    float light_pdf;
    this->sampleLight(lightInter,light_pdf);

    Vector3f wi = normalize(-ray.direction);
    Vector3f wo = normalize(lightInter.coords-obj_intersection.coords);
    Vector3f material_n = normalize(obj_intersection.normal);
    Vector3f light_n = normalize(lightInter.normal);

    Intersection light_ray = this->intersect(Ray(obj_intersection.coords,wo));

    if(light_ray.happened && (light_ray.coords-lightInter.coords).norm() < 1e-2){
        L0 = lightInter.emit * obj_intersection.m->eval(wo,wi,material_n) * (dotProduct(material_n,wo)*dotProduct(light_n,-wo)/dotProduct(lightInter.coords-obj_intersection.coords,lightInter.coords-obj_intersection.coords))/light_pdf;
        format(L0);
    }

    //indirectly
    Vector3f L1 = Vector3f(0.0,0.0,0.0);
    float P_RR = this->RussianRoulette;

    if(get_random_float() < P_RR){

        Vector3f thisObject2otherObject_dir = obj_intersection.m->sample(ray.direction,obj_intersection.normal);

        Ray thisObject2otherObject_ray(obj_intersection.coords,thisObject2otherObject_dir);

        Intersection otherObject = this->intersect(thisObject2otherObject_ray);


        if(otherObject.happened && !otherObject.m->hasEmission()){
            L1 = this->castRay(thisObject2otherObject_ray,depth+1) * obj_intersection.m->eval(-ray.direction,thisObject2otherObject_dir,obj_intersection.normal) * dotProduct(obj_intersection.normal,thisObject2otherObject_dir)/obj_intersection.m->pdf(-ray.direction,thisObject2otherObject_dir,obj_intersection.normal)/P_RR;
            format(L1);
        }
    }

    return L0 + L1;       
}

// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {
// 	Intersection inter = intersect(ray);

// 	if (inter.happened)
// 	{
// 		// 如果射线第一次打到光源，直接返回
// 		if (inter.m->hasEmission())
// 		{
// 			if (depth == 0) 
// 			{
// 				return inter.m->getEmission();
// 			}
// 			else return Vector3f(0, 0, 0);
// 		}

// 		Vector3f L_dir(0, 0, 0);
// 		Vector3f L_indir(0, 0, 0);

// 		// 随机 sample 灯光，用该 sample 的结果判断射线是否击中光源
// 		Intersection lightInter;
// 		float pdf_light = 0.0f;
// 		sampleLight(lightInter, pdf_light);

// 		// 物体表面法线
// 		auto& N = inter.normal;
// 		// 灯光表面法线
// 		auto& NN = lightInter.normal;

// 		auto& objPos = inter.coords;
// 		auto& lightPos = lightInter.coords;

// 		auto diff = lightPos - objPos;
// 		auto lightDir = diff.normalized();
// 		float lightDistance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

// 		Ray light(objPos, lightDir);
// 		Intersection light2obj = intersect(light);

// 		// 如果反射击中光源
// 		if (light2obj.happened && (light2obj.coords - lightPos).norm() < 1e-2)
// 		{
// 			Vector3f f_r = inter.m->eval(ray.direction, lightDir, N);
// 			L_dir = lightInter.emit * f_r * dotProduct(lightDir, N) * dotProduct(-lightDir, NN) / lightDistance / pdf_light;
// 		}

// 		if (get_random_float() < RussianRoulette)
// 		{
// 			Vector3f nextDir = inter.m->sample(ray.direction, N).normalized();

// 			Ray nextRay(objPos, nextDir);
// 			Intersection nextInter = intersect(nextRay);
// 			if (nextInter.happened && !nextInter.m->hasEmission())
// 			{
// 				float pdf = inter.m->pdf(ray.direction, nextDir, N);
// 				Vector3f f_r = inter.m->eval(ray.direction, nextDir, N);
// 				L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextDir, N) / pdf / RussianRoulette;
// 			}
// 		}

// 		return L_dir + L_indir;
// 	}

// 	return Vector3f(0, 0, 0);
// }
