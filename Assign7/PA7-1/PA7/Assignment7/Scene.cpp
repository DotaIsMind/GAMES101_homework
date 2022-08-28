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
    // 最大深度渲染为黑色
    if (depth > this->maxDepth) {return Vector3f(0.0, 0.0, 0.0);}

    Vector3f hitColor = this->backgroundColor;

    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened) return Vector3f();
    // ray.direction取反，与课程一致
    hitColor = shade(intersection, -ray.direction);

    return hitColor;

}

Vector3f Scene::shade(Intersection &insect, Vector3f wo) const
{
    // 在main中定义了16spp，
    // TODO:判断光源并返回光源颜色吗？
    // 自发光项目，也就是光源,注视这里光源可能变成黑色？
    if (insect.m->hasEmission()){
        return insect.m->getEmission();
    }
    const float epsilon = 0.0005f;

    // 直接光照渲染
    Vector3f L_dir;
    {
        // 定义采样密度
        float light_pdf;
        // 在光源上根据面积均匀采样
        Intersection hitLight;
        sampleLight(hitLight, light_pdf);
        // 光源到材质的光线方向(归一化处理)
        Vector3f light2Obj = insect.coords - hitLight.coords;
        Vector3f light2ObjDir = light2Obj.normalized();


        // 检查光线是否被遮挡
        auto t = intersect(Ray(insect.coords, -light2ObjDir));
        if ((t.distance - light2Obj.norm()) > -epsilon){
            // 没有遮挡,通过入射方向(摄像机到材质表面），出射方向，法线计算f_r

            Vector3f f_r = insect.m->eval(light2ObjDir, wo, insect.normal);
            // 渲染点到光源的距离
            float r2 = dotProduct(light2Obj, light2Obj);
            // 计算法线和渲染点出射方向的夹角
            float cosA = std::max(0.0f, dotProduct(insect.normal, -light2ObjDir));
            float cosB = std::max(0.0f, dotProduct(hitLight.normal, light2ObjDir));

            L_dir = hitLight.emit * f_r * cosA * cosB / r2 / light_pdf;

        }

    }

    Vector3f L_indir;
    // 间接光照渲染
    {
        // 如果没有达到终止条件，递归求解
        if (get_random_float() < RussianRoulette){
            // 当前着色点到贡献给他间接光照的光源采样
            // 入射方向：材质表面到摄像机的反向
            Vector3f hit2IndirLight = insect.m->sample(-wo, insect.normal);
            Vector3f hit2IndirLightDir = hit2IndirLight.normalized();
            // shoot a ray from object to inderLight
            Ray indirRay(insect.coords, hit2IndirLightDir);
            Intersection nextInsect = intersect(indirRay);
            float pdf = insect.m->pdf(-hit2IndirLightDir, wo, insect.normal);
            if (nextInsect.happened && !(nextInsect.m->hasEmission())) {
                Vector3f f_r = insect.m->eval(-hit2IndirLightDir, wo, insect.normal);
                float cosX = std::max(0.0f, dotProduct(hit2IndirLightDir, insect.normal));
                L_indir = shade(nextInsect, -hit2IndirLightDir) * f_r * cosX / pdf / RussianRoulette;
            }


//            // 采样密度
//            // 入射方向：sample得到的出射方向的反向，出射方向：材质表面到摄像机
//            float pdf = insect.m->pdf(-hit2IndirLightDir, wo,insect.normal);
//            // 如果击中了一个具有反射性质的物体，即采样密度>epsilon,需要递归进行再次求解
//            if (pdf>epsilon){
//                Intersection nextInsect = intersect(Ray(insect.coords, hit2IndirLightDir));
//                // 且该物体不是光源,shade本身具有光源判断，是否还要加入光源判断？
//                if (nextInsect.happened && !nextInsect.m->hasEmission()){
//                    // 计算下一个点的直接光照*该方向上的百分比
//                    // 对间接光源采样
//                    Vector3f f_r = insect.m->eval(-hit2IndirLightDir, wo, insect.normal);
//                    float cosX = std::max(0.0f, dotProduct(hit2IndirLightDir, insect.normal));
//                    L_indir = shade(nextInsect, -hit2IndirLightDir) * f_r * cosX / pdf / RussianRoulette;
//                }
//            }

        }
    }
    return L_dir + L_indir;
//    return L_dir;
}