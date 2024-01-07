#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    // root = recursiveBuild(primitives);
    root = recursiveBuild_SAH(primitives);
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
    // Create leaf _BVHBuildNode_
    node->bounds = objects[0]->getBounds();
    node->object = objects[0];
    node->left = nullptr;
    node->right = nullptr;
    return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
            });
            break;
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild_SAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
    // Create leaf _BVHBuildNode_
    node->bounds = objects[0]->getBounds();
    node->object = objects[0];
    node->left = nullptr;
    node->right = nullptr;
    return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild_SAH(std::vector{objects[0]});
        node->right = recursiveBuild_SAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
            });
            break;
        }

        float boundsArea = bounds.SurfaceArea();
        const int bucketNum = 10;
        Bucket bucket[bucketNum];
        for (int i = 0; i < objects.size(); ++i)
        {
            int b;
            switch (dim)
            {
            case 0: b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).x * bucketNum;
                break;
            case 1: b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).y * bucketNum;
                break;
            case 2: b = centroidBounds.Offset(objects[i]->getBounds().Centroid()).z * bucketNum;
                break;
            }
            if (b == bucketNum)
                b = bucketNum - 1;
            bucket[b].bounds = Union(bucket[b].bounds, objects[i]->getBounds());
            bucket[b].cnt++;
        }

        float costRes = std::numeric_limits<double>::infinity();
        int partitionInd = 0;
        int offset = 0;
        for (int i = 0; i < bucketNum - 1; ++i)
        {
            Bounds3 boundsLeft, boundsRight;
            int cntLeft = 0, cntRight = 0;
            for (int j = 0; j < i + 1; ++j)
            {
                cntLeft += bucket[j].cnt;
                boundsLeft = Union(boundsLeft, bucket[j].bounds);
            }
            for (int j = i + 1; j < bucketNum; ++j)
            {
                cntRight += bucket[j].cnt;
                boundsRight = Union(boundsRight, bucket[j].bounds);
            }
            float cost = cntLeft * boundsLeft.SurfaceArea() / boundsArea + cntRight * boundsRight.SurfaceArea() / boundsArea + 0.125f;
            if (cost < costRes)
            {
                partitionInd = i;
                costRes = cost;
                offset = cntLeft;
            }
        }

        auto beginning = objects.begin();
        auto midding = objects.begin() + offset;
        auto ending = objects.end();
        auto leftshapes = std::vector<Object*>(beginning, midding);
        auto rightshapes = std::vector<Object*>(midding, ending);

        node->left = recursiveBuild_SAH(leftshapes);
        node->right = recursiveBuild_SAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if (!node->bounds.IntersectP(ray, ray.direction_inv, std::array<int, 3>({ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0})))
        return Intersection();

    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);

    Intersection hitLeft = BVHAccel::getIntersection(node->left, ray);
    Intersection hitRight = BVHAccel::getIntersection(node->right, ray);

    return hitLeft.distance < hitRight.distance ? hitLeft : hitRight;

}