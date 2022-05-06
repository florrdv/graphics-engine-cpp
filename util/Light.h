#include "Color.h"
#include "../lib/vector3d/vector3d.h"

class Light {
public:
    Color ambientLight;
    Color diffuseLight;
    Color specularLight;
};

class InfLight : public Light {
public:
    Vector3D ldVector;
};

class PointLight : public Light {
public:
    Vector3D location;
    double spotAngle;
};