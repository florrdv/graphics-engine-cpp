#include <list>
#include "Color.h"
#include "../lib/vector3d/vector3d.h"

class Light {
public:
    Color ambientLight;
    Color diffuseLight;
    Color specularLight;

    Light(Color a, Color d, Color s): ambientLight(a), diffuseLight(d), specularLight(s) {};
    virtual ~Light() = default;
};

class InfLight : public Light {
public:
    Vector3D ldVector;

    InfLight(Color a, Color d, Color s, Vector3D direction): Light(a, d, s), ldVector(direction) {};
};

class PointLight : public Light {
public:
    Vector3D location;
    double spotAngle;

    PointLight(Color a, Color d, Color s, Vector3D l): Light(a, d, s), location(l), spotAngle(0) {};
};

typedef std::list<Light*> Lights3D;