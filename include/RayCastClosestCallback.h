#ifndef RAYCASTCLOSESTCALLBACK_H
#define RAYCASTCLOSESTCALLBACK_H

#include "Box2d.h"

class RayCastClosestCallback : public b2RayCastCallback
{
    public:
    b2Body* m_body;
    b2Vec2 m_point;

    RayCastClosestCallback() { m_body = NULL; }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
    {
        m_body = fixture->GetBody();
        m_point = point;
        return fraction;
    }
};

#endif // RAYCASTCLOSESTCALLBACK_H
