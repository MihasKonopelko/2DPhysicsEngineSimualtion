#ifndef AVATARPARTICLECOLFILTER_H
#define AVATARPARTICLECOLFILTER_H

#include <Box2D.h>


class CollisionFilter : public b2ContactFilter
{
    public:
        CollisionFilter();
        virtual ~CollisionFilter();
        bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) override;

    protected:
    private:
};

#endif // AVATARPARTICLECOLFILTER_H
