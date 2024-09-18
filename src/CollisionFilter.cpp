#include "CollisionFilter.h"
#include "FixtureUserDataContainer.h"
#include <iostream>


CollisionFilter::CollisionFilter()
{
    //ctor
}

CollisionFilter::~CollisionFilter()
{
    //dtor
}

bool CollisionFilter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB)
{
    FixtureUserDataContainer* dataA = static_cast<FixtureUserDataContainer*>(fixtureA->GetUserData());
    FixtureUserDataContainer* dataB = static_cast<FixtureUserDataContainer*>(fixtureB->GetUserData());

    if(dataA != NULL && dataB != NULL)
    {
        if((dataA->GetName() == "ValveBody" && dataB->GetName() == "Ignition Trigger") ||
           (dataB->GetName() == "ValveBody" && dataA->GetName() == "Ignition Trigger"))
        {
            return false;
        }

        if((dataA->GetName() == "ValveBody" && dataB->GetName() == "Fuel") ||
           (dataB->GetName() == "ValveBody" && dataA->GetName() == "Fuel"))
        {
            return false;
        }

        if((dataA->GetName() == "ValveBody" && dataB->GetName() == "Corpus") ||
           (dataB->GetName() == "ValveBody" && dataA->GetName() == "Corpus"))
        {
            return false;
        }

        if((dataA->GetName() == "ValveCover" && dataB->GetName() == "Corpus") ||
           (dataB->GetName() == "ValveCover" && dataA->GetName() == "Corpus"))
        {
            return true;
        }


        if ((dataA->GetName() == "Fuel" && dataB->GetName() == "Connecting Rod") ||
            (dataB->GetName() == "Fuel" && dataA->GetName() == "Connecting Rod"))
        {
            return false;
        }

        if ((dataA->GetName() == "Fuel" && dataB->GetName() == "Crankshaft") ||
            (dataB->GetName() == "Fuel" && dataA->GetName() == "Crankshaft"))
        {
            return false;
        }
    }

    return true;
}
