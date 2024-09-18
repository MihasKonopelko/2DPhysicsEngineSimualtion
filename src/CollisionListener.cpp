#include <CollisionListener.h>
#include <iostream>
#include <string>
#include <FixtureUserDataContainer.h>

CollisionListener* CollisionListener::m_instance = NULL;

CollisionListener* CollisionListener::CreateListener()
{
	if(m_instance == NULL)
    {
		m_instance = new CollisionListener();
    }
	return m_instance;
}

CollisionListener::CollisionListener()
{

}

void CollisionListener::BeginContact(b2Contact* contact)
{
    b2Fixture* l_fixtureA = contact->GetFixtureA();
    b2Fixture* l_fixtureB = contact->GetFixtureB();

    void* l_objectA;
    void* l_objectB;

    FixtureUserDataContainer* dataA = static_cast<FixtureUserDataContainer*>(l_fixtureA->GetUserData());
    FixtureUserDataContainer* dataB = static_cast<FixtureUserDataContainer*>(l_fixtureB->GetUserData());

    if(dataA != nullptr && dataB != nullptr)
    {


        if(dataA->GetName() == "Exhaust Lock" && dataB->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = true;
        }
        else if(dataB->GetName() == "Exhaust Lock" && dataA->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = true;
        }



        if(dataA->GetName() == "Combustion Chamber Lock" && dataB->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = true;
        }

        else if(dataB->GetName() == "Combustion Chamber Lock" && dataA->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = true;
        }



        if(dataA->GetName() == "Ignition Trigger" && dataB->GetName() == "Piston")
        {

            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = true;
        }
        else if(dataB->GetName() == "Ignition Trigger" && dataA->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = true;
        }
    }
}

void CollisionListener::EndContact(b2Contact* contact)
{
    b2Fixture* l_fixtureA = contact->GetFixtureA();
    b2Fixture* l_fixtureB = contact->GetFixtureB();

    void* l_objectA;
    void* l_objectB;

    FixtureUserDataContainer* dataA = static_cast<FixtureUserDataContainer*>(l_fixtureA->GetUserData());
    FixtureUserDataContainer* dataB = static_cast<FixtureUserDataContainer*>(l_fixtureB->GetUserData());

    if(dataA != nullptr && dataB != nullptr)
    {
        if(dataA->GetName() == "Exhaust Lock" && dataB->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = false;
        }
        else if(dataB->GetName() == "Exhaust Lock" && dataA->GetName() == "Piston")
        {
            SensorArea* area = static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = false;
        }



        if(dataA->GetName() == "Combustion Chamber Lock" && dataB->GetName() == "Piston")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = false;
        }
        else if(dataB->GetName() == "Combustion Chamber Lock" && dataA->GetName() == "Piston")
        {

            SensorArea* area =  static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = false;
        }



        if(dataA->GetName() == "Ignition Trigger" && dataB->GetName() == "Piston Top")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureA->GetBody()->GetUserData());
            area->touched = false;
        }
        else if(dataB->GetName() == "Ignition Trigger" && dataA->GetName() == "Piston Top")
        {
            SensorArea* area =  static_cast<SensorArea*>(l_fixtureB->GetBody()->GetUserData());
            area->touched = false;
        }
    }
}
