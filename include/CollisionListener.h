#ifndef COLLISIONLISTENER_H
#define COLLISIONLISTENER_H

#include "Box2D.h"
#include "Globals.h"

class CollisionListener: public b2ContactListener
{
public:
	CollisionListener();

	static CollisionListener* m_instance;
	static CollisionListener* CreateListener();

	void BeginContact(b2Contact* contact) override;
	void EndContact(b2Contact* contact) override;

// Non inherited methods


private:

};

#endif // COLLISIONLISTENER_H
