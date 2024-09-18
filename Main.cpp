#include "Box2D.h"

#include "B2Renderer.h"
#include "CollisionListener.h"
#include "CollisionFilter.h"
#include "FixtureUserDataContainer.h"
#include "Globals.h"
#include "RayCastClosestCallback.h"

#include <vector>
#include <map>
#include <stdlib.h>
#include <time.h>


// ----------------------------------------------------------------------------------------------------
// Methods
void InitVertsList();
void Update(sf::RenderWindow& window, b2World& world);
void Draw(sf::RenderWindow& window, b2World& world);
void WorldStep(b2World& world);


void CreateEngineBodies(b2World& world);
void CreateCorpus(b2World& world);
void CreatePiston(b2World& world);
void CreateCrankshaft(b2World& world);
void CreateConnectRod(b2World& world);
void CreateReedValve(b2World& world);

void CreateJoints(b2World& world);

void CreateAirPressureZones(b2World& world);
void CreateSensors(b2World& world);

void SpawnFuelParticles(b2World& world, b2Vec2 position, int totalParticles);


void PolygonMaker(b2Body* body, float density, b2Vec2 verts[], int size,  FixtureUserDataContainer* data);
void AddPolygonVerts(std::vector<b2Vec2*>& vertVec, std::vector<int>& vertsSizeVec, int size, b2Vec2* verts);
void AddRelativeJoint(b2World& world, b2Body* bodyA, b2Body* bodyB, std::string name, b2Vec2 jointPos, bool collideConnected,  bool testMotor = false);
void AddPrismaticJoint(b2World& world, b2Body* bodyA, b2Body* bodyB,std::string name, b2Vec2 axis, bool collideConnected, bool enableMotor);
void AddAirPressureArea(b2World& world, b2Vec2* verts, int size, std::string name);
void AddSensor(b2World& world, b2Vec2* verts, int size, std::string name);


void ForceUpdate(b2World& world);
void UpdateAirPressureZonesState(b2World& world);
void AffectBodiesInAirPressureZones();
void ParticleRemover(b2World& world);
void EnginePhysics(b2World& world);
void FuelPhysics();

// I-Force's methods (which I modified a bit) from explosion tutorial (mentioned here in case if I will forget to put him in LR)
void ApplyBlastImpulse(b2Body* body, b2Vec2 blastCenter, b2Vec2 applyPoint, float blastPower);
void ExplodeRaycast(b2World& world, b2Body* body);



// ----------------------------------------------------------------------------------------------------
// Variables
std::vector<b2Vec2*> m_corpusVertsVec;
std::vector<b2Vec2*> m_crankshaftVertsVec;
std::vector<b2Vec2*> m_pistonVertsVec;
std::vector<b2Vec2*> m_conRodVertsVec;

std::vector<int> m_corpusVertsSizeVec;
std::vector<int> m_crankshaftVertsSizeVec;
std::vector<int> m_pistonVertsSizeVec;
std::vector<int> m_conRodVertsSizeVec;


std::map<std::string, b2Body*> m_bodies;
std::map<std::string, b2Joint*> m_joints;

std::map <std::string, AirPressureArea*> m_airAreas;
std::map <std::string, SensorArea*> m_sensorAreas;

std::vector<b2Body*> m_fuelParticles;

bool m_engineOn = false;
bool m_engineStarted = false;


// Special Constants

// Marks where Piston separates into Top and Base.
const int c_startIndexOfPistonTop = 7;
const int c_maxParticles = 200;

const int c_conbustionForce = 1000;
const int c_conbustionRadius = 400;
const int c_conbustionSpreadDirs = 32;

int main()
{
    srand (time(NULL));
    // Set Screen.
    sf::RenderWindow l_window(sf::VideoMode(375, 547), "2 Stroke Engine| Press Enter to Start Engine", sf::Style::Close);

    // Set Debug Draw.
    DebugDraw l_debugDraw(l_window);
    l_debugDraw.SetFlags(b2Draw::e_shapeBit);

    // Set World.
    b2Vec2 l_gravity(0, 0);
    b2World l_world(l_gravity);
    l_world.SetContactListener(new CollisionListener());
    l_world.SetContactFilter(new CollisionFilter());
    l_world.SetDebugDraw(&l_debugDraw);


    // Hard-coded variables.
    InitVertsList();

    // Create Engine
    CreateEngineBodies(l_world);
    CreateJoints(l_world);

    // Air Pressure Zones
    CreateAirPressureZones(l_world);

    // Sensors
    CreateSensors(l_world);

    // Some oil leftovers spawn.
    SpawnFuelParticles(l_world, b2Vec2(25, 352), 35);
    SpawnFuelParticles(l_world, b2Vec2(166, 28), 20);
    SpawnFuelParticles(l_world, b2Vec2(204, 384), 30);

    // Game Loop.
    while(l_window.isOpen())
    {
        Update(l_window, l_world);
        Draw(l_window, l_world);
    }

    return 0;
}



void InitVertsList()
{
    // Engine Corpus

    // Upper Engine Corpus.
    {
        /*01*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(2,329), b2Vec2(2, 340), b2Vec2(55, 340), b2Vec2(55, 329)});
        /*02*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(55, 329), b2Vec2(55, 340), b2Vec2(69, 347), b2Vec2(69, 329)});
        /*03*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(69, 347), b2Vec2(75, 357), b2Vec2(75, 347)});
        /*04*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(69, 329), b2Vec2(69, 347), b2Vec2(102, 347), b2Vec2(102, 329)});
        /*05*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(75, 347), b2Vec2(75, 365), b2Vec2(102, 365), b2Vec2(102, 347)});
        /*06*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(75, 365), b2Vec2(75, 403), b2Vec2(93, 387), b2Vec2(102, 365)});
        /*07*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(83, 329), b2Vec2(102, 329), b2Vec2(102, 320), b2Vec2(91, 320)});
        /*08*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(91, 320), b2Vec2(102, 320), b2Vec2(102, 4), b2Vec2(91, 4)});
        /*09*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(102, 66), b2Vec2(102, 179),  b2Vec2(112, 179), b2Vec2(112, 66)});
        /*10*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 6, new b2Vec2[6]{b2Vec2(102, 66), b2Vec2(112, 66), b2Vec2(131, 60), b2Vec2(149, 42), b2Vec2(155, 22), b2Vec2(102, 22)});
        /*11*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(102, 179), b2Vec2(102, 189), b2Vec2(112, 179)});
        /*12*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(102, 4), b2Vec2(102, 22),  b2Vec2(315, 22), b2Vec2(315, 4)});
        /*13*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 6, new b2Vec2[6]{b2Vec2(315, 22), b2Vec2(315, 66), b2Vec2(277, 66), b2Vec2(242, 57), b2Vec2(215, 45), b2Vec2(186, 22)});
        /*14*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(294, 66), b2Vec2(294, 168), b2Vec2(315, 168), b2Vec2(315, 66)});
        /*15*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(315, 156), b2Vec2(315, 168), b2Vec2(368, 168), b2Vec2(368, 156)});
    }

    // Lower Engine Corpus.
    {
        /*16*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(2,363), b2Vec2(2, 374), b2Vec2(46, 374), b2Vec2(46, 363)});
        /*17*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(42, 374), b2Vec2(42, 543),  b2Vec2(53, 543), b2Vec2(53, 374)});
        /*18*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{ b2Vec2(46, 363), b2Vec2(46, 374), b2Vec2(53, 374), b2Vec2(51, 368)});
        /*19*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(34, 374), b2Vec2(42, 374), b2Vec2(42, 385)});
        /*20*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(53, 528), b2Vec2(53, 543), b2Vec2(316, 543), b2Vec2(316, 528)});
        /*21*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(331, 363), b2Vec2(316, 363), b2Vec2(316, 543), b2Vec2(331, 543)});
        /*22*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(294, 347), b2Vec2(315, 347), b2Vec2(315, 213), b2Vec2(294, 213)});
        /*23*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(315, 213), b2Vec2(315, 224), b2Vec2(367, 224), b2Vec2(367, 213)});
        /*24*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(315, 347), b2Vec2(314, 363), b2Vec2(331, 363)});
        /*25*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(295, 347), b2Vec2(315, 400), b2Vec2(315, 347)});
        /*26*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 6, new b2Vec2[6]{b2Vec2(53, 423), b2Vec2(53, 528), b2Vec2(93, 528), b2Vec2(93, 445), b2Vec2(75, 428), b2Vec2(63, 423)});
        /*27*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(312, 445), b2Vec2(312, 528),b2Vec2(316, 528), b2Vec2(316, 426)});
        /*28*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(93, 445), b2Vec2(93, 528), b2Vec2(116, 528), b2Vec2(116, 492)});
        /*29*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(116, 492), b2Vec2(116, 528), b2Vec2(145, 528), b2Vec2(145, 517)});
        /*30*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(145, 517), b2Vec2(145, 528),  b2Vec2(185, 528)});
        /*31*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(258, 517),b2Vec2(218, 528), b2Vec2(258, 528)});
        /*32*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(287, 492), b2Vec2(258, 517), b2Vec2(258, 528), b2Vec2(287, 528)});
        /*33*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(312, 445), b2Vec2(287, 492), b2Vec2(287, 528), b2Vec2(312, 528)});
        /*34*/AddPolygonVerts(m_corpusVertsVec, m_corpusVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(53, 411),b2Vec2(53, 423), b2Vec2(63, 423)});
    }

    // ----------------------------------------------------------------------------------------------------
    // Piston
    {
        /*01*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(114, 78), b2Vec2(114, 184), b2Vec2(291, 184), b2Vec2(291, 78)});
        /*02*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(114, 184), b2Vec2(114, 224),b2Vec2(123, 224), b2Vec2(123, 184)});
        /*03*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(283, 184), b2Vec2(283, 224), b2Vec2(291, 224), b2Vec2(291, 184)});
        /*04*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(123, 184), b2Vec2(123, 224), b2Vec2(153, 197), b2Vec2(153, 184)});
        /*05*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(153, 184), b2Vec2(153, 197), b2Vec2(188, 184)});
        /*06*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(283, 184), b2Vec2(253, 184), b2Vec2(253, 197), b2Vec2(283, 224)});
        /*07*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(253, 184), b2Vec2(218, 184), b2Vec2(253, 197)});
        /*08*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(114, 78), b2Vec2(131, 78), b2Vec2(131, 73)});
        /*09*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(131, 73), b2Vec2(131, 78), b2Vec2(149, 78), b2Vec2(149, 60)});
        /*10*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(149, 60), b2Vec2(149, 78), b2Vec2(158, 78), b2Vec2(158, 49)});
        /*11*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(158, 49), b2Vec2(158, 78), b2Vec2(165, 78), b2Vec2(165, 33)});
        /*12*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(165, 33), b2Vec2(165, 78), b2Vec2(181, 78), b2Vec2(181, 33)});
        /*13*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(181, 33), b2Vec2(181, 78), b2Vec2(214, 78), b2Vec2(214, 57)});
        /*14*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(214, 57), b2Vec2(214, 78), b2Vec2(243, 78), b2Vec2(243, 70)});
        /*15*/AddPolygonVerts(m_pistonVertsVec, m_pistonVertsSizeVec, 3, new b2Vec2[3]{b2Vec2(243, 70), b2Vec2(243, 78), b2Vec2(291, 78)});
    }


    // ----------------------------------------------------------------------------------------------------
    // Connecting Rod

    // Bridge
    {
        AddPolygonVerts(m_conRodVertsVec, m_conRodVertsSizeVec, 4,
                        new b2Vec2[4]{
                        b2Vec2(186, 165),
                        b2Vec2(186, 317),
                        b2Vec2(220, 317),
                        b2Vec2(220, 165)});
    }

    // Lower Head
    {
        int l_radius = 39;
        b2Vec2* l_vertices = new b2Vec2[8];
        l_vertices[0] =  b2Vec2(203,348);
        for (int i = 0; i < 7; i++)
        {
            float l_angle = i / 3.0 * -180 * DEG_TO_RAD;
            l_vertices[i+1] =  b2Vec2(  l_vertices[0].x + l_radius * cosf(l_angle),
                                        l_vertices[0].y + l_radius * sinf(l_angle) );
        }
            AddPolygonVerts(m_conRodVertsVec, m_conRodVertsSizeVec, 8, l_vertices);
    }

    // Upper Head
    {
        int l_radius = 39;
        b2Vec2* l_vertices = new b2Vec2[8];
        l_vertices[0] =  b2Vec2(203,135);
        for (int i = 0; i < 7; i++)
        {
            float l_angle = i / 3.0 * -180 * DEG_TO_RAD;
            l_vertices[i+1] =  b2Vec2(  l_vertices[0].x + l_radius * cosf(l_angle),
                                        l_vertices[0].y + l_radius * sinf(l_angle) );
        }
            AddPolygonVerts(m_conRodVertsVec, m_conRodVertsSizeVec, 8, l_vertices);
    }

    // Crankshaft

    // Lower Half - Circle
    {
        int l_radius = 92;
        b2Vec2* l_vertices = new b2Vec2[8];
        l_vertices[0] =  b2Vec2(203,417);
        for (int i = 0; i < 7; i++)
        {
            float l_angle = i / 6.0 * 180 * DEG_TO_RAD;
            l_vertices[i+1] =  b2Vec2(l_vertices[0].x + l_radius * cosf(l_angle),
                                      l_vertices[0].y + l_radius * sinf(l_angle) );
        }
        AddPolygonVerts(m_crankshaftVertsVec, m_crankshaftVertsSizeVec, 8, l_vertices);
    }


    // Upper Half - Circle
    {
        int l_radius = 39;
        b2Vec2* l_vertices = new b2Vec2[8];
        l_vertices[0] =  b2Vec2(203,348);
        for (int i = 0; i < 7; i++)
        {
            float l_angle = i / 6.0 * -180 * DEG_TO_RAD;
            l_vertices[i+1] =  b2Vec2(l_vertices[0].x + l_radius * cosf(l_angle),
                                      l_vertices[0].y + l_radius * sinf(l_angle) );
        }
        AddPolygonVerts(m_crankshaftVertsVec, m_crankshaftVertsSizeVec, 8, l_vertices);
    }

    // Middle of Crankshaft.
    AddPolygonVerts(m_crankshaftVertsVec, m_crankshaftVertsSizeVec, 4, new b2Vec2[4]{b2Vec2(164, 348), b2Vec2(164, 417), b2Vec2(242, 417), b2Vec2(242, 348)});

}


void Update(sf::RenderWindow& window, b2World& world)
{
    sf::Event l_sfmlEvent;

    while (window.pollEvent(l_sfmlEvent))
    {
        switch(l_sfmlEvent.type)
        {
            case sf::Event::Closed:
                window.close();
                break;


            case sf::Event::KeyPressed:
                if(sf::Keyboard::isKeyPressed(sf::Keyboard::Return))
                {
                    m_engineOn = true;
                }
                break;



        }
    }

    WorldStep(world);
    ForceUpdate(world);

}

void Draw(sf::RenderWindow& window, b2World& world)
{
    window.clear(sf::Color::White);
    world.DrawDebugData();
    window.display();
}

void WorldStep(b2World& world)
{
	int32 velocityIterations = 6;
	int32 positionIterations = 4;

	world.Step( UPDATE_TICKS,
                velocityIterations,
                positionIterations);
}



void CreateEngineBodies(b2World& world)
{
    CreateCorpus(world);
    CreatePiston(world);
    CreateCrankshaft(world);
    CreateConnectRod(world);
    CreateReedValve(world);
}

void CreateCorpus(b2World& world)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_kinematicBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    FixtureUserDataContainer* l_data = new FixtureUserDataContainer("Corpus");
    l_bodyDef.fixedRotation = true;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    for(int i = 0; i < m_corpusVertsSizeVec.size(); i++)
    {
        PolygonMaker(l_body, .05f, m_corpusVertsVec.at(i), m_corpusVertsSizeVec.at(i), l_data);
    }

    m_bodies["Corpus"] = l_body;
}

void CreatePiston(b2World& world)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_dynamicBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;

    l_bodyDef.fixedRotation = true;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    for(int i = 0; i < m_pistonVertsVec.size(); i++)
    {
        if(i < c_startIndexOfPistonTop)
        {
            PolygonMaker(l_body, .005f, m_pistonVertsVec.at(i), m_pistonVertsSizeVec.at(i), new FixtureUserDataContainer("Piston"));
        }
        else
        {
            PolygonMaker(l_body, .005f, m_pistonVertsVec.at(i), m_pistonVertsSizeVec.at(i), new FixtureUserDataContainer("Piston Top"));
        }
    }

    m_bodies["Piston"] = l_body;
}

void CreateCrankshaft(b2World& world)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_dynamicBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    FixtureUserDataContainer* l_data =  new FixtureUserDataContainer("Crankshaft");
    l_bodyDef.fixedRotation = false;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    for(int i = 0; i < m_crankshaftVertsVec.size(); i++)
    {
        PolygonMaker(l_body, .005f, m_crankshaftVertsVec.at(i), m_crankshaftVertsSizeVec.at(i), l_data);
    }
    m_bodies["Crankshaft"] = l_body;
}

void CreateConnectRod(b2World& world)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_dynamicBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    FixtureUserDataContainer* l_data =  new FixtureUserDataContainer("Connecting Rod");
    l_bodyDef.fixedRotation = false;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    for(int i = 0; i < m_conRodVertsVec.size(); i++)
    {
        PolygonMaker(l_body, .0005f, m_conRodVertsVec.at(i), m_conRodVertsSizeVec.at(i), l_data);
    }
    m_bodies["Connecting Rod"] = l_body;
}

void CreateReedValve(b2World& world)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_dynamicBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    l_bodyDef.fixedRotation = false;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    PolygonMaker(l_body, .001f, new b2Vec2[4]{b2Vec2(92, 393), b2Vec2(81, 404), b2Vec2(81, 427), b2Vec2(92, 436)}, 4, new FixtureUserDataContainer("ValveCover"));
    PolygonMaker(l_body, .001f, new b2Vec2[4]{b2Vec2(35, 410), b2Vec2(35, 421), b2Vec2(81, 421), b2Vec2(81, 410)}, 4, new FixtureUserDataContainer("ValveBody"));

    // Valve Head
    {
        int l_radius = 9;
        b2Vec2* l_vertices = new b2Vec2[8];
        l_vertices[0] =  b2Vec2(35,415);

        for (int i = 0; i < 7; i++)
        {
            float l_angle = (90 + (i / 6.0 * 180) )* DEG_TO_RAD;
            l_vertices[i+1] =  b2Vec2(  l_vertices[0].x + l_radius * cosf(l_angle),
                                        l_vertices[0].y + l_radius * sinf(l_angle) );
        }
        PolygonMaker(l_body, .001f, l_vertices, 8, new FixtureUserDataContainer("ValveHead"));
    }


    m_bodies["Reed Valve"] = l_body;
}

void CreateJoints(b2World& world)
{
    AddRelativeJoint(world, m_bodies["Corpus"], m_bodies["Crankshaft"], "Corpus-Crankshaft Joint", b2Vec2(203, 417), false);
    AddRelativeJoint(world, m_bodies["Connecting Rod"], m_bodies["Crankshaft"], "Rod-Crankshaft Joint", b2Vec2(203,348), false);
    AddRelativeJoint(world, m_bodies["Connecting Rod"], m_bodies["Piston"], "Rod-Piston Joint", b2Vec2(203,135), false);
    AddPrismaticJoint(world, m_bodies["Piston"], m_bodies["Corpus"], "Piston Prismatic Joint",b2Vec2(0.0f, 1.0f), false, false);
    AddPrismaticJoint(world, m_bodies["Reed Valve"], m_bodies["Corpus"], "Valve Prismatic Joint", b2Vec2(1.0f, 0.0f), true, false);
}

void CreateAirPressureZones(b2World& world)
{
    AddAirPressureArea(world, new b2Vec2[4]{b2Vec2(102,22),b2Vec2(102,215),b2Vec2(294,215),b2Vec2(294,22)}, 4, "Combustion Chamber");
    AddAirPressureArea(world, new b2Vec2[4]{b2Vec2(2,340),b2Vec2(2,445),b2Vec2(93,445),b2Vec2(93,340)}, 4, "Intake Port");
    AddAirPressureArea(world, new b2Vec2[4]{b2Vec2(102,179),b2Vec2(102,528),b2Vec2(112,528),b2Vec2(112,179)}, 4, "Air Suck Left");
    AddAirPressureArea(world, new b2Vec2[4]{b2Vec2(112,220),b2Vec2(112, 445),b2Vec2(320, 445),b2Vec2(320,220)}, 4, "Air Suck Right");
    AddAirPressureArea(world, new b2Vec2[4]{b2Vec2(112,445),b2Vec2(112, 528),b2Vec2(320, 528),b2Vec2(320,445)}, 4, "Air Suck Bottom");
}

void CreateSensors(b2World& world)
{
    AddSensor(world, new b2Vec2[4]{b2Vec2(285,167),b2Vec2(285, 170),b2Vec2(295, 170),b2Vec2(295,167)}, 4, "Exhaust Lock");
    AddSensor(world, new b2Vec2[4]{b2Vec2(112,178),b2Vec2(112, 180),b2Vec2(116, 180),b2Vec2(116,178)}, 4, "Combustion Chamber Lock");
    AddSensor(world, new b2Vec2[4]{b2Vec2(359,169),b2Vec2(359, 212),b2Vec2(367, 212),b2Vec2(367,169)}, 4, "Particle Remover Exhaust");
    AddSensor(world, new b2Vec2[4]{b2Vec2(3,340),b2Vec2(3, 364),b2Vec2(5, 364),b2Vec2(5,340)}, 4, "Particle Remover Intake");
    AddSensor(world, new b2Vec2[4]{b2Vec2(112,22),b2Vec2(112, 80),b2Vec2(293, 80),b2Vec2(293,22)}, 4, "Ignition Trigger");
}



void SpawnFuelParticles(b2World& world, b2Vec2 position, int totalParticles)
{
    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_dynamicBody;

    l_bodyDef.angle = 0;
    l_bodyDef.fixedRotation = false;
    b2CircleShape l_shape;
    l_shape.m_radius = 4;
    b2FixtureDef l_fixture;

    l_fixture.shape = &l_shape;
    l_fixture.userData = new FixtureUserDataContainer("Fuel");
    l_fixture.density = .001f;
    l_fixture.friction = 0;
    l_fixture.restitution = 0;

    if(m_fuelParticles.size() + totalParticles >= c_maxParticles)
    {
        totalParticles = m_fuelParticles.size() - c_maxParticles;
    }


    for (int i = 0; i < totalParticles; ++i)
    {
        Fuel* l_fuel = new Fuel();
        l_fuel->burned = false;
        l_fuel->burning = false;
        l_bodyDef.position.Set( position.x + (-5 + rand() % 11),
                                position.y + (-5 + rand() % 11));

        l_bodyDef.userData = l_fuel;

        l_shape.m_p.Set(0,0);
        l_fixture.shape = &l_shape;

        b2Body* l_body = world.CreateBody(&l_bodyDef);
        l_body->CreateFixture(&l_fixture);
        m_fuelParticles.push_back(l_body);
    }
}



void PolygonMaker(b2Body* body, float density, b2Vec2 verts[], int size, FixtureUserDataContainer* data)
{

    b2PolygonShape l_shape;
    l_shape.Set(verts,size);

    b2FixtureDef l_fixture;
    l_fixture.shape = &l_shape;
    l_fixture.density = density;
    l_fixture.friction = 0;
    l_fixture.restitution = 0;
    l_fixture.userData = data;

    body->CreateFixture(&l_fixture);
}

void AddPolygonVerts(std::vector<b2Vec2*>& vertVec, std::vector<int>& vertsSizeVec, int size, b2Vec2* verts)
{
    vertVec.push_back(verts);
    vertsSizeVec.push_back(size);
}

void AddRelativeJoint(b2World& world, b2Body* bodyA, b2Body* bodyB, std::string name, b2Vec2 jointPos, bool collideConnected, bool testMotor)
{
    b2RevoluteJointDef l_jointDef;
    l_jointDef.bodyA = bodyA;
    l_jointDef.bodyB = bodyB;
    l_jointDef.collideConnected = collideConnected;
    l_jointDef.localAnchorA.Set(jointPos.x, jointPos.y);
    l_jointDef.localAnchorB.Set(jointPos.x, jointPos.y);
    l_jointDef.maxMotorTorque = 600000000000000;
    l_jointDef.enableMotor = false;

    if(testMotor)
    {
        l_jointDef.motorSpeed = .2f;
    }

    b2RevoluteJoint* l_joint =  (b2RevoluteJoint*)world.CreateJoint( &l_jointDef );
    m_joints[name] = l_joint;
}

void AddPrismaticJoint(b2World& world, b2Body* bodyA, b2Body* bodyB, std::string name, b2Vec2 axis, bool collideConnected, bool enableMotor)
{
    b2PrismaticJointDef l_prisJointDef;
    l_prisJointDef.bodyA = bodyA;
    l_prisJointDef.bodyB = bodyB;
    l_prisJointDef.localAxisA = axis;
    l_prisJointDef.collideConnected = collideConnected;
    l_prisJointDef.maxMotorForce = 10000000000.0f;
    l_prisJointDef.enableMotor = enableMotor;


    b2PrismaticJoint* l_prisJoint =  (b2PrismaticJoint*)world.CreateJoint( &l_prisJointDef );
    m_joints[name] =l_prisJoint;
}

void AddAirPressureArea(b2World& world, b2Vec2* verts, int size, std::string name)
{
    AirPressureArea* l_area = new AirPressureArea();
    l_area->awake = false;
    l_area->name = name;

    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_staticBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    l_bodyDef.fixedRotation = true;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    b2PolygonShape l_shape;
    l_shape.Set(verts, size);
    b2FixtureDef l_fixture;
    l_fixture.userData = new FixtureUserDataContainer(name);
    l_fixture.shape = &l_shape;
    l_fixture.density = 0;
    l_fixture.isSensor = true;
    l_fixture.friction = 0;
    l_fixture.restitution = 0;
    l_body->CreateFixture(&l_fixture);

    l_area->sensor = l_body;
    l_body->SetUserData(l_area);
    m_airAreas[name] = l_area;
}

void AddSensor(b2World& world, b2Vec2* verts, int size, std::string name)
{
    SensorArea* l_area = new SensorArea();
    l_area->touched = false;
    l_area->name = name;

    b2BodyDef l_bodyDef;
    l_bodyDef.type = b2_staticBody;
    l_bodyDef.position.Set(0, 0);
    l_bodyDef.angle = 0;
    l_bodyDef.fixedRotation = true;
    b2Body* l_body = world.CreateBody(&l_bodyDef);

    b2PolygonShape l_shape;
    l_shape.Set(verts, size);
    b2FixtureDef l_fixture;
    l_fixture.userData = new FixtureUserDataContainer(name);
    l_fixture.shape = &l_shape;
    l_fixture.density = 0;
    l_fixture.isSensor = true;
    l_fixture.friction = 0;
    l_fixture.restitution = 0;
    l_body->CreateFixture(&l_fixture);

    l_area->sensor = l_body;
    l_body->SetUserData(l_area);
    m_sensorAreas[name] = l_area;
}



void ForceUpdate(b2World& world)
{
    // Constant force to Reed Valve
    m_bodies["Reed Valve"]->ApplyForceToCenter(b2Vec2(-10,0), true);

    UpdateAirPressureZonesState(world);
    AffectBodiesInAirPressureZones();
    ParticleRemover(world);

    EnginePhysics(world);
    FuelPhysics();
}

void UpdateAirPressureZonesState(b2World& world)
{
    // Manage Intake Port based on Piston Y Velocity.
    {
        float l_pistonVelY = m_bodies["Piston"]->GetLinearVelocity().y;

        if(l_pistonVelY < -0.7f && m_airAreas["Intake Port"]->awake != true)
        {
            m_airAreas["Intake Port"]->awake = true;
        }

        else if (l_pistonVelY > 0.9f && m_airAreas["Intake Port"]->awake != false)
        {
            m_airAreas["Intake Port"]->awake = false;
            SpawnFuelParticles(world, b2Vec2(25, 352), 20);
        }
    }


    // Manage Combustion Chamber based on Exhaust Port state.
    {
        if(m_sensorAreas["Exhaust Lock"]->touched)
        {
            m_airAreas["Combustion Chamber"]->awake = false;
        }

        else if(m_sensorAreas["Exhaust Lock"]->touched == false)
        {
            m_airAreas["Combustion Chamber"]->awake = true;
        }

    }


    // Manage Air Suck if Combustion Chamber is open.
    {
        if(m_sensorAreas["Combustion Chamber Lock"]->touched)
        {
            m_airAreas["Air Suck Left"]->awake = false;
            m_airAreas["Air Suck Right"]->awake = false;
            m_airAreas["Air Suck Bottom"]->awake = false;
        }

        else if(m_sensorAreas["Combustion Chamber Lock"]->touched == false)
        {
            m_airAreas["Air Suck Left"]->awake = true;
            m_airAreas["Air Suck Right"]->awake = true;
            m_airAreas["Air Suck Bottom"]->awake = true;
        }
    }
}

void AffectBodiesInAirPressureZones()
{
    // If Intake Port is awake.
    if(m_airAreas["Intake Port"]->awake)
    {
        // Move Valve and let all fuel in.
        m_bodies["Reed Valve"]->ApplyForceToCenter(b2Vec2(15,0), true);

        for (b2ContactEdge* ce = m_airAreas["Intake Port"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(1,1), true);
        }
    }

    // If Exhaust Port is awake.
    if(m_airAreas["Combustion Chamber"]->awake)
    {
        // Move all particle inside of it.
        for (b2ContactEdge* ce = m_airAreas["Combustion Chamber"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(2,1.5f), true);
        }
    }

    //If Air Suck Port is awake. (if one awake - all awake)
    if(m_airAreas["Air Suck Right"]->awake)
    {
        // Move all particle inside of it.
        for (b2ContactEdge* ce = m_airAreas["Air Suck Right"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(-1,0.1f), true);
        }

        for (b2ContactEdge* ce = m_airAreas["Air Suck Left"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(0,-4), true);
        }

        for (b2ContactEdge* ce =  m_airAreas["Air Suck Bottom"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(0,-3), true);
        }
    }

    //Else if Air Suck Port is "sleeping". (if one sleeping - all sleeping)
    else if(!m_airAreas["Air Suck Right"]->awake)
    {
        // Move all particle inside of it.
        for (b2ContactEdge* ce = m_airAreas["Air Suck Right"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(0,.01f), true);
        }

        for (b2ContactEdge* ce = m_airAreas["Air Suck Left"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(0,.01f), true);
        }

        for (b2ContactEdge* ce =  m_airAreas["Air Suck Bottom"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            l_body->ApplyForceToCenter(b2Vec2(0,.01f), true);
        }
    }
}

void ParticleRemover(b2World& world)
{
    for (b2ContactEdge* ce = m_sensorAreas["Particle Remover Exhaust"]->sensor->GetContactList(); ce; ce = ce->next)
    {
        b2Body* l_body = ce->other;
        world.DestroyBody(l_body);

        m_fuelParticles.erase(std::remove(m_fuelParticles.begin(), m_fuelParticles.end(), l_body), m_fuelParticles.end());
    }

    for (b2ContactEdge* ce = m_sensorAreas["Particle Remover Intake"]->sensor->GetContactList(); ce; ce = ce->next)
    {
        b2Body* l_body = ce->other;
        world.DestroyBody(l_body);

        m_fuelParticles.erase(std::remove(m_fuelParticles.begin(), m_fuelParticles.end(), l_body), m_fuelParticles.end());
    }
}

void EnginePhysics(b2World& world)
{
    if(m_engineOn == true &&  !m_engineStarted)
    {
        m_sensorAreas["Ignition Trigger"]->touched = true;
        m_engineStarted = true;
    }


    if(m_sensorAreas["Ignition Trigger"]->touched && m_engineStarted)
    {
        for (b2ContactEdge* ce = m_sensorAreas["Ignition Trigger"]->sensor->GetContactList(); ce; ce = ce->next)
        {
            b2Body* l_body = ce->other;
            if(((FixtureUserDataContainer*)l_body->GetFixtureList()->GetUserData())->GetName() == "Fuel")
            {
                Fuel* l_fuel = (Fuel*)l_body->GetUserData();

                if(!l_fuel->burned)
                {
                    //l_body->ApplyLinearImpulse(b2Vec2(0,20), l_body->GetWorldCenter(),true);
                    ExplodeRaycast(world,l_body );
                    if(!l_fuel->burning)
                    {
                        l_fuel->burning = true;

                    }
                }
            }
        }
    }
}

void FuelPhysics()
{
    for( b2Body* body : m_fuelParticles)
    {
        Fuel* l_fuel = (Fuel*)body->GetUserData();

        if(l_fuel->burning &&  m_sensorAreas["Combustion Chamber Lock"]->touched == false)
        {
           l_fuel->burning = false;
           l_fuel->burned = true;
        }
    }

}





void ApplyBlastImpulse(b2Body* body, b2Vec2 blastCenter, b2Vec2 applyPoint, float blastPower)
{
    b2Vec2 l_blastDir = applyPoint - blastCenter;
    float l_distance = l_blastDir.Normalize();

    //ignore bodies exactly at the blast point
    if ( l_distance != 0 )
    {
        float l_invDistance = 1 / l_distance;
        float l_impulseMag = blastPower * l_invDistance * l_invDistance;
        body->ApplyLinearImpulse( l_impulseMag * l_blastDir, applyPoint, true);
    }
}

void ExplodeRaycast(b2World& world, b2Body* body)
{
    if ( body )
    {
        b2Vec2 l_center = body->GetPosition();
        RayCastClosestCallback l_callback;

        for (int i = 0; i < c_conbustionSpreadDirs; i++)
        {
            float l_angle = (i / (float)c_conbustionSpreadDirs) * 360.0 * DEG_TO_RAD;

            b2Vec2 l_raycastDirection( sinf(l_angle), cosf(l_angle));
            b2Vec2 l_raycastEnd = l_center + c_conbustionRadius * l_raycastDirection;
            world.RayCast(&l_callback, l_center, l_raycastEnd);

            if (l_callback.m_body )
            {
                ApplyBlastImpulse(l_callback.m_body, l_center, l_callback.m_point, (c_conbustionForce / (float)c_conbustionSpreadDirs));
            }
        }
    }
}
