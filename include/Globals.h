#pragma once

#define METRESTOPIXELS 1
#define PIXELSTOMETRES 1.f / METRESTOPIXELS

#define FPS  60
#define UPDATE_TICKS  1.f/FPS
#define PI  3.1415926536f
#define RAD_TO_DEG  180 / PI
#define DEG_TO_RAD  PI / 180

struct AirPressureArea
{
    b2Body* sensor;
    std::string name;
    bool awake;
};

struct SensorArea
{
    b2Body* sensor;
    std::string name;
    bool touched;
};

struct Fuel
{
    bool burned;
    bool burning;
};




class Globals
{

};
