#include "Connection.h"
#include "GUI.h"
#include "Ipc.hpp"

Connection::Connection(Parameters* params)
{
    (void)params;
}

void Connection::StartConnection()
{
}

void Connection::Connect()
{
}

std::istringstream Connection::Send(std::string buffer)
{
    (void)buffer;
    std::string rec = "";
    return std::istringstream(rec);
}

std::istringstream Connection::SendTracker(int id, double a, double b, double c, double qw, double qx, double qy, double qz, double time, double smoothing)
{
    (void)id;
    (void)a;
    (void)b;
    (void)c;
    (void)qw;
    (void)qx;
    (void)qy;
    (void)qz;
    (void)time;
    (void)smoothing;
    return Send("");
}

std::istringstream Connection::SendStation(int id, double a, double b, double c, double qw, double qx, double qy, double qz)
{
    (void)id;
    (void)a;
    (void)b;
    (void)c;
    (void)qw;
    (void)qx;
    (void)qy;
    (void)qz;
    return Send("");
}

bool GetDigitalActionState(vr::VRActionHandle_t action)
{
    (void)action;
    return false;
}

int Connection::GetButtonStates()
{
    return 0;
}

void Connection::GetControllerPose(double outpose[])
{
    (void)outpose;
    return;
}
