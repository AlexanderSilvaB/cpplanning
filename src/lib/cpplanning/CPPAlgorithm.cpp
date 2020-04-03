#include "CPPAlgorithm.hpp"
#include "cpplanning.hpp"
#include <iostream>

using namespace std;

CPPAlgorithm::CPPAlgorithm()
{
    robot = -1;
    cp = NULL;
}

CPPAlgorithm::~CPPAlgorithm()
{
    robot = -1;
    cp = NULL;
}

bool CPPAlgorithm::isValid()
{
    return !(robot < 0 || cp == NULL);   
}

bool CPPAlgorithm::isVerbose()
{
    if(!isValid())
        return false;
    return cp->isVerbose();
}

bool CPPAlgorithm::setPosition(float x, float y, float theta)
{
    if(!isValid())
        return false;
    return cp->setPosition(robot, x, y, theta);
}

bool CPPAlgorithm::getPosition(float *x, float *y, float *theta)
{
    if(!isValid())
        return false;
    return cp->getPosition(robot, x, y, theta);
}

bool CPPAlgorithm::setVelocity(float linear, float angular)
{
    if(!isValid())
        return false;
    return cp->setVelocity(robot, linear, angular);
}

bool CPPAlgorithm::getVelocity(float *linear, float *angular)
{
    if(!isValid())
        return false;
    return cp->getVelocity(robot, linear, angular);
}

bool CPPAlgorithm::setGoal(float x, float y, float theta)
{
    if(!isValid())
        return false;
    return cp->setGoal(robot, x, y, theta);
}

bool CPPAlgorithm::getGoal(float *x, float *y, float *theta)
{
    if(!isValid())
        return false;
    return cp->getGoal(robot, x, y, theta);
}

void CPPAlgorithm::getObstacles(std::vector<Shape> &obstacles, float e0)
{
    if(!isValid())
        return;
    cp->getObstacles(obstacles, robot, e0);
}

bool CPPAlgorithm::done(float e0)
{
    if(!isValid())
        return false;
    return cp->done(robot, e0);
}

void CPPAlgorithm::set(CPPlanning *cp, int robot)
{
    this->cp = cp;
    this->robot = robot;
}

void CPPAlgorithm::run()
{
    if(!isValid())
        return;
}