#ifndef _CPPLANNING_CPPALGORITHM_HPP_
#define _CPPLANNING_CPPALGORITHM_HPP_

#include <vector>
#include "Common.hpp"

class CPPlanning;

class CPPAlgorithm
{
    private:
        int robot;
        CPPlanning *cp;

    protected:
        bool isValid();
        bool isVerbose();
        bool setPosition(float x = 0, float y = 0, float theta = 0);
        bool getPosition(float *x, float *y, float *theta);
        bool setVelocity(float linear = 0, float angular = 0);
        bool getVelocity(float *linear, float *angular);
        bool setGoal(float x = 0, float y = 0, float theta = 0);
        bool getGoal(float *x, float *y, float *theta);
        void getObstacles(std::vector<Shape> &obstacles, float e0 = -1);
        bool done(float e0 = 0);

    public:
        CPPAlgorithm();
        virtual ~CPPAlgorithm();

        void set(CPPlanning *cp, int robot);
        virtual void run();
};

#endif
