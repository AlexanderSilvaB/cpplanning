#include <iostream>
#include <cpplanning/cpplanning.hpp>
#include <ctime>
#include <cstdlib>
#include "PotentialField.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    srand(time(NULL));

    CPPlanning cp(500, 500, 5.0f, 5.0f);
    cp.setGrid(10);

    PotentialField pf;

    int N = 20;
    for(int i = 0; i < N; i++)
    {
        int r = cp.createRobot(0, 0, 0);
        cp.setAlgorithm(r, &pf);
    }

    for(int i = 0; i < 10; i++)
        cp.createObstacle((rand() % 500 - 250) / 100.0f, (rand() % 500 - 250) / 100.0f);

    cp.forever();

    return 0;
}