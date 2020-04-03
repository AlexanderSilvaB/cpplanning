#ifndef _CPPLANNING_CPPLANNING_HPP_
#define _CPPLANNING_CPPLANNING_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "Common.hpp"
#include "CPPAlgorithm.hpp"

class CPPlanning
{
    private:
        bool running;
        bool verbose;
        std::vector<Shape> shapes;
        cv::Mat img;
        int rw, rh;
        float sw, sh;
        CPPAlgorithm *defaultAlg;
        int grid;

        float fixAngle(float rad);
        void drawGrid();
        void drawBackground();
        void updateShape(Shape& shape, float dt);
        void drawShape(Shape& shape);

    public:
        CPPlanning(int width = 500, int height = 500, float widthM = -1, float heightM = -1);
        virtual ~CPPlanning();

        void setGrid(int grid);
        int getGrid();

        void setVerbose(bool verbose);
        bool isVerbose();

        int createRobot(float x = 0, float y = 0, float theta = 0);
        int createObstacle(float x = 0, float y = 0, float theta = 0);

        bool setPosition(int id, float x = 0, float y = 0, float theta = 0);
        bool getPosition(int id, float *x, float *y, float *theta);
        bool setVelocity(int id, float linear = 0, float angular = 0);
        bool getVelocity(int id, float *linear, float *angular);
        bool setGoal(int id, float x = 0, float y = 0, float theta = 0);
        bool getGoal(int id, float *x, float *y, float *theta);
        void setAlgorithm(int id, CPPAlgorithm *alg = NULL);
        void setDefaultAlgorithm(CPPAlgorithm *alg = NULL);
        void getObstacles(std::vector<Shape> &obstacles, int id = -1, float e0 = -1);
        bool done(int id, float e0 = 0);

        bool update();
        void forever();
        void finish();
};

#endif
