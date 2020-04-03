#include "cpplanning.hpp"
#include <cmath>

using namespace std;
using namespace cv;

CPPlanning::CPPlanning(int width, int height, float widthM, float heightM)
{
    defaultAlg = NULL;
    grid = -1;
    verbose = false;

    int m = std::min(width, height);

    if(widthM <= 0)
        widthM = width;

    if(heightM <= 0)
        heightM = height;

    sw = width / widthM;
    sh = height / heightM;
    

    rw = m / 40;
    rh = (int)(rw * 1.3);
    img = Mat::zeros(height, width, CV_8UC3);
    running = true;
}

CPPlanning::~CPPlanning()
{
    running = false;
}

float CPPlanning::fixAngle(float rad)
{
    rad = std::fmod(rad, 2 * PI);
    if (rad > PI)
        rad -= 2 * PI;
    else if(rad < -PI)
        rad += 2 * PI;
    return rad;
}

void CPPlanning::setGrid(int grid)
{
    this->grid = grid;
}

int CPPlanning::getGrid()
{
    return grid;
}

void CPPlanning::setVerbose(bool verbose)
{
    this->verbose = verbose;
}

bool CPPlanning::isVerbose()
{
    return verbose;
}

int CPPlanning::createRobot(float x, float y, float theta)
{
    Shape shape;
    shape.type = ROBOT;
    shape.x = x;
    shape.y = y;
    shape.theta = theta;
    shape.vs = 0;
    shape.vt = 0;
    shape.alg = NULL;
    shape.gx = x;
    shape.gy = y;
    shape.gtheta = theta;

    shapes.push_back(shape);
    return shapes.size() - 1;
}

int CPPlanning::createObstacle(float x, float y, float theta)
{
    Shape shape;
    shape.type = OBSTACLE;
    shape.x = x;
    shape.y = y;
    shape.theta = theta;
    shape.vs = 0;
    shape.vt = 0;
    shape.alg = NULL;
    shape.gx = x;
    shape.gy = y;
    shape.gtheta = theta;

    shapes.push_back(shape);
    return shapes.size() - 1;
}

bool CPPlanning::setPosition(int id, float x, float y, float theta)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    shapes[id].x = x;
    shapes[id].y = y;
    shapes[id].theta = theta;
    return true;
}

bool CPPlanning::getPosition(int id, float *x, float *y, float *theta)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    *x = shapes[id].x;
    *y = shapes[id].y;
    *theta = shapes[id].theta;
    return true;
}

bool CPPlanning::setVelocity(int id, float linear, float angular)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    angular = fixAngle(angular);

    shapes[id].vs = linear;
    shapes[id].vt = angular;
    return true;
}

bool CPPlanning::getVelocity(int id, float *linear, float *angular)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    *linear = shapes[id].vs;
    *angular = shapes[id].vt;
    return true;
}

bool CPPlanning::setGoal(int id, float x, float y, float theta)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    shapes[id].gx = x;
    shapes[id].gy = y;
    shapes[id].gtheta = theta;
    return true;
}

bool CPPlanning::getGoal(int id, float *x, float *y, float *theta)
{
    if(id < 0 || id >= shapes.size())
        return false;
    
    *x = shapes[id].gx;
    *y = shapes[id].gy;
    *theta = shapes[id].gtheta;
    return true;
}

bool CPPlanning::done(int id, float e0)
{
    if(id < 0 || id >= shapes.size())
        return false;

    float dx = shapes[id].gx - shapes[id].x;
    float dy = shapes[id].gy - shapes[id].y;
    float distance = sqrt(dx*dx + dy*dy);
    return distance <= e0;
}

void CPPlanning::getObstacles(std::vector<Shape> &obstacles, int id, float e0)
{
    obstacles.clear();

    Shape robot;
    if(id >= 0 && id < shapes.size())
        robot = shapes[id];

    for(int i = 0; i < shapes.size(); i++)
    {
        Shape shape = shapes[i];
        // if(shape.type == OBSTACLE)
        if(i != id)
        {
            if(id < 0 || id >= shapes.size() || e0 <= 0)
                obstacles.push_back(shape);
            else
            {
                float dx = shape.x - robot.x;
                float dy = shape.y - robot.y;
                float distance = sqrt(dx*dx + dy*dy);
                if(distance <= e0 && distance > 0)
                    obstacles.push_back(shape);
            }
        }
    }
}

bool CPPlanning::update()
{
    drawBackground();
    drawGrid();

    float dt = 0.03;
    CPPAlgorithm *alg;
    
    for(int i = 0; i < shapes.size(); i++)
    {
        alg = defaultAlg;
        if(shapes[i].alg != NULL)
            alg = shapes[i].alg;
        
        if(alg != NULL)
        {
            alg->set(this, i);
            alg->run();
        }

        updateShape(shapes[i], dt);
        drawShape(shapes[i]);
    }

    imshow("CPPlanning", img);
    waitKey(30);
    return running;
}

void CPPlanning::finish()
{
    running = false;
}

void CPPlanning::forever()
{
    while(update()){}
}

void CPPlanning::setAlgorithm(int id, CPPAlgorithm *alg)
{
    if(id < 0 || id >= shapes.size())
        return;

    shapes[id].alg = alg;   
}

void CPPlanning::setDefaultAlgorithm(CPPAlgorithm *alg)
{
    defaultAlg = alg;
}

void CPPlanning::drawBackground()
{
    img.setTo(Scalar(77, 128, 47));
}

void CPPlanning::drawGrid()
{
    if(grid <= 0)
        return;
    
    Scalar color(150, 150, 150);
    for(int y = 0; y < img.rows; y += grid)
    {
        line(img, Point(0, y), Point(img.cols, y), color, 1);
    }

    for(int x = 0; x < img.cols; x += grid)
    {
        line(img, Point(x, 0), Point(x, img.rows), color, 1);
    }
}

void CPPlanning::updateShape(Shape& shape, float dt)
{
    float vs = shape.vs * dt;
    float wt = shape.vt * dt;
    shape.x += vs * cos( shape.theta + wt * 0.5 );
    shape.y += vs * sin( shape.theta + wt * 0.5 );
    shape.theta += wt;

    shape.theta = fixAngle(shape.theta);
}

void CPPlanning::drawShape(Shape& shape)
{
    int w_ = img.cols / 2;
    int h_ = img.rows / 2;

    if(shape.type == ROBOT)
    {
        Scalar color(255, 126, 28);
        RotatedRect body(Point(w_ + shape.x * sw, img.rows - (h_ + shape.y * sh)), Size(rh, rw), -shape.theta * 57.2958);

        Point2f vertices2f[4];
        body.points(vertices2f);

        Point vertices[4];    
        for(int i = 0; i < 4; ++i)
            vertices[i] = vertices2f[i];

        fillConvexPoly(img, vertices, 4, color);

        
        Point head( (vertices[2].x + vertices[3].x) / 2, (vertices[2].y + vertices[3].y) / 2 );
        Point arrow( head.x + rw * 0.5f * cos(-shape.theta), head.y + rw * 0.5f * sin(-shape.theta) );

        arrowedLine(img, head, arrow, color, 2, CV_AA);
        // circle(img, head, rw / 2, color, CV_FILLED);

        color = Scalar(217, 28, 255);
        circle(img, Point(w_ + shape.gx * sw, img.rows - (h_ + shape.gy * sh)), rw * 0.5f, color, CV_FILLED);
    }
    else
    {
        Scalar color(0, 0, 255);
        circle(img, Point(w_ + shape.x * sw, img.rows - (h_ + shape.y * sh)), rw * 0.5f, color, CV_FILLED);
    }
}