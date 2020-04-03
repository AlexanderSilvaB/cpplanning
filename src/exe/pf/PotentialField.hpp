#ifndef _POTENTIAL_FIELD_
#define _POTENTIAL_FIELD_

#include <cpplanning/CPPAlgorithm.hpp>
#include <cmath>
#include <iostream>

using namespace std;

class PotentialField : public CPPAlgorithm
{
    private: 
        vector<Shape> obstacles;
        float e0, vmax, Kw;
        float x, y, theta;
        float xg, yg, thetag;
        float minD;
        float p, Fattx, Fatty, Frepx, Frepy, Ftotx, Ftoty, dx, dy, Katt, Krep, k;
        float v, w;

    public:
        PotentialField() : CPPAlgorithm()
        {
            e0 = 3.0f;
            vmax = 0.5f;
            Kw = 1.0f;

            Katt = 1.0f;
            Krep = 0.1f;

            minD = 0.35f;
        }

        void run()
        {
            if(!isValid())
                return;
            
            if(done(minD))
            {
                xg = (rand() % 500 - 250) / 100.0f;
                yg = (rand() % 500 - 250) / 100.0f;
                setGoal(xg, yg);
            }

            getGoal(&xg, &yg, &thetag);
            getPosition(&x, &y, &theta);

            dx = xg - x;
            dy = yg - y;
            p = sqrt(dx*dx + dy*dy);

            if(isVerbose())
                cout << "( " << x << ", " << y << " ) -> ( " << xg << ", " << yg << " ) -> " << p << endl;

            Fattx = Katt * dx;
            Fatty = Katt * dy;
            
            Frepx = 0;
            Frepy = 0;
            getObstacles(obstacles, e0);

            
            for(int i = 0; i < obstacles.size(); i++)
            {
                dx = obstacles[i].x - x;
                dy = obstacles[i].y - y;
                p = sqrt(dx*dx + dy*dy);
                k = (1.0f / (p*p*p)) * ((1.0f / e0) - (1.0f / p));
                Frepx += k * dx;
                Frepy += k * dy;
            }

            Frepx *= Krep;
            Frepy *= Krep;

            Ftotx = Fattx + Frepx;
            Ftoty = Fatty + Frepy;

            p = sqrt(Ftotx*Ftotx + Ftoty*Ftoty);
            v = min(p, vmax);
            float a = atan2(Ftoty, Ftotx);
            float b = a - theta;
            w = Kw * (atan2(Ftoty, Ftotx) - theta);
            setVelocity(v, w);

            if(isVerbose())
                cout << "( " << v << ", " << (w * 57) << "º )" << endl;
        }
};

#endif
