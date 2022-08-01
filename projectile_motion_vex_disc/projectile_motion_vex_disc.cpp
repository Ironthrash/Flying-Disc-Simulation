
#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <vector>
#include <string>


using namespace std;

float abs2 (float value){
    if (value >= 0){
        return value;
    }
    else{
        return -value;
    }
}

int main(){

    static double g = -9.81;

    static double m = 0.175;

    static double RHO = 1.23;

    static double AREA = 0.0568;

    static double CL0 = 0.1;
    static double CLA = 1.4;

    static double CD0 = 0.08;
    static double CDA = 2.72;

    static double ALPHA0 = -4;

    static double xDes = 7;
    static double yDes = 7;

    static double maxError = 0.05;

    static double Kp = 0.5;

    static double xInit = 0;
    static double yInit = 1;

    static double vInit = 14;

    double x = xInit;
    double y = yInit;

    double v = vInit;
    double vAngle = 40;

    double vx = v*cos(vAngle*(M_PI/180));
    double vy = v*sin(vAngle*(M_PI/180));

    double deltavx = 0;
    double deltavy = 0;

    double ALPHA = 40;

    double time = 0;
    double deltaT = 0.001;

    double CL = CL0 + CLA*ALPHA*M_PI/180;
    double CD = CD0 + CDA*pow((ALPHA - ALPHA0)*M_PI/180, 2);

    double test = 51;

    //fw << "Test for doubles. double: " << test << endl;

    //fw.close();

    int iterations = 0;

    while(abs2(y - yDes) > maxError){

        x = xInit;
        y = yInit;

        time = 0;

        while(x < xDes){

            deltavx = -RHO*pow(vx, 2)*AREA*CD*deltaT;

            deltavy = (RHO*pow(vx, 2)*AREA*CL/2/m+g)*deltaT;

            vx = vx + deltavx;
            vy = vy + deltavy;

            x = x + vx*deltaT;
            y = y + vy*deltaT;

            time = time + deltaT;

            //cout << "x = " << x << endl << "y = " << y << endl << "at time " << time << " seconds" << endl;

            //fw << x << ", " << y << ", " << time << endl;

        }

        v = v - Kp * (y - yDes);

        vx = v*cos(vAngle*(M_PI/180));
        vy = v*sin(vAngle*(M_PI/180));

        iterations = iterations + 1;

    }

    cout << "number of internal iterations: " << iterations << endl;

    ofstream fw("C:\\Users\\chris\\Documents\\vex-disc-sim(13).txt", std::ofstream::out);

    cout << "y value after termination: " << y << ", desired y value: " << yDes << ", velocity after termination: " << v << endl;

    x = xInit;
    y = yInit;

    time = 0;

    while(y > 0){

        

        deltavx = -RHO*pow(vx, 2)*AREA*CD*deltaT;

        deltavy = (RHO*pow(vx, 2)*AREA*CL/2/m+g)*deltaT;

        vx = vx + deltavx;
        vy = vy + deltavy;

        x = x + vx*deltaT;
        y = y + vy*deltaT;

        time = time + deltaT;

        //cout << "x = " << x << endl << "y = " << y << endl << "at time " << time << " seconds" << endl;

        fw << x << ", " << y << ", " << time << endl;

    }

    

    fw.close();

    cout << "task completed successfully" << endl;

}
