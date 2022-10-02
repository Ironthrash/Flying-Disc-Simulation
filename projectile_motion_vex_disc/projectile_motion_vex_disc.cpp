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
//just a better absolute value function

int main(){

    float g = -9.81;
    //the force of gravity

    float m = 0.175;
    //the mass of the projectile

    float RHO = 1.23;
    //the density of air in metric units

    float AREA = 0.041;
    //the approximate area of the flat side of the frisbee

    float CL0 = 0.1;
    //the coefficient of lift independent of the angle of attack
    float CLA = 1.4;
    //the coefficient of lift dependent on the angle of attack

    float CD0 = 0.08;
    //the coefficient of drag independent of the angle of attack
    float CDA = 2.72;
    //the coefficient of drag dependent on the angle of attack

    float ALPHA0 = 0;
    //the angle of attack where the coefficient of drag is the lowest

    float xDes = 1.905144705982368;
    //the x coordinate we want the frisbee to pass through
    float yDes = 0.762;
    //the y coordinate we want the frisbee to pass through

    float maxError = 0.001;
    //the level of error we want our disc to be under

    float Kp = 1.8;
    //a constant used as part of a PID loop later

    float xInit = 0;
    //the initial x position of the frisbee
    float yInit = 0.3;
    //the initial y position of the frisbee

    float vInit = 1;
    //the initial velocity of the frisbee

    float x = xInit;
    //the current x position of the frisbee
    float y = yInit;
    //the current y position of the frisbee

    float v = vInit;
    //the current velocity of the frisbee
    float vAngle = 40;
    //the initial angle of the velocity of the frisbee (should probably be called vAngleInit(whatever, suck it(I'm looking at you Ahmad)))

    float vx = v*cos(vAngle*(M_PI/180));
    //the current x component of the velocity of the frisbee
    float vy = v*sin(vAngle*(M_PI/180));
    //the current y component of the velocity of the frisbee

    float deltavx = 0;
    //the difference in the x component of the velocity of the frisbee
    float deltavy = 0;
    //the difference in the y compoenet of the velocity of the frisbee

    float discAngle = 40;

    float ALPHA = 40;
    //the angle of the attack of the frisbee

    float time = 0;
    //the current time of the simulation
    float deltaT = 0.0001;
    //the difference in time between each iteration

    float CL = CL0 + CLA*ALPHA*M_PI/180;
    //the coefficient of lift of the frisbee
    float CD = CD0 + CDA*pow((ALPHA - ALPHA0)*M_PI/180, 2);
    //the coefficient of drag of the frisbee

    int iterations = 0;
    //a count of the number of iterations that it takes to find the required velocity value

    while(abs2(y - yDes) > maxError){
        x = xInit;
        //resets the x value after the simulation
        y = yInit;
        //resets the y value after the simulation

        time = 0;
        //resets the time after the simulation

        while(x < xDes){
            ALPHA = discAngle - ((atan(vy/vx)) * (180/M_PI));

            CL = CL0 + CLA*ALPHA*M_PI/180;
            CD = CD0 + CDA*pow((ALPHA - ALPHA0)*M_PI/180, 2);

            deltavx = -RHO*pow(vx, 2)*AREA*CD*deltaT;
            //calculates the change in the x component of the velocity over the change in time
            deltavy = (RHO*pow(vx, 2)*AREA*CL/2/m+g)*deltaT;
            //calculates the change in the y component of the velocity over the change in time

            vx = vx + deltavx;
            //changes the x component of the velocity based on the deltavx
            vy = vy + deltavy;
            //changes the y component of the velocity based on the deltavy

            x = x + vx*deltaT;
            //changes the x position of the robot based on the x component of the velocity and the delta time
            y = y + vy*deltaT;
            //changes the y position of the robot based on the y component of the velocity and the delta time

            time = time + deltaT;
            //changes the time in the simulation based on the delta time
        }
        //simulates the path of the disc until the desired x position

        v = v - Kp * (y - yDes);
        //changes the initial velocity based on the difference between the simulated y value & the desired y value, and the value of Kp
        //doesn't use the vInit variable because it is not necessary

        vx = v*cos(vAngle*(M_PI/180));
        //recalculates the x component of the velocity
        vy = v*sin(vAngle*(M_PI/180));
        //recalculates the y component of the velocity

        iterations = iterations + 1;
        //increases the number of iterations by 1
    }
    //a loop that is used to find the required velocity value
    //in the loop the initial velocity is set to a value that is known to send the frisbee to within a certain range of the desired value
    //& that value is simply carried over the final simulation. This value could instead be outputed to be used for other applications.

    cout << "number of internal iterations: " << iterations << endl;
    //prints the number of iterations to the output

    ofstream fw("C:\\Users\\chris\\Documents\\original-flight-sim-test-with-velocity.txt", std::ofstream::out);
    //opens up a text file. ^^^^^^^^^^^^^^^^^^^^^^^^^^       ^ This is the text files name
    //                      CHANGE THIS TO A FOLDER ON YOUR OWN COMPUTER BEFORE RUNNING. I have literally no idea what will happen if you don't.

    cout << "y value after termination: " << y << ", x value after termination: " << x << ", desired y value: " << yDes << ", desired x value: " << xDes << ", velocity after termination: " << v << endl;
    //prints out the final simulated y value, the desired y value, and the required velocity to the output

    x = xInit;
    //resets the x value of the frisbee
    y = yInit;
    //resets the y value of the frisbees

    time = 0;
    //resets the time of the simulation

    while(y > 0){
        ALPHA = discAngle - atan(vy/vx);

        CL = CL0 + CLA*ALPHA*M_PI/180;
        CD = CD0 + CDA*pow((ALPHA - ALPHA0)*M_PI/180, 2);

        deltavx = -RHO*pow(vx, 2)*AREA*CD*deltaT;
        //calculates the change in the x component of the velocity over the change in time
        deltavy = (RHO*pow(vx, 2)*AREA*CL/2/m+g)*deltaT;
        //calculates the change in the y component of the velocity over the change in time

        vx = vx + deltavx;
        //changes the x component of the velocity based on the deltavx
        vy = vy + deltavy;
        //changes the y component of the velocity based on the deltavx

        x = x + vx*deltaT;
        //changes the x position of the robot based on the x component of the velocity and the delta time
        y = y + vy*deltaT;
        //changes the y position of the robot based on the y component of the velocity and the delta time

        time = time + deltaT;
        //changes the time in the simulation based on the delta time

        fw << x << ", " << y << ", " << vx << ", " << vy << ", " << time << endl;
        //exports the x & y values of the frisbee, and the final time of the simulation in the text file
    }
    //does a final simulation of the frisbee so that the final result can be exported as a text file

    fw.close();
    //closes the text file

    cout << "task completed successfully" << endl;
    //prints the completion of the task in the output
}
