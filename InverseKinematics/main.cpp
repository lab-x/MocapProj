//
//  main.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "point.h"
#include "fabrik.h"
#include "quaternion.h"
#include "vector3.h"

#define PI 3.14159265
float posX = 2.0f;
float posY = 2.0f;
float posZ = 0.0f;
float fov = -40.0f;
float rotx = 168;
float roty = 180;

int jointNum = 0;
float tolerance = 0.01;
float moveStep = 0.2;
float epsilon = moveStep * 10;
float goal[3];
std::vector<float> endPosition[3];

//****************************************************
// Global Variables
//****************************************************
Fabrik fabrik(tolerance, epsilon);

void generateLinks() {
    Position p0(0.0001, 0.0001, 10);
    Position p1(0, 10, 10);
    Position p2(0, 20, 10);
    Position p3(0, 30, 10);
    Position goal(0, 3, 68);
    
    Axes AxesInit;
//    printf("%lf  %lf  %lf \n",AxesInit.GetXAxis().getX(),AxesInit.GetXAxis().getY(),AxesInit.GetXAxis().getZ());
//    printf("%lf  %lf  %lf \n",AxesInit.GetYAxis().getX(),AxesInit.GetYAxis().getY(),AxesInit.GetYAxis().getZ());
//    printf("%lf  %lf  %lf \n",AxesInit.GetZAxis().getX(),AxesInit.GetZAxis().getY(),AxesInit.GetZAxis().getZ());
    
    Point P0(p0, AxesInit);
    Point P1(p1, AxesInit);
    Point P2(p2, AxesInit);
    Point P3(p3, AxesInit);
    Point GOAL(goal, AxesInit);
    
    fabrik.setGoal(GOAL);
    fabrik.setJoints(P0,P1,P2,P3);
    jointNum = 4;
}

int main(int argc, const char * argv[]) {
    // insert code here...
    
    //FABRIK PART...
    generateLinks();
    fabrik.compute();
    Point* joints = fabrik.getJoints();
    
    for (int i = 0; i < jointNum; i++) {
        
        printf("joint %d \nPosition:%lf,%lf,%lf\n",i,joints[i].getPosition().getValues()[0],joints[i].getPosition().getValues()[1],joints[i].getPosition().getValues()[2]);
        
        printf("Axis-X:  %lf  %lf  %lf \n",joints[i].getAxes().GetXAxis().getX(),joints[i].getAxes().GetXAxis().getY(),joints[i].getAxes().GetXAxis().getZ());
        printf("Axis-Y:  %lf  %lf  %lf \n",joints[i].getAxes().GetYAxis().getX(),joints[i].getAxes().GetYAxis().getY(),joints[i].getAxes().GetYAxis().getZ());
        printf("Axis-Z:  %lf  %lf  %lf \n\n",joints[i].getAxes().GetZAxis().getX(),joints[i].getAxes().GetZAxis().getY(),joints[i].getAxes().GetZAxis().getZ());
    }
    //GENERATE  WORLD QUATERNIONS.
    //GENERATE EULER ANGLES AND FORM .bvh FOMAT.
    

/* ---- ----- ----
 Test Funcions:
    Quaternion q1(0.707,0.707,0,0);
    Quaternion q2(0.707,0, 0.707,0);
    Quaternion q3 = q1*q2;
    printf("%lf  %lf  %lf  %lf\n",q3.getW(),q3.getX(),q3.getY(),q3.getZ());
    
    Vector3 A(1,0,0);
    Vector3 B(0,1,0);
    // q3 helps rotate A to B
    q3 = Quaternion::v2q(A,B);
    // then apply q to A should result in B
    Vector3 C(Quaternion::rotVbyQ(A, q3));
    printf("%lf  %lf  %lf \n", C.getX(), C.getY(), C.getZ());
 ---- ----- ---- */
    return 0;
}
