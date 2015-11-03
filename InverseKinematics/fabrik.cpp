//
//  fabrik.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include "point.h"
#include "fabrik.h"

Fabrik::Fabrik(float tolerance, float eps) {
    tol = tolerance;
    epsilon = eps; //Error for out of bounds correction
}

void Fabrik::setGoal(Point x) {
    goal = x;
}

void Fabrik::setJoints(Point one, Point two, Point three, Point four) {
    joints[0] = one;
    joints[1] = two;
    joints[2] = three;
    joints[3] = four;
    
    float d0 = (two.getPosition() - one.getPosition()).getDistance();
    float d1 = (three.getPosition() - two.getPosition()).getDistance();
    float d2 = (four.getPosition() - three.getPosition()).getDistance();
    d[0] = d0;
    d[1] = d1;
    d[2] = d2;
}

Point* Fabrik::getJoints() {
    return joints;
}

// Fixes the out-of-reach problem.  Forces the last link to maintain its length,
// going as close as possible toward the goal.

/*void Fabrik::shrinkEnd() {
    float dCalc = (joints[3] - joints[2]).getDistance();
    float dExact = d[2];
    
    if (dCalc - dExact > 0.0001 || dCalc - dExact < -0.0001) {
        Point vector = goal - joints[3];
        vector.normalize();
        double factor = dExact / vector.getDistance();
        joints[4] = joints[3] + (vector * factor);
        goal = joints[4];
    }
}*/


//Use previous point's orientation to build up position constraint area and cut a segment with proper length
//Position Constraint --> get joint[i]  //find the line(L) passing through Pi+1 and Pi
//fitch the segment on L that satisfied the distance constraint (expressed by di)

void Fabrik::Orientation_Constraint(Point ThisJP, Point PrevJP){
    
}

void Fabrik::Position_Constraint(Point ThisJP, Point PrevJP){
    
}





// Uses the FABRIK algorithm to compute IK.
void Fabrik::compute() {
    int n = 3; //number of joints
    Point p0 = joints[0];
    Point p1 = joints[1];
    Point p2 = joints[2];
    Point p3 = joints[3];
    Point t = goal;
    
    float dist = (p0.getPosition() - goal.getPosition()).getDistance();
    
    if ((dist - epsilon) > d[0] + d[1] + d[2]) {
//Target unreachable
        float r, lambda;
        for (int i = 0; i < 3; i++) {
            r = (goal.getPosition() - joints[i].getPosition()).getDistance();
            lambda = d[i] / r;
            Position P =(1-lambda) * joints[i].getPosition() + lambda * goal.getPosition();
            Axis A = joints[i].getAxis();
            Point tmp(P,A);
            joints[i+1] = tmp;           // joints[i+1] = (1-lambda) * joints[i] + lambda * goal;
        }
    }
    else {
//Target reachable
        Point b = p0;
        float difA = (p3.getPosition() - t.getPosition()).getDistance();
        while (difA > tol) {
            //STAGE ONE: Forward Reaching
            joints[n] = t;
            //[Xn, Yn,Zn] = [Xt, Yt, Zt];
            for (int i = n-1; i >= 0; i--) {
                
//Use previous point's orientation to build up position constraint area and cut a segment with proper length
                //Position Constraint --> get joint[i]  //find the line(L) passing through Pi+1 and Pi
                //fitch the segment on L that satisfied the distance constraint (expressed by di)
                float r = (joints[i+1].getPosition() - joints[i].getPosition()).getDistance();
                float lambda = d[i] / r;
                //Final Joint[i];
                Point tmp(((1-lambda) * joints[i+1].getPosition()) + (lambda * joints[i].getPosition()), joints[i].getAxis());
                joints[i] = tmp; //((1-lambda) * joints[i+1]) + (lambda * joints[i]);
                
//After we got the position of new Pi, we apply the previous orientation[X, Y, Z] on that point and we check the oritation boundary, then apply the new orientation satisfied the constraint.
                
                //Point Xt = (joints[i+1] - joints[i])/r;
                //float Angle = acos(<Xi,Xt>);
                //Axis = cross(Xi,Xt);
                //[Xi,Yi,Zi] = rotat([Xi,Yi,Zi],angle,axis);
                //[Xi,Yi,Zi] = constraint_orientation([Xi,Yi,Zi],[Xi+1,Yi+1,Zi+1],[Ub,LB]);
            }
            
            //STAGE 2: Backward Reaching
            joints[0] = b;
            for (int i = 0; i < n-1; i++) {
                float r = (joints[i+1].getPosition() - joints[i].getPosition()).getDistance();
                float lambda = d[i] / r;
                Point tmp(((1-lambda) * joints[i].getPosition()) + (lambda * joints[i+1].getPosition()), joints[i+1].getAxis());
                joints[i+1] = tmp; //((1-lambda) * joints[i]) + (lambda * joints[i+1]);
            }
            difA = (joints[n].getPosition() - t.getPosition()).getDistance();
        }
    }
    
    //shrinkEnd();
}