//
//  fabrik.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "vector3.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include "point.h"
#include "axes.h"
#include "quaternion.h"
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

void Fabrik::shrinkEnd() {
    float dCalc = (joints[3].getPosition() - joints[2].getPosition()).getDistance();
    float dExact = d[2];
    
    if (dCalc - dExact > 0.0001 || dCalc - dExact < -0.0001) {
        Position vector(goal.getPosition() - joints[3].getPosition());
        vector.normalize();
        double factor = dExact / vector.getDistance();
        joints[4].setPosition(joints[3].getPosition()+vector*factor);
        //joints[4] = joints[3] + (vector * factor);
        goal = joints[4];
    }
}


void Fabrik::SetOrientation(Point &This, Point Previous){
    Axes tmp;
    Position X(This.getPosition() - Previous.getPosition());
    X.normalize();
    //Bone Vector AxisX
    //ForwardStage: segment<p[i+1],p[i]> / BackwardStage: segment<p[i-1],p[i]>
    Vector3 AxisX(X.getX(),X.getY(),X.getZ());
    
    Quaternion Rotor = Quaternion::v2q(This.getAxes().GetXAxis(), AxisX);
    Vector3 AxisY(Quaternion::rotVbyQ(This.getAxes().GetYAxis(), Rotor));
    Vector3 AxisZ(Quaternion::rotVbyQ(This.getAxes().GetZAxis(), Rotor));
    Axes axes(AxisX,AxisY,AxisZ);
    This.setAxes(axes);
    Orientation_Constraint(This, Previous);
}

//Use previous point's orientation to build up position constraint area and cut a segment with proper length
//Position Constraint --> get joint[i]  //find the line(L) passing through Pi+1 and Pi
//fitch the segment on L that satisfied the distance constraint (expressed by di)

/* --------- --------- ---------
 REMAINING PROBLEMS:
 HOW TO PASS BOUNDARY VALUE INTO THIS FUNC?
 ARE THE AXES VALUE SET INTO POINT 'THIS'?
 MAY angleY angleZ BE THE SAME?
 SELECT BOUNDARY TO DECIDE q1?
 
 Quaternion Rotor(q1 * invRotor);//MAY BE WRONG
 --------- --------- ---------*/

void Fabrik::Orientation_Constraint(Point &This, Point Previous){
    float epsilon = 0.001;
    float boundary;
    //Check Colinear
    Vector3 X_cur(This.getAxes().GetXAxis());
    Vector3 Y_cur(This.getAxes().GetYAxis());
                                                    //   Vector3 Z_cur(This.getAxes().GetZAxis());
    Vector3 X_pre(Previous.getAxes().GetXAxis());
    Vector3 Y_pre(Previous.getAxes().GetYAxis());
    Vector3 Z_pre(Previous.getAxes().GetZAxis());
    Vector3 cross(Vector3::cross(X_cur, X_pre));
    
    Vector3 tmpY;
    Vector3 tmpZ;
    Quaternion rotor;
    // Not Colinear
    if(Vector3::Getlen(cross) > epsilon){
        // rotor helps rotate X_cur to X_pre
        rotor = Quaternion::v2q(X_cur, X_pre);
    }
    // Colinear
    else{
        rotor = Quaternion(1,0,0,0);
    }
    // Apply rotor to X_cur should result in X_pre
    tmpY = Quaternion::rotVbyQ(Y_cur, rotor);
    float angleY = acosf(Vector3::Dot(tmpY, Y_pre)/(Vector3::Getlen(tmpY)*Vector3::Getlen(Y_pre)));
    
    // ?   tmpZ = Quaternion::rotVbyQ(Z_cur, rotor);
    // ?   float angleZ = acosf(Vector3::Dot(tmpZ, Z_pre)/(Vector3::Getlen(tmpZ)*Vector3::Getlen(Z_pre)));
    
    if(angleY > boundary)
    {
        float w = cos(boundary/2);
        Vector3 axis(Vector3::Normalize(X_pre));
        axis = axis * sin(boundary/2);
        Quaternion q1(w, axis.getX(), axis.getY(), axis.getZ());    //ROTOR HELPS TO ROTATE TO THE BOUND
    
        Quaternion invRotor = Quaternion::conjugate(rotor);         //COLINEAR MAKER
        Quaternion Rotor(q1 * invRotor);
        
        Vector3 X_new(Quaternion::rotVbyQ(X_pre, Rotor));
        Vector3 Y_new(Quaternion::rotVbyQ(Y_pre, Rotor));
        Vector3 Z_new(Quaternion::rotVbyQ(Z_pre, Rotor));
        Axes axes(X_new, Y_new, Z_new);
        This.setAxes(axes);
    
    }

}

void Fabrik::Position_Constraint(Point &This, Point Previous){
    
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
            joints[i+1].setPosition((1-lambda) * joints[i].getPosition() + lambda * goal.getPosition());
            // joints[i+1] = (1-lambda) * joints[i] + lambda * goal;
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
                //joints[i] = ((1-lambda) * joints[i+1]) + (lambda * joints[i]);
                joints[i].setPosition((1-lambda) * joints[i+1].getPosition() + lambda * joints[i].getPosition());
                
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
                joints[i+1].setPosition((1-lambda) * joints[i].getPosition() + lambda * joints[i+1].getPosition());
                 
                //joints[i+1] = ((1-lambda) * joints[i]) + (lambda * joints[i+1]);
            }
            difA = (joints[n].getPosition() - t.getPosition()).getDistance();
        }
    }
    
    //shrinkEnd();
}