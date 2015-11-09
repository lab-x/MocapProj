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
    for (int i = 0; i<3; i++)
    {
        bone[i] = Vector3(1, 0, 0);
        QW[i] = Quaternion(1,0,0,0);
        QL[i] = Quaternion(1,0,0,0);
        Euler[i] = Vector3(0,0,0);
    }
}

Point* Fabrik::getJoints() {
    return joints;
}
Vector3* Fabrik::getEulers() {
    return Euler;
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

void Fabrik::SetOrientation(Point &This, Point Previous, int Type){
    Axes tmp;
    Position X(This.getPosition() - Previous.getPosition());
    X.normalize();
    //Bone Vector AxisX
    //ForwardStage: segment<p[i+1],p[i]> / BackwardStage: segment<p[i-1],p[i]>
    Vector3 AxisX(X.getX(),X.getY(),X.getZ());
    if(Type == 1)
    {
        Quaternion Rotor = Quaternion::v2q(This.getFWDAxes().GetXAxis(), AxisX);
        Vector3 AxisY(Quaternion::rotVbyQ(This.getFWDAxes().GetYAxis(), Rotor));
        Vector3 AxisZ(Quaternion::rotVbyQ(This.getFWDAxes().GetZAxis(), Rotor));
        Axes axes(AxisX,AxisY,AxisZ);
        This.setFWDAxes(axes);
    }
    else if(Type == 2)
    {
        Quaternion Rotor = Quaternion::v2q(This.getBWDAxes().GetXAxis(), AxisX);
        Vector3 AxisY(Quaternion::rotVbyQ(This.getBWDAxes().GetYAxis(), Rotor));
        Vector3 AxisZ(Quaternion::rotVbyQ(This.getBWDAxes().GetZAxis(), Rotor));
        Axes axes(AxisX,AxisY,AxisZ);
        This.setBWDAxes(axes);
    }
    Orientation_Constraint(This, Previous, Type);
}

void Fabrik::Orientation_Constraint(Point &This, Point Previous, int Type){
    float epsilon = 0.001;
    float boundary = 180;
    //Check Colinear
    Vector3 X_cur, Y_cur, X_pre, Y_pre, Z_pre;
    if(Type == 1){
        X_cur = This.getFWDAxes().GetXAxis();
        Y_cur = This.getFWDAxes().GetYAxis();     //   Vector3 Z_cur(This.getAxes().GetZAxis());
        X_pre = Previous.getFWDAxes().GetXAxis();
        Y_pre = Previous.getFWDAxes().GetYAxis();
        Z_pre = Previous.getFWDAxes().GetZAxis();
        
    }
    else{
        X_cur = This.getBWDAxes().GetXAxis();
        Y_cur = This.getBWDAxes().GetYAxis();     //   Vector3 Z_cur(This.getAxes().GetZAxis());
        X_pre = Previous.getBWDAxes().GetXAxis();
        Y_pre = Previous.getBWDAxes().GetYAxis();
        Z_pre = Previous.getBWDAxes().GetZAxis();
    }
    
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
        if(Type == 1){
            This.setFWDAxes(axes);
        }
        else{
            This.setBWDAxes(axes);
        }
    }
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

void Fabrik::Rotation_Constraint(Point &This, Point Previous, Axes PprevAxes){
// Assuming the rotational constraint only applies on X axis, in another word, relation between two linked bones.
    float Bound1 = 60;
    float Bound2 = 30;
    float Bound3 = 60;
    float Bound4 = 30;
    float theta1,theta2;
    // the BASE COORDINATE of PREVIOUS JOINT
    Vector3 AxisX(PprevAxes.GetXAxis());
    Vector3 AxisY(PprevAxes.GetYAxis());
    Vector3 AxisZ(PprevAxes.GetZAxis());
    Position Origin = Previous.getPosition();       //Set PrevPoint as Origin
    Position bone(This.getPosition() - Origin);
    Vector3 BONE(bone.getX(),bone.getY(),bone.getZ());
    //Project BONE on XYZ axes.
    float x = Vector3::Dot(BONE, AxisX);
    float y = Vector3::Dot(BONE, AxisY);
    float z = Vector3::Dot(BONE, AxisZ);
    //Project BONE into X-Y & X-Z planes.
    Position Pxy(x,y,0);
    Position Pxz(x,0,z);
    //  theta can't be 90, othewise the tangent value would be infinite.
   
    if(x){
        theta1 = atanf(y/x);   //Rot around Y
        theta2 = atanf(z/x);    //Rot around Z
    }
    else{
        theta1 = 90;
        theta2 = 90;
    }
    if (theta1 > Bound1)
        theta1 = Bound1;
    if (theta1 < Bound2)
        theta1 = Bound2;
    if (theta2 > Bound3)
        theta2 = Bound3;
    if (theta2 < Bound4)
        theta2 = Bound4;
    
    
    float Scalar = bone.getDistance();
    Vector3 tmp0 = AxisX * Scalar;
    
    //RotRoundZ theta1
    float S1 = sin(theta1/2);
    Quaternion RotRoundZ(theta1/2, S1*AxisZ.getX(), S1*AxisZ.getY(), S1*AxisZ.getZ());
    //RotRoundY theta2
    float S2 = sin(theta2/2);
    Quaternion RotRoundY(theta2/2, S2*AxisY.getX(), S2*AxisY.getY(), S2*AxisY.getZ());
 
    Vector3 tmp1 = Quaternion::rotVbyQ(tmp0, RotRoundY);
    Vector3 target = Quaternion::rotVbyQ(tmp1, RotRoundZ);
    Position tar(target.getX(),target.getY(),target.getZ());
    
    
    
    
    //P’i =di (new_P’i - Pi+1)/|| (new_P’i - Pi+1) ||
    Position dist = tar - Previous.getPosition();
    float len = sqrt(pow(dist.getX(), 2) + pow(dist.getY(), 2) + pow(dist.getZ(), 2));
    tar = dist*(10/len); 
    This.setPosition(tar+Previous.getPosition());
    
  }

// Uses the FABRIK algorithm to compute IK.






void Fabrik::compute() {
    int n = 4;
    Point p0 = joints[0];
    Point p1 = joints[1];
    Point p2 = joints[2];
    Point p3 = joints[3];
    Point t = goal;
    float r[3];
    float lambda[3];
    float dist;
    dist = (p0.getPosition() - goal.getPosition()).getDistance();
    int i;
    float dmax;
    float difA;
    Position b;
    for (i = 0; i<3-1; i++ ){
        dmax += d[i];
    }
    //UNEACHABLE
    if (dist > dmax){
        for(i = 0; i<n-1;i++){
            r[i] = (joints[i].getPosition()-goal.getPosition()).getDistance();
            lambda[i] = d[i]/r[i];
            joints[i+1].setPosition((1 - lambda[i]) * joints[i].getPosition() + lambda[i]*goal.getPosition());
        }
    }
    //REACHABLE
    else{
        b = joints[0].getPosition();
        Position distA(joints[n-1].getPosition()-goal.getPosition());
        difA = distA.getDistance();
        
        while (difA > tol)
        {
            //   STAGE1 FORWARD REACHING
            joints[n-1].setPosition(goal.getPosition());
            for(i = n-2; i>=0; i--){
                r[i] = (joints[i+1].getPosition() - joints[i].getPosition()).getDistance();
                lambda[i] = d[i]/r[i];
                Position P = ((1-lambda[i]) * joints[i+1].getPosition() + lambda[i] * joints[i].getPosition());
                joints[i].setPosition(P);
            }
            // STAGE2 BACKWARD REACHING
            joints[0].setPosition(b);
            for(i = 0; i<n-2; i++){
                r[i] = (joints[i].getPosition() - joints[i+1].getPosition()).getDistance();
                lambda[i] = d[i]/r[i];
                Position P = (1-lambda[i])*joints[i].getPosition() + lambda[i]* joints[i+1].getPosition();
                joints[i+1].setPosition(P);
            }
            difA = (joints[n-1].getPosition() - goal.getPosition()).getDistance();
        }
    }
    shrinkEnd();
    GenBones();
    GenQW();
    GenQL();
    GenEuler();
    
}

void Fabrik::GenBones(){
    Position b[3];
    int i = 0;
    for(i = 0; i< 3; i++)
    {
        b[i] = joints[i+1].getPosition()-joints[i].getPosition();
        bone[i] = Vector3(b[i].getX(), b[i].getY(), b[i].getZ());
    }
}
void Fabrik::GenQW() {
    
    int n = 3; //number of joints  (Shoulder Elbow Wrist)

    for(int i = 0; i<n; i++)
    {
        QW[i] = Quaternion::v2q(Vector3(1,0,0), Vector3::Normalize(bone[i]));
    }
}
void Fabrik::GenQL() {
    int n = 3; //number of joints  (Shoulder Elbow Wrist)
    QL[0] = QW[0];
    for(int i = 1; i<n; i++)
    {
        Quaternion invQParent = Quaternion::conjugate(QW[i-1]);
        QL[i] = invQParent * QW[i];
    }
    
}
void Fabrik::GenEuler(){
    for(int i=0; i<3; i++){
        Euler[i] = Quaternion::Quat2Angle(QL[i]) * 57.3;
    }
}
    