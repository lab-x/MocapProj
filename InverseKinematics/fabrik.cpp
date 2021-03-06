//
//  fabrik.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "fabrik.h"


Fabrik::Fabrik(float tolerance, float eps) {
    tol = tolerance;
    epsilon = eps; //Error for out of bounds correction
}

void Fabrik::setGoal(Joint x) {
    goal = x;
}

void Fabrik::setJoints(Joint one, Joint two, Joint three, Joint four) {
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
        Position b = joints[i+1].getPosition() - joints[i].getPosition();
        initialBone[i] = Vector3 (b.getX(), b.getY(), b.getZ()) ;
        bone[i] = initialBone[i] ;

        bone[i] = Vector3(1, 0, 0);
        QW[i] = Quaternion(1,0,0,0);
        QL[i] = Quaternion(1,0,0,0);
        Euler[i] = Vector3(0,0,0);
    }

}

Joint* Fabrik::getJoints() {
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
        Position vector(goal.getPosition() - joints[2].getPosition());
        vector.normalize();
        double factor = dExact / vector.getDistance();
        joints[3].setPosition(joints[2].getPosition()+vector*factor);
        //joints[4] = joints[3] + (vector * factor);
        goal = joints[3];
    }
}

void Fabrik::SetOrientation(Joint &This, Joint Previous, int Type){
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

void Fabrik::Orientation_Constraint(Joint &This, Joint Previous, int Type){
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

//Use previous Joint's orientation to build up position constraint area and cut a segment with proper length
//Position Constraint --> get joint[i]  //find the line(L) passing through Pi+1 and Pi
//fitch the segment on L that satisfied the distance constraint (expressed by di)
/* --------- --------- ---------
 REMAINING PROBLEMS:
 HOW TO PASS BOUNDARY VALUE INTO THIS FUNC?
 ARE THE AXES VALUE SET INTO Joint 'THIS'?
 MAY angleY angleZ BE THE SAME?
 SELECT BOUNDARY TO DECIDE q1?
 Quaternion Rotor(q1 * invRotor);//MAY BE WRONG
 --------- --------- ---------*/

void Fabrik::Rotation_Constraint(Joint &This, Joint Previous, Axes PprevAxes){
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
    Position Origin = Previous.getPosition();       //Set PrevJoint as Origin
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
void Fabrik::compute(int LinkNo) {
    int n = 4;
    Joint p0 = joints[0];
    Joint p1 = joints[1];
    Joint p2 = joints[2];
    Joint p3 = joints[3];
    Joint t = goal;
    float r[3];
    float lambda[3];
    float dist;
    dist = (p0.getPosition() - goal.getPosition()).getDistance();
    int i;
    float dmax;
    float difA;
    Position b;
    for (i = 0; i<n-1; i++ ){
        dmax += d[i];
    }
    //UNREACHABLE
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
    int isNan = 0;
    for (int i = 0; i < 4; i++) {
     if( joints[i].getPosition().getX() != joints[i].getPosition().getX()){
         isNan = 1;
         break;
     }
    }
    if(isNan){
        printf("Nan\n");
        Axes AxesInit;
        Position p0 (0,0.4,0);
        Position p1 (0.2,0.4,0);
        Position p2 (0.4,0.4,0);
        Position p3 (0.6,0.4,0);
        Joint P0(p0, AxesInit, AxesInit);
        Joint P1(p1, AxesInit, AxesInit);
        Joint P2(p2, AxesInit, AxesInit);
        Joint P3(p3, AxesInit, AxesInit);
        setJoints(P0,P1,P2,P3);
        Fabrik::compute(LinkNo);
    }
    else{
        isNan = 0;
        //shrinkEnd();
        GenBones(LinkNo);
        GenQW(LinkNo);
        GenQL(LinkNo);
        GenEuler(LinkNo);
    }
}

//This -->Joints[i] thisID-->i Dir-->BWD/FWD  LinkNo-->(1:4);
void Fabrik::Position_Constraints(Joint &This, int thisID, int Dir, int LinkNo){
    Position P[3];
    Vector3 Bones[2];
    int i;
    if(Dir == 0){
    //fwd  use joints[i+2],[i+1].[i]
        for(i =0; i<3;i++){
            P[i] =joints[thisID + i].getPosition();
        }
    }
    else{
    //bwd   use joints[i],[i-1],[i-2]
        for(i =0; i<3;i++){
            P[i] =joints[thisID + 2 - i].getPosition();
        }
    }
    // Generate 2 Bones' Vector
    for(i = 0; i< 2; i++){
        Position b = P[i+1] - P[i];
        Bones[i] = Vector3(b.getX(), b.getY(), b.getZ());
    }
    
    Quaternion Q = Quaternion::v2q(Vector3::Normalize(bone[0]), Vector3::Normalize(bone[1]));
    
    /*
    //Generate 2 bone vectors' world quaternion
    for(int i = 0; i<2; i++){
        Vector3 I = Vector3::Normalize(initialBone[i]) ;
        QW[i] = Quaternion::v2q(I, Vector3::Normalize(bone[i]));
    }
    Quaternion invQParent = Quaternion::conjugate(QW[0]);
    Quaternion QL = invQParent * QW[1];
    Vector3 Euler = Quaternion::Quat2Angle(QL) * 57.3;
    */

    if(Dir == 0){
        //fwd  use joints[i+2],[i+1].[i]
//        Set P[0] set[i]
    }
    else{
        //bwd   use joints[i],[i-1],[i-2]
//        Set P[2]  set[i]
    }
}


void Fabrik::GenBones(int LinkNo){
    Position b[3];
    int i = 0;
    for(i = 0; i< 3; i++)
    {
        b[i] = joints[i+1].getPosition()-joints[i].getPosition();
        bone[i] = Vector3(b[i].getX(), b[i].getY(), b[i].getZ());
    }
}
void Fabrik::GenQW(int LinkNo) {
    
    int n = 3; //number of joints  (Shoulder Elbow Wrist)

    for(int i = 0; i<n; i++){
//        QW[i] = Quaternion::v2q(Vector3(1,0,0), Vector3::Normalize(bone[i]));
        QW[i] = Quaternion::v2q(Vector3::Normalize(bone[LinkNo]), Vector3::Normalize(bone[i]));

    }
}
void Fabrik::GenQL(int LinkNo) {
    int n = 3; //number of joints  (Shoulder Elbow Wrist)
    QL[0] = QW[0];
    for(int i = 1; i<n; i++){
        Quaternion invQParent = Quaternion::conjugate(QW[i-1]);
        QL[i] = invQParent * QW[i];
    }
    
}
void Fabrik::GenEuler(int LinkNo){
    for(int i=0; i<3; i++){
        Euler[i] = Quaternion::Quat2Angle(QL[i]) * 57.3;
    }
}
    