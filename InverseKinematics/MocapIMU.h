//
//  MocapIMU.h
//  InverseKinematics
//
//  Created by MengTsao on 12/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__MocapIMU__
#define __InverseKinematics__MocapIMU__

#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include "quaternion.h"
#include "vector3.h"
#include "position.h"
#include "MemsNode.h"

class MocapIMU {
    MemsNode Nodes[17];
    Quaternion QW[17];
    Quaternion QL[17];
    Vector3 EulerOut[17];
    Position HipPosition;
    
public:
    MocapIMU();
    void calibrate();
    void compute();
    void genQW();
    void genQL();
    void genEuler();
    bool CalibrationDone();
};
#endif /* defined(__InverseKinematics__MocapIMU__) */
