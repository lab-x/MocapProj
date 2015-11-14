//
//  MemsNode.cpp
//  InverseKinematics
//
//  Created by MengTsao on 13/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "MemsNode.h"
MemsNode::MemsNode(int id, int parentId){
    ID  = id;
    PID = parentId;
    isCali = 0;
    Qraw   = Quaternion();
    Traw   = Quaternion();
    Araw   = Quaternion();
    Qinit  = Quaternion();
    QCali  = Quaternion();
    QW = Quaternion();
    QL = Quaternion();
    Euler = Vector3(0,0,0);
}
int MemsNode::getPID(){
    return this->PID;
}


int MemsNode::getCaliFlag(){
    return this->isCali;
}
void MemsNode::setCaliFlag(int flag){
    this->isCali = flag;
}

Quaternion MemsNode::getQraw(){
    return this->Qraw;
}
void MemsNode::setQraw(Quaternion raw){
    this->Qraw = raw;
}
Quaternion MemsNode::getQT(){
    return this->Traw;
}
void MemsNode::setQT(Quaternion qT){
    this->Traw = qT;
}
Quaternion MemsNode::getQA(){
    return this->Araw;
}
void MemsNode::setQA(Quaternion qA){
    this->Araw = qA;
}

Quaternion MemsNode::getQinit(){
    return this->Qinit;
}
void MemsNode::setQinit(Quaternion init){
    this->Qinit = init;
}
Quaternion MemsNode::getQCali(){
    return this->QCali;
}
void MemsNode::setQCali(Quaternion cali){
    this->QCali = cali;
}
Quaternion MemsNode::getQW(){
    return (this->QW);
}
void MemsNode::setQW(Quaternion qw){
    this->QW = qw;
}
Quaternion MemsNode::getQL(){
    return (this->QL);
}
void MemsNode::setQL(Quaternion ql){
    this->QL = ql;
}
Vector3 MemsNode::getEuler(){
    return (this->Euler);
}
void MemsNode::setEuler(Vector3 angles){
    this->Euler = angles;
}