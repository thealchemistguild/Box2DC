//
//  Box2DC.c
//  Box2DC
//
//  Created by Ryan Joseph on 5/15/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#include "Box2DC.h"
#include "Box2D.h"
#include "WorldCallbackCenter.h"
#include "DebugDraw.h"
#include "StructConverters.h"

#include <memory.h>
#include <string.h>
#include <stdio.h>

EXTERNAL_FUNCTIONS_BEGIN
    
#pragma mark World Manifold

void Box2DWorldManifoldInitialize(Box2DWorldManifold &worldManifold, Box2DManifold manifold, Box2DTransform xfA, float32 radiusA, Box2DTransform xfB, float32 radiusB) {
    b2WorldManifold _worldManifold = Box2DWorldManifoldMake(worldManifold);
    b2Manifold _manifold = Box2DManifoldMake(manifold);
    _worldManifold.Initialize(&_manifold, Box2DTransformMake(xfA), radiusA, Box2DTransformMake(xfB), radiusB);
    worldManifold = Box2DWorldManifoldMake(_worldManifold);
}

#pragma mark Vector2

void Box2DVectorSetZero(Box2DVector2 &vec) {
    vec.x = 0.0f;
    vec.y = 0.0f;
}

void Box2DVectorSet(Box2DVector2 &vec, float32 x_, float32 y_) {
    vec.x = x_;
    vec.y = y_;
}

float32 Box2DVectorLength(Box2DVector2 vec) {
    return b2Sqrt(vec.x * vec.x + vec.y * vec.y);
}

float32 Box2DVectorLengthSquared(Box2DVector2 vec) {
    return vec.x * vec.x + vec.y * vec.y;
}

float32 Box2DVectorNormalize(Box2DVector2 &vec) {
    float32 length = Box2DVectorLength(vec);
    if (length < b2_epsilon) {
        return 0.0f;
    }
    float32 invLength = 1.0f / length;
    vec.x *= invLength;
    vec.y *= invLength;
    
    return length;
}

bool Box2DVectorIsValid(Box2DVector2 vec) {
    return b2IsValid(vec.x) && b2IsValid(vec.y);
}

Box2DVector2 Box2DVectorSkew(Box2DVector2 vec) {
    return Box2DVector2Make(b2Vec2(-vec.y, vec.x));
}

Box2DVector2 Box2DVectorNegate (Box2DVector2 vec) {
    b2Vec2 v;
    v.Set(-vec.x, -vec.y);
    return Box2DVector2Make(v);
}

Box2DVector2 Box2DVectorAddVector (Box2DVector2 vec, Box2DVector2 add) {
    vec.x += add.x;
    vec.y += add.y;
    return vec;
}

Box2DVector2 Box2DVectorSubtractVector (Box2DVector2 vec, Box2DVector2 subtract) {
    vec.x -= subtract.x;
    vec.y -= subtract.y;
    return vec;
}

Box2DVector2 Box2DVectorMultiply (Box2DVector2 vec, float32 a) {
    vec.x *= a;
    vec.y *= a;
    return vec;
}

#pragma mark Transform
    
Box2DTransform Box2DTransformMake (Box2DVector2 position, Box2DRotation rotation) {
    b2Transform out = b2Transform(Box2DVector2Make(position), Box2DRotationMake(rotation));
    return Box2DTransformMake(out);
}

void Box2DTransformSetIdentity(Box2DTransform &transform) {
    b2Transform _transform = Box2DTransformMake(transform);
    _transform.SetIdentity();
    transform = Box2DTransformMake(_transform);
}

void Box2DTransformSet(Box2DTransform &transform, Box2DVector2 position, float32 angle) {
    b2Transform _transform = Box2DTransformMake(transform);
    _transform.Set(Box2DVector2Make(position), angle);
    transform = Box2DTransformMake(_transform);
}

#pragma mark Rotation
    
Box2DRotation Box2DRotationMake (float32 angle) {
    b2Rot out = b2Rot(angle);
    return Box2DRotationMake(out);
}

void Box2DRotationSet(Box2DRotation &rotation, float32 angle) {
    b2Rot _rotation = Box2DRotationMake(rotation);
    _rotation.Set(angle);
    rotation = Box2DRotationMake(_rotation);
}

void Box2DRotationSetIdentity(Box2DRotation &rotation) {
    b2Rot _rotation = Box2DRotationMake(rotation);
    _rotation.SetIdentity();
    rotation = Box2DRotationMake(_rotation);
}

float32 Box2DRotationGetAngle(Box2DRotation rotation) {
    b2Rot _rotation = Box2DRotationMake(rotation);
    return _rotation.GetAngle();
}

Box2DVector2 Box2DRotationGetXAxis(Box2DRotation rotation) {
    b2Rot _rotation = Box2DRotationMake(rotation);
    return Box2DVector2Make(_rotation.GetXAxis());
}

Box2DVector2 Box2DRotationGetYAxis(Box2DRotation rotation) {
    b2Rot _rotation = Box2DRotationMake(rotation);
    return Box2DVector2Make(_rotation.GetYAxis());
}

#pragma mark AABB

bool Box2DAABBIsValid(Box2DAABB aabb) {
    return Box2DAABBMake(aabb).IsValid();
}

Box2DVector2 Box2DAABBGetCenter(Box2DAABB aabb) {
    return Box2DVector2Make(Box2DAABBMake(aabb).GetCenter());
}

Box2DVector2 Box2DAABBGetExtents(Box2DAABB aabb) {
    return Box2DVector2Make(Box2DAABBMake(aabb).GetExtents());
}

float32 Box2DAABBGetPerimeter(Box2DAABB aabb) {
    return Box2DAABBMake(aabb).GetPerimeter();
}

void Box2DAABBCombine(Box2DAABB &ioAABB, Box2DAABB aabb) {
    b2AABB out = Box2DAABBMake(ioAABB);
    out.Combine(Box2DAABBMake(aabb));
    ioAABB = Box2DAABBMake(out);
}

void Box2DAABBCombine2(Box2DAABB &ioAABB, Box2DAABB aabb1, Box2DAABB aabb2) {
    b2AABB out = Box2DAABBMake(ioAABB);
    out.Combine(Box2DAABBMake(aabb1), Box2DAABBMake(aabb2));
    ioAABB = Box2DAABBMake(out);
}

bool Box2DAABBContains(Box2DAABB thisAABB, Box2DAABB aabb) {
    return Box2DAABBMake(thisAABB).Contains(Box2DAABBMake(aabb));
}

bool Box2DAABBRayCast(Box2DAABB aabb, Box2DRayCastOutput &output, Box2DRayCastInput input) {
    b2RayCastOutput _output = Box2DRayCastOutputMake(output);
    return Box2DAABBMake(aabb).RayCast(&_output, Box2DRayCastInputMake(input));
    output = Box2DRayCastOutputMake(_output);
}

#pragma mark Body Definition

Box2DBodyDefinition Box2DBodyDefinitionDefault () {
    return Box2DBodyDefinitionMake(b2BodyDef());
}

#pragma mark Fixture Definition

Box2DFixtureDefinition Box2DFixtureDefinitionDefault () {
    return Box2DFixtureDefinitionMake(b2FixtureDef());
}

#pragma mark Joint Definitions

#pragma mark Distance Joint Definition

Box2DDistanceJointDefinition Box2DDistanceJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2DistanceJointDef());
}

void Box2DDistanceJointDefinitionInitialize(Box2DDistanceJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchorA, Box2DVector2 anchorB) {
    b2DistanceJointDef _def = Box2DJointDefinitionMake(def);
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchorA), Box2DVector2Make(anchorB));
    def = Box2DJointDefinitionMake(_def);
}

#pragma mark Friction Joint Definition

Box2DFrictionJointDefinition Box2DFrictionJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2FrictionJointDef());
}

void Box2DFrictionJointDefinitionInitialize(Box2DFrictionJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor) {
    b2FrictionJointDef _def = Box2DJointDefinitionMake(def);
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchor));
    def = Box2DJointDefinitionMake(_def);
}

#pragma mark Motor Joint Definition

Box2DMotorJointDefinition Box2DMotorJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2MotorJointDef());
}

void Box2DMotorJointDefinitionInitialize(Box2DMotorJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB) {
    b2MotorJointDef _def = Box2DJointDefinitionMake(def);
    _def.Initialize(bodyA, bodyB);
    def = Box2DJointDefinitionMake(_def);
}


#pragma mark Revolute Joint Definition

Box2DRevoluteJointDefinition Box2DRevoluteJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2RevoluteJointDef());
}

void Box2DRevoluteJointDefinitionInitialize (Box2DRevoluteJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor) {
    b2RevoluteJointDef _def = Box2DJointDefinitionMake(def);
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchor));
    def = Box2DJointDefinitionMake(_def);
}


#pragma mark Pulley Joint Definition

Box2DPulleyJointDefinition Box2DPulleyJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2PulleyJointDef());
}

void Box2DPulleyJointDefinitionInitialize (Box2DPulleyJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB,
                                           Box2DVector2 groundAnchorA, Box2DVector2 groundAnchorB,
                                           Box2DVector2 anchorA, Box2DVector2 anchorB,
                                           float32 ratio) {
    b2PulleyJointDef _def = Box2DJointDefinitionMake(def);
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(groundAnchorA), Box2DVector2Make(groundAnchorB), Box2DVector2Make(anchorA), Box2DVector2Make(anchorB), ratio);
    def = Box2DJointDefinitionMake(_def);

}

#pragma mark Prismatic Joint Definition

Box2DPrismaticJointDefinition Box2DPrismaticJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2PrismaticJointDef());
}

void Box2DPrismaticJointDefinitionInitialize(Box2DPrismaticJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor, Box2DVector2 axis) {
    b2PrismaticJointDef _def;
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchor), Box2DVector2Make(axis));
    def = Box2DJointDefinitionMake(_def);
}

# pragma mark Weld Joint Definition
        
Box2DWeldJointDefinition Box2DWeldJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2WeldJointDef());
}

void Box2DWeldJointDefinitionInitialize (Box2DWeldJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor) {
    b2WeldJointDef _def;
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchor));
    def = Box2DJointDefinitionMake(_def);
}

# pragma mark Wheel Joint Definition

Box2DWheelJointDefinition Box2DWheelJointDefinitionDefault () {
    return Box2DJointDefinitionMake(b2WheelJointDef());
}

void Box2DWheelJointDefinitionInitialize(Box2DWheelJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor, Box2DVector2 axis) {
    b2WheelJointDef _def;
    _def.Initialize(bodyA, bodyB, Box2DVector2Make(anchor), Box2DVector2Make(axis));
    def = Box2DJointDefinitionMake(_def);
}

#pragma mark Joints

#pragma mark Joint

Box2DJointType Box2DJointGetType(Box2DJoint joint) {
    return joint->GetType();
}

Box2DBody Box2DJointGetBodyA(Box2DJoint joint) {
    return joint->GetBodyA();
}

Box2DBody Box2DJointGetBodyB(Box2DJoint joint) {
    return joint->GetBodyB();
}
    
Box2DJoint Box2DJointGetNext(Box2DJoint joint) {
    return joint->GetNext();
}
    
void* Box2DJointGetUserData(Box2DJoint joint) {
    return joint->GetUserData();
}

void Box2DJointSetUserData(Box2DJoint joint, void* data) {
    joint->SetUserData(data);
}

bool Box2DJointIsActive(Box2DJoint joint) {
    return joint->IsActive();
}

bool Box2DJointGetCollideConnected(Box2DJoint joint) {
    return joint->GetCollideConnected();
}

VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetAnchorA(Box2DJoint joint) {
    return Box2DVector2Make(joint->GetAnchorA());
}

VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetAnchorB(Box2DJoint joint) {
    return Box2DVector2Make(joint->GetAnchorB());
}

VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetReactionForce(Box2DJoint joint, float32 inv_dt) {
    return Box2DVector2Make(joint->GetReactionForce(inv_dt));
}

VIRTUAL_FUNCTION float32 Box2DJointGetReactionTorque(Box2DJoint joint, float32 inv_dt) {
    return joint->GetReactionTorque(inv_dt);
}

VIRTUAL_FUNCTION void Box2DJointDump(Box2DJoint joint) {
    joint->Dump();
}

VIRTUAL_FUNCTION void Box2DJointShiftOrigin(Box2DJoint joint, Box2DVector2 newOrigin) {
    joint->ShiftOrigin(Box2DVector2Make(newOrigin));
}


#pragma mark Distance Joint

Box2DVector2 Box2DDistanceJointGetReactionForce(Box2DDistanceJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DDistanceJointGetReactionTorque(Box2DDistanceJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DDistanceJointGetLocalAnchorA(Box2DDistanceJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DDistanceJointGetLocalAnchorB(Box2DDistanceJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorB());
}

void Box2DDistanceJointSetLength(Box2DDistanceJoint joint, float32 length) {
    joint->SetLength(length);
}

float32 Box2DDistanceJointGetLength(Box2DDistanceJoint joint) {
    return joint->GetLength();
}

void Box2DDistanceJointSetFrequency(Box2DDistanceJoint joint, float32 hz) {
    joint->SetFrequency(hz);
}

float32 Box2DDistanceJointGetFrequency(Box2DDistanceJoint joint) {
    return joint->GetFrequency();
}

void Box2DDistanceJointSetDampingRatio(Box2DDistanceJoint joint, float32 ratio) {
    joint->SetDampingRatio(ratio);
}

float32 Box2DDistanceJointGetDampingRatio(Box2DDistanceJoint joint) {
    return joint->GetDampingRatio();
}

void Box2DDistanceJointDump(Box2DDistanceJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Friction Joint

Box2DVector2 Box2DFrictionJointGetAnchorA(Box2DFrictionJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DFrictionJointGetAnchorB(Box2DFrictionJoint joint) {
    return Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DFrictionJointGetReactionForce(Box2DFrictionJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DFrictionJointGetReactionTorque(Box2DFrictionJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DFrictionJointGetLocalAnchorA(Box2DFrictionJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DFrictionJointGetLocalAnchorB(Box2DFrictionJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorB());
}

void Box2DFrictionJointSetMaxForce(Box2DFrictionJoint joint, float32 force) {
    joint->SetMaxForce(force);
}

float32 Box2DFrictionJointGetMaxForce(Box2DFrictionJoint joint) {
    return joint->GetMaxForce();
}

void Box2DFrictionJointSetMaxTorque(Box2DFrictionJoint joint, float32 torque) {
    joint->SetMaxTorque(torque);
}

float32 Box2DFrictionJointGetMaxTorque(Box2DFrictionJoint joint) {
    return joint->GetMaxTorque();
}

void Box2DFrictionJointDump(Box2DDistanceJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Gear Joint

Box2DVector2 Box2DGearJointGetAnchorA(Box2DGearJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DGearJointGetAnchorB(Box2DGearJoint joint) {
    return Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DGearJointGetReactionForce(Box2DGearJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DGearJointGetReactionTorque(Box2DGearJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DJoint Box2DGearJointGetJoint1(Box2DGearJoint joint) {
    return joint->GetJoint1();
}

Box2DJoint Box2DGearJointGetJoint2(Box2DGearJoint joint) {
    return joint->GetJoint2();
}

void Box2DGearJointSetRatio(Box2DGearJoint joint, float32 ratio) {
    joint->SetRatio(ratio);
}

float32 Box2DGearJointGetRatio(Box2DGearJoint joint) {
    return joint->GetRatio();
}

void Box2DGearJointDump(Box2DGearJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Motor Joint

Box2DVector2 Box2DMotorJointGetAnchorA(Box2DMotorJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DMotorJointGetAnchorB(Box2DMotorJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DMotorJointGetReactionForce(Box2DMotorJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DMotorJointGetReactionTorque(Box2DMotorJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

void Box2DMotorJointSetLinearOffset(Box2DMotorJoint joint, Box2DVector2 linearOffset) {
    joint->SetLinearOffset(Box2DVector2Make(linearOffset));
}

Box2DVector2 Box2DMotorJointGetLinearOffset(Box2DMotorJoint joint) {
    return Box2DVector2Make(joint->GetLinearOffset());
}

void Box2DMotorJointSetAngularOffset(Box2DMotorJoint joint, float32 angularOffset) {
    joint->SetAngularOffset(angularOffset);
}

float32 Box2DMotorJointGetAngularOffset(Box2DMotorJoint joint) {
    return joint->GetAngularOffset();
}

void Box2DMotorJointSetMaxForce(Box2DMotorJoint joint, float32 force) {
    joint->SetMaxForce(force);
}

float32 Box2DMotorJointGetMaxForce(Box2DMotorJoint joint) {
    return joint->GetMaxForce();
}

void Box2DMotorJointSetMaxTorque(Box2DMotorJoint joint, float32 torque) {
    joint->SetMaxTorque(torque);
}

float32 Box2DMotorJointGetMaxTorque(Box2DFrictionJoint joint) {
    return joint->GetMaxTorque();
}

void Box2DMotorJointDump(Box2DMotorJoint joint) {
    Box2DJointDump(joint);
}   

#pragma mark Mouse Joint

Box2DVector2 Box2DMouseJointGetAnchorA(Box2DMouseJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DMouseJointGetAnchorB(Box2DMouseJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DMouseJointGetReactionForce(Box2DMouseJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DMouseJointGetReactionTorque(Box2DMouseJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

void Box2DMouseJointSetTarget(Box2DMouseJoint joint, const Box2DVector2 target) {
    joint->SetTarget(Box2DVector2Make(target));
}

Box2DVector2 Box2DMouseJointGetTarget(Box2DMouseJoint joint) {
    return Box2DVector2Make(joint->GetTarget());
}

void Box2DMouseJointSetMaxForce(Box2DMouseJoint joint, float32 force) {
    joint->SetMaxForce(force);
}

float32 Box2DMouseJointGetMaxForce(Box2DMouseJoint joint) {
    return joint->GetMaxForce();
}

void Box2DMouseJointSetFrequency(Box2DMouseJoint joint, float32 hz) {
    joint->SetFrequency(hz);
}

float32 Box2DMouseJointGetFrequency(Box2DMouseJoint joint) {
    return joint->GetFrequency();
}

void Box2DMouseJointSetDampingRatio(Box2DMouseJoint joint, float32 ratio) {
    joint->SetDampingRatio(ratio);
}

float32 Box2DMouseJointGetDampingRatio(Box2DMouseJoint joint) {
    return joint->GetDampingRatio();
}

void Box2DMouseJointDump(Box2DMouseJoint joint) {
    Box2DJointDump(joint);
}

void Box2DMouseJointShiftOrigin(Box2DMouseJoint joint, Box2DVector2 newOrigin) {
    Box2DJointShiftOrigin(joint, newOrigin);
}

#pragma mark Prismatic Joint

Box2DVector2 Box2DPrismaticJointGetAnchorA(Box2DPrismaticJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DPrismaticJointGetAnchorB(Box2DPrismaticJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DPrismaticJointGetReactionForce(Box2DPrismaticJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DPrismaticJointGetReactionTorque(Box2DPrismaticJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DPrismaticJointGetLocalAnchorA(Box2DPrismaticJoint joint) {
    return Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DPrismaticJointGetLocalAnchorB(Box2DPrismaticJoint joint) {
    return Box2DVector2Make(joint->GetLocalAnchorB());
}

Box2DVector2 Box2DPrismaticJointGetLocalAxisA(Box2DPrismaticJoint joint) {
    return Box2DVector2Make(joint->GetLocalAxisA());
}

float32 Box2DPrismaticJointGetReferenceAngle(Box2DPrismaticJoint joint) {
    return joint->GetReferenceAngle();
}

float32 Box2DPrismaticJointGetJointTranslation(Box2DPrismaticJoint joint) {
    return joint->GetJointTranslation();
}

float32 Box2DPrismaticJointGetJointSpeed(Box2DPrismaticJoint joint) {
    return joint->GetJointSpeed();
}

bool Box2DPrismaticJointIsLimitEnabled(Box2DPrismaticJoint joint) {
    return joint->IsLimitEnabled();
}

void Box2DPrismaticJointEnableLimit(Box2DPrismaticJoint joint, bool flag) {
    joint->EnableLimit(flag);
}

float32 Box2DPrismaticJointGetLowerLimit(Box2DPrismaticJoint joint) {
    return joint->GetLowerLimit();
}

float32 Box2DPrismaticJointGetUpperLimit(Box2DPrismaticJoint joint) {
    return joint->GetUpperLimit();
}

void Box2DPrismaticJointSetLimits(Box2DPrismaticJoint joint, float32 lower, float32 upper) {
    joint->SetLimits(lower, upper);
}

bool Box2DPrismaticJointIsMotorEnabled(Box2DPrismaticJoint joint) {
    return joint->IsMotorEnabled();
}

void Box2DPrismaticJointEnableMotor(Box2DPrismaticJoint joint, bool flag) {
    joint->EnableMotor(flag);
}

void Box2DPrismaticJointSetMotorSpeed(Box2DPrismaticJoint joint, float32 speed) {
    joint->SetMotorSpeed(speed);
}

float32 GBox2DPrismaticJointGetMotorSpeed(Box2DPrismaticJoint joint) {
    return joint->GetMotorSpeed();
}

void Box2DPrismaticJointSetMaxMotorForce(Box2DPrismaticJoint joint, float32 force) {
    joint->SetMaxMotorForce(force);
}

float32 Box2DPrismaticJointGetMaxMotorForce(Box2DPrismaticJoint joint) {
    return joint->GetMaxMotorForce();
}

float32 Box2DPrismaticJointGetMotorForce(Box2DPrismaticJoint joint, float32 inv_dt) {
    return joint->GetMotorForce(inv_dt);
}

void Box2DPrismaticJointDump(Box2DPrismaticJoint joint) {
    Box2DJointDump(joint);
}


#pragma mark Pulley Joint

Box2DVector2 Box2DPulleyJointGetAnchorA(Box2DPulleyJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DPulleyJointGetAnchorB(Box2DPulleyJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DPulleyJointGetReactionForce(Box2DPulleyJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DPulleyJointGetReactionTorque(Box2DPulleyJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DPulleyJointGetGroundAnchorA(Box2DPulleyJoint joint) {
    return Box2DVector2Make(joint->GetGroundAnchorA());
}

Box2DVector2 Box2DPulleyJointGetGroundAnchorB(Box2DPulleyJoint joint) {
    return Box2DVector2Make(joint->GetGroundAnchorB());
}

float32 Box2DPulleyJointGetLengthA(Box2DPulleyJoint joint) {
    return joint->GetLengthA();
}

float32 Box2DPulleyJointGetLengthB(Box2DPulleyJoint joint) {
    return joint->GetLengthB();
}

float32 Box2DPulleyJointGetRatio(Box2DPulleyJoint joint) {
    return joint->GetRatio();
}

float32 Box2DPulleyJointGetCurrentLengthA(Box2DPulleyJoint joint) {
    return joint->GetCurrentLengthA();
}

float32 Box2DPulleyJointGetCurrentLengthB(Box2DPulleyJoint joint) {
    return joint->GetCurrentLengthB();
}

void Box2DPulleyJointDump(Box2DPulleyJoint joint) {
    Box2DJointDump(joint);
}

void Box2DPulleyJointShiftOrigin(Box2DPulleyJoint joint, Box2DVector2 newOrigin) {
    Box2DJointShiftOrigin(joint, newOrigin);
}

#pragma mark Revolute Joint

Box2DVector2 Box2DRevoluteJointGetLocalAnchorA(Box2DRevoluteJoint joint) {
    return Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DRevoluteJointGetLocalAnchorB(Box2DRevoluteJoint joint) {
    return Box2DVector2Make(joint->GetLocalAnchorB());
}

float32 Box2DRevoluteJointGetReferenceAngle(Box2DRevoluteJoint joint) {
    return joint->GetReferenceAngle();
}

float32 Box2DRevoluteJointGetJointAngle(Box2DRevoluteJoint joint) {
    return joint->GetJointAngle();
}

float32 Box2DRevoluteJointGetJointSpeed(Box2DRevoluteJoint joint) {
    return joint->GetJointSpeed();
}

bool Box2DRevoluteJointIsLimitEnabled(Box2DRevoluteJoint joint) {
    return joint->IsLimitEnabled();
}

void Box2DRevoluteJointEnableLimit(Box2DRevoluteJoint joint, bool flag) {
    joint->EnableLimit(flag);
}

float32 Box2DRevoluteJointGetLowerLimit(Box2DRevoluteJoint joint) {
    return joint->GetLowerLimit();
}

float32 Box2DRevoluteJointGetUpperLimit(Box2DRevoluteJoint joint) {
    return joint->GetUpperLimit();
}

void Box2DRevoluteJointSetLimits(Box2DRevoluteJoint joint, float32 lower, float32 upper) {
    joint->SetLimits(lower, upper);
}

bool Box2DRevoluteJointIsMotorEnabled(Box2DRevoluteJoint joint) {
    return joint->IsMotorEnabled();
}

void Box2DRevoluteJointEnableMotor(Box2DRevoluteJoint joint, bool flag) {
    joint->EnableMotor(flag);
}

void Box2DRevoluteJointSetMotorSpeed(Box2DRevoluteJoint joint, float32 speed) {
    joint->SetMotorSpeed(speed);
}

float32 Box2DRevoluteJointGetMotorSpeed(Box2DRevoluteJoint joint) {
    return joint->GetMotorSpeed();
}

void Box2DRevoluteJointSetMaxMotorTorque(Box2DRevoluteJoint joint, float32 torque) {
    joint->SetMaxMotorTorque(torque);
}

float32 Box2DRevoluteJointGetMaxMotorTorque(Box2DRevoluteJoint joint) {
    return joint->GetMaxMotorTorque();
}

Box2DVector2 Box2DRevoluteJointGetReactionForce(Box2DRevoluteJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DRevoluteJointGetReactionTorque(Box2DRevoluteJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

float32 Box2DRevoluteJointGetMotorTorque(Box2DRevoluteJoint joint, float32 inv_dt) {
    return joint->GetMotorTorque(inv_dt);
}

void Box2DRevoluteJointDump(Box2DRevoluteJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Rope Joint

Box2DVector2 Box2DRopeJointGetAnchorA(Box2DRopeJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DRopeJointGetAnchorB(Box2DRopeJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DRopeJointGetReactionForce(Box2DRopeJoint joint, float32 inv_dt) {
    return  Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DRopeJointGetReactionTorque(Box2DRopeJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DRopeJointGetLocalAnchorA(Box2DRopeJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DRopeJointGetLocalAnchorB(Box2DRopeJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorB());
}

void Box2DRopeJointSetMaxLength(Box2DRopeJoint joint, float32 length) {
    joint->SetMaxLength(length);
}

float32 Box2DRopeJointGetMaxLength(Box2DRopeJoint joint) {
    return joint->GetMaxLength();
}

Box2DLimitState Box2DRopeJointGetLimitState(Box2DRopeJoint joint) {
    return joint->GetLimitState();
}

void Box2DRopeJointDump(Box2DRopeJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Weld Joint

Box2DVector2 Box2DWeldJointGetAnchorA(Box2DWeldJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DWeldJointGetAnchorB(Box2DWeldJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DWeldJointGetReactionForce(Box2DWeldJoint joint, float32 inv_dt) {
    return  Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DWeldJointGetReactionTorque(Box2DWeldJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DWeldJointGetLocalAnchorA(Box2DWeldJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DWeldJointGetLocalAnchorB(Box2DWeldJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorB());
}

float32 Box2DWeldJointGetReferenceAngle(Box2DWeldJoint joint) {
    return joint->GetReferenceAngle();
}

void Box2DWeldJointSetFrequency(Box2DWeldJoint joint, float32 hz) {
    joint->SetFrequency(hz);
}

float32 Box2DWeldJointGetFrequency(Box2DWeldJoint joint) {
    return joint->GetFrequency();
}

void Box2DWeldJointSetDampingRatio(Box2DWeldJoint joint, float32 ratio) {
    joint->SetDampingRatio(ratio);
}

float32 Box2DWeldJointGetDampingRatio(Box2DWeldJoint joint) {
    return joint->GetDampingRatio();
}

void Box2DWeldJointDump(Box2DWeldJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Wheel Joint

void Box2DWheelJointGetDefinition(Box2DWheelJoint joint, Box2DWheelJointDefinition &def) {
    TO_DO
    /*
    b2WheelJointDef _def;
    joint->GetDefinition(&_def);
    def = Box2DJointDefinitionMake(_def);
    */
}

Box2DVector2 Box2DWheelJointGetAnchorA(Box2DWheelJoint joint) {
    return Box2DJointGetAnchorA(joint);
}

Box2DVector2 Box2DWheelJointGetAnchorB(Box2DWheelJoint joint) {
    return  Box2DJointGetAnchorB(joint);
}

Box2DVector2 Box2DWheelJointGetReactionForce(Box2DWheelJoint joint, float32 inv_dt) {
    return  Box2DJointGetReactionForce(joint, inv_dt);
}

float32 Box2DWheelJointGetReactionTorque(Box2DWheelJoint joint, float32 inv_dt) {
    return Box2DJointGetReactionTorque(joint, inv_dt);
}

Box2DVector2 Box2DWheelJointGetLocalAnchorA(Box2DWheelJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorA());
}

Box2DVector2 Box2DWheelJointGetLocalAnchorB(Box2DWheelJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAnchorB());
}

Box2DVector2 Box2DWheelJointGetLocalAxisA(Box2DWheelJoint joint) {
    return  Box2DVector2Make(joint->GetLocalAxisA());
}

float32 Box2DWheelJointGetJointTranslation(Box2DWheelJoint joint) {
    return joint->GetJointTranslation();
}

float32 Box2DWheelJointGetJointSpeed(Box2DWheelJoint joint) {
    return joint->GetJointSpeed();
}

bool Box2DWheelJointIsMotorEnabled(Box2DWheelJoint joint) {
    return joint->IsMotorEnabled();
}

void Box2DWheelJointEnableMotor(Box2DWheelJoint joint, bool flag) {
    joint->EnableMotor(flag);
}

void Box2DWheelJointSetMotorSpeed(Box2DWheelJoint joint, float32 speed) {
    joint->SetMotorSpeed(speed);
}

float32 Box2DWheelJointGetMotorSpeed(Box2DWheelJoint joint) {
    return joint->GetMotorSpeed();
}

void Box2DWheelJointSetMaxMotorTorque(Box2DWheelJoint joint, float32 torque) {
    joint->SetMaxMotorTorque(torque);
}

float32 Box2DWheelJointGetMaxMotorTorque(Box2DWheelJoint joint) {
    return joint->GetMaxMotorTorque();
}

float32 Box2DWheelJointGetMotorTorque(Box2DWheelJoint joint, float32 inv_dt) {
    return joint->GetMotorTorque(inv_dt);
}

void Box2DWheelJointSetSpringFrequencyHz(Box2DWheelJoint joint, float32 hz) {
    joint->SetSpringFrequencyHz(hz);
}

float32 Box2DWheelJointGetSpringFrequencyHz(Box2DWheelJoint joint) {
    return joint->GetSpringFrequencyHz();
}

void Box2DWheelJointSetSpringDampingRatio(Box2DWheelJoint joint, float32 ratio) {
    joint->SetSpringDampingRatio(ratio);
}

float32 Box2DWheelJointGetSpringDampingRatio(Box2DWheelJoint joint) {
    return joint->GetSpringDampingRatio();
}

void Box2DWheelJointDump(Box2DWheelJoint joint) {
    Box2DJointDump(joint);
}

#pragma mark Fixture

Box2DShapeType Box2DFixtureGetType(Box2DFixture fixture) {
    return fixture->GetType();
}

Box2DShape Box2DFixtureGetShape(Box2DFixture fixture) {
    return fixture->GetShape();
}
    
void Box2DFixtureSetSensor(Box2DFixture fixture, bool sensor) {
    fixture->SetSensor(sensor);
}

bool Box2DFixtureIsSensor(Box2DFixture fixture) {
    return fixture->IsSensor();
}

void Box2DFixtureSetFilterData(Box2DFixture fixture, Box2DFilter filter) {
    fixture->SetFilterData(Box2DFilterMake(filter));
}

Box2DFilter Box2DFixtureGetFilterData(Box2DFixture fixture) {
    return Box2DFilterMake(fixture->GetFilterData());
}

void Box2DFixtureRefilter(Box2DFixture fixture) {
    fixture->Refilter();
}

Box2DBody Box2DFixtureGetBody(Box2DFixture fixture) {
    return fixture->GetBody();
}
    
Box2DFixture Box2DFixtureGetNext(Box2DFixture fixture) {
    return fixture->GetNext();
}
    
void* Box2DFixtureGetUserData(Box2DFixture fixture) {
    return fixture->GetUserData();
}

void Box2DFixtureSetUserData(Box2DFixture fixture, void* data) {
    fixture->SetUserData(data);
}

bool Box2DFixtureTestPoint(Box2DFixture fixture, Box2DVector2 p) {
    return fixture->TestPoint(Box2DVector2Make(p));
}

bool Box2DFixtureRayCast(Box2DFixture fixture, Box2DRayCastOutput &output, Box2DRayCastInput input, int32 childIndex) {
    b2RayCastOutput _output = Box2DRayCastOutputMake(output);
    return fixture->RayCast(&_output, Box2DRayCastInputMake(input), childIndex);
    output = Box2DRayCastOutputMake(_output);
}

void Box2DFixtureGetMassData(Box2DFixture fixture, Box2DMassData &massData) {
    b2MassData _massData;
    fixture->GetMassData(&_massData);
    massData = Box2DMassDataMake(_massData);
}

void Box2DFixtureSetDensity(Box2DFixture fixture, float32 density) {
    fixture->SetDensity(density);
}

float32 Box2DFixtureGetDensity(Box2DFixture fixture) {
    return fixture->GetDensity();
}

float32 Box2DFixtureGetFriction(Box2DFixture fixture) {
    return fixture->GetFriction();
}

void Box2DFixtureSetFriction(Box2DFixture fixture, float32 friction) {
    fixture->SetFriction(friction);
}

float32 Box2DFixtureGetRestitution(Box2DFixture fixture) {
    return fixture->GetRestitution();
}

void Box2DFixtureSetRestitution(Box2DFixture fixture, float32 restitution) {
    fixture->SetRestitution(restitution);
}

Box2DAABB Box2DFixtureGetAABB(Box2DFixture fixture, int32 childIndex) {
    return Box2DAABBMake(fixture->GetAABB(childIndex));
}

void Box2DFixtureDump(Box2DFixture fixture, int32 bodyIndex) {
    fixture->Dump(bodyIndex);
}

#pragma mark Shape

Box2DShapeType Box2DShapeGetType(Box2DShape shape) {
    return shape->GetType();
}

float32 Box2DShapeGetRadius(Box2DShape shape) {
    return shape->m_radius;
}

void Box2DShapeSetRadius (Box2DShape shape, float32 radius) {
    shape->m_radius = radius;
}

Box2DShape Box2DShapeClone(Box2DShape shape, Box2DBlockAllocator allocator) {
    return (Box2DShape) shape->Clone(allocator);
}

VIRTUAL_FUNCTION int32 Box2DShapeGetChildCount(Box2DShape shape) {
    return shape->GetChildCount();
}

VIRTUAL_FUNCTION bool Box2DShapeTestPoint(Box2DShape shape, Box2DTransform transform, Box2DVector2 p) {
    return shape->TestPoint(Box2DTransformMake(transform), Box2DVector2Make(p));
}

VIRTUAL_FUNCTION bool Box2DShapeRayCast(Box2DShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex) {
    b2RayCastOutput _output;
    return shape->RayCast(&_output, Box2DRayCastInputMake(input), Box2DTransformMake(transform), childIndex);
    output = Box2DRayCastOutputMake(_output);
}

VIRTUAL_FUNCTION void Box2DShapeComputeAABB(Box2DShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex) {
    b2AABB _aabb;
    return shape->ComputeAABB(&_aabb, Box2DTransformMake(transform), childIndex);
    aabb = Box2DAABBMake(_aabb);
}

VIRTUAL_FUNCTION void Box2DShapeComputeMass(Box2DShape shape, Box2DMassData &massData, float32 density) {
    b2MassData _massData;
    return shape->ComputeMass(&_massData, density);
    massData = Box2DMassDataMake(_massData);
}

#pragma mark Edge Shape

Box2DEdgeShape Box2DEdgeShapeCreate () {
    return new b2EdgeShape();
}

void Box2DEdgeShapeSet (Box2DEdgeShape shape, Box2DVector2 v1, Box2DVector2 v2) {
    shape->Set(Box2DVector2Make(v1), Box2DVector2Make(v2));
}

Box2DEdgeShape Box2DEdgeShapeClone(Box2DEdgeShape shape, Box2DBlockAllocator allocator) {
    return (Box2DEdgeShape) Box2DShapeClone(shape, allocator);
}

int32 Box2DEdgeShapeGetChildCount(Box2DEdgeShape shape) {
    return Box2DShapeGetChildCount(shape);
}

bool Box2DEdgeShapeTestPoint(Box2DEdgeShape shape, Box2DTransform transform, Box2DVector2 p) {
    return Box2DShapeTestPoint(shape, transform, p);
}

bool Box2DEdgeShapeRayCast(Box2DEdgeShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex) {
    return Box2DShapeRayCast(shape, output, input, transform, childIndex);
}

void Box2DEdgeShapeComputeAABB(Box2DEdgeShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex) {
    Box2DShapeComputeAABB(shape, aabb, transform, childIndex);
}

void Box2DEdgeShapeComputeMass(Box2DEdgeShape shape, Box2DMassData &massData, float32 density) {
    Box2DShapeComputeMass(shape, massData, density);
}
    
    
#pragma mark Circle Shape
Box2DCircleShape Box2DCircleShapeCreate () {
    return new b2CircleShape();
}

void Box2DCircleShapeSetPosition (Box2DCircleShape shape, Box2DVector2 point) {
    shape->m_p.Set(point.x, point.y);
}

Box2DCircleShape Box2DCircleShapeClone(Box2DCircleShape shape, Box2DBlockAllocator allocator) {
    return (Box2DCircleShape) Box2DShapeClone(shape, allocator);
}

int32 Box2DCircleShapeGetChildCount(Box2DCircleShape shape) {
    return Box2DShapeGetChildCount(shape);
}

bool Box2DCircleShapeTestPoint(Box2DCircleShape shape, Box2DTransform transform, Box2DVector2 p) {
    return Box2DShapeTestPoint(shape, transform, p);
}

bool Box2DCircleShapeRayCast(Box2DCircleShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex) {
    return Box2DShapeRayCast(shape, output, input, transform, childIndex);
}

void Box2DCircleShapeComputeAABB(Box2DCircleShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex) {
    Box2DShapeComputeAABB(shape, aabb, transform, childIndex);
}

void Box2DCircleShapeComputeMass(Box2DCircleShape shape, Box2DMassData &massData, float32 density) {
    Box2DShapeComputeMass(shape, massData, density);
}

int32 Box2DCircleShapeGetSupport(Box2DCircleShape shape, Box2DVector2 d) {
    return shape->GetSupport(Box2DVector2Make(d));
}

Box2DVector2 Box2DCircleShapeGetSupportVertex(Box2DCircleShape shape, Box2DVector2 d) {
    return Box2DVector2Make(shape->GetSupportVertex(Box2DVector2Make(d)));
}

int32 Box2DCircleShapeGetVertexCount(Box2DCircleShape shape) {
    return shape->GetVertexCount();
}

Box2DVector2 Box2DCircleShapeGetVertex(Box2DCircleShape shape, int32 index) {
    return Box2DVector2Make(shape->GetVertex(index));
}
        
#pragma mark Polygon Shape

Box2DPolygonShape Box2DPolygonShapeCreate () {
    return new b2PolygonShape();
}

Box2DPolygonShape Box2DPolygonShapeClone(Box2DPolygonShape shape, Box2DBlockAllocator allocator) {
    return (Box2DPolygonShape) Box2DShapeClone(shape, allocator);
}

int32 Box2DPolygonShapeGetChildCount(Box2DPolygonShape shape) {
    return Box2DShapeGetChildCount(shape);
}

void Box2DPolygonShapeSet (Box2DPolygonShape shape, Box2DVector2 *points, int32 count) {
    b2Vec2 vertices[b2_maxPolygonVertices];
    b2Vec2ArrayMake((b2Vec2*)&vertices, points, count);
    shape->Set(vertices, count);
}

void Box2DPolygonShapeSetAsBox (Box2DPolygonShape shape, float32 hx, float32 hy) {
    shape->SetAsBox(hx, hy);
}

void Box2DPolygonShapeSetAsBox2 (Box2DPolygonShape shape, float32 hx, float32 hy, Box2DVector2 center, float32 angle) {
    shape->SetAsBox(hx, hy, Box2DVector2Make(center), angle);
}

bool Box2DPolygonShapeTestPoint(Box2DPolygonShape shape, Box2DTransform transform, Box2DVector2 p) {
    return Box2DShapeTestPoint(shape, transform, p);
}

bool Box2DPolygonShapeRayCast(Box2DPolygonShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex) {
    return Box2DShapeRayCast(shape, output, input, transform, childIndex);
}

void Box2DPolygonShapeComputeAABB(Box2DPolygonShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex) {
    Box2DShapeComputeAABB(shape, aabb, transform, childIndex);
}

void Box2DPolygonShapeComputeMass(Box2DPolygonShape shape, Box2DMassData &massData, float32 density) {
    Box2DShapeComputeMass(shape, massData, density);
}

int32 Box2DPolygonShapeGetVertexCount(Box2DPolygonShape shape) {
    return shape->GetVertexCount();
}

Box2DVector2 Box2DPolygonShapeGetVertex(Box2DPolygonShape shape, int32 index) {
    return Box2DVector2Make(shape->GetVertex(index));
}

bool Box2DPolygonShapeValidate(Box2DPolygonShape shape) {
    return shape->Validate();
}

#pragma mark Chain Shape

Box2DChainShape Box2DChainShapeCreate () {
    return new b2ChainShape();
}

void Box2DChainShapeCreateLoop(Box2DChainShape shape, Box2DVector2 *vertices, int32 count) {
    b2Vec2 _vertices[b2_maxPolygonVertices];
    b2Vec2ArrayMake((b2Vec2*)&_vertices, vertices, count);
    shape->CreateLoop(_vertices, count);
}

void Box2DChainShapeCreateChain(Box2DChainShape shape, Box2DVector2 *vertices, int32 count) {
    b2Vec2 _vertices[b2_maxPolygonVertices];
    b2Vec2ArrayMake((b2Vec2*)&_vertices, vertices, count);
    shape->CreateChain(_vertices, count);
}

void Box2DChainShapeSetPrevVertex(Box2DChainShape shape, Box2DVector2 prevVertex) {
    shape->SetPrevVertex(Box2DVector2Make(prevVertex));
}

void Box2DChainShapeSetNextVertex(Box2DChainShape shape, Box2DVector2 nextVertex) {
    shape->SetNextVertex(Box2DVector2Make(nextVertex));
}

Box2DChainShape Box2DChainShapeClone(Box2DChainShape shape, Box2DBlockAllocator allocator) {
    return (Box2DChainShape) Box2DShapeClone(shape, allocator);
}

int32 Box2DChainShapeGetChildCount(Box2DChainShape shape) {
    return Box2DShapeGetChildCount(shape);
}

void Box2DChainShapeGetChildEdge(Box2DChainShape shape, Box2DEdgeShape edge, int32 index) {
    return shape->GetChildEdge(edge, index);
}

bool Box2DChainShapeTestPoint(Box2DChainShape shape, Box2DTransform transform, Box2DVector2 p) {
    return Box2DShapeTestPoint(shape, transform, p);
}

bool Box2DChainShapeRayCast(Box2DChainShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex) {
    return Box2DShapeRayCast(shape, output, input, transform, childIndex);
}

void Box2DChainShapeComputeAABB(Box2DChainShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex) {
    Box2DShapeComputeAABB(shape, aabb, transform, childIndex);
}

void Box2DChainShapeComputeMass(Box2DChainShape shape, Box2DMassData &massData, float32 density) {
    Box2DShapeComputeMass(shape, massData, density);
}
    
#pragma mark Body

Box2DFixture Box2DBodyCreateFixture(Box2DBody body, Box2DFixtureDefinition def) {
    b2FixtureDef _def = Box2DFixtureDefinitionMake(def);
    return body->CreateFixture(&_def);
}

Box2DFixture Box2DBodyCreateFixtureWithShape(Box2DBody body, Box2DShape shape, float32 density) {
    return body->CreateFixture(shape, density);
}

void Box2DBodyDestroyFixture(Box2DBody body, Box2DFixture fixture) {
    body->DestroyFixture(fixture);
}

void Box2DBodySetTransform(Box2DBody body, Box2DVector2 position, float32 angle) {
    body->SetTransform(Box2DVector2Make(position), angle);
}

Box2DTransform Box2DBodyGetTransform(Box2DBody body) {
    return Box2DTransformMake(body->GetTransform());
}

Box2DVector2 Box2DBodyGetPosition(Box2DBody body) {
    return Box2DVector2Make(body->GetPosition());
}

float32 Box2DBodyGetAngle(Box2DBody body) {
    return body->GetAngle();
}

Box2DVector2 Box2DBodyGetWorldCenter(Box2DBody body) {
    return Box2DVector2Make(body->GetWorldCenter());
}

Box2DVector2 Box2DBodyGetLocalCenter(Box2DBody body) {
    return Box2DVector2Make(body->GetLocalCenter());
}

void Box2DBodySetLinearVelocity(Box2DBody body, Box2DVector2 v) {
    body->SetLinearVelocity(Box2DVector2Make(v));
}

Box2DVector2 Box2DBodyGetLinearVelocity(Box2DBody body) {
    return Box2DVector2Make(body->GetLinearVelocity());
}

void Box2DBodySetAngularVelocity(Box2DBody body, float32 omega) {
    body->SetAngularVelocity(omega);
}

float32 Box2DBodyGetAngularVelocity(Box2DBody body) {
    return body->GetAngularVelocity();
}

void Box2DBodyApplyForce(Box2DBody body, Box2DVector2 force, Box2DVector2 point, bool wake) {
    body->ApplyForce(Box2DVector2Make(force), Box2DVector2Make(point), wake);
}

void Box2DBodyApplyForceToCenter(Box2DBody body, Box2DVector2 force, bool wake) {
    body->ApplyForceToCenter(Box2DVector2Make(force), wake);
}

void Box2DBodyApplyTorque(Box2DBody body, float32 torque, bool wake) {
    body->ApplyTorque(torque, wake);
}

void Box2DBodyApplyAngularImpulse(Box2DBody body, float32 impulse, bool wake) {
    body->ApplyAngularImpulse(impulse, wake);
}

void Box2DBodyApplyLinearImpulse(Box2DBody body, Box2DVector2 impulse, Box2DVector2 point, bool wake) {
    body->ApplyLinearImpulse(Box2DVector2Make(impulse), Box2DVector2Make(point), wake);
}

float32 Box2DBodyGetMass(Box2DBody body) {
    return body->GetMass();
}

float32 Box2DBodyGetInertia(Box2DBody body) {
    return body->GetInertia();
}

void Box2DBodyGetMassData(Box2DBody body, Box2DMassData &data) {
    b2MassData _data = Box2DMassDataMake(data);
    body->GetMassData(&_data);
    data = Box2DMassDataMake(_data);
}

void Box2DBodySetMassData(Box2DBody body, Box2DMassData data) {
    b2MassData _data = Box2DMassDataMake(data);
    body->SetMassData(&_data);
}

void Box2DBodyResetMassData(Box2DBody body) {
    body->ResetMassData();
}

Box2DVector2 Box2DBodyGetWorldPoint(Box2DBody body, Box2DVector2 localPoint) {
    return Box2DVector2Make(body->GetWorldPoint(Box2DVector2Make(localPoint)));
}

Box2DVector2 Box2DBodyGetWorldVector(Box2DBody body, Box2DVector2 localVector) {
    return Box2DVector2Make(body->GetWorldVector(Box2DVector2Make(localVector)));
}

Box2DVector2 Box2DBodyGetLocalPoint(Box2DBody body, Box2DVector2 worldPoint) {
    return Box2DVector2Make(body->GetLocalPoint(Box2DVector2Make(worldPoint)));
}

Box2DVector2 Box2DBodyGetLocalVector(Box2DBody body, Box2DVector2 worldVector) {
    return Box2DVector2Make(body->GetLocalVector(Box2DVector2Make(worldVector)));
}

Box2DVector2 Box2DBodyGetLinearVelocityFromWorldPoint(Box2DBody body, Box2DVector2 worldPoint) {
    return Box2DVector2Make(body->GetLinearVelocityFromWorldPoint(Box2DVector2Make(worldPoint)));
}

Box2DVector2 Box2DBodyGetLinearVelocityFromLocalPoint(Box2DBody body, Box2DVector2 localPoint) {
    return Box2DVector2Make(body->GetLinearVelocityFromLocalPoint(Box2DVector2Make(localPoint)));
}

float32 Box2DBodyGetLinearDamping(Box2DBody body) {
    return body->GetLinearDamping();
}

void Box2DBodySetLinearDamping(Box2DBody body, float32 linearDamping) {
    body->SetLinearDamping(linearDamping);
}

float32 Box2DBodyGetAngularDamping(Box2DBody body) {
    return body->GetAngularDamping();
}

void Box2DBodySetAngularDamping(Box2DBody body, float32 angularDamping) {
    body->SetAngularDamping(angularDamping);
}

float32 Box2DBodyGetGravityScale(Box2DBody body) {
    return body->GetGravityScale();
}

void Box2DBodySetGravityScale(Box2DBody body, float32 scale) {
    body->SetGravityScale(scale);
}

void Box2DBodySetType(Box2DBody body, Box2DBodyType type) {
    body->SetType((b2BodyType) type);
}

Box2DBodyType Box2DBodyGetType(Box2DBody body) {
    return body->GetType();
}

void Box2DBodySetBullet(Box2DBody body, bool flag) {
    body->SetBullet(flag);
}

bool Box2DBodyIsBullet(Box2DBody body) {
    return body->IsBullet();
}

void Box2DBodySetSleepingAllowed(Box2DBody body, bool flag) {
    body->SetSleepingAllowed(flag);
}

bool Box2DBodyIsSleepingAllowed(Box2DBody body) {
    return body->IsSleepingAllowed();
}

void Box2DBodySetAwake(Box2DBody body, bool flag) {
    body->SetAwake(flag);
}

bool Box2DBodyIsAwake(Box2DBody body) {
    return body->IsAwake();
}

void Box2DBodySetActive(Box2DBody body, bool flag) {
    body->SetActive(flag);
}

bool Box2DBodyIsActive(Box2DBody body) {
    return body->IsActive();
}

void Box2DBodySetFixedRotation(Box2DBody body, bool flag) {
    body->SetFixedRotation(flag);
}

bool Box2DBodyIsFixedRotation(Box2DBody body) {
    return body->IsFixedRotation();
}
    
Box2DFixture Box2DBodyGetFixtureList(Box2DBody body) {
    return body->GetFixtureList();
}

void* Box2DBodyGetJointList(Box2DBody body) {
    TO_DO
    return NULL;
}
    
void* Box2DBodyGetContactList(Box2DBody body) {
    TO_DO
    return NULL;
}
    
Box2DBody Box2DBodyGetNext(Box2DBody body) {
    return body->GetNext();
}
    
void* Box2DBodyGetUserData(Box2DBody body) {
    return body->GetUserData();
}

void Box2DBodySetUserData(Box2DBody body, void* data) {
    body->SetUserData(data);
}

Box2DWorld Box2DBodyGetWorld(Box2DBody body) {
    return body->GetWorld();
}
    
void Box2DBodyDump(Box2DBody body) {
    body->Dump();
}

#pragma mark World

Box2DWorld Box2DWorldCreate (Box2DVector2 gravity) {
    return new b2World(Box2DVector2Make(gravity));
}

void Box2DWorldSetDestructionListener(Box2DWorld world, Box2DWorldCallbackCenter listener) {
    world->SetDestructionListener((b2DestructionListener*)listener);
}

void Box2DWorldSetContactFilter(Box2DWorld world, Box2DWorldCallbackCenter filter) {
    world->SetContactFilter((b2ContactFilter*)filter);
}

void Box2DWorldSetContactListener(Box2DWorld world, Box2DWorldCallbackCenter listener) {
    world->SetContactListener((b2ContactListener*)listener);
}

void Box2DWorldSetDebugDraw(Box2DWorld world, Box2DDebugDraw debugDraw) {
    world->SetDebugDraw((b2Draw*)debugDraw);
}

Box2DBody Box2DWorldCreateBody(Box2DWorld world, Box2DBodyDefinition def) {
    b2BodyDef _def = Box2DBodyDefinitionMake(def);
    return world->CreateBody(&_def);
}

void Box2DWorldDestroyBody(Box2DWorld world, Box2DBody body) {
    world->DestroyBody(body);
}

Box2DJoint Box2DWorldCreateJoint(Box2DWorld world, Box2DJointDefinition *def) {
    b2JointDef genDef = _Box2DJointDefinitionMake(*def);
    return world->CreateJoint(&genDef);
}

void Box2DWorldDestroyJoint(Box2DWorld world, Box2DJoint joint) {
    world->DestroyJoint(joint);
}

void Box2DWorldStep(Box2DWorld world, float32 timeStep, int32 velocityIterations, int32 positionIterations) {
    world->Step(timeStep, velocityIterations, positionIterations);
}

void Box2DWorldClearForces(Box2DWorld world) {
    world->ClearForces();
}

void Box2DWorldDrawDebugData(Box2DWorld world) {
    world->DrawDebugData();
}

void Box2DWorldQueryAABB(Box2DWorld world, Box2DWorldCallbackCenter callback, Box2DAABB aabb) {
    world->QueryAABB((b2QueryCallback*)callback, Box2DAABBMake(aabb));
}

void Box2DWorldRayCast(Box2DWorld world, Box2DWorldCallbackCenter callback, Box2DVector2 point1, Box2DVector2 point2) {
    world->RayCast((b2RayCastCallback*)callback, Box2DVector2Make(point1), Box2DVector2Make(point2));
}

Box2DBody Box2DWorldGetBodyList(Box2DWorld world) {
    return world->GetBodyList();
}

Box2DJoint Box2DWorldGetJointList(Box2DWorld world) {
    return world->GetJointList();
}

Box2DContact Box2DWorldGetContactList(Box2DWorld world) {
    return world->GetContactList();
}

void Box2DWorldSetAllowSleeping(Box2DWorld world, bool flag) {
    world->SetAllowSleeping(flag);
}

bool Box2DWorldGetAllowSleeping(Box2DWorld world) {
    return world->GetAllowSleeping();
}

void Box2DWorldSetWarmStarting(Box2DWorld world, bool flag) {
    world->SetWarmStarting(flag);
}

bool Box2DWorldGetWarmStarting(Box2DWorld world) {
    return world->GetWarmStarting();
}

void Box2DWorldSetContinuousPhysics(Box2DWorld world, bool flag) {
    world->SetContinuousPhysics(flag);
}

bool Box2DWorldGetContinuousPhysics(Box2DWorld world) {
    return world->GetContinuousPhysics();
}

void Box2DWorldSetSubStepping(Box2DWorld world, bool flag) {
    world->SetSubStepping(flag);
}

bool Box2DWorldGetSubStepping(Box2DWorld world) {
    return world->GetSubStepping();
}

int32 Box2DWorldGetProxyCount(Box2DWorld world) {
    return world->GetProxyCount();
}

int32 Box2DWorldGetBodyCount(Box2DWorld world) {
    return world->GetBodyCount();
}

int32 Box2DWorldGetJointCount(Box2DWorld world) {
    return world->GetJointCount();
}

int32 Box2DWorldGetContactCount(Box2DWorld world) {
    return world->GetContactCount();
}

int32 Box2DWorldGetTreeHeight(Box2DWorld world) {
    return world->GetTreeHeight();
}

int32 Box2DWorldGetTreeBalance(Box2DWorld world) {
    return world->GetTreeBalance();
}

float32 Box2DWorldGetTreeQuality(Box2DWorld world) {
    return world->GetTreeQuality();
}

void Box2DWorldSetGravity(Box2DWorld world, Box2DVector2 gravity) {
    world->SetGravity(Box2DVector2Make(gravity));
}

Box2DVector2 Box2DWorldGetGravity(Box2DWorld world) {
    return Box2DVector2Make(world->GetGravity());
}

bool Box2DWorldIsLocked(Box2DWorld world) {
    return world->IsLocked();
}

void Box2DWorldSetAutoClearForces(Box2DWorld world, bool flag) {
    world->SetAutoClearForces(flag);
}

bool Box2DWorldGetAutoClearForces(Box2DWorld world) {
    return world->GetAutoClearForces();
}

void Box2DWorldShiftOrigin(Box2DWorld world, Box2DVector2 newOrigin) {
    world->ShiftOrigin(Box2DVector2Make(newOrigin));
}

void Box2DWorldGetContactManager(Box2DWorld world, Box2DContactManager &manager) {
    *manager = world->GetContactManager();
}

b2Profile Box2DWorldGetProfile(Box2DWorld world) {
    return world->GetProfile();
}

void Box2DWorldDump(Box2DWorld world) {
    world->Dump();
}

#pragma mark Contact Manager

void Box2DContactManagerAddPair(Box2DContactManager manager, void* proxyUserDataA, void* proxyUserDataB) {
    manager->AddPair(proxyUserDataA, proxyUserDataB);
}

void Box2DContactManagerFindNewContacts(Box2DContactManager manager) {
    manager->FindNewContacts();
}

void Box2DContactManagerDestroy(Box2DContactManager manager, Box2DContact c) {
    manager->Destroy(c);
}

void Box2DContactManagerCollide(Box2DContactManager manager) {
    manager->Collide();
}


#pragma mark Contact

Box2DManifold Box2DContactGetManifold(Box2DContact contact) {
    return Box2DManifoldMake(*contact->GetManifold());
}
    
void Box2DContactGetWorldManifold(Box2DContact contact, Box2DWorldManifold &worldManifold) {
    b2WorldManifold _worldManifold;
    contact->GetWorldManifold(&_worldManifold);
    worldManifold = Box2DWorldManifoldMake(_worldManifold);
}

bool Box2DContactIsTouching(Box2DContact contact) {
    return contact->IsTouching();
}

void Box2DContactSetEnabled(Box2DContact contact, bool flag) {
    contact->SetEnabled(flag);
}

bool Box2DContactIsEnabled(Box2DContact contact) {
    return contact->IsEnabled();
}

Box2DContact Box2DContactGetNext(Box2DContact contact) {
    return contact->GetNext();
}

Box2DFixture Box2DContactGetFixtureA(Box2DContact contact) {
    return contact->GetFixtureA();
}

int32 Box2DContactGetChildIndexA(Box2DContact contact) {
    return contact->GetChildIndexA();
}

Box2DFixture Box2DContactGetFixtureB(Box2DContact contact) {
    return contact->GetFixtureB();
}

int32 Box2DContactGetChildIndexB(Box2DContact contact) {
    return contact->GetChildIndexB();
}

void Box2DContactSetFriction(Box2DContact contact, float32 friction) {
    contact->SetFriction(friction);
}

float32 Box2DContactGetFriction(Box2DContact contact) {
    return contact->GetFriction();
}

void Box2DContactResetFriction(Box2DContact contact) {
    contact->ResetFriction();
}

void Box2DContactSetRestitution(Box2DContact contact, float32 restitution) {
    contact->SetRestitution(restitution);
}

float32 Box2DContactGetRestitution(Box2DContact contact) {
    return contact->GetRestitution();
}

void Box2DContactResetRestitution(Box2DContact contact) {
    contact->ResetRestitution();
}

void Box2DContactSetTangentSpeed(Box2DContact contact, float32 speed) {
    contact->SetTangentSpeed(speed);
}

float32 Box2DContactGetTangentSpeed(Box2DContact contact) {
    return contact->GetTangentSpeed();
}

VIRTUAL_FUNCTION void Box2DContactEvaluate(Box2DContact contact, Box2DManifold &manifold, Box2DTransform xfA, Box2DTransform xfB) {
    b2Manifold _manifold;
    contact->Evaluate(&_manifold, Box2DTransformMake(xfA), Box2DTransformMake(xfB));
    manifold = Box2DManifoldMake(_manifold);
}

#pragma mark Utilities
    
void Box2DRelease (void* &_class) {
    delete _class;
    _class = NULL;
}

b2Version Box2DGetCurrentVersion () {
    return b2_version;
}

EXTERNAL_FUNCTIONS_END
