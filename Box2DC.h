//
//  Box2DC.h
//  Box2DC
//
//  Created by Ryan Joseph on 5/23/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#ifndef BOX2DC_BOX2DC_H
#define BOX2DC_BOX2DC_H

#include "Box2D.h"

#define VIRTUAL_FUNCTION
#define EXTERNAL_FUNCTIONS_BEGIN extern "C" {
#define EXTERNAL_FUNCTIONS_END }
#define TO_DO

// Opaque types for callback wrappers
typedef void* Box2DWorldCallbackCenter;
typedef void* Box2DDebugDraw;

#pragma mark Types

typedef b2BodyType Box2DBodyType;
typedef b2Shape::Type Box2DShapeType;
typedef b2JointType Box2DJointType;
typedef b2Manifold::Type Box2DManifoldType;
typedef b2LimitState Box2DLimitState;

#pragma mark Opaque Classes

typedef b2BlockAllocator *Box2DBlockAllocator;

typedef b2World *Box2DWorld;
typedef b2Body *Box2DBody;
typedef b2Fixture *Box2DFixture;
typedef b2Contact *Box2DContact;
typedef b2ContactManager *Box2DContactManager;

typedef b2EdgeShape *Box2DEdgeShape;
typedef b2Shape *Box2DShape;
typedef b2CircleShape *Box2DCircleShape;
typedef b2PolygonShape *Box2DPolygonShape;
typedef b2ChainShape *Box2DChainShape;

typedef b2Joint *Box2DJoint;
typedef b2DistanceJoint *Box2DDistanceJoint;
typedef b2FrictionJoint *Box2DFrictionJoint;
typedef b2GearJoint *Box2DGearJoint;
typedef b2MotorJoint *Box2DMotorJoint;
typedef b2MouseJoint *Box2DMouseJoint;
typedef b2PrismaticJoint *Box2DPrismaticJoint;
typedef b2PulleyJoint *Box2DPulleyJoint;
typedef b2RevoluteJoint *Box2DRevoluteJoint;
typedef b2RopeJoint *Box2DRopeJoint;
typedef b2WeldJoint *Box2DWeldJoint;
typedef b2WheelJoint *Box2DWheelJoint;

#pragma mark General Structs

struct Box2DVector2 {
    float32 x;
    float32 y;
};

struct Box2DRotation {
	float32 s, c;
};

struct Box2DTransform {
	Box2DVector2 p;
	Box2DRotation q;
};

struct Box2DFilter {
	uint16 categoryBits;
	uint16 maskBits;
	int16 groupIndex;
};

struct Box2DRayCastInput {
	Box2DVector2 p1, p2;
	float32 maxFraction;
};

struct Box2DRayCastOutput {
	Box2DVector2 normal;
	float32 fraction;
};

struct Box2DMassData {
	float32 mass;
	Box2DVector2 center;
	float32 I;
};

struct Box2DAABB {
	Box2DVector2 lowerBound;
	Box2DVector2 upperBound;
};

struct Box2DManifoldPoint {
	Box2DVector2 localPoint;
	float32 normalImpulse;
	float32 tangentImpulse;
	b2ContactID id;
};

struct Box2DManifold {
	Box2DManifoldPoint points[b2_maxManifoldPoints];
	Box2DVector2 localNormal;
	Box2DVector2 localPoint;
    Box2DManifoldType type;
	int32 pointCount;
};

struct Box2DWorldManifold {
	Box2DVector2 normal;
	Box2DVector2 points[b2_maxManifoldPoints];
};

struct Box2DContactImpulse
{
	float32 normalImpulses[b2_maxManifoldPoints];
	float32 tangentImpulses[b2_maxManifoldPoints];
	int32 count;
};

#pragma mark Definition Structs

struct Box2DBodyDefinition {
	Box2DBodyType type;
	Box2DVector2 position;
	float32 angle;
	Box2DVector2 linearVelocity;
	float32 angularVelocity;
	float32 linearDamping;
	float32 angularDamping;
    bool allowSleep;
	bool awake;
	bool fixedRotation;
	bool bullet;
	bool active;
	void* userData;
	float32 gravityScale;
};

struct Box2DFixtureDefinition {
	Box2DShape shape;
	void* userData;
	float32 friction;
	float32 restitution;
	float32 density;
	bool isSensor;
	Box2DFilter filter;
};

struct Box2DJointDefinition {
	Box2DJointType type;
	void* userData;
	Box2DBody bodyA;
	Box2DBody bodyB;
	bool collideConnected;
};

struct Box2DDistanceJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	float32 length;
	float32 frequencyHz;
	float32 dampingRatio;
};

struct Box2DFrictionJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	float32 maxForce;
	float32 maxTorque;
};

struct Box2DGearJointDefinition {
    Box2DJointDefinition joint;
	Box2DJoint joint1;
	Box2DJoint joint2;
	float32 ratio;
};

struct Box2DMotorJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 linearOffset;
	float32 angularOffset;
	float32 maxForce;
	float32 maxTorque;
	float32 correctionFactor;
};

struct Box2DMouseJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 target;
	float32 maxForce;
	float32 frequencyHz;
	float32 dampingRatio;
};

struct Box2DPulleyJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 groundAnchorA;
	Box2DVector2 groundAnchorB;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	float32 lengthA;
	float32 lengthB;
	float32 ratio;
};

struct Box2DPrismaticJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	Box2DVector2 localAxisA;
	float32 referenceAngle;
	bool enableLimit;
	float32 lowerTranslation;
	float32 upperTranslation;
	bool enableMotor;
	float32 maxMotorForce;
	float32 motorSpeed;
};

struct Box2DRevoluteJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	float32 referenceAngle;
	bool enableLimit;
	float32 lowerAngle;
	float32 upperAngle;
	bool enableMotor;
	float32 motorSpeed;
	float32 maxMotorTorque;
};

struct Box2DRopeJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	float32 maxLength;
};

struct Box2DWeldJointDefinition {
    Box2DJointDefinition joint;
    Box2DVector2 localAnchorA;
    Box2DVector2 localAnchorB;
    float32 referenceAngle;
    float32 frequencyHz;
    float32 dampingRatio;
};

struct Box2DWheelJointDefinition {
    Box2DJointDefinition joint;
	Box2DVector2 localAnchorA;
	Box2DVector2 localAnchorB;
	Box2DVector2 localAxisA;
	bool enableMotor;
	float32 maxMotorTorque;
	float32 motorSpeed;
	float32 frequencyHz;
	float32 dampingRatio;
};

typedef struct Box2DVector2 Box2DVector2;
typedef Box2DVector2 Box2DVerticesArray[b2_maxPolygonVertices];

typedef struct Box2DTransform Box2DTransform;
typedef struct Box2DRotation Box2DRotation;
typedef struct Box2DFilter Box2DFilter;
typedef struct Box2DRayCastInput Box2DRayCastInput;
typedef struct Box2DRayCastOutput Box2DRayCastOutput;
typedef struct Box2DMassData Box2DMassData;
typedef struct Box2DAABB Box2DAABB;
typedef struct Box2DManifold Box2DManifold;
typedef struct Box2DManifoldPoint Box2DManifoldPoint;
typedef struct Box2DContactImpulse Box2DContactImpulse;

typedef struct Box2DBodyDefinition Box2DBodyDefinition;
typedef struct Box2DFixtureDefinition Box2DFixtureDefinition;
typedef struct Box2DJointDefinition Box2DJointDefinition;
typedef struct Box2DDistanceJointDefinition Box2DDistanceJointDefinition;
typedef struct Box2DFrictionJointDefinition Box2DFrictionJointDefinition;
typedef struct Box2DGearJointDefinition Box2DGearJointDefinition;
typedef struct Box2DMotorJointDefinition Box2DMotorJointDefinition;
typedef struct Box2DMouseJointDefinition Box2DMouseJointDefinition;
typedef struct Box2DPrismaticJointDefinition Box2DPrismaticJointDefinition;
typedef struct Box2DPulleyJointDefinition Box2DPulleyJointDefinition;
typedef struct Box2DRevoluteJointDefinition Box2DRevoluteJointDefinition;
typedef struct Box2DRopeJointDefinition Box2DRopeJointDefinition;
typedef struct Box2DWeldJointDefinition Box2DWeldJointDefinition;
typedef struct Box2DWheelJointDefinition Box2DWheelJointDefinition;

#pragma mark World Manifold

void Box2DWorldManifoldInitialize(Box2DWorldManifold &worldManifold, Box2DManifold manifold, Box2DTransform xfA, float32 radiusA, Box2DTransform xfB, float32 radiusB);

#pragma mark Vector2

void Box2DVectorSetZero(Box2DVector2 &vec);
void Box2DVectorSet(Box2DVector2 &vec, float32 x_, float32 y_);
float32 Box2DVectorLength(Box2DVector2 vec);
float32 Box2DVectorLengthSquared(Box2DVector2 vec);
float32 Box2DVectorNormalize(Box2DVector2 &vec);
bool Box2DVectorIsValid(Box2DVector2 vec);
Box2DVector2 Box2DVectorSkew(Box2DVector2 vec);
Box2DVector2 Box2DVectorNegate (Box2DVector2 vec);
Box2DVector2 Box2DVectorAddVector (Box2DVector2 vec, Box2DVector2 add);
Box2DVector2 Box2DVectorSubtractVector (Box2DVector2 vec, Box2DVector2 subtract);
Box2DVector2 Box2DVectorMultiply (Box2DVector2 vec, float32 a);

#pragma mark Transform

Box2DTransform Box2DTransformMake (Box2DVector2 position, Box2DRotation rotation);
void Box2DTransformSetIdentity(Box2DTransform &transform);
void Box2DTransformSet(Box2DTransform &transform, Box2DVector2 position, float32 angle);

#pragma mark Rotation

Box2DRotation Box2DRotationMake (float32 angle);
void Box2DRotationSet(Box2DRotation &rotation, float32 angle);
void Box2DRotationSetIdentity(Box2DRotation &rotation);
float32 Box2DRotationGetAngle(Box2DRotation rotation);
Box2DVector2 Box2DRotationGetXAxis(Box2DRotation rotation);
Box2DVector2 Box2DRotationGetYAxis(Box2DRotation rotation);

#pragma mark AABB

bool Box2DAABBIsValid(Box2DAABB aabb);
Box2DVector2 Box2DAABBGetCenter(Box2DAABB aabb);
Box2DVector2 Box2DAABBGetExtents(Box2DAABB aabb);
float32 Box2DAABBGetPerimeter(Box2DAABB aabb);
void Box2DAABBCombine(Box2DAABB &ioAABB, Box2DAABB aabb);
void Box2DAABBCombine2(Box2DAABB &ioAABB, Box2DAABB aabb1, Box2DAABB aabb2);
bool Box2DAABBContains(Box2DAABB thisAABB, Box2DAABB aabb);
bool Box2DAABBRayCast(Box2DAABB aabb, Box2DRayCastOutput &output, Box2DRayCastInput input);

#pragma mark Body Definition

Box2DBodyDefinition Box2DBodyDefinitionDefault ();

#pragma mark Fixture Definition

Box2DFixtureDefinition Box2DFixtureDefinitionDefault ();

#pragma mark Joint Definitions


#pragma mark Distance Joint Definition

Box2DDistanceJointDefinition Box2DDistanceJointDefinitionDefault ();
void Box2DDistanceJointDefinitionInitialize(Box2DDistanceJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchorA, Box2DVector2 anchorB);

#pragma mark Friction Joint Definition

Box2DFrictionJointDefinition Box2DFrictionJointDefinitionDefault ();
void Box2DFrictionJointDefinitionInitialize(Box2DFrictionJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor);

#pragma mark Motor Joint Definition

Box2DMotorJointDefinition Box2DMotorJointDefinitionDefault ();
void Box2DMotorJointDefinitionInitialize(Box2DMotorJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB);


#pragma mark Revolute Joint Definition

Box2DRevoluteJointDefinition Box2DRevoluteJointDefinitionDefault ();
void Box2DRevoluteJointDefinitionInitialize (Box2DRevoluteJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor);


#pragma mark Pulley Joint Definition

Box2DPulleyJointDefinition Box2DPulleyJointDefinitionDefault ();
void Box2DPulleyJointDefinitionInitialize (Box2DPulleyJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB,
                                           Box2DVector2 groundAnchorA, Box2DVector2 groundAnchorB,
                                           Box2DVector2 anchorA, Box2DVector2 anchorB,
                                           float32 ratio);

#pragma mark Prismatic Joint Definition

Box2DPrismaticJointDefinition Box2DPrismaticJointDefinitionDefault ();
void Box2DPrismaticJointDefinitionInitialize(Box2DPrismaticJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor, Box2DVector2 axis);

#pragma mark Weld Joint Definition

Box2DWeldJointDefinition Box2DWeldJointDefinitionDefault ();
void Box2DWeldJointDefinitionInitialize (Box2DWeldJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor);

#pragma mark Wheel Joint Definition

Box2DWheelJointDefinition Box2DWheelJointDefinitionDefault ();
void Box2DWheelJointDefinitionInitialize(Box2DWheelJointDefinition &def, Box2DBody bodyA, Box2DBody bodyB, Box2DVector2 anchor, Box2DVector2 axis);

#pragma mark Joints


#pragma mark Joint

Box2DJointType Box2DJointGetType(Box2DJoint joint);
Box2DBody Box2DJointGetBodyA(Box2DJoint joint);
Box2DBody Box2DJointGetBodyB(Box2DJoint joint);
Box2DJoint Box2DJointGetNext(Box2DJoint joint);
void* Box2DJointGetUserData(Box2DJoint joint);
void Box2DJointSetUserData(Box2DJoint joint, void* data);
bool Box2DJointIsActive(Box2DJoint joint);
bool Box2DJointGetCollideConnected(Box2DJoint joint);
VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetAnchorA(Box2DJoint joint);
VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetAnchorB(Box2DJoint joint);
VIRTUAL_FUNCTION Box2DVector2 Box2DJointGetReactionForce(Box2DJoint joint, float32 inv_dt);
VIRTUAL_FUNCTION float32 Box2DJointGetReactionTorque(Box2DJoint joint, float32 inv_dt);
VIRTUAL_FUNCTION void Box2DJointDump(Box2DJoint joint);
VIRTUAL_FUNCTION void Box2DJointShiftOrigin(Box2DJoint joint, Box2DVector2 newOrigin);


#pragma mark Distance Joint

Box2DVector2 Box2DDistanceJointGetReactionForce(Box2DDistanceJoint joint, float32 inv_dt);
float32 Box2DDistanceJointGetReactionTorque(Box2DDistanceJoint joint, float32 inv_dt);
Box2DVector2 Box2DDistanceJointGetLocalAnchorA(Box2DDistanceJoint joint);
Box2DVector2 Box2DDistanceJointGetLocalAnchorB(Box2DDistanceJoint joint);
void Box2DDistanceJointSetLength(Box2DDistanceJoint joint, float32 length);
float32 Box2DDistanceJointGetLength(Box2DDistanceJoint joint);
void Box2DDistanceJointSetFrequency(Box2DDistanceJoint joint, float32 hz);
float32 Box2DDistanceJointGetFrequency(Box2DDistanceJoint joint);
void Box2DDistanceJointSetDampingRatio(Box2DDistanceJoint joint, float32 ratio);
float32 Box2DDistanceJointGetDampingRatio(Box2DDistanceJoint joint);
void Box2DDistanceJointDump(Box2DDistanceJoint joint);

#pragma mark Friction Joint

Box2DVector2 Box2DFrictionJointGetAnchorA(Box2DFrictionJoint joint);
Box2DVector2 Box2DFrictionJointGetAnchorB(Box2DFrictionJoint joint);
Box2DVector2 Box2DFrictionJointGetReactionForce(Box2DFrictionJoint joint, float32 inv_dt);
float32 Box2DFrictionJointGetReactionTorque(Box2DFrictionJoint joint, float32 inv_dt);
Box2DVector2 Box2DFrictionJointGetLocalAnchorA(Box2DFrictionJoint joint);
Box2DVector2 Box2DFrictionJointGetLocalAnchorB(Box2DFrictionJoint joint);
void Box2DFrictionJointSetMaxForce(Box2DFrictionJoint joint, float32 force);
float32 Box2DFrictionJointGetMaxForce(Box2DFrictionJoint joint);
void Box2DFrictionJointSetMaxTorque(Box2DFrictionJoint joint, float32 torque);
float32 Box2DFrictionJointGetMaxTorque(Box2DFrictionJoint joint);
void Box2DFrictionJointDump(Box2DDistanceJoint joint);

#pragma mark Gear Joint

Box2DVector2 Box2DGearJointGetAnchorA(Box2DGearJoint joint);
Box2DVector2 Box2DGearJointGetAnchorB(Box2DGearJoint joint);
Box2DVector2 Box2DGearJointGetReactionForce(Box2DGearJoint joint, float32 inv_dt);
float32 Box2DGearJointGetReactionTorque(Box2DGearJoint joint, float32 inv_dt);
Box2DJoint Box2DGearJointGetJoint1(Box2DGearJoint joint);
Box2DJoint Box2DGearJointGetJoint2(Box2DGearJoint joint);
void Box2DGearJointSetRatio(Box2DGearJoint joint, float32 ratio);
float32 Box2DGearJointGetRatio(Box2DGearJoint joint);
void Box2DGearJointDump(Box2DGearJoint joint);

#pragma mark Motor Joint

Box2DVector2 Box2DMotorJointGetAnchorA(Box2DMotorJoint joint);
Box2DVector2 Box2DMotorJointGetAnchorB(Box2DMotorJoint joint);
Box2DVector2 Box2DMotorJointGetReactionForce(Box2DMotorJoint joint, float32 inv_dt);
float32 Box2DMotorJointGetReactionTorque(Box2DMotorJoint joint, float32 inv_dt);
void Box2DMotorJointSetLinearOffset(Box2DMotorJoint joint, Box2DVector2 linearOffset);
Box2DVector2 Box2DMotorJointGetLinearOffset(Box2DMotorJoint joint);
void Box2DMotorJointSetAngularOffset(Box2DMotorJoint joint, float32 angularOffset);
float32 Box2DMotorJointGetAngularOffset(Box2DMotorJoint joint);
void Box2DMotorJointSetMaxForce(Box2DMotorJoint joint, float32 force);
float32 Box2DMotorJointGetMaxForce(Box2DMotorJoint joint);
void Box2DMotorJointSetMaxTorque(Box2DMotorJoint joint, float32 torque);
float32 Box2DMotorJointGetMaxTorque(Box2DFrictionJoint joint);
void Box2DMotorJointDump(Box2DMotorJoint joint);


#pragma mark Mouse Joint

Box2DVector2 Box2DMouseJointGetAnchorA(Box2DMouseJoint joint);
Box2DVector2 Box2DMouseJointGetAnchorB(Box2DMouseJoint joint);
Box2DVector2 Box2DMouseJointGetReactionForce(Box2DMouseJoint joint, float32 inv_dt);
float32 Box2DMouseJointGetReactionTorque(Box2DMouseJoint joint, float32 inv_dt);
void Box2DMouseJointSetTarget(Box2DMouseJoint joint, const Box2DVector2 target);
Box2DVector2 Box2DMouseJointGetTarget(Box2DMouseJoint joint);
void Box2DMouseJointSetMaxForce(Box2DMouseJoint joint, float32 force);
float32 Box2DMouseJointGetMaxForce(Box2DMouseJoint joint);
void Box2DMouseJointSetFrequency(Box2DMouseJoint joint, float32 hz);
float32 Box2DMouseJointGetFrequency(Box2DMouseJoint joint);
void Box2DMouseJointSetDampingRatio(Box2DMouseJoint joint, float32 ratio);
float32 Box2DMouseJointGetDampingRatio(Box2DMouseJoint joint);
void Box2DMouseJointDump(Box2DMouseJoint joint);
void Box2DMouseJointShiftOrigin(Box2DMouseJoint joint, Box2DVector2 newOrigin);

#pragma mark Prismatic Joint

Box2DVector2 Box2DPrismaticJointGetAnchorA(Box2DPrismaticJoint joint);
Box2DVector2 Box2DPrismaticJointGetAnchorB(Box2DPrismaticJoint joint);
Box2DVector2 Box2DPrismaticJointGetReactionForce(Box2DPrismaticJoint joint, float32 inv_dt);
float32 Box2DPrismaticJointGetReactionTorque(Box2DPrismaticJoint joint, float32 inv_dt);
Box2DVector2 Box2DPrismaticJointGetLocalAnchorA(Box2DPrismaticJoint joint);
Box2DVector2 Box2DPrismaticJointGetLocalAnchorB(Box2DPrismaticJoint joint);
Box2DVector2 Box2DPrismaticJointGetLocalAxisA(Box2DPrismaticJoint joint);
float32 Box2DPrismaticJointGetReferenceAngle(Box2DPrismaticJoint joint);
float32 Box2DPrismaticJointGetJointTranslation(Box2DPrismaticJoint joint);
float32 Box2DPrismaticJointGetJointSpeed(Box2DPrismaticJoint joint);
bool Box2DPrismaticJointIsLimitEnabled(Box2DPrismaticJoint joint);
void Box2DPrismaticJointEnableLimit(Box2DPrismaticJoint joint, bool flag);
float32 Box2DPrismaticJointGetLowerLimit(Box2DPrismaticJoint joint);
float32 Box2DPrismaticJointGetUpperLimit(Box2DPrismaticJoint joint);
void Box2DPrismaticJointSetLimits(Box2DPrismaticJoint joint, float32 lower, float32 upper);
bool Box2DPrismaticJointIsMotorEnabled(Box2DPrismaticJoint joint);
void Box2DPrismaticJointEnableMotor(Box2DPrismaticJoint joint, bool flag);
void Box2DPrismaticJointSetMotorSpeed(Box2DPrismaticJoint joint, float32 speed);
float32 GBox2DPrismaticJointGetMotorSpeed(Box2DPrismaticJoint joint);
void Box2DPrismaticJointSetMaxMotorForce(Box2DPrismaticJoint joint, float32 force);
float32 Box2DPrismaticJointGetMaxMotorForce(Box2DPrismaticJoint joint);
float32 Box2DPrismaticJointGetMotorForce(Box2DPrismaticJoint joint, float32 inv_dt);
void Box2DPrismaticJointDump(Box2DPrismaticJoint joint);


#pragma mark Pulley Joint

Box2DVector2 Box2DPulleyJointGetAnchorA(Box2DPulleyJoint joint);
Box2DVector2 Box2DPulleyJointGetAnchorB(Box2DPulleyJoint joint);
Box2DVector2 Box2DPulleyJointGetReactionForce(Box2DPulleyJoint joint, float32 inv_dt);
float32 Box2DPulleyJointGetReactionTorque(Box2DPulleyJoint joint, float32 inv_dt);
Box2DVector2 Box2DPulleyJointGetGroundAnchorA(Box2DPulleyJoint joint);
Box2DVector2 Box2DPulleyJointGetGroundAnchorB(Box2DPulleyJoint joint);
float32 Box2DPulleyJointGetLengthA(Box2DPulleyJoint joint);
float32 Box2DPulleyJointGetLengthB(Box2DPulleyJoint joint);
float32 Box2DPulleyJointGetRatio(Box2DPulleyJoint joint);
float32 Box2DPulleyJointGetCurrentLengthA(Box2DPulleyJoint joint);
float32 Box2DPulleyJointGetCurrentLengthB(Box2DPulleyJoint joint);
void Box2DPulleyJointDump(Box2DPulleyJoint joint);
void Box2DPulleyJointShiftOrigin(Box2DPulleyJoint joint, Box2DVector2 newOrigin);

#pragma mark Revolute Joint

Box2DVector2 Box2DRevoluteJointGetLocalAnchorA(Box2DRevoluteJoint joint);
Box2DVector2 Box2DRevoluteJointGetLocalAnchorB(Box2DRevoluteJoint joint);
float32 Box2DRevoluteJointGetReferenceAngle(Box2DRevoluteJoint joint);
float32 Box2DRevoluteJointGetJointAngle(Box2DRevoluteJoint joint);
float32 Box2DRevoluteJointGetJointSpeed(Box2DRevoluteJoint joint);
bool Box2DRevoluteJointIsLimitEnabled(Box2DRevoluteJoint joint);
void Box2DRevoluteJointEnableLimit(Box2DRevoluteJoint joint, bool flag);
float32 Box2DRevoluteJointGetLowerLimit(Box2DRevoluteJoint joint);
float32 Box2DRevoluteJointGetUpperLimit(Box2DRevoluteJoint joint);
void Box2DRevoluteJointSetLimits(Box2DRevoluteJoint joint, float32 lower, float32 upper);
bool Box2DRevoluteJointIsMotorEnabled(Box2DRevoluteJoint joint);
void Box2DRevoluteJointEnableMotor(Box2DRevoluteJoint joint, bool flag);
void Box2DRevoluteJointSetMotorSpeed(Box2DRevoluteJoint joint, float32 speed);
float32 Box2DRevoluteJointGetMotorSpeed(Box2DRevoluteJoint joint);
void Box2DRevoluteJointSetMaxMotorTorque(Box2DRevoluteJoint joint, float32 torque);
float32 Box2DRevoluteJointGetMaxMotorTorque(Box2DRevoluteJoint joint);
Box2DVector2 Box2DRevoluteJointGetReactionForce(Box2DRevoluteJoint joint, float32 inv_dt);
float32 Box2DRevoluteJointGetReactionTorque(Box2DRevoluteJoint joint, float32 inv_dt);
float32 Box2DRevoluteJointGetMotorTorque(Box2DRevoluteJoint joint, float32 inv_dt);
void Box2DRevoluteJointDump(Box2DRevoluteJoint joint);

#pragma mark Rope Joint

Box2DVector2 Box2DRopeJointGetAnchorA(Box2DRopeJoint joint);
Box2DVector2 Box2DRopeJointGetAnchorB(Box2DRopeJoint joint);
Box2DVector2 Box2DRopeJointGetReactionForce(Box2DRopeJoint joint, float32 inv_dt);
float32 Box2DRopeJointGetReactionTorque(Box2DRopeJoint joint, float32 inv_dt);
Box2DVector2 Box2DRopeJointGetLocalAnchorA(Box2DRopeJoint joint);
Box2DVector2 Box2DRopeJointGetLocalAnchorB(Box2DRopeJoint joint);
void Box2DRopeJointSetMaxLength(Box2DRopeJoint joint, float32 length);
float32 Box2DRopeJointGetMaxLength(Box2DRopeJoint joint);
Box2DLimitState Box2DRopeJointGetLimitState(Box2DRopeJoint joint);
void Box2DRopeJointDump(Box2DRopeJoint joint);

#pragma mark Weld Joint

Box2DVector2 Box2DWeldJointGetAnchorA(Box2DWeldJoint joint);
Box2DVector2 Box2DWeldJointGetAnchorB(Box2DWeldJoint joint);
Box2DVector2 Box2DWeldJointGetReactionForce(Box2DWeldJoint joint, float32 inv_dt);
float32 Box2DWeldJointGetReactionTorque(Box2DWeldJoint joint, float32 inv_dt);
Box2DVector2 Box2DWeldJointGetLocalAnchorA(Box2DWeldJoint joint);
Box2DVector2 Box2DWeldJointGetLocalAnchorB(Box2DWeldJoint joint);
float32 Box2DWeldJointGetReferenceAngle(Box2DWeldJoint joint);
void Box2DWeldJointSetFrequency(Box2DWeldJoint joint, float32 hz);
float32 Box2DWeldJointGetFrequency(Box2DWeldJoint joint);
void Box2DWeldJointSetDampingRatio(Box2DWeldJoint joint, float32 ratio);
float32 Box2DWeldJointGetDampingRatio(Box2DWeldJoint joint);
void Box2DWeldJointDump(Box2DWeldJoint joint);

#pragma mark Wheel Joint

void Box2DWheelJointGetDefinition(Box2DWheelJoint joint, Box2DWheelJointDefinition &def);
Box2DVector2 Box2DWheelJointGetAnchorA(Box2DWheelJoint joint);
Box2DVector2 Box2DWheelJointGetAnchorB(Box2DWheelJoint joint);
Box2DVector2 Box2DWheelJointGetReactionForce(Box2DWheelJoint joint, float32 inv_dt);
float32 Box2DWheelJointGetReactionTorque(Box2DWheelJoint joint, float32 inv_dt);
Box2DVector2 Box2DWheelJointGetLocalAnchorA(Box2DWheelJoint joint);
Box2DVector2 Box2DWheelJointGetLocalAnchorB(Box2DWheelJoint joint);
Box2DVector2 Box2DWheelJointGetLocalAxisA(Box2DWheelJoint joint);
float32 Box2DWheelJointGetJointTranslation(Box2DWheelJoint joint);
float32 Box2DWheelJointGetJointSpeed(Box2DWheelJoint joint);
bool Box2DWheelJointIsMotorEnabled(Box2DWheelJoint joint);
void Box2DWheelJointEnableMotor(Box2DWheelJoint joint, bool flag);
void Box2DWheelJointSetMotorSpeed(Box2DWheelJoint joint, float32 speed);
float32 Box2DWheelJointGetMotorSpeed(Box2DWheelJoint joint);
void Box2DWheelJointSetMaxMotorTorque(Box2DWheelJoint joint, float32 torque);
float32 Box2DWheelJointGetMaxMotorTorque(Box2DWheelJoint joint);
float32 Box2DWheelJointGetMotorTorque(Box2DWheelJoint joint, float32 inv_dt);
void Box2DWheelJointSetSpringFrequencyHz(Box2DWheelJoint joint, float32 hz);
float32 Box2DWheelJointGetSpringFrequencyHz(Box2DWheelJoint joint);
void Box2DWheelJointSetSpringDampingRatio(Box2DWheelJoint joint, float32 ratio);
float32 Box2DWheelJointGetSpringDampingRatio(Box2DWheelJoint joint);
void Box2DWheelJointDump(Box2DWheelJoint joint);

#pragma mark Fixture

Box2DShapeType Box2DFixtureGetType(Box2DFixture fixture);
Box2DShape Box2DFixtureGetShape(Box2DFixture fixture);
void Box2DFixtureSetSensor(Box2DFixture fixture, bool sensor);
bool Box2DFixtureIsSensor(Box2DFixture fixture);
void Box2DFixtureSetFilterData(Box2DFixture fixture, Box2DFilter filter);
Box2DFilter Box2DFixtureGetFilterData(Box2DFixture fixture);
void Box2DFixtureRefilter(Box2DFixture fixture);
Box2DBody Box2DFixtureGetBody(Box2DFixture fixture);
Box2DFixture Box2DFixtureGetNext(Box2DFixture fixture);
void* Box2DFixtureGetUserData(Box2DFixture fixture);
void Box2DFixtureSetUserData(Box2DFixture fixture, void* data);
bool Box2DFixtureTestPoint(Box2DFixture fixture, Box2DVector2 p);
bool Box2DFixtureRayCast(Box2DFixture fixture, Box2DRayCastOutput &output, Box2DRayCastInput input, int32 childIndex);
void Box2DFixtureGetMassData(Box2DFixture fixture, Box2DMassData &massData);
void Box2DFixtureSetDensity(Box2DFixture fixture, float32 density);
float32 Box2DFixtureGetDensity(Box2DFixture fixture);
float32 Box2DFixtureGetFriction(Box2DFixture fixture);
void Box2DFixtureSetFriction(Box2DFixture fixture, float32 friction);
float32 Box2DFixtureGetRestitution(Box2DFixture fixture);
void Box2DFixtureSetRestitution(Box2DFixture fixture, float32 restitution);
Box2DAABB Box2DFixtureGetAABB(Box2DFixture fixture, int32 childIndex);
void Box2DFixtureDump(Box2DFixture fixture, int32 bodyIndex);

#pragma mark Shape

Box2DShapeType Box2DShapeGetType(Box2DShape shape);
float32 Box2DShapeGetRadius(Box2DShape shape);
void Box2DShapeSetRadius (Box2DShape shape, float32 radius);
Box2DShape Box2DShapeClone(Box2DShape shape, Box2DBlockAllocator allocator);
VIRTUAL_FUNCTION int32 Box2DShapeGetChildCount(Box2DShape shape);
VIRTUAL_FUNCTION bool Box2DShapeTestPoint(Box2DShape shape, Box2DTransform transform, Box2DVector2 p);
VIRTUAL_FUNCTION bool Box2DShapeRayCast(Box2DShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex);
VIRTUAL_FUNCTION void Box2DShapeComputeAABB(Box2DShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex);
VIRTUAL_FUNCTION void Box2DShapeComputeMass(Box2DShape shape, Box2DMassData &massData, float32 density);

#pragma mark Edge Shape

Box2DEdgeShape Box2DEdgeShapeCreate ();
void Box2DEdgeShapeSet (Box2DEdgeShape shape, Box2DVector2 v1, Box2DVector2 v2);
Box2DEdgeShape Box2DEdgeShapeClone(Box2DEdgeShape shape, Box2DBlockAllocator allocator);
int32 Box2DEdgeShapeGetChildCount(Box2DEdgeShape shape);
bool Box2DEdgeShapeTestPoint(Box2DEdgeShape shape, Box2DTransform transform, Box2DVector2 p);
bool Box2DEdgeShapeRayCast(Box2DEdgeShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex);
void Box2DEdgeShapeComputeAABB(Box2DEdgeShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex);
void Box2DEdgeShapeComputeMass(Box2DEdgeShape shape, Box2DMassData &massData, float32 density);


#pragma mark Circle Shape
Box2DCircleShape Box2DCircleShapeCreate ();
void Box2DCircleShapeSetPosition (Box2DCircleShape shape, Box2DVector2 point);
Box2DCircleShape Box2DCircleShapeClone(Box2DCircleShape shape, Box2DBlockAllocator allocator);
int32 Box2DCircleShapeGetChildCount(Box2DCircleShape shape);
bool Box2DCircleShapeTestPoint(Box2DCircleShape shape, Box2DTransform transform, Box2DVector2 p);
bool Box2DCircleShapeRayCast(Box2DCircleShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex);
void Box2DCircleShapeComputeAABB(Box2DCircleShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex);
void Box2DCircleShapeComputeMass(Box2DCircleShape shape, Box2DMassData &massData, float32 density);
int32 Box2DCircleShapeGetSupport(Box2DCircleShape shape, Box2DVector2 d);
Box2DVector2 Box2DCircleShapeGetSupportVertex(Box2DCircleShape shape, Box2DVector2 d);
int32 Box2DCircleShapeGetVertexCount(Box2DCircleShape shape);
Box2DVector2 Box2DCircleShapeGetVertex(Box2DCircleShape shape, int32 index);

#pragma mark Polygon Shape

Box2DPolygonShape Box2DPolygonShapeCreate ();
Box2DPolygonShape Box2DPolygonShapeClone(Box2DPolygonShape shape, Box2DBlockAllocator allocator);
int32 Box2DPolygonShapeGetChildCount(Box2DPolygonShape shape);
void Box2DPolygonShapeSet (Box2DPolygonShape shape, Box2DVector2 *points, int32 count);
void Box2DPolygonShapeSetAsBox (Box2DPolygonShape shape, float32 hx, float32 hy);
void Box2DPolygonShapeSetAsBox2 (Box2DPolygonShape shape, float32 hx, float32 hy, Box2DVector2 center, float32 angle);
bool Box2DPolygonShapeTestPoint(Box2DPolygonShape shape, Box2DTransform transform, Box2DVector2 p);
bool Box2DPolygonShapeRayCast(Box2DPolygonShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex);
void Box2DPolygonShapeComputeAABB(Box2DPolygonShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex);
void Box2DPolygonShapeComputeMass(Box2DPolygonShape shape, Box2DMassData &massData, float32 density);
int32 Box2DPolygonShapeGetVertexCount(Box2DPolygonShape shape);
Box2DVector2 Box2DPolygonShapeGetVertex(Box2DPolygonShape shape, int32 index);
bool Box2DPolygonShapeValidate(Box2DPolygonShape shape);

#pragma mark Chain Shape

Box2DChainShape Box2DChainShapeCreate ();
void Box2DChainShapeCreateLoop(Box2DChainShape shape, Box2DVector2 *vertices, int32 count);
void Box2DChainShapeCreateChain(Box2DChainShape shape, Box2DVector2 *vertices, int32 count);
void Box2DChainShapeSetPrevVertex(Box2DChainShape shape, Box2DVector2 prevVertex);
void Box2DChainShapeSetNextVertex(Box2DChainShape shape, Box2DVector2 nextVertex);
Box2DChainShape Box2DChainShapeClone(Box2DChainShape shape, Box2DBlockAllocator allocator);
int32 Box2DChainShapeGetChildCount(Box2DChainShape shape);
void Box2DChainShapeGetChildEdge(Box2DChainShape shape, Box2DEdgeShape edge, int32 index);
bool Box2DChainShapeTestPoint(Box2DChainShape shape, Box2DTransform transform, Box2DVector2 p);
bool Box2DChainShapeRayCast(Box2DChainShape shape, Box2DRayCastOutput &output, Box2DRayCastInput input, Box2DTransform transform, int32 childIndex);
void Box2DChainShapeComputeAABB(Box2DChainShape shape, Box2DAABB &aabb, Box2DTransform transform, int32 childIndex);
void Box2DChainShapeComputeMass(Box2DChainShape shape, Box2DMassData &massData, float32 density);

#pragma mark Body

Box2DFixture Box2DBodyCreateFixture(Box2DBody body, Box2DFixtureDefinition def);
Box2DFixture Box2DBodyCreateFixtureWithShape(Box2DBody body, Box2DShape shape, float32 density);
void Box2DBodyDestroyFixture(Box2DBody body, Box2DFixture fixture);
void Box2DBodySetTransform(Box2DBody body, Box2DVector2 position, float32 angle);
Box2DTransform Box2DBodyGetTransform(Box2DBody body);
Box2DVector2 Box2DBodyGetPosition(Box2DBody body);
float32 Box2DBodyGetAngle(Box2DBody body);
Box2DVector2 Box2DBodyGetWorldCenter(Box2DBody body);
Box2DVector2 Box2DBodyGetLocalCenter(Box2DBody body);
void Box2DBodySetLinearVelocity(Box2DBody body, Box2DVector2 v);
Box2DVector2 Box2DBodyGetLinearVelocity(Box2DBody body);
void Box2DBodySetAngularVelocity(Box2DBody body, float32 omega);
float32 Box2DBodyGetAngularVelocity(Box2DBody body);
void Box2DBodyApplyForce(Box2DBody body, Box2DVector2 force, Box2DVector2 point, bool wake);
void Box2DBodyApplyForceToCenter(Box2DBody body, Box2DVector2 force, bool wake);
void Box2DBodyApplyTorque(Box2DBody body, float32 torque, bool wake);
void Box2DBodyApplyAngularImpulse(Box2DBody body, float32 impulse, bool wake);
void Box2DBodyApplyLinearImpulse(Box2DBody body, Box2DVector2 impulse, Box2DVector2 point, bool wake);
float32 Box2DBodyGetMass(Box2DBody body);
float32 Box2DBodyGetInertia(Box2DBody body);
void Box2DBodyGetMassData(Box2DBody body, Box2DMassData &data);
void Box2DBodySetMassData(Box2DBody body, Box2DMassData data);
void Box2DBodyResetMassData(Box2DBody body);
Box2DVector2 Box2DBodyGetWorldPoint(Box2DBody body, Box2DVector2 localPoint);
Box2DVector2 Box2DBodyGetWorldVector(Box2DBody body, Box2DVector2 localVector);
Box2DVector2 Box2DBodyGetLocalPoint(Box2DBody body, Box2DVector2 worldPoint);
Box2DVector2 Box2DBodyGetLocalVector(Box2DBody body, Box2DVector2 worldVector);
Box2DVector2 Box2DBodyGetLinearVelocityFromWorldPoint(Box2DBody body, Box2DVector2 worldPoint);
Box2DVector2 Box2DBodyGetLinearVelocityFromLocalPoint(Box2DBody body, Box2DVector2 localPoint);
float32 Box2DBodyGetLinearDamping(Box2DBody body);
void Box2DBodySetLinearDamping(Box2DBody body, float32 linearDamping);
float32 Box2DBodyGetAngularDamping(Box2DBody body);
void Box2DBodySetAngularDamping(Box2DBody body, float32 angularDamping);
float32 Box2DBodyGetGravityScale(Box2DBody body);
void Box2DBodySetGravityScale(Box2DBody body, float32 scale);
void Box2DBodySetType(Box2DBody body, Box2DBodyType type);
Box2DBodyType Box2DBodyGetType(Box2DBody body);
void Box2DBodySetBullet(Box2DBody body, bool flag);
bool Box2DBodyIsBullet(Box2DBody body);
void Box2DBodySetSleepingAllowed(Box2DBody body, bool flag);
bool Box2DBodyIsSleepingAllowed(Box2DBody body);
void Box2DBodySetAwake(Box2DBody body, bool flag);
bool Box2DBodyIsAwake(Box2DBody body);
void Box2DBodySetActive(Box2DBody body, bool flag);
bool Box2DBodyIsActive(Box2DBody body);
void Box2DBodySetFixedRotation(Box2DBody body, bool flag);
bool Box2DBodyIsFixedRotation(Box2DBody body);
Box2DFixture Box2DBodyGetFixtureList(Box2DBody body);
void* Box2DBodyGetJointList(Box2DBody body);
void* Box2DBodyGetContactList(Box2DBody body);
Box2DBody Box2DBodyGetNext(Box2DBody body);
void* Box2DBodyGetUserData(Box2DBody body);
void Box2DBodySetUserData(Box2DBody body, void* data);
Box2DWorld Box2DBodyGetWorld(Box2DBody body);
void Box2DBodyDump(Box2DBody body);

/*
#pragma mark World Callbacks

Box2DWorldCallbackCenter Box2DWorldCallbackCenterCreate (Box2DWorldCallbacks callbacks);

#pragma mark Debug Draw

Box2DDebugDraw Box2DDebugDrawCreate (Box2DDebugDrawCallbacks callbacks);
void Box2DDebugDrawSetFlags(Box2DDebugDraw debugDraw, uint32 flags);
uint32 Box2DDebugDrawGetFlags(Box2DDebugDraw debugDraw);
void Box2DDebugDrawAppendFlags(Box2DDebugDraw debugDraw, uint32 flags);
void Box2DDebugDrawClearFlags(Box2DDebugDraw debugDraw, uint32 flags);
void Box2DDebugDrawPolygon(Box2DDebugDraw debugDraw, Box2DVector2* vertices, int32 vertexCount, Box2DColor color);
void Box2DDebugDrawSolidPolygon(Box2DDebugDraw debugDraw, Box2DVector2* vertices, int32 vertexCount, Box2DColor color);
void Box2DDebugDrawCircle(Box2DDebugDraw debugDraw, Box2DVector2 center, float32 radius, Box2DColor color);
void Box2DDebugDrawSolidCircle(Box2DDebugDraw debugDraw, Box2DVector2 center, float32 radius, Box2DVector2 axis, Box2DColor color);
void Box2DDebugDrawSegment(Box2DDebugDraw debugDraw, Box2DVector2 p1, Box2DVector2 p2, Box2DColor color);
void Box2DDebugDrawTransform(Box2DDebugDraw debugDraw, Box2DTransform xf);
*/

#pragma mark World

Box2DWorld Box2DWorldCreate (Box2DVector2 gravity);
void Box2DWorldSetDestructionListener(Box2DWorld world, Box2DWorldCallbackCenter listener);
void Box2DWorldSetContactFilter(Box2DWorld world, Box2DWorldCallbackCenter filter);
void Box2DWorldSetContactListener(Box2DWorld world, Box2DWorldCallbackCenter listener);
void Box2DWorldSetDebugDraw(Box2DWorld world, Box2DDebugDraw debugDraw);
Box2DBody Box2DWorldCreateBody(Box2DWorld world, Box2DBodyDefinition def);
void Box2DWorldDestroyBody(Box2DWorld world, Box2DBody body);
Box2DJoint Box2DWorldCreateJoint(Box2DWorld world, Box2DJointDefinition *def);
void Box2DWorldDestroyJoint(Box2DWorld world, Box2DJoint joint);
void Box2DWorldStep(Box2DWorld world, float32 timeStep, int32 velocityIterations, int32 positionIterations);
void Box2DWorldClearForces(Box2DWorld world);
void Box2DWorldDrawDebugData(Box2DWorld world);
void Box2DWorldQueryAABB(Box2DWorld world, Box2DWorldCallbackCenter callback, Box2DAABB aabb);
void Box2DWorldRayCast(Box2DWorld world, Box2DWorldCallbackCenter callback, Box2DVector2 point1, Box2DVector2 point2);
Box2DBody Box2DWorldGetBodyList(Box2DWorld world);
Box2DJoint Box2DWorldGetJointList(Box2DWorld world);
Box2DContact Box2DWorldGetContactList(Box2DWorld world);
void Box2DWorldSetAllowSleeping(Box2DWorld world, bool flag);
bool Box2DWorldGetAllowSleeping(Box2DWorld world);
void Box2DWorldSetWarmStarting(Box2DWorld world, bool flag);
bool Box2DWorldGetWarmStarting(Box2DWorld world);
void Box2DWorldSetContinuousPhysics(Box2DWorld world, bool flag);
bool Box2DWorldGetContinuousPhysics(Box2DWorld world);
void Box2DWorldSetSubStepping(Box2DWorld world, bool flag);
bool Box2DWorldGetSubStepping(Box2DWorld world);
int32 Box2DWorldGetProxyCount(Box2DWorld world);
int32 Box2DWorldGetBodyCount(Box2DWorld world);
int32 Box2DWorldGetJointCount(Box2DWorld world);
int32 Box2DWorldGetContactCount(Box2DWorld world);
int32 Box2DWorldGetTreeHeight(Box2DWorld world);
int32 Box2DWorldGetTreeBalance(Box2DWorld world);
float32 Box2DWorldGetTreeQuality(Box2DWorld world);
void Box2DWorldSetGravity(Box2DWorld world, Box2DVector2 gravity);
Box2DVector2 Box2DWorldGetGravity(Box2DWorld world);
bool Box2DWorldIsLocked(Box2DWorld world);
void Box2DWorldSetAutoClearForces(Box2DWorld world, bool flag);
bool Box2DWorldGetAutoClearForces(Box2DWorld world);
void Box2DWorldShiftOrigin(Box2DWorld world, Box2DVector2 newOrigin);
void Box2DWorldGetContactManager(Box2DWorld world, Box2DContactManager &manager);
b2Profile Box2DWorldGetProfile(Box2DWorld world);
void Box2DWorldDump(Box2DWorld world);

#pragma mark Contact Manager

void Box2DContactManagerAddPair(Box2DContactManager manager, void* proxyUserDataA, void* proxyUserDataB);
void Box2DContactManagerFindNewContacts(Box2DContactManager manager);
void Box2DContactManagerDestroy(Box2DContactManager manager, Box2DContact c);
void Box2DContactManagerCollide(Box2DContactManager manager);


#pragma mark Contact

Box2DManifold Box2DContactGetManifold(Box2DContact contact);
void Box2DContactGetWorldManifold(Box2DContact contact, Box2DWorldManifold &worldManifold);
bool Box2DContactIsTouching(Box2DContact contact);
void Box2DContactSetEnabled(Box2DContact contact, bool flag);
bool Box2DContactIsEnabled(Box2DContact contact);
Box2DContact Box2DContactGetNext(Box2DContact contact);
Box2DFixture Box2DContactGetFixtureA(Box2DContact contact);
int32 Box2DContactGetChildIndexA(Box2DContact contact);
Box2DFixture Box2DContactGetFixtureB(Box2DContact contact);
int32 Box2DContactGetChildIndexB(Box2DContact contact);
void Box2DContactSetFriction(Box2DContact contact, float32 friction);
float32 Box2DContactGetFriction(Box2DContact contact);
void Box2DContactResetFriction(Box2DContact contact);
void Box2DContactSetRestitution(Box2DContact contact, float32 restitution);
float32 Box2DContactGetRestitution(Box2DContact contact);
void Box2DContactResetRestitution(Box2DContact contact);
void Box2DContactSetTangentSpeed(Box2DContact contact, float32 speed);
float32 Box2DContactGetTangentSpeed(Box2DContact contact);
VIRTUAL_FUNCTION void Box2DContactEvaluate(Box2DContact contact, Box2DManifold &manifold, Box2DTransform xfA, Box2DTransform xfB);

#pragma mark Utilities

void Box2DRelease (void* &_class);
b2Version Box2DGetCurrentVersion ();

#endif