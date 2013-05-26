//
//  Box2DC.h
//  Box2DC
//
//  Created by Ryan Joseph on 5/23/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#import "Box2D.h"

#define VIRTUAL_FUNCTION
#define EXTERNAL_FUNCTIONS_BEGIN extern "C" {
#define EXTERNAL_FUNCTIONS_END }
#define TO_DO

/*
 TO-DO:
 
    - Functions to invoke callbacks for DebugDraw and WorlCallbackCenter
    - b2BroadPhase
*/

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

#pragma mark Struct Converters

Box2DVector2 Box2DVector2Make (b2Vec2 in);
b2Vec2 Box2DVector2Make (Box2DVector2 in);
Box2DFilter Box2DFilterMake (b2Filter in);
b2Filter Box2DFilterMake (Box2DFilter in);
Box2DRotation Box2DRotationMake (b2Rot in);
b2Rot Box2DRotationMake (Box2DRotation in);
Box2DTransform Box2DTransformMake (b2Transform in);
b2Transform Box2DTransformMake (Box2DTransform in);
Box2DRayCastInput Box2DRayCastInputMake (b2RayCastInput in);
b2RayCastInput Box2DRayCastInputMake (Box2DRayCastInput in);
Box2DRayCastOutput Box2DRayCastOutputMake (b2RayCastOutput in);
b2RayCastOutput Box2DRayCastOutputMake (Box2DRayCastOutput in);
Box2DMassData Box2DMassDataMake (b2MassData in);
b2MassData Box2DMassDataMake (Box2DMassData in);
Box2DAABB Box2DAABBMake (b2AABB in);
b2AABB Box2DAABBMake (Box2DAABB in);
Box2DManifoldPoint Box2DManifoldPointMake (b2ManifoldPoint in);
b2ManifoldPoint Box2DManifoldPointMake (Box2DManifoldPoint in);
Box2DManifold Box2DManifoldMake (b2Manifold in);
b2Manifold Box2DManifoldMake (Box2DManifold in);
Box2DWorldManifold Box2DWorldManifoldMake (b2WorldManifold in);
b2WorldManifold Box2DWorldManifoldMake (Box2DWorldManifold in);
Box2DContactImpulse Box2DContactImpulseMake (b2ContactImpulse in);
b2ContactImpulse Box2DContactImpulseMake (Box2DContactImpulse in);
Box2DBodyDefinition Box2DBodyDefinitionMake (b2BodyDef in);
b2BodyDef Box2DBodyDefinitionMake (Box2DBodyDefinition in);
Box2DFixtureDefinition Box2DFixtureDefinitionMake (b2FixtureDef in);
b2FixtureDef Box2DFixtureDefinitionMake (Box2DFixtureDefinition in);
Box2DJointDefinition _Box2DJointDefinitionMake (b2JointDef in);
b2JointDef _Box2DJointDefinitionMake (Box2DJointDefinition in);
Box2DDistanceJointDefinition Box2DJointDefinitionMake (b2DistanceJointDef in);
b2DistanceJointDef Box2DJointDefinitionMake (Box2DDistanceJointDefinition in);
Box2DFrictionJointDefinition Box2DJointDefinitionMake (b2FrictionJointDef in);
b2FrictionJointDef Box2DJointDefinitionMake (Box2DFrictionJointDefinition in);
Box2DGearJointDefinition Box2DJointDefinitionMake (b2GearJointDef in);
b2GearJointDef Box2DJointDefinitionMake (Box2DGearJointDefinition in);
Box2DMotorJointDefinition Box2DJointDefinitionMake (b2MotorJointDef in);
b2MotorJointDef Box2DJointDefinitionMake (Box2DMotorJointDefinition in);
Box2DPrismaticJointDefinition Box2DJointDefinitionMake (b2PrismaticJointDef in);
b2PrismaticJointDef Box2DJointDefinitionMake (Box2DPrismaticJointDefinition in);
Box2DPulleyJointDefinition Box2DJointDefinitionMake (b2PulleyJointDef in);
b2PulleyJointDef Box2DJointDefinitionMake (Box2DPulleyJointDefinition in);
Box2DRevoluteJointDefinition Box2DJointDefinitionMake (b2RevoluteJointDef in);
b2RevoluteJointDef Box2DJointDefinitionMake (Box2DRevoluteJointDefinition in);
Box2DRopeJointDefinition Box2DJointDefinitionMake (b2RopeJointDef in);
b2RopeJointDef Box2DJointDefinitionMake (Box2DRopeJointDefinition in);
Box2DWeldJointDefinition Box2DJointDefinitionMake (b2WeldJointDef in);
b2WeldJointDef Box2DJointDefinitionMake (Box2DWeldJointDefinition in);
Box2DWheelJointDefinition Box2DJointDefinitionMake (b2WheelJointDef in);
b2WheelJointDef Box2DJointDefinitionMake (Box2DWheelJointDefinition in);

void Box2DVerticesArrayMake (Box2DVerticesArray &out, const b2Vec2* vertices, int32 vertexCount);