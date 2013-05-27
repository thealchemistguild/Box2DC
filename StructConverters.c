//
//  StructConverters.c
//  Box2DC
//
//  Created by Ryan Joseph on 5/23/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#include "Box2D.h"
#include "Box2DC.h"

Box2DVector2 Box2DVector2Make (b2Vec2 in) {
    Box2DVector2 out;
    out.x = in.x;
    out.y = in.y;
    return out;
}

b2Vec2 Box2DVector2Make (Box2DVector2 in) {
    b2Vec2 out;
    out.x = in.x;
    out.y = in.y;
    return out;
}

Box2DFilter Box2DFilterMake (b2Filter in) {
    Box2DFilter out;
    out.categoryBits = in.categoryBits;
    out.maskBits = in.maskBits;
    out.groupIndex = in.groupIndex;
    return out;
}

b2Filter Box2DFilterMake (Box2DFilter in) {
    b2Filter out;
    out.categoryBits = in.categoryBits;
    out.maskBits = in.maskBits;
    out.groupIndex = in.groupIndex;
    return out;
}

Box2DRotation Box2DRotationMake (b2Rot in) {
    Box2DRotation out;
    out.s = in.s;
    out.c = in.c;
    return out;
}

b2Rot Box2DRotationMake (Box2DRotation in) {
    b2Rot out;
    out.s = in.s;
    out.c = in.c;
    return out;
}

Box2DTransform Box2DTransformMake (b2Transform in) {
    Box2DTransform out;
    out.p = Box2DVector2Make(in.p);
    out.q = Box2DRotationMake(in.q);
    return out;
}

b2Transform Box2DTransformMake (Box2DTransform in) {
    b2Transform out;
    out.p = Box2DVector2Make(in.p);
    out.q = Box2DRotationMake(in.q);
    return out;
}

Box2DRayCastInput Box2DRayCastInputMake (b2RayCastInput in) {
    Box2DRayCastInput out;
    out.p1 = Box2DVector2Make(in.p1);
    out.p2 = Box2DVector2Make(in.p2);
    out.maxFraction = in.maxFraction;
    return out;
}

b2RayCastInput Box2DRayCastInputMake (Box2DRayCastInput in) {
    b2RayCastInput out;
    out.p1 = Box2DVector2Make(in.p1);
    out.p2 = Box2DVector2Make(in.p2);
    out.maxFraction = in.maxFraction;
    return out;
}

Box2DRayCastOutput Box2DRayCastOutputMake (b2RayCastOutput in) {
	Box2DRayCastOutput out;
    out.normal = Box2DVector2Make(in.normal);
    out.fraction = in.fraction;
    return out;
}

b2RayCastOutput Box2DRayCastOutputMake (Box2DRayCastOutput in) {
	b2RayCastOutput out;
    out.normal = Box2DVector2Make(in.normal);
    out.fraction = in.fraction;
    return out;
}

Box2DMassData Box2DMassDataMake (b2MassData in) {
	Box2DMassData out;
    out.mass = in.mass;
    out.center = Box2DVector2Make(in.center);
    out.I = in.I;
    return out;
}

b2MassData Box2DMassDataMake (Box2DMassData in) {
	b2MassData out;
    out.mass = in.mass;
    out.center = Box2DVector2Make(in.center);
    out.I = in.I;
    return out;
}

Box2DAABB Box2DAABBMake (b2AABB in) {
	Box2DAABB out;
    out.lowerBound = Box2DVector2Make(in.lowerBound);
    out.upperBound = Box2DVector2Make(in.upperBound);
    return out;
}

b2AABB Box2DAABBMake (Box2DAABB in) {
	b2AABB out;
    out.lowerBound = Box2DVector2Make(in.lowerBound);
    out.upperBound = Box2DVector2Make(in.upperBound);
    return out;
}

Box2DManifoldPoint Box2DManifoldPointMake (b2ManifoldPoint in) {
	Box2DManifoldPoint out;
    out.localPoint = Box2DVector2Make(in.localPoint);
    out.normalImpulse = in.normalImpulse;
    out.tangentImpulse = in.tangentImpulse;
    out.id = in.id;
    return out;
}

b2ManifoldPoint Box2DManifoldPointMake (Box2DManifoldPoint in) {
	b2ManifoldPoint out;
    out.localPoint = Box2DVector2Make(in.localPoint);
    out.normalImpulse = in.normalImpulse;
    out.tangentImpulse = in.tangentImpulse;
    out.id = in.id;
    return out;
}

Box2DManifold Box2DManifoldMake (b2Manifold in) {
	Box2DManifold out;
    TO_DO
    //b2ManifoldPoint points[b2_maxManifoldPoints];
    out.localNormal = Box2DVector2Make(in.localNormal);
    out.localPoint = Box2DVector2Make(in.localPoint);
    out.type = in.type;
    out.pointCount = in.pointCount;
    return out;
}

b2Manifold Box2DManifoldMake (Box2DManifold in) {
	b2Manifold out;
    TO_DO
    //b2ManifoldPoint points[b2_maxManifoldPoints];
    out.localNormal = Box2DVector2Make(in.localNormal);
    out.localPoint = Box2DVector2Make(in.localPoint);
    out.type = (b2Manifold::Type) in.type;
    out.pointCount = in.pointCount;
    return out;
}

Box2DWorldManifold Box2DWorldManifoldMake (b2WorldManifold in) {
    Box2DWorldManifold out;
    out.normal = Box2DVector2Make(in.normal);
    TO_DO
	//Box2DVector2 points[b2_maxManifoldPoints];
    return out;
}

b2WorldManifold Box2DWorldManifoldMake (Box2DWorldManifold in) {
    b2WorldManifold out;
    out.normal = Box2DVector2Make(in.normal);
    TO_DO
	//Box2DVector2 points[b2_maxManifoldPoints];
    return out;
}

Box2DContactImpulse Box2DContactImpulseMake (b2ContactImpulse in) {
	Box2DContactImpulse out;
    TO_DO
    //out.normalImpulses = in.normalImpulses;
    //out.tangentImpulses = in.tangentImpulses;
    out.count = in.count;
    return out;
}

b2ContactImpulse Box2DContactImpulseMake (Box2DContactImpulse in) {
	b2ContactImpulse out;
    TO_DO
    //out.normalImpulses = in.normalImpulses;
    //out.tangentImpulses = in.tangentImpulses;
    out.count = in.count;
    return out;
}

Box2DBodyDefinition Box2DBodyDefinitionMake (b2BodyDef in) {
    Box2DBodyDefinition out;
    out.type = in.type;
    out.position = Box2DVector2Make(in.position);
    out.angle = in.angle;
    out.linearVelocity = Box2DVector2Make(in.linearVelocity);
    out.angularVelocity = in.angularVelocity;
    out.linearDamping = in.linearDamping;
    out.angularDamping = in.angularDamping;
    out.allowSleep = in.allowSleep;
    out.awake = in.awake;
    out.fixedRotation = in.fixedRotation;
    out.bullet = in.bullet;
    out.active = in.active;
    out.userData = in.userData;
    out.gravityScale = in.gravityScale;
    return out;
}

b2BodyDef Box2DBodyDefinitionMake (Box2DBodyDefinition in) {
    b2BodyDef out;
    out.type = in.type;
    out.position = Box2DVector2Make(in.position);
    out.angle = in.angle;
    out.linearVelocity = Box2DVector2Make(in.linearVelocity);
    out.angularVelocity = in.angularVelocity;
    out.linearDamping = in.linearDamping;
    out.angularDamping = in.angularDamping;
    out.allowSleep = in.allowSleep;
    out.awake = in.awake;
    out.fixedRotation = in.fixedRotation;
    out.bullet = in.bullet;
    out.active = in.active;
    out.userData = in.userData;
    out.gravityScale = in.gravityScale;
    return out;
}

Box2DFixtureDefinition Box2DFixtureDefinitionMake (b2FixtureDef in) {
    Box2DFixtureDefinition out;
    out.shape = (Box2DShape) in.shape;
    out.userData = in.userData;
    out.friction = in.friction;
    out.restitution = in.restitution;
    out.density = in.density;
    out.isSensor = in.isSensor;
    out.filter = Box2DFilterMake(in.filter);
    return out;
}

b2FixtureDef Box2DFixtureDefinitionMake (Box2DFixtureDefinition in) {
    b2FixtureDef out;
    out.shape = (Box2DShape) in.shape;
    out.userData = in.userData;
    out.friction = in.friction;
    out.restitution = in.restitution;
    out.density = in.density;
    out.isSensor = in.isSensor;
    out.filter = Box2DFilterMake(in.filter);
    return out;
}

Box2DJointDefinition _Box2DJointDefinitionMake (b2JointDef in) {
    Box2DJointDefinition out;
    out.type = in.type;
    out.userData = in.userData;
    out.bodyA = in.bodyA;
    out.bodyB = in.bodyB;
    out.collideConnected = in.collideConnected;
    return out;
}

b2JointDef _Box2DJointDefinitionMake (Box2DJointDefinition in) {
    b2JointDef out;
    out.type = in.type;
    out.userData = in.userData;
    out.bodyA = in.bodyA;
    out.bodyB = in.bodyB;
    out.collideConnected = in.collideConnected;
    return out;
}

void Box2DJointDefinitionCopyStruct (void* in, Box2DJointDefinition def) {
    b2JointDef jd = _Box2DJointDefinitionMake(def);
    memcpy(in, &jd, sizeof(b2JointDef));
}

Box2DDistanceJointDefinition Box2DJointDefinitionMake (b2DistanceJointDef in) {
    Box2DDistanceJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.length = in.length;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

b2DistanceJointDef Box2DJointDefinitionMake (Box2DDistanceJointDefinition in) {
    b2DistanceJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.length = in.length;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

Box2DFrictionJointDefinition Box2DJointDefinitionMake (b2FrictionJointDef in) {
    Box2DFrictionJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.maxForce = in.maxForce;
    out.maxTorque = in.maxTorque;
    return out;
}

b2FrictionJointDef Box2DJointDefinitionMake (Box2DFrictionJointDefinition in) {
    b2FrictionJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.maxForce = in.maxForce;
    out.maxTorque = in.maxTorque;
    return out;
}

Box2DGearJointDefinition Box2DJointDefinitionMake (b2GearJointDef in) {
    Box2DGearJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.joint1 = in.joint1;
    out.joint2 = in.joint2;
    out.ratio = in.ratio;
    return out;
}

b2GearJointDef Box2DJointDefinitionMake (Box2DGearJointDefinition in) {
    b2GearJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.joint1 = in.joint1;
    out.joint2 = in.joint2;
    out.ratio = in.ratio;
    return out;
}

Box2DMotorJointDefinition Box2DJointDefinitionMake (b2MotorJointDef in) {
    Box2DMotorJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.linearOffset = Box2DVector2Make(in.linearOffset);
    out.angularOffset = in.angularOffset;
    out.maxForce = in.maxForce;
    out.maxTorque = in.maxTorque;
    out.correctionFactor = in.correctionFactor;
    return out;
}

b2MotorJointDef Box2DJointDefinitionMake (Box2DMotorJointDefinition in) {
    b2MotorJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.linearOffset = Box2DVector2Make(in.linearOffset);
    out.angularOffset = in.angularOffset;
    out.maxForce = in.maxForce;
    out.maxTorque = in.maxTorque;
    out.correctionFactor = in.correctionFactor;
    return out;
}

Box2DPrismaticJointDefinition Box2DJointDefinitionMake (b2PrismaticJointDef in) {
    Box2DPrismaticJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.localAxisA = Box2DVector2Make(in.localAxisA);
    out.referenceAngle = in.referenceAngle;
    out.enableLimit = in.enableLimit;
    out.lowerTranslation = in.lowerTranslation;
    out.upperTranslation = in.upperTranslation;
    out.enableMotor = in.enableMotor;
    out.maxMotorForce = in.maxMotorForce;
    out.motorSpeed = in.motorSpeed;
    return out;
}

b2PrismaticJointDef Box2DJointDefinitionMake (Box2DPrismaticJointDefinition in) {
    b2PrismaticJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.localAxisA = Box2DVector2Make(in.localAxisA);
    out.referenceAngle = in.referenceAngle;
    out.enableLimit = in.enableLimit;
    out.lowerTranslation = in.lowerTranslation;
    out.upperTranslation = in.upperTranslation;
    out.enableMotor = in.enableMotor;
    out.maxMotorForce = in.maxMotorForce;
    out.motorSpeed = in.motorSpeed;
    return out;
}

Box2DPulleyJointDefinition Box2DJointDefinitionMake (b2PulleyJointDef in) {
    Box2DPulleyJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.groundAnchorA = Box2DVector2Make(in.groundAnchorA);
    out.groundAnchorB = Box2DVector2Make(in.groundAnchorB);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.lengthA = in.lengthA;
    out.lengthB = in.lengthB;
    out.ratio = in.ratio;
    return out;
}

b2PulleyJointDef Box2DJointDefinitionMake (Box2DPulleyJointDefinition in) {
    b2PulleyJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.groundAnchorA = Box2DVector2Make(in.groundAnchorA);
    out.groundAnchorB = Box2DVector2Make(in.groundAnchorB);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.lengthA = in.lengthA;
    out.lengthB = in.lengthB;
    out.ratio = in.ratio;
    return out;
}

Box2DRevoluteJointDefinition Box2DJointDefinitionMake (b2RevoluteJointDef in) {
    Box2DRevoluteJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.referenceAngle = in.referenceAngle;
    out.enableLimit = in.enableLimit;
    out.lowerAngle = in.lowerAngle;
    out.upperAngle = in.upperAngle;
    out.enableMotor = in.enableMotor;
    out.motorSpeed = in.motorSpeed;
    out.maxMotorTorque = in.maxMotorTorque;
    return out;
}

b2RevoluteJointDef Box2DJointDefinitionMake (Box2DRevoluteJointDefinition in) {
    b2RevoluteJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.referenceAngle = in.referenceAngle;
    out.enableLimit = in.enableLimit;
    out.lowerAngle = in.lowerAngle;
    out.upperAngle = in.upperAngle;
    out.enableMotor = in.enableMotor;
    out.motorSpeed = in.motorSpeed;
    out.maxMotorTorque = in.maxMotorTorque;
    return out;
}

Box2DRopeJointDefinition Box2DJointDefinitionMake (b2RopeJointDef in) {
    Box2DRopeJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.maxLength = in.maxLength;
    return out;
}

b2RopeJointDef Box2DJointDefinitionMake (Box2DRopeJointDefinition in) {
    b2RopeJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.maxLength = in.maxLength;
    return out;
}

Box2DWeldJointDefinition Box2DJointDefinitionMake (b2WeldJointDef in) {
    Box2DWeldJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.referenceAngle = in.referenceAngle;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

b2WeldJointDef Box2DJointDefinitionMake (Box2DWeldJointDefinition in) {
    b2WeldJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.referenceAngle = in.referenceAngle;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

Box2DWheelJointDefinition Box2DJointDefinitionMake (b2WheelJointDef in) {
    Box2DWheelJointDefinition out;
    out.joint = _Box2DJointDefinitionMake(in);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.localAxisA = Box2DVector2Make(in.localAxisA);
    out.enableMotor = in.enableMotor;
    out.maxMotorTorque = in.maxMotorTorque;
    out.motorSpeed = in.motorSpeed;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

b2WheelJointDef Box2DJointDefinitionMake (Box2DWheelJointDefinition in) {
    b2WheelJointDef out;
    Box2DJointDefinitionCopyStruct(&out, in.joint);
    out.localAnchorA = Box2DVector2Make(in.localAnchorA);
    out.localAnchorB = Box2DVector2Make(in.localAnchorB);
    out.localAxisA = Box2DVector2Make(in.localAxisA);
    out.enableMotor = in.enableMotor;
    out.maxMotorTorque = in.maxMotorTorque;
    out.motorSpeed = in.motorSpeed;
    out.frequencyHz = in.frequencyHz;
    out.dampingRatio = in.dampingRatio;
    return out;
}

void Box2DVerticesArrayMake (Box2DVerticesArray &out, const b2Vec2* vertices, int32 vertexCount) {
    for (int i=0; i<vertexCount; i++) {
        out[i] = Box2DVector2Make(vertices[i]);
    }
}

void b2Vec2ArrayMake (b2Vec2* out, const Box2DVector2* vertices, int32 vertexCount) {
    for (int i=0; i<vertexCount; i++) {
        out[i] = Box2DVector2Make(vertices[i]);
    }
}