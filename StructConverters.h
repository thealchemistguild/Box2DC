//
//  StructConverters.h
//  Box2DC
//
//  Created by Ryan Joseph on 5/26/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#ifndef BOX2DC_STRUCT_CONVERTERS_H
#define BOX2DC_STRUCT_CONVERTERS_H

#include "Box2D.h"
#include "Box2DC.h"

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
void b2Vec2ArrayMake (b2Vec2* out, const Box2DVector2* vertices, int32 vertexCount);

#endif