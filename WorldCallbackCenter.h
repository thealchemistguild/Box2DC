//
//  WorldCallbackCenter.h
//  Box2DC
//
//  Created by Ryan Joseph on 5/23/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#import "Box2D.h"
#import "Box2DC.h"

// b2DestructionListener
typedef void (*Box2DWorldCallbackSayGoodbyeJoint)(Box2DJoint joint, void* userData);
typedef void (*Box2DWorldCallbackSayGoodbyeFixture)(Box2DFixture fixture, void* userData);

// b2ContactFilter
typedef bool (*Box2DWorldCallbackShouldCollide)(Box2DFixture fixtureA, Box2DFixture fixtureB, void* userData);

// b2ContactListener
typedef void (*Box2DWorldCallbackBeginContact)(Box2DContact contact, void* userData);
typedef void (*Box2DWorldCallbackEndContact)(Box2DContact contact, void* userData);
typedef void (*Box2DWorldCallbackPreSolve)(Box2DContact contact, Box2DManifold oldManifold, void* userData);
typedef void (*Box2DWorldCallbackPostSolve)(Box2DContact contact, Box2DContactImpulse impulse, void* userData);

// b2QueryCallback
typedef bool (*Box2DWorldCallbackQueryReportFixture)(Box2DFixture fixture, void* userData);

// b2RayCastCallback
typedef float32 (*Box2DWorldCallbackRayCastReportFixture)(Box2DFixture fixture, Box2DVector2 point, Box2DVector2 normal, float32 fraction, void* userData);

struct Box2DWorldCallbacks {
    void* userData;
    Box2DWorldCallbackSayGoodbyeJoint sayGoodbyeJoint;
    Box2DWorldCallbackSayGoodbyeFixture sayGoodbyeFixture;
    Box2DWorldCallbackShouldCollide shouldCollide;
    Box2DWorldCallbackBeginContact beginContact;
    Box2DWorldCallbackEndContact endContact;
    Box2DWorldCallbackPreSolve preSolve;
    Box2DWorldCallbackPostSolve postSolve;
    Box2DWorldCallbackQueryReportFixture queryReportFixture;
    Box2DWorldCallbackRayCastReportFixture rayCastReportFixture;
};
typedef struct Box2DWorldCallbacks Box2DWorldCallbacks;

class b2WorldCallbackCenter: public b2DestructionListener, b2ContactFilter, b2ContactListener, b2QueryCallback, b2RayCastCallback
{
public:
    
    b2WorldCallbackCenter(Box2DWorldCallbacks callbacks);
    
    // b2DestructionListener
    void SayGoodbye(b2Joint* joint);
	void SayGoodbye(b2Fixture* fixture);
    
    // b2ContactFilter
    bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB);
    
    // b2ContactListener
    void BeginContact(b2Contact* contact);
	void EndContact(b2Contact* contact);
	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);

    // b2QueryCallback
    bool ReportFixture(b2Fixture* fixture);
    
    // b2RayCastCallback
	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction);
    
private:
    Box2DWorldCallbacks m_callbacks;
};

typedef b2WorldCallbackCenter* Box2DWorldCallbackCenter;