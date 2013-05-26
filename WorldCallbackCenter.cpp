//
//  WorldCallbackCenter.cpp
//  Box2DC
//
//  Created by Ryan Joseph on 5/23/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#import "Box2D.h"
#import "WorldCallbackCenter.h"

b2WorldCallbackCenter::b2WorldCallbackCenter(Box2DWorldCallbacks callbacks) {
    m_callbacks = callbacks;
}

// b2DestructionListener

void b2WorldCallbackCenter::SayGoodbye(b2Joint* joint) {
    if (m_callbacks.sayGoodbyeJoint) m_callbacks.sayGoodbyeJoint(joint, m_callbacks.userData);
}

void b2WorldCallbackCenter::SayGoodbye(b2Fixture* fixture) {
    if (m_callbacks.sayGoodbyeFixture) m_callbacks.sayGoodbyeFixture(fixture, m_callbacks.userData);
}

// b2ContactFilter
bool b2WorldCallbackCenter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) {
    if (m_callbacks.shouldCollide) {
        return m_callbacks.shouldCollide(fixtureA, fixtureB, m_callbacks.userData);
    } else {
        return false;
    }
}

// b2ContactListener

void b2WorldCallbackCenter::BeginContact(b2Contact* contact) {
    if (m_callbacks.beginContact) m_callbacks.beginContact(contact, m_callbacks.userData);
}

void b2WorldCallbackCenter::EndContact(b2Contact* contact) {
    if (m_callbacks.endContact) m_callbacks.endContact(contact, m_callbacks.userData);
}

void b2WorldCallbackCenter::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
    if (m_callbacks.preSolve) m_callbacks.preSolve(contact, Box2DManifoldMake(*oldManifold), m_callbacks.userData);
}

void b2WorldCallbackCenter::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
    if (m_callbacks.postSolve) m_callbacks.postSolve(contact, Box2DContactImpulseMake(*impulse), m_callbacks.userData);
}

// b2QueryCallback
bool b2WorldCallbackCenter::ReportFixture(b2Fixture* fixture) {
    if (m_callbacks.queryReportFixture) {
        return m_callbacks.queryReportFixture(fixture, m_callbacks.userData);
    } else {
        return false;
    }
}

// b2RayCastCallback
float32 b2WorldCallbackCenter::ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) {
    if (m_callbacks.rayCastReportFixture) {
        return m_callbacks.rayCastReportFixture(fixture, Box2DVector2Make(point), Box2DVector2Make(normal), fraction, m_callbacks.userData);
    } else {
        return false;
    }
}
