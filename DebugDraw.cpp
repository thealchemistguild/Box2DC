//
//  DebugDraw.cpp
//  Box2DC
//
//  Created by Ryan Joseph on 5/24/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#include "DebugDraw.h"
#import "Box2DC.h"

b2DebugDraw::b2DebugDraw(Box2DDebugDrawCallbacks callbacks) {
    m_callbacks = callbacks;
}

Box2DColor Box2DColorMake (b2Color in) {
    Box2DColor out;
    out.r = in.r;
    out.g = in.g;
    out.b = in.b;
    return out;
}

b2Color Box2DColorMake (Box2DColor in) {
    b2Color out;
    out.r = in.r;
    out.g = in.g;
    out.b = in.b;
    return out;
}

void b2DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    Box2DVerticesArray _vertices;
    Box2DVerticesArrayMake(_vertices, vertices, vertexCount);
    if (m_callbacks.drawPolygon) m_callbacks.drawPolygon(&_vertices, vertexCount, Box2DColorMake(color), m_callbacks.userData);
}

void b2DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    Box2DVerticesArray _vertices;
    Box2DVerticesArrayMake(_vertices, vertices, vertexCount);
    if (m_callbacks.drawSolidPolygon) m_callbacks.drawSolidPolygon(&_vertices, vertexCount, Box2DColorMake(color), m_callbacks.userData);
}

void b2DebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
    if (m_callbacks.drawCircle) m_callbacks.drawCircle(Box2DVector2Make(center), radius, Box2DColorMake(color), m_callbacks.userData);
}

void b2DebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
    if (m_callbacks.drawSolidCircle) m_callbacks.drawSolidCircle(Box2DVector2Make(center), radius, Box2DVector2Make(axis), Box2DColorMake(color), m_callbacks.userData);
}

void b2DebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
    if (m_callbacks.drawSegment) m_callbacks.drawSegment(Box2DVector2Make(p1), Box2DVector2Make(p2), Box2DColorMake(color), m_callbacks.userData);
}

void b2DebugDraw::DrawTransform(const b2Transform& xf) {
    if (m_callbacks.drawTransform) m_callbacks.drawTransform(Box2DTransformMake(xf), m_callbacks.userData);
}
