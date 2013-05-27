//
//  DebugDraw.cpp
//  Box2DC
//
//  Created by Ryan Joseph on 5/24/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#include "DebugDraw.h"
#include "Box2DC.h"
#include "StructConverters.h"

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

Box2DDebugDraw Box2DDebugDrawCreate (Box2DDebugDrawCallbacks callbacks) {
    return new b2DebugDraw(callbacks);
}

void Box2DDebugDrawSetFlags(b2DebugDraw* debugDraw, uint32 flags) {
    debugDraw->SetFlags(flags);
}

uint32 Box2DDebugDrawGetFlags(b2DebugDraw* debugDraw) {
    return debugDraw->GetFlags();
}

void Box2DDebugDrawAppendFlags(b2DebugDraw* debugDraw, uint32 flags) {
    debugDraw->AppendFlags(flags);
}

void Box2DDebugDrawClearFlags(b2DebugDraw* debugDraw, uint32 flags) {
    debugDraw->ClearFlags(flags);
}

void Box2DDebugDrawPolygon(b2DebugDraw* debugDraw, Box2DVector2* vertices, int32 vertexCount, Box2DColor color) {
    b2Vec2 _vertices[b2_maxPolygonVertices];
    b2Vec2ArrayMake((b2Vec2*)&_vertices, vertices, vertexCount);
    debugDraw->DrawPolygon(_vertices, vertexCount, Box2DColorMake(color));
}

void Box2DDebugDrawSolidPolygon(b2DebugDraw* debugDraw, Box2DVector2* vertices, int32 vertexCount, Box2DColor color) {
    b2Vec2 _vertices[b2_maxPolygonVertices];
    b2Vec2ArrayMake((b2Vec2*)&_vertices, vertices, vertexCount);
    debugDraw->DrawPolygon(_vertices, vertexCount, Box2DColorMake(color));
}

void Box2DDebugDrawCircle(b2DebugDraw* debugDraw, Box2DVector2 center, float32 radius, Box2DColor color) {
    debugDraw->DrawCircle(Box2DVector2Make(center), radius, Box2DColorMake(color));
}

void Box2DDebugDrawSolidCircle(b2DebugDraw* debugDraw, Box2DVector2 center, float32 radius, Box2DVector2 axis, Box2DColor color) {
    debugDraw->DrawSolidCircle(Box2DVector2Make(center), radius, Box2DVector2Make(axis), Box2DColorMake(color));
}

void Box2DDebugDrawSegment(b2DebugDraw* debugDraw, Box2DVector2 p1, Box2DVector2 p2, Box2DColor color) {
    debugDraw->DrawSegment(Box2DVector2Make(p1), Box2DVector2Make(p2), Box2DColorMake(color));
}

void Box2DDebugDrawTransform(b2DebugDraw* debugDraw, Box2DTransform xf) {
    debugDraw->DrawTransform(Box2DTransformMake(xf));
}