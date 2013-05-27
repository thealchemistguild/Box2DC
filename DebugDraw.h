//
//  DebugDraw.h
//  Box2DC
//
//  Created by Ryan Joseph on 5/24/13.
//  Copyright (c) 2013 Ryan Joseph. All rights reserved.
//

#ifndef BOX2DC_DEBUG_DRAW_H
#define BOX2DC_DEBUG_DRAW_H

#include "Box2D.h"
#include "Box2DC.h"

struct Box2DColor {
	float32 r, g, b;
};

typedef void (*Box2DDebugDrawCallbackDrawPolygon)(Box2DVerticesArray* vertices, int32 vertexCount, Box2DColor color, void* userData);
typedef void (*Box2DDebugDrawCallbackDrawSolidPolygon)(Box2DVerticesArray* vertices, int32 vertexCount, Box2DColor color, void* userData);
typedef void (*Box2DDebugDrawCallbackDrawCircle)(Box2DVector2 center, float32 radius, Box2DColor color, void* userData);
typedef void (*Box2DDebugDrawCallbackDrawSolidCircle)(Box2DVector2 center, float32 radius, Box2DVector2 axis, Box2DColor color, void* userData);
typedef void (*Box2DDebugDrawCallbackDrawSegment)(Box2DVector2 p1, Box2DVector2 p2, Box2DColor color, void* userData);
typedef void (*Box2DDebugDrawCallbackDrawTransform)(Box2DTransform xf, void* userData);

struct Box2DDebugDrawCallbacks {
    void* userData;
    Box2DDebugDrawCallbackDrawPolygon drawPolygon;
    Box2DDebugDrawCallbackDrawSolidPolygon drawSolidPolygon;
    Box2DDebugDrawCallbackDrawCircle drawCircle;
    Box2DDebugDrawCallbackDrawSolidCircle drawSolidCircle;
    Box2DDebugDrawCallbackDrawSegment drawSegment;
    Box2DDebugDrawCallbackDrawTransform drawTransform;
};

class b2DebugDraw: public b2Draw {
public:
	b2DebugDraw(Box2DDebugDrawCallbacks callbacks);
        
	void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
	void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color);
	void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color);
	void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);
	void DrawTransform(const b2Transform& xf);
private:
    Box2DDebugDrawCallbacks m_callbacks;
};

b2Color Box2DColorMake (Box2DColor in);
Box2DColor Box2DColorMake (b2Color in);
Box2DColor Box2DColorMake (float32 r, float32 g, float32 b);

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

#endif
