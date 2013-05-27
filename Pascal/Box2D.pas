{$mode objfpc}
{$modeswitch objectivec1}
{$linklib libBox2DC.dylib}

unit Box2D;
interface
uses
	CTypes;

{=============================================}
{@! ___Types___ } 
{=============================================}
type
	Int8 = cint8;
	Int16 = cint16;
	Int32 = cint32;
	UInt8 = cuint8;
	UInt16 = cuint16;
	UInt32 = cuint32;
	Float32 = cfloat;
	Float64 = cdouble;

const
	Box2DMaxManifoldPoints = 2;

const
	Box2DMaxPolygonVertices = 8;
	
const
	Box2DBodyTypeStaticBody = 0;
	Box2DBodyTypeKinematicBody = 1;
	Box2DBodyTypeDynamicBody = 2;

const
	Box2DShapeTypeCircle = 0;
	Box2DShapeTypeEdge = 1;
	Box2DShapeTypePolygon = 2;
	Box2DShapeTypeChain = 3;
	Box2DShapeTypeCount = 4;

const
	Box2DJointTypeUnknownJoint = 0;
	Box2DJointTypeRevoluteJoint = 1;
	Box2DJointTypePrismaticJoint = 2;
	Box2DJointTypeDistanceJoint = 3;
	Box2DJointTypePulleyJoint = 4;
	Box2DJointTypeMouseJoint = 5;
	Box2DJointTypeGearJoint = 6;
	Box2DJointTypeWheelJoint = 7;
	Box2DJointTypeWeldJoint = 8;
	Box2DJointTypeFrictionJoint = 9;
	Box2DJointTypeRopeJoint = 10;
	Box2DJointTypeMotorJoint = 11;

const
	Box2DManifoldTypeCircles = 0;
	Box2DManifoldTypeFaceA = 1;
	Box2DManifoldTypeFaceB = 2;

const
	Box2DContactFeatureVertex = 0;
	Box2DContactFeatureFace = 1;

const
	Box2DLimitStateInactiveLimit = 0;
	Box2DLimitStateAtLowerLimit = 1;
	Box2DLimitStateAtUpperLimit = 2;
	Box2DLimitStateEqualLimits = 3;

const
	Box2DDrawFlagShapeBit = $0001;
	Box2DDrawFlagJointBit	= $0002;
	Box2DDrawFlagAABBBit = $0004;
	Box2DDrawFlagPairBit = $0008;
	Box2DDrawFlagCenterOfMassBit = $0010;

// Pixel to metres ratio. Box2D uses metres as the unit for measurement.
// This ratio defines how many pixels correspond to 1 Box2D "metre"
// Box2D is optimized for objects of 1x1 metre therefore it makes sense
// to define the ratio so that your most common object type is 1x1 metre.
const
	PTM_RATIO = 32;

const
	BOX2D_PI = 3.14159265359;

type
	_Box2DOpaqueClass = record
	end;
	Box2DOpaqueClass = ^_Box2DOpaqueClass;

// ??? WARNING: not sure how these enums should be translated
type
	Box2DBodyType = cint;
	Box2DShapeType = cint;
	Box2DJointType = cint;
	Box2DManifoldType = cint;
	Box2DLimitState = cint;
	
type
	Box2DBlockAllocator = Box2DOpaqueClass;
	Box2DWorld = Box2DOpaqueClass;
	Box2DBody = Box2DOpaqueClass;
	Box2DFixture = Box2DOpaqueClass;
	Box2DContact = Box2DOpaqueClass;
	Box2DContactManager = Box2DOpaqueClass;
	Box2DEdgeShape = Box2DOpaqueClass;
	Box2DShape = Box2DOpaqueClass;
	Box2DCircleShape = Box2DOpaqueClass;
	Box2DPolygonShape = Box2DOpaqueClass;
	Box2DChainShape = Box2DOpaqueClass;
	Box2DJoint = Box2DOpaqueClass;
	Box2DDistanceJoint = Box2DOpaqueClass;
	Box2DFrictionJoint = Box2DOpaqueClass;
	Box2DGearJoint = Box2DOpaqueClass;
	Box2DMotorJoint = Box2DOpaqueClass;
	Box2DMouseJoint = Box2DOpaqueClass;
	Box2DPrismaticJoint = Box2DOpaqueClass;
	Box2DPulleyJoint = Box2DOpaqueClass;
	Box2DRevoluteJoint = Box2DOpaqueClass;
	Box2DRopeJoint = Box2DOpaqueClass;
	Box2DWeldJoint = Box2DOpaqueClass;
	Box2DWheelJoint = Box2DOpaqueClass;

type
	Box2DVersion = record
		major: int32;
		minor: int32;
		revision: int32;
	end;

type
	Box2DVector2 = record
		x: Float32;
		y: Float32;
	end;
	Box2DVerticesArray = array[0..Box2DMaxPolygonVertices] of Box2DVector2;
	Box2DVerticesArrayPointer = ^Box2DVerticesArray;
	
type
	Box2DRotation = record
		s: Float32;
		c: Float32;
	end;
	
type
	Box2DTransform = record
		p: Box2DVector2;
		q: Box2DRotation;
	end;
	
type
	Box2DFilter = record
		categoryBits: UInt16;
		maskBits: UInt16;
		groupIndex: int16;
	end;

type
	Box2DRayCastInput = record
		p1: Box2DVector2;
		p2: Box2DVector2;
		maxFraction: Float32;
	end;

type
	Box2DRayCastOutput = record
		normal: Box2DVector2;
		fraction: Float32;
	end;
	
type
	Box2DMassData = record
		mass: Float32;
		center: Box2DVector2;
		i: Float32;
	end;

type
	Box2DAABB = record
		lowerBound: Box2DVector2;
		upperBound: Box2DVector2;
	end;

type
	Box2DProfile = record
		step: float32;
		collide: float32;
		solve: float32;
		solveInit: float32;
		solveVelocity: float32;
		solvePosition: float32;
		broadphase: float32;
		solveTOI: float32;
	end; 

type 
	Box2DContactFeature = record
		indexA: uint8;
		indexB: uint8;
		typeA: uint8;
		typeB: uint8;
	end;

type
	Box2DContactID = record
		cf: Box2DContactFeature;
		key: uint32;
	end;

type
	Box2DManifoldPoint = record
		localPoint: Box2DVector2;
		normalImpulse: Float32;
		tangentImpulse: Float32;
		id: Box2DContactID;
	end;

type
	Box2DManifold = record
		points: array[0..Box2DMaxManifoldPoints] of Box2DManifoldPoint;
		localNormal: Box2DVector2;
		localPoint: Box2DVector2;
		theType: Box2DManifoldType;
		pointCount: int32;
	end;
	
type
	Box2DWorldManifold = record
		normal: Box2DVector2;
		points: array[0..Box2DMaxManifoldPoints] of Box2DVector2;
	end;

type
	Box2DContactImpulse = record
		normalImpulses: array[0..Box2DMaxManifoldPoints] of Float32;
		tangentImpulses: array[0..Box2DMaxManifoldPoints] of Float32;
		count: int32;
	end;

type
	Box2DBodyDefinition = record
		theType: Box2DBodyType;
		position: Box2DVector2;
		angle: Float32;
		linearVelocity: Box2DVector2;
		angularVelocity: Float32;
		linearDamping: Float32;
		angularDamping: Float32;
	  allowSleep: cbool;
		awake: cbool;
		fixedRotation: cbool;
		bullet: cbool;
		active: cbool;
		userData: pointer;
		gravityScale: Float32;
	end;

type
	Box2DFixtureDefinition = record
		shape: Box2DShape;
		userData: pointer;
		friction: Float32;
		restitution: Float32;
		density: Float32;
		isSensor: cbool;
		filter: Box2DFilter;
	end;

type
	Box2DJointDefinition = record
		theType: Box2DJointType;
		userData: pointer;
		bodyA: Box2DBody;
		bodyB: Box2DBody;
		collideConnected: cbool;
	end;
	
type
	Box2DDistanceJointDefinition = record
	  joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		length: Float32;
		frequencyHz: Float32;
		dampingRatio: Float32;
	end;

type
	Box2DFrictionJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		maxForce: Float32;
		maxTorque: Float32;
	end;
	
type
	Box2DGearJointDefinition = record
		joint: Box2DJointDefinition;
		joint1: Box2DJoint;
		joint2: Box2DJoint;
		ratio: Float32;
	end;

type
	Box2DMotorJointDefinition = record
		joint: Box2DJointDefinition;
		linearOffset: Box2DVector2;
		angularOffset: Float32;
		maxForce: Float32;
		maxTorque: Float32;
		correctionFactor: Float32;
	end;

type
	Box2DMouseJointDefinition = record
		joint: Box2DJointDefinition;
		target: Box2DVector2;
		maxForce: Float32;
		frequencyHz: Float32;
		dampingRatio: Float32;
	end;

type
	Box2DPulleyJointDefinition = record
		joint: Box2DJointDefinition;
		groundAnchorA: Box2DVector2;
		groundAnchorB: Box2DVector2;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		lengthA: Float32;
		lengthB: Float32;
		ratio: Float32;
	end;

type
	Box2DPrismaticJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		localAxisA: Box2DVector2;
		referenceAngle: Float32;
		enableLimit: cbool;
		lowerTranslation: Float32;
		upperTranslation: Float32;
		enableMotor: cbool;
		maxMotorForce: Float32;
		motorSpeed: Float32;
	end;

type
	Box2DRevoluteJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		referenceAngle: Float32;
		enableLimit: cbool;
		lowerAngle: Float32;
		upperAngle: Float32;
		enableMotor: cbool;
		motorSpeed: Float32;
		maxMotorTorque: Float32;
	end;

type
	Box2DRopeJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		maxLength: Float32;
	end;

type
	Box2DWeldJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		referenceAngle: Float32;
		frequencyHz: Float32;
		dampingRatio: Float32;
	end;

type
	Box2DWheelJointDefinition = record
		joint: Box2DJointDefinition;
		localAnchorA: Box2DVector2;
		localAnchorB: Box2DVector2;
		localAxisA: Box2DVector2;
		enableMotor: cbool;
		maxMotorTorque: Float32;
		motorSpeed: Float32;
		frequencyHz: Float32;
		dampingRatio: Float32;
	end;

type
	Box2DJointDefinitionPointer = pointer;

{=============================================}
{@! ___World Callbacks___ } 
{=============================================}

type
	Box2DWorldCallbackSayGoodbyeJoint = procedure (joint: Box2DJoint; userData: pointer); cdecl;
	Box2DWorldCallbackSayGoodbyeFixture = procedure (fixture: Box2DFixture; userData: pointer); cdecl;
	Box2DWorldCallbackShouldCollide = function (fixtureA: Box2DFixture; fixtureB: Box2DFixture; userData: pointer): cbool; cdecl;
	Box2DWorldCallbackBeginContact = procedure (contact: Box2DContact; userData: pointer); cdecl;
	Box2DWorldCallbackEndContact = procedure (contact: Box2DContact; userData: pointer); cdecl;
	Box2DWorldCallbackPreSolve = procedure (contact: Box2DContact; oldManifold: Box2DManifold; userData: pointer); cdecl;
	Box2DWorldCallbackPostSolve = procedure (contact: Box2DContact; impulse: Box2DContactImpulse; userData: pointer); cdecl;
	Box2DWorldCallbackQueryReportFixture = function (fixture: Box2DFixture; userData: pointer): cbool; cdecl;
	Box2DWorldCallbackRayCastReportFixture = function (fixture: Box2DFixture; point: Box2DVector2; normal: Box2DVector2; fraction: float32; userData: pointer): float32; cdecl;

type
	Box2DWorldCallbacks = record
		userData: pointer;
		sayGoodbyeJoint: Box2DWorldCallbackSayGoodbyeJoint;
		sayGoodbyeFixture: Box2DWorldCallbackSayGoodbyeFixture;
		shouldCollide: Box2DWorldCallbackShouldCollide;
		beginContact: Box2DWorldCallbackBeginContact;
		endContact: Box2DWorldCallbackEndContact;
		preSolve: Box2DWorldCallbackPreSolve;
		postSolve: Box2DWorldCallbackPostSolve;
		queryReportFixture: Box2DWorldCallbackQueryReportFixture;
		rayCastReportFixture: Box2DWorldCallbackRayCastReportFixture;
	end;

type
	Box2DWorldCallbackCenter = Box2DOpaqueClass;

{=============================================}
{@! ___Draw Callbacks___ } 
{=============================================}
type
	Box2DColor = record
		red: float32;
		green: float32;
		blue: float32;
	end;

type
	Box2DDebugDrawCallbackDrawPolygon = procedure (vertices: Box2DVerticesArrayPointer; vertexCount: int32; color: Box2DColor; userData: pointer); cdecl;
	Box2DDebugDrawCallbackDrawSolidPolygon = procedure (vertices: Box2DVerticesArrayPointer; vertexCount: int32; color: Box2DColor; userData: pointer); cdecl;
	Box2DDebugDrawCallbackDrawCircle = procedure (center: Box2DVector2; radius: float32; color: Box2DColor; userData: pointer); cdecl;
	Box2DDebugDrawCallbackDrawSolidCircle = procedure (center: Box2DVector2; radius: float32; axis: Box2DVector2; color: Box2DColor; userData: pointer); cdecl;
	Box2DDebugDrawCallbackDrawSegment = procedure (p1: Box2DVector2; p2: Box2DVector2; color: Box2DColor; userData: pointer); cdecl;
	Box2DDebugDrawCallbackDrawTransform = procedure (xf: Box2DTransform; userData: pointer); cdecl;

type
	Box2DDebugDrawCallbacks = record
		userData: pointer;
		drawPolygon: Box2DDebugDrawCallbackDrawPolygon;
		drawSolidPolygon: Box2DDebugDrawCallbackDrawSolidPolygon;
		drawCircle: Box2DDebugDrawCallbackDrawCircle;
		drawSolidCircle: Box2DDebugDrawCallbackDrawSolidCircle;
		drawSegment: Box2DDebugDrawCallbackDrawSegment;
		drawTransform: Box2DDebugDrawCallbackDrawTransform;
	end;

type
	Box2DDebugDraw = Box2DOpaqueClass;

{=============================================}
{@! ___Functions___ } 
{=============================================}

{=============================================}
{@! ___World Manifold___ } 
{=============================================}

procedure Box2DWorldManifoldInitialize (var worldManifold: Box2DWorldManifold; manifold: Box2DManifold; xfA: Box2DTransform; radiusA: float32; xfB: Box2DTransform; radiusB: float32); cdecl; external;

{=============================================}
{@! ___Vector___ } 
{=============================================}

function Box2DVectorMake (x: float32; y: float32): Box2DVector2;
procedure Box2DVectorShow (vec: Box2DVector2);

procedure Box2DVectorSetZero (var vec: Box2DVector2); cdecl; external;
procedure Box2DVectorSet (var vec: Box2DVector2; x_: float32; y_: float32); cdecl; external;
function Box2DVectorLength (vec: Box2DVector2): float32; cdecl; external;
function Box2DVectorLengthSquared (vec: Box2DVector2): float32; cdecl; external;
function Box2DVectorNormalize (var vec: Box2DVector2): float32; cdecl; external;
function Box2DVectorIsValid (vec: Box2DVector2): cbool; cdecl; external;
function Box2DVectorSkew (vec: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DVectorNegate (vec: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DVectorAddVector (vec: Box2DVector2; add: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DVectorSubtractVector (vec: Box2DVector2; subtract: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DVectorMultiply (vec: Box2DVector2; a: float32): Box2DVector2; cdecl; external;

{=============================================}
{@! ___Transform___ } 
{=============================================}
    
function Box2DTransformMake (position: Box2DVector2; rotation: Box2DRotation): Box2DTransform; cdecl; external;
procedure Box2DTransformSetIdentity (var transform: Box2DTransform); cdecl; external;
procedure Box2DTransformSet (var transform: Box2DTransform; position: Box2DVector2; angle: float32); cdecl; external;

{=============================================}
{@! ___Rotation___ } 
{=============================================}
    
function Box2DRotationMake (angle: float32): Box2DRotation; cdecl; external;
procedure Box2DRotationSet (var rotation: Box2DRotation; angle: float32); cdecl; external;
procedure Box2DRotationSetIdentity (var rotation: Box2DRotation); cdecl; external;
function Box2DRotationGetAngle (rotation: Box2DRotation): float32; cdecl; external;
function Box2DRotationGetXAxis (rotation: Box2DRotation): Box2DVector2; cdecl; external;
function Box2DRotationGetYAxis (rotation: Box2DRotation): Box2DVector2; cdecl; external;

{=============================================}
{@! ___AABB___ } 
{=============================================}

function Box2DAABBIsValid (aabb: Box2DAABB): cbool; cdecl; external;
function Box2DAABBGetCenter (aabb: Box2DAABB): Box2DVector2; cdecl; external;
function Box2DAABBGetExtents (aabb: Box2DAABB): Box2DVector2; cdecl; external;
function Box2DAABBGetPerimeter (aabb: Box2DAABB): float32; cdecl; external;
procedure Box2DAABBCombine (var ioAABB: Box2DAABB; aabb: Box2DAABB); cdecl; external;
procedure Box2DAABBCombine2 (var ioAABB: Box2DAABB; aabb1, aabb2: Box2DAABB); cdecl; external;
function Box2DAABBContains (thisAABB: Box2DAABB; aabb: Box2DAABB): cbool; cdecl; external;
function Box2DAABBRayCast (aabb: Box2DAABB; var output: Box2DRayCastOutput; input: Box2DRayCastInput): cbool; cdecl; external;

{=============================================}
{@! ___Definitions___ } 
{=============================================}

{=============================================}
{@! ___Body Definition___ } 
{=============================================}

function Box2DBodyDefinitionDefault: Box2DBodyDefinition; cdecl; external;

{=============================================}
{@! ___Fixture Definition___ } 
{=============================================}

function Box2DFixtureDefinitionDefault: Box2DFixtureDefinition; cdecl; external;

{=============================================}
{@! ___Distance Joint Definition___ } 
{=============================================}

function Box2DDistanceJointDefinitionDefault: Box2DDistanceJointDefinition; cdecl; external;
procedure Box2DDistanceJointDefinitionInitialize (var def: Box2DDistanceJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchorA: Box2DVector2; anchorB: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Friction Joint Definition___ } 
{=============================================}

function Box2DFrictionJointDefinitionDefault: Box2DFrictionJointDefinition; cdecl; external;
procedure Box2DFrictionJointDefinitionInitialize (var def: Box2DFrictionJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchor: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Motor Joint Definition___ } 
{=============================================}

function Box2DMotorJointDefinitionDefault: Box2DMotorJointDefinition; cdecl; external;
procedure Box2DMotorJointDefinitionInitialize (var def: Box2DMotorJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody); cdecl; external;

{=============================================}
{@! ___Revolute Joint Definition___ } 
{=============================================}

function Box2DRevoluteJointDefinitionDefault: Box2DRevoluteJointDefinition; cdecl; external;
procedure Box2DRevoluteJointDefinitionInitialize (var def: Box2DRevoluteJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchor: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Pulley Joint Definition___ } 
{=============================================}

function Box2DPulleyJointDefinitionDefault: Box2DPulleyJointDefinition; cdecl; external;
procedure Box2DPulleyJointDefinitionInitialize (var def: Box2DPulleyJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody;
                                           groundAnchorA: Box2DVector2; groundAnchorB: Box2DVector2;
                                           anchorA: Box2DVector2; anchorB: Box2DVector2;
                                           ratio: float32); cdecl; external;

{=============================================}
{@! ___Prismatic Joint Definition___ } 
{=============================================}

function Box2DPrismaticJointDefinitionDefault: Box2DPrismaticJointDefinition; cdecl; external;
procedure Box2DPrismaticJointDefinitionInitialize (var def: Box2DPrismaticJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchor: Box2DVector2; axis: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Weld Joint Definition___ } 
{=============================================}
        
function Box2DWeldJointDefinitionDefault: Box2DWeldJointDefinition; cdecl; external;
procedure Box2DWeldJointDefinitionInitialize (var def: Box2DWeldJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchor: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Wheel Joint Definition___ } 
{=============================================}

function Box2DWheelJointDefinitionDefault: Box2DWheelJointDefinition; cdecl; external;
procedure Box2DWheelJointDefinitionInitialize (var def: Box2DWheelJointDefinition; bodyA: Box2DBody; bodyB: Box2DBody; anchor: Box2DVector2; axis: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Joints___ } 
{=============================================}

{=============================================}
{@! ___Joint___ } 
{=============================================}

function Box2DJointGetType (joint: Box2DJoint): Box2DJointType; cdecl; external;
function Box2DJointGetBodyA (joint: Box2DJoint): Box2DBody; cdecl; external;
function Box2DJointGetBodyB (joint: Box2DJoint): Box2DBody; cdecl; external;    
function Box2DJointGetNext (joint: Box2DJoint): Box2DJoint; cdecl; external;    
function Box2DJointGetUserData(joint: Box2DJoint): pointer; cdecl; external;

procedure Box2DJointSetUserData (joint: Box2DJoint; data: pointer); cdecl; external;
function Box2DJointIsActive (joint: Box2DJoint): cbool; cdecl; external;
function Box2DJointGetCollideConnected (joint: Box2DJoint): cbool; cdecl; external;

{=============================================}
{@! ___Distance Joint___ } 
{=============================================}

function Box2DDistanceJointGetReactionForce (joint: Box2DDistanceJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DDistanceJointGetReactionTorque (joint: Box2DDistanceJoint; inv_dt: float32): float32; cdecl; external;
function Box2DDistanceJointGetLocalAnchorA (joint: Box2DDistanceJoint): Box2DVector2; cdecl; external;
function Box2DDistanceJointGetLocalAnchorB (joint: Box2DDistanceJoint): Box2DVector2; cdecl; external;
procedure Box2DDistanceJointSetLength (joint: Box2DDistanceJoint; length: float32); cdecl; external;
function Box2DDistanceJointGetLength (joint: Box2DDistanceJoint): float32; cdecl; external;
procedure Box2DDistanceJointSetFrequency (joint: Box2DDistanceJoint; hz: float32); cdecl; external;
function Box2DDistanceJointGetFrequency (joint: Box2DDistanceJoint): float32; cdecl; external;
procedure Box2DDistanceJointSetDampingRatio (joint: Box2DDistanceJoint; ratio: float32); cdecl; external;
function Box2DDistanceJointGetDampingRatio (joint: Box2DDistanceJoint): float32; cdecl; external;
procedure Box2DDistanceJointDump (joint: Box2DDistanceJoint); cdecl; external;

{=============================================}
{@! ___Friction Joint___ } 
{=============================================}

function Box2DFrictionJointGetAnchorA (joint: Box2DFrictionJoint): Box2DVector2; cdecl; external;
function Box2DFrictionJointGetAnchorB (joint: Box2DFrictionJoint): Box2DVector2; cdecl; external;
function Box2DFrictionJointGetReactionForce (joint: Box2DFrictionJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DFrictionJointGetReactionTorque (joint: Box2DFrictionJoint; inv_dt: float32): float32; cdecl; external;
function Box2DFrictionJointGetLocalAnchorA (joint: Box2DFrictionJoint): Box2DVector2; cdecl; external;
function Box2DFrictionJointGetLocalAnchorB (joint: Box2DFrictionJoint): Box2DVector2; cdecl; external;
procedure Box2DFrictionJointSetMaxForce (joint: Box2DFrictionJoint; force: float32); cdecl; external;
function Box2DFrictionJointGetMaxForce (joint: Box2DFrictionJoint): float32; cdecl; external;
procedure Box2DFrictionJointSetMaxTorque (joint: Box2DFrictionJoint; torque: float32); cdecl; external;
function Box2DFrictionJointGetMaxTorque (joint: Box2DFrictionJoint): float32; cdecl; external;
procedure Box2DFrictionJointDump (joint: Box2DDistanceJoint); cdecl; external;

{=============================================}
{@! ___Gear Joint___ } 
{=============================================}

function Box2DGearJointGetAnchorA (joint: Box2DGearJoint): Box2DVector2; cdecl; external;
function Box2DGearJointGetAnchorB (joint: Box2DGearJoint): Box2DVector2; cdecl; external;
function Box2DGearJointGetReactionForce (joint: Box2DGearJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DGearJointGetReactionTorque (joint: Box2DGearJoint; inv_dt: float32): float32; cdecl; external;
function Box2DGearJointGetJoint1 (joint: Box2DGearJoint): Box2DJoint; cdecl; external;
function Box2DGearJointGetJoint2 (joint: Box2DGearJoint): Box2DJoint; cdecl; external;
procedure Box2DGearJointSetRatio (joint: Box2DGearJoint; ratio: float32); cdecl; external;
function Box2DGearJointGetRatio (joint: Box2DGearJoint): float32; cdecl; external;
procedure Box2DGearJointDump (joint: Box2DGearJoint); cdecl; external;

{=============================================}
{@! ___Motor Joint___ } 
{=============================================}

function Box2DMotorJointGetAnchorA (joint: Box2DMotorJoint): Box2DVector2; cdecl; external;
function Box2DMotorJointGetAnchorB (joint: Box2DMotorJoint): Box2DVector2; cdecl; external;
function Box2DMotorJointGetReactionForce (joint: Box2DMotorJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DMotorJointGetReactionTorque (joint: Box2DMotorJoint; inv_dt: float32): float32; cdecl; external;
procedure Box2DMotorJointSetLinearOffset (joint: Box2DMotorJoint; linearOffset: Box2DVector2); cdecl; external;
function Box2DMotorJointGetLinearOffset (joint: Box2DMotorJoint): Box2DVector2; cdecl; external;
procedure Box2DMotorJointSetAngularOffset (joint: Box2DMotorJoint; angularOffset: float32); cdecl; external;
function Box2DMotorJointGetAngularOffset (joint: Box2DMotorJoint): float32; cdecl; external;
procedure Box2DMotorJointSetMaxForce (joint: Box2DMotorJoint; force: float32); cdecl; external;
function Box2DMotorJointGetMaxForce (joint: Box2DMotorJoint): float32; cdecl; external;
procedure Box2DMotorJointSetMaxTorque (joint: Box2DMotorJoint; torque: float32); cdecl; external;
function Box2DMotorJointGetMaxTorque (joint: Box2DFrictionJoint): float32; cdecl; external;
procedure Box2DMotorJointDump (joint: Box2DMotorJoint); cdecl; external;   

{=============================================}
{@! ___Mouse Joint___ } 
{=============================================}

function Box2DMouseJointGetAnchorA (joint: Box2DMouseJoint): Box2DVector2; cdecl; external;
function Box2DMouseJointGetAnchorB (joint: Box2DMouseJoint): Box2DVector2; cdecl; external;
function Box2DMouseJointGetReactionForce (joint: Box2DMouseJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DMouseJointGetReactionTorque (joint: Box2DMouseJoint; inv_dt: float32): float32; cdecl; external;
procedure Box2DMouseJointSetTarget (joint: Box2DMouseJoint; const target: Box2DVector2); cdecl; external;
function Box2DMouseJointGetTarget (joint: Box2DMouseJoint): Box2DVector2; cdecl; external;
procedure Box2DMouseJointSetMaxForce (joint: Box2DMouseJoint; force: float32); cdecl; external;
function Box2DMouseJointGetMaxForce (joint: Box2DMouseJoint): float32; cdecl; external;
procedure Box2DMouseJointSetFrequency (joint: Box2DMouseJoint; hz: float32); cdecl; external;
function Box2DMouseJointGetFrequency (joint: Box2DMouseJoint): float32; cdecl; external;
procedure Box2DMouseJointSetDampingRatio (joint: Box2DMouseJoint; ratio: float32); cdecl; external;
function Box2DMouseJointGetDampingRatio (joint: Box2DMouseJoint): float32; cdecl; external;
procedure Box2DMouseJointDump (joint: Box2DMouseJoint); cdecl; external;
procedure Box2DMouseJointShiftOrigin (joint: Box2DMouseJoint; newOrigin: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Prismatic Joint___ } 
{=============================================}

function Box2DPrismaticJointGetAnchorA (joint: Box2DPrismaticJoint): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetAnchorB (joint: Box2DPrismaticJoint): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetReactionForce (joint: Box2DPrismaticJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetReactionTorque (joint: Box2DPrismaticJoint; inv_dt: float32): float32; cdecl; external;
function Box2DPrismaticJointGetLocalAnchorA (joint: Box2DPrismaticJoint): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetLocalAnchorB (joint: Box2DPrismaticJoint): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetLocalAxisA (joint: Box2DPrismaticJoint): Box2DVector2; cdecl; external;
function Box2DPrismaticJointGetReferenceAngle (joint: Box2DPrismaticJoint): float32; cdecl; external;
function Box2DPrismaticJointGetJointTranslation (joint: Box2DPrismaticJoint): float32; cdecl; external;
function Box2DPrismaticJointGetJointSpeed (joint: Box2DPrismaticJoint): float32; cdecl; external;
function Box2DPrismaticJointIsLimitEnabled (joint: Box2DPrismaticJoint): cbool; cdecl; external;
procedure Box2DPrismaticJointEnableLimit (joint: Box2DPrismaticJoint; flag: cbool); cdecl; external;
function Box2DPrismaticJointGetLowerLimit (joint: Box2DPrismaticJoint): float32; cdecl; external;
function Box2DPrismaticJointGetUpperLimit (joint: Box2DPrismaticJoint): float32; cdecl; external;
procedure Box2DPrismaticJointSetLimits (joint: Box2DPrismaticJoint; lower: float32; upper: float32); cdecl; external;
function Box2DPrismaticJointIsMotorEnabled (joint: Box2DPrismaticJoint): cbool; cdecl; external;
procedure Box2DPrismaticJointEnableMotor (joint: Box2DPrismaticJoint; flag: cbool); cdecl; external;
procedure Box2DPrismaticJointSetMotorSpeed (joint: Box2DPrismaticJoint; speed: float32); cdecl; external;
function GBox2DPrismaticJointGetMotorSpeed (joint: Box2DPrismaticJoint): float32; cdecl; external;
procedure Box2DPrismaticJointSetMaxMotorForce (joint: Box2DPrismaticJoint; force: float32); cdecl; external;
function Box2DPrismaticJointGetMaxMotorForce (joint: Box2DPrismaticJoint): float32; cdecl; external;
function Box2DPrismaticJointGetMotorForce (joint: Box2DPrismaticJoint; inv_dt: float32): float32; cdecl; external;
procedure Box2DPrismaticJointDump (joint: Box2DPrismaticJoint); cdecl; external;

{=============================================}
{@! ___Pulley Joint___ } 
{=============================================}

function Box2DPulleyJointGetAnchorA (joint: Box2DPulleyJoint): Box2DVector2; cdecl; external;
function Box2DPulleyJointGetAnchorB (joint: Box2DPulleyJoint): Box2DVector2; cdecl; external;
function Box2DPulleyJointGetReactionForce (joint: Box2DPulleyJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DPulleyJointGetReactionTorque (joint: Box2DPulleyJoint; inv_dt: float32): float32; cdecl; external;
function Box2DPulleyJointGetGroundAnchorA (joint: Box2DPulleyJoint): Box2DVector2; cdecl; external;
function Box2DPulleyJointGetGroundAnchorB (joint: Box2DPulleyJoint): Box2DVector2; cdecl; external;
function Box2DPulleyJointGetLengthA (joint: Box2DPulleyJoint): float32; cdecl; external;
function Box2DPulleyJointGetLengthB (joint: Box2DPulleyJoint): float32; cdecl; external;
function Box2DPulleyJointGetRatio (joint: Box2DPulleyJoint): float32; cdecl; external;
function Box2DPulleyJointGetCurrentLengthA (joint: Box2DPulleyJoint): float32; cdecl; external;
function Box2DPulleyJointGetCurrentLengthB (joint: Box2DPulleyJoint): float32; cdecl; external;
procedure Box2DPulleyJointDump (joint: Box2DPulleyJoint); cdecl; external;
procedure Box2DPulleyJointShiftOrigin (joint: Box2DPulleyJoint; newOrigin: Box2DVector2); cdecl; external;

{=============================================}
{@! ___Revolute Joint___ } 
{=============================================}

function Box2DRevoluteJointGetLocalAnchorA (joint: Box2DRevoluteJoint): Box2DVector2; cdecl; external;
function Box2DRevoluteJointGetLocalAnchorB (joint: Box2DRevoluteJoint): Box2DVector2; cdecl; external;
function Box2DRevoluteJointGetReferenceAngle (joint: Box2DRevoluteJoint): float32; cdecl; external;
function Box2DRevoluteJointGetJointAngle (joint: Box2DRevoluteJoint): float32; cdecl; external;
function Box2DRevoluteJointGetJointSpeed (joint: Box2DRevoluteJoint): float32; cdecl; external;
function Box2DRevoluteJointIsLimitEnabled (joint: Box2DRevoluteJoint): cbool; cdecl; external;
procedure Box2DRevoluteJointEnableLimit (joint: Box2DRevoluteJoint; flag: cbool); cdecl; external;
function Box2DRevoluteJointGetLowerLimit (joint: Box2DRevoluteJoint): float32; cdecl; external;
function Box2DRevoluteJointGetUpperLimit (joint: Box2DRevoluteJoint): float32; cdecl; external;
procedure Box2DRevoluteJointSetLimits (joint: Box2DRevoluteJoint; lower: float32; upper: float32); cdecl; external;
function Box2DRevoluteJointIsMotorEnabled (joint: Box2DRevoluteJoint): cbool; cdecl; external;
procedure Box2DRevoluteJointEnableMotor (joint: Box2DRevoluteJoint; flag: cbool); cdecl; external;
procedure Box2DRevoluteJointSetMotorSpeed (joint: Box2DRevoluteJoint; speed: float32); cdecl; external;
function Box2DRevoluteJointGetMotorSpeed (joint: Box2DRevoluteJoint): float32; cdecl; external;
procedure Box2DRevoluteJointSetMaxMotorTorque (joint: Box2DRevoluteJoint; torque: float32); cdecl; external;
function Box2DRevoluteJointGetMaxMotorTorque (joint: Box2DRevoluteJoint): float32; cdecl; external;
function Box2DRevoluteJointGetReactionForce (joint: Box2DRevoluteJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DRevoluteJointGetReactionTorque (joint: Box2DRevoluteJoint; inv_dt: float32): float32; cdecl; external;
function Box2DRevoluteJointGetMotorTorque (joint: Box2DRevoluteJoint; inv_dt: float32): float32; cdecl; external;
procedure Box2DRevoluteJointDump (joint: Box2DRevoluteJoint); cdecl; external;

{=============================================}
{@! ___Rope Joint___ } 
{=============================================}

function Box2DRopeJointGetAnchorA (joint: Box2DRopeJoint): Box2DVector2; cdecl; external;
function Box2DRopeJointGetAnchorB (joint: Box2DRopeJoint): Box2DVector2; cdecl; external;
function Box2DRopeJointGetReactionForce (joint: Box2DRopeJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DRopeJointGetReactionTorque (joint: Box2DRopeJoint; inv_dt: float32): float32; cdecl; external;
function Box2DRopeJointGetLocalAnchorA (joint: Box2DRopeJoint): Box2DVector2; cdecl; external;
function Box2DRopeJointGetLocalAnchorB (joint: Box2DRopeJoint): Box2DVector2; cdecl; external;
procedure Box2DRopeJointSetMaxLength (joint: Box2DRopeJoint; length: float32); cdecl; external;
function Box2DRopeJointGetMaxLength (joint: Box2DRopeJoint): float32; cdecl; external;
function Box2DRopeJointGetLimitState (joint: Box2DRopeJoint): Box2DLimitState; cdecl; external;
procedure Box2DRopeJointDump (joint: Box2DRopeJoint); cdecl; external;

{=============================================}
{@! ___Weld Joint___ } 
{=============================================}

function Box2DWeldJointGetAnchorA (joint: Box2DWeldJoint): Box2DVector2; cdecl; external;
function Box2DWeldJointGetAnchorB (joint: Box2DWeldJoint): Box2DVector2; cdecl; external;
function Box2DWeldJointGetReactionForce (joint: Box2DWeldJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DWeldJointGetReactionTorque (joint: Box2DWeldJoint; inv_dt: float32): float32; cdecl; external;
function Box2DWeldJointGetLocalAnchorA (joint: Box2DWeldJoint): Box2DVector2; cdecl; external;
function Box2DWeldJointGetLocalAnchorB (joint: Box2DWeldJoint): Box2DVector2; cdecl; external;
function Box2DWeldJointGetReferenceAngle (joint: Box2DWeldJoint): float32; cdecl; external;
procedure Box2DWeldJointSetFrequency (joint: Box2DWeldJoint; hz: float32); cdecl; external;
function Box2DWeldJointGetFrequency (joint: Box2DWeldJoint): float32; cdecl; external;
procedure Box2DWeldJointSetDampingRatio (joint: Box2DWeldJoint; ratio: float32); cdecl; external;
function Box2DWeldJointGetDampingRatio (joint: Box2DWeldJoint): float32; cdecl; external;
procedure Box2DWeldJointDump (joint: Box2DWeldJoint); cdecl; external;

{=============================================}
{@! ___Wheel Joint___ } 
{=============================================}

procedure Box2DWheelJointGetDefinition (joint: Box2DWheelJoint; var def: Box2DWheelJointDefinition); cdecl; external;
function Box2DWheelJointGetAnchorA (joint: Box2DWheelJoint): Box2DVector2; cdecl; external;
function Box2DWheelJointGetAnchorB (joint: Box2DWheelJoint): Box2DVector2; cdecl; external;
function Box2DWheelJointGetReactionForce (joint: Box2DWheelJoint; inv_dt: float32): Box2DVector2; cdecl; external;
function Box2DWheelJointGetReactionTorque (joint: Box2DWheelJoint; inv_dt: float32): float32; cdecl; external;
function Box2DWheelJointGetLocalAnchorA (joint: Box2DWheelJoint): Box2DVector2; cdecl; external;
function Box2DWheelJointGetLocalAnchorB (joint: Box2DWheelJoint): Box2DVector2; cdecl; external;
function Box2DWheelJointGetLocalAxisA (joint: Box2DWheelJoint): Box2DVector2; cdecl; external;
function Box2DWheelJointGetJointTranslation (joint: Box2DWheelJoint): float32; cdecl; external;
function Box2DWheelJointGetJointSpeed (joint: Box2DWheelJoint): float32; cdecl; external;
function Box2DWheelJointIsMotorEnabled (joint: Box2DWheelJoint): cbool; cdecl; external;
procedure Box2DWheelJointEnableMotor (joint: Box2DWheelJoint; flag: cbool); cdecl; external;
procedure Box2DWheelJointSetMotorSpeed (joint: Box2DWheelJoint; speed: float32); cdecl; external;
function Box2DWheelJointGetMotorSpeed (joint: Box2DWheelJoint): float32; cdecl; external;
procedure Box2DWheelJointSetMaxMotorTorque (joint: Box2DWheelJoint; torque: float32); cdecl; external;
function Box2DWheelJointGetMaxMotorTorque (joint: Box2DWheelJoint): float32; cdecl; external;
function Box2DWheelJointGetMotorTorque (joint: Box2DWheelJoint; inv_dt: float32): float32; cdecl; external;
procedure Box2DWheelJointSetSpringFrequencyHz (joint: Box2DWheelJoint; hz: float32); cdecl; external;
function Box2DWheelJointGetSpringFrequencyHz (joint: Box2DWheelJoint): float32; cdecl; external;
procedure Box2DWheelJointSetSpringDampingRatio (joint: Box2DWheelJoint; ratio: float32); cdecl; external;
function Box2DWheelJointGetSpringDampingRatio (joint: Box2DWheelJoint): float32; cdecl; external;
procedure Box2DWheelJointDump (joint: Box2DWheelJoint); cdecl; external;

{=============================================}
{@! ___Fixture___ } 
{=============================================}

function Box2DFixtureGetType (fixture: Box2DFixture): Box2DShapeType; cdecl; external;
function Box2DFixtureGetShape (fixture: Box2DFixture): Box2DShape; cdecl; external;    
procedure Box2DFixtureSetSensor (fixture: Box2DFixture; sensor: cbool); cdecl; external;
function Box2DFixtureIsSensor (fixture: Box2DFixture): cbool; cdecl; external;
procedure Box2DFixtureSetFilterData (fixture: Box2DFixture; filter: Box2DFilter); cdecl; external;
function Box2DFixtureGetFilterData (fixture: Box2DFixture): Box2DFilter; cdecl; external;
procedure Box2DFixtureRefilter (fixture: Box2DFixture); cdecl; external;
function Box2DFixtureGetBody (fixture: Box2DFixture): Box2DBody; cdecl; external;    
function Box2DFixtureGetNext (fixture: Box2DFixture): Box2DFixture; cdecl; external;    
function Box2DFixtureGetUserData(fixture: Box2DFixture): pointer; cdecl; external;

procedure Box2DFixtureSetUserData (fixture: Box2DFixture; data: pointer); cdecl; external;
function Box2DFixtureTestPoint (fixture: Box2DFixture; p: Box2DVector2): cbool; cdecl; external;
function Box2DFixtureRayCast (fixture: Box2DFixture; var output: Box2DRayCastOutput; input: Box2DRayCastInput; childIndex: int32): cbool; cdecl; external;
procedure Box2DFixtureGetMassData (fixture: Box2DFixture; var massData: Box2DMassData); cdecl; external;
procedure Box2DFixtureSetDensity (fixture: Box2DFixture; density: float32); cdecl; external;
function Box2DFixtureGetDensity (fixture: Box2DFixture): float32; cdecl; external;
function Box2DFixtureGetFriction (fixture: Box2DFixture): float32; cdecl; external;
procedure Box2DFixtureSetFriction (fixture: Box2DFixture; friction: float32); cdecl; external;
function Box2DFixtureGetRestitution (fixture: Box2DFixture): float32; cdecl; external;
procedure Box2DFixtureSetRestitution (fixture: Box2DFixture; restitution: float32); cdecl; external;
function Box2DFixtureGetAABB (fixture: Box2DFixture; childIndex: int32): Box2DAABB; cdecl; external;
procedure Box2DFixtureDump (fixture: Box2DFixture; bodyIndex: int32); cdecl; external;

{=============================================}
{@! ___Shape___ } 
{=============================================}

function Box2DShapeGetType (shape: Box2DShape): Box2DShapeType; cdecl; external;   
function Box2DShapeGetRadius (shape: Box2DShape): float32; cdecl; external;   
procedure Box2DShapeSetRadius (shape: Box2DShape; radius: float32); cdecl; external;
function Box2DShapeClone (shape: Box2DShape; allocator: Box2DBlockAllocator): Box2DShape; cdecl; external;
function Box2DShapeGetChildCount (shape: Box2DShape): int32; cdecl; external;
function Box2DShapeTestPoint (shape: Box2DShape; transform: Box2DTransform; p: Box2DVector2): cbool; cdecl; external;
function Box2DShapeRayCast (shape: Box2DShape; var output: Box2DRayCastOutput; input: Box2DRayCastInput; transform: Box2DTransform; childIndex: int32): cbool; cdecl; external;
procedure Box2DShapeComputeAABB (shape: Box2DShape; var aabb: Box2DAABB; transform: Box2DTransform; childIndex: int32); cdecl; external;
procedure Box2DShapeComputeMass (shape: Box2DShape; var massData: Box2DMassData; density: float32); cdecl; external;

{=============================================}
{@! ___Edge Shape___ } 
{=============================================}

function Box2DEdgeShapeCreate: Box2DEdgeShape; cdecl; external;
procedure Box2DEdgeShapeSet (shape: Box2DEdgeShape; v1, v2: Box2DVector2); cdecl; external;
function Box2DEdgeShapeClone (shape: Box2DEdgeShape; allocator: Box2DBlockAllocator): Box2DEdgeShape; cdecl; external;
function Box2DEdgeShapeGetChildCount (shape: Box2DEdgeShape): int32; cdecl; external;
function Box2DEdgeShapeTestPoint (shape: Box2DEdgeShape; transform: Box2DTransform; p: Box2DVector2): cbool; cdecl; external;
function Box2DEdgeShapeRayCast (shape: Box2DEdgeShape; var output: Box2DRayCastOutput; input: Box2DRayCastInput; transform: Box2DTransform; childIndex: int32): cbool; cdecl; external;
procedure Box2DEdgeShapeComputeAABB (shape: Box2DEdgeShape; var aabb: Box2DAABB; transform: Box2DTransform; childIndex: int32); cdecl; external;
procedure Box2DEdgeShapeComputeMass (shape: Box2DEdgeShape; var massData: Box2DMassData; density: float32); cdecl; external;    
    
{=============================================}
{@! ___Circle Shape___ } 
{=============================================}

function Box2DCircleShapeCreate: Box2DCircleShape; cdecl; external;
procedure Box2DCircleShapeSetPosition (shape: Box2DCircleShape; point: Box2DVector2); cdecl; external;
function Box2DCircleShapeClone (shape: Box2DCircleShape; allocator: Box2DBlockAllocator): Box2DCircleShape; cdecl; external;
function Box2DCircleShapeGetChildCount (shape: Box2DCircleShape): int32; cdecl; external;
function Box2DCircleShapeTestPoint (shape: Box2DCircleShape; transform: Box2DTransform; p: Box2DVector2): cbool; cdecl; external;
function Box2DCircleShapeRayCast (shape: Box2DCircleShape; var output: Box2DRayCastOutput; input: Box2DRayCastInput; transform: Box2DTransform; childIndex: int32): cbool; cdecl; external;
procedure Box2DCircleShapeComputeAABB (shape: Box2DCircleShape; var aabb: Box2DAABB; transform: Box2DTransform; childIndex: int32); cdecl; external;
procedure Box2DCircleShapeComputeMass (shape: Box2DCircleShape; var massData: Box2DMassData; density: float32); cdecl; external;
function Box2DCircleShapeGetSupport (shape: Box2DCircleShape; d: Box2DVector2): int32; cdecl; external;
function Box2DCircleShapeGetSupportVertex (shape: Box2DCircleShape; d: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DCircleShapeGetVertexCount (shape: Box2DCircleShape): int32; cdecl; external;
function Box2DCircleShapeGetVertex (shape: Box2DCircleShape; index: int32): Box2DVector2; cdecl; external;        

{=============================================}
{@! ___Polygon Shape___ } 
{=============================================}

function Box2DPolygonShapeCreate: Box2DPolygonShape; cdecl; external;
function Box2DPolygonShapeClone (shape: Box2DPolygonShape; allocator: Box2DBlockAllocator): Box2DPolygonShape; cdecl; external;
function Box2DPolygonShapeGetChildCount (shape: Box2DPolygonShape): int32; cdecl; external;
procedure Box2DPolygonShapeSet (shape: Box2DPolygonShape; points: Box2DVerticesArrayPointer; count: int32); cdecl; external;
procedure Box2DPolygonShapeSetAsBox (shape: Box2DPolygonShape; hx: float32; hy: float32); cdecl; external;
procedure Box2DPolygonShapeSetAsBox2 (shape: Box2DPolygonShape; hx: float32; hy: float32; center: Box2DVector2; angle: float32); cdecl; external;
function Box2DPolygonShapeTestPoint (shape: Box2DPolygonShape; transform: Box2DTransform; p: Box2DVector2): cbool; cdecl; external;
function Box2DPolygonShapeRayCast (shape: Box2DPolygonShape; var output: Box2DRayCastOutput; input: Box2DRayCastInput; transform: Box2DTransform; childIndex: int32): cbool; cdecl; external;
procedure Box2DPolygonShapeComputeAABB (shape: Box2DPolygonShape; var aabb: Box2DAABB; transform: Box2DTransform; childIndex: int32); cdecl; external;
procedure Box2DPolygonShapeComputeMass (shape: Box2DPolygonShape; var massData: Box2DMassData; density: float32); cdecl; external;
function Box2DPolygonShapeGetVertexCount (shape: Box2DPolygonShape): int32; cdecl; external;
function Box2DPolygonShapeGetVertex (shape: Box2DPolygonShape; index: int32): Box2DVector2; cdecl; external;
function Box2DPolygonShapeValidate (shape: Box2DPolygonShape): cbool; cdecl; external;

{=============================================}
{@! ___Chain Shape___ } 
{=============================================}

function Box2DChainShapeCreate: Box2DChainShape; cdecl; external;
procedure Box2DChainShapeCreateLoop (shape: Box2DChainShape; vertices: Box2DVerticesArrayPointer; count: int32); cdecl; external;
procedure Box2DChainShapeCreateChain (shape: Box2DChainShape; vertices: Box2DVerticesArrayPointer; count: int32); cdecl; external;
procedure Box2DChainShapeSetPrevVertex (shape: Box2DChainShape; prevVertex: Box2DVector2); cdecl; external;
procedure Box2DChainShapeSetNextVertex (shape: Box2DChainShape; nextVertex: Box2DVector2); cdecl; external;
function Box2DChainShapeClone (shape: Box2DChainShape; allocator: Box2DBlockAllocator): Box2DChainShape; cdecl; external;
function Box2DChainShapeGetChildCount (shape: Box2DChainShape): int32; cdecl; external;
procedure Box2DChainShapeGetChildEdge (shape: Box2DChainShape; edge: Box2DEdgeShape; index: int32); cdecl; external;
function Box2DChainShapeTestPoint (shape: Box2DChainShape; transform: Box2DTransform; p: Box2DVector2): cbool; cdecl; external;
function Box2DChainShapeRayCast (shape: Box2DChainShape; var output: Box2DRayCastOutput; input: Box2DRayCastInput; transform: Box2DTransform; childIndex: int32): cbool; cdecl; external;
procedure Box2DChainShapeComputeAABB (shape: Box2DChainShape; var aabb: Box2DAABB; transform: Box2DTransform; childIndex: int32); cdecl; external;
procedure Box2DChainShapeComputeMass (shape: Box2DChainShape; var massData: Box2DMassData; density: float32); cdecl; external;    

{=============================================}
{@! ___Body___ } 
{=============================================}

function Box2DBodyCreateFixture (body: Box2DBody; def: Box2DFixtureDefinition): Box2DFixture; cdecl; external;
function Box2DBodyCreateFixtureWithShape (body: Box2DBody; shape: Box2DShape; density: float32): Box2DFixture; cdecl; external;
procedure Box2DBodyDestroyFixture (body: Box2DBody; fixture: Box2DFixture); cdecl; external;
procedure Box2DBodySetTransform (body: Box2DBody; position: Box2DVector2; angle: float32); cdecl; external;
function Box2DBodyGetTransform (body: Box2DBody): Box2DTransform; cdecl; external;
function Box2DBodyGetPosition (body: Box2DBody): Box2DVector2; cdecl; external;
function Box2DBodyGetAngle (body: Box2DBody): float32; cdecl; external;
function Box2DBodyGetWorldCenter (body: Box2DBody): Box2DVector2; cdecl; external;
function Box2DBodyGetLocalCenter (body: Box2DBody): Box2DVector2; cdecl; external;
procedure Box2DBodySetLinearVelocity (body: Box2DBody; v: Box2DVector2); cdecl; external;
function Box2DBodyGetLinearVelocity (body: Box2DBody): Box2DVector2; cdecl; external;
procedure Box2DBodySetAngularVelocity (body: Box2DBody; omega: float32); cdecl; external;
function Box2DBodyGetAngularVelocity (body: Box2DBody): float32; cdecl; external;
procedure Box2DBodyApplyForce (body: Box2DBody; force: Box2DVector2; point: Box2DVector2; wake: cbool); cdecl; external;
procedure Box2DBodyApplyForceToCenter (body: Box2DBody; force: Box2DVector2; wake: cbool); cdecl; external;
procedure Box2DBodyApplyTorque (body: Box2DBody; torque: float32; wake: cbool); cdecl; external;
procedure Box2DBodyApplyAngularImpulse (body: Box2DBody; impulse: float32; wake: cbool); cdecl; external;
procedure Box2DBodyApplyLinearImpulse (body: Box2DBody; impulse: Box2DVector2; point: Box2DVector2; wake: cbool); cdecl; external;
function Box2DBodyGetMass (body: Box2DBody): float32; cdecl; external;
function Box2DBodyGetInertia (body: Box2DBody): float32; cdecl; external;
procedure Box2DBodyGetMassData (body: Box2DBody; var data: Box2DMassData); cdecl; external;
procedure Box2DBodySetMassData (body: Box2DBody; data: Box2DMassData); cdecl; external;
procedure Box2DBodyResetMassData (body: Box2DBody); cdecl; external;
function Box2DBodyGetWorldPoint (body: Box2DBody; localPoint: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetWorldVector (body: Box2DBody; localVector: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetLocalPoint (body: Box2DBody; worldPoint: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetLocalVector (body: Box2DBody; worldVector: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetLinearVelocityFromWorldPoint (body: Box2DBody; worldPoint: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetLinearVelocityFromLocalPoint (body: Box2DBody; localPoint: Box2DVector2): Box2DVector2; cdecl; external;
function Box2DBodyGetLinearDamping (body: Box2DBody): float32; cdecl; external;
procedure Box2DBodySetLinearDamping (body: Box2DBody; linearDamping: float32); cdecl; external;
function Box2DBodyGetAngularDamping (body: Box2DBody): float32; cdecl; external;
procedure Box2DBodySetAngularDamping (body: Box2DBody; angularDamping: float32); cdecl; external;
function Box2DBodyGetGravityScale (body: Box2DBody): float32; cdecl; external;
procedure Box2DBodySetGravityScale (body: Box2DBody; scale: float32); cdecl; external;
procedure Box2DBodySetType (body: Box2DBody; theType: Box2DBodyType); cdecl; external;
function Box2DBodyGetType (body: Box2DBody): Box2DBodyType; cdecl; external;
procedure Box2DBodySetBullet (body: Box2DBody; flag: cbool); cdecl; external;
function Box2DBodyIsBullet (body: Box2DBody): cbool; cdecl; external;
procedure Box2DBodySetSleepingAllowed (body: Box2DBody; flag: cbool); cdecl; external;
function Box2DBodyIsSleepingAllowed (body: Box2DBody): cbool; cdecl; external;
procedure Box2DBodySetAwake (body: Box2DBody; flag: cbool); cdecl; external;
function Box2DBodyIsAwake (body: Box2DBody): cbool; cdecl; external;
procedure Box2DBodySetActive (body: Box2DBody; flag: cbool); cdecl; external;
function Box2DBodyIsActive (body: Box2DBody): cbool; cdecl; external;
procedure Box2DBodySetFixedRotation (body: Box2DBody; flag: cbool); cdecl; external;
function Box2DBodyIsFixedRotation (body: Box2DBody): cbool; cdecl; external;    
function Box2DBodyGetFixtureList (body: Box2DBody): Box2DFixture; cdecl; external;
function Box2DBodyGetJointList(body: Box2DBody): pointer; cdecl; external;    
function Box2DBodyGetContactList(body: Box2DBody): pointer; cdecl; external;    
function Box2DBodyGetNext (body: Box2DBody): Box2DBody; cdecl; external;    
function Box2DBodyGetUserData(body: Box2DBody): pointer; cdecl; external;
procedure Box2DBodySetUserData (body: Box2DBody; data: pointer); cdecl; external;
function Box2DBodyGetWorld (body: Box2DBody): Box2DWorld; cdecl; external;    
procedure Box2DBodyDump (body: Box2DBody); cdecl; external;

{=============================================}
{@! ___World Callbacks___ } 
{=============================================}

function Box2DWorldCallbackCenterCreate (callbacks: Box2DWorldCallbacks): Box2DWorldCallbackCenter; cdecl; external;

{=============================================}
{@! ___Debug Draw___ } 
{=============================================}
function Box2DDebugDrawCreate (callbacks: Box2DDebugDrawCallbacks): Box2DDebugDraw; cdecl; external;
procedure Box2DDebugDrawSetFlags(debugDraw: Box2DDebugDraw; flags: uint32); cdecl; external;
function Box2DDebugDrawGetFlags(debugDraw: Box2DDebugDraw): uint32; cdecl; external;
procedure Box2DDebugDrawAppendFlags(debugDraw: Box2DDebugDraw; flags: uint32); cdecl; external;
procedure Box2DDebugDrawClearFlags(debugDraw: Box2DDebugDraw; flags: uint32); cdecl; external;
procedure Box2DDebugDrawPolygon(debugDraw: Box2DDebugDraw; vertices: Box2DVerticesArrayPointer; vertexCount: int32; color: Box2DColor); cdecl; external;
procedure Box2DDebugDrawSolidPolygon(debugDraw: Box2DDebugDraw; vertices: Box2DVerticesArrayPointer; vertexCount: int32; color: Box2DColor); cdecl; external;
procedure Box2DDebugDrawCircle(debugDraw: Box2DDebugDraw; center: Box2DVector2; radius: float32; color: Box2DColor); cdecl; external;
procedure Box2DDebugDrawSolidCircle(debugDraw: Box2DDebugDraw; center: Box2DVector2; radius: float32; axis: Box2DVector2; color: Box2DColor); cdecl; external;
procedure Box2DDebugDrawSegment(debugDraw: Box2DDebugDraw; p1: Box2DVector2; p2: Box2DVector2; color: Box2DColor); cdecl; external;
procedure Box2DDebugDrawTransform(debugDraw: Box2DDebugDraw; xf: Box2DTransform); cdecl; external;

function Box2DColorMake (r: float32; g: float32; b: float32): Box2DColor;

{=============================================}
{@! ___World___ } 
{=============================================}

function Box2DWorldCreate (gravity: Box2DVector2): Box2DWorld; cdecl; external;
procedure Box2DWorldSetDestructionListener (world: Box2DWorld; listener: Box2DWorldCallbackCenter); cdecl; external;
procedure Box2DWorldSetContactFilter (world: Box2DWorld; filter: Box2DWorldCallbackCenter); cdecl; external;
procedure Box2DWorldSetContactListener (world: Box2DWorld; listener: Box2DWorldCallbackCenter); cdecl; external;
procedure Box2DWorldSetDebugDraw (world: Box2DWorld; debugDraw: Box2DDebugDraw); cdecl; external;
function Box2DWorldCreateBody (world: Box2DWorld; def: Box2DBodyDefinition): Box2DBody; cdecl; external;
procedure Box2DWorldDestroyBody (world: Box2DWorld; body: Box2DBody); cdecl; external;
function Box2DWorldCreateJoint (world: Box2DWorld; def: Box2DJointDefinitionPointer): Box2DJoint; cdecl; external;
procedure Box2DWorldDestroyJoint (world: Box2DWorld; joint: Box2DJoint); cdecl; external;
procedure Box2DWorldStep (world: Box2DWorld; timeStep: float32; velocityIterations: int32; positionIterations: int32); cdecl; external;
procedure Box2DWorldClearForces (world: Box2DWorld); cdecl; external;
procedure Box2DWorldDrawDebugData (world: Box2DWorld); cdecl; external;
procedure Box2DWorldQueryAABB (world: Box2DWorld; callback: Box2DWorldCallbackCenter; aabb: Box2DAABB); cdecl; external;
procedure Box2DWorldRayCast (world: Box2DWorld; callback: Box2DWorldCallbackCenter; point1, point2: Box2DVector2); cdecl; external;
function Box2DWorldGetBodyList (world: Box2DWorld): Box2DBody; cdecl; external;
function Box2DWorldGetJointList (world: Box2DWorld): Box2DJoint; cdecl; external;
function Box2DWorldGetContactList (world: Box2DWorld): Box2DContact; cdecl; external;
procedure Box2DWorldSetAllowSleeping (world: Box2DWorld; flag: cbool); cdecl; external;
function Box2DWorldGetAllowSleeping (world: Box2DWorld): cbool; cdecl; external;
procedure Box2DWorldSetWarmStarting (world: Box2DWorld; flag: cbool); cdecl; external;
function Box2DWorldGetWarmStarting (world: Box2DWorld): cbool; cdecl; external;
procedure Box2DWorldSetContinuousPhysics (world: Box2DWorld; flag: cbool); cdecl; external;
function Box2DWorldGetContinuousPhysics (world: Box2DWorld): cbool; cdecl; external;
procedure Box2DWorldSetSubStepping (world: Box2DWorld; flag: cbool); cdecl; external;
function Box2DWorldGetSubStepping (world: Box2DWorld): cbool; cdecl; external;
function Box2DWorldGetProxyCount (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetBodyCount (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetJointCount (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetContactCount (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetTreeHeight (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetTreeBalance (world: Box2DWorld): int32; cdecl; external;
function Box2DWorldGetTreeQuality (world: Box2DWorld): float32; cdecl; external;
procedure Box2DWorldSetGravity (world: Box2DWorld; gravity: Box2DVector2); cdecl; external;
function Box2DWorldGetGravity (world: Box2DWorld): Box2DVector2; cdecl; external;
function Box2DWorldIsLocked (world: Box2DWorld): cbool; cdecl; external;
procedure Box2DWorldSetAutoClearForces (world: Box2DWorld; flag: cbool); cdecl; external;
function Box2DWorldGetAutoClearForces (world: Box2DWorld): cbool; cdecl; external;
procedure Box2DWorldShiftOrigin (world: Box2DWorld; newOrigin: Box2DVector2); cdecl; external;
procedure Box2DWorldGetContactManager (world: Box2DWorld; var manager: Box2DContactManager); cdecl; external;
function Box2DWorldGetProfile (world: Box2DWorld): Box2DProfile; cdecl; external;
procedure Box2DWorldDump (world: Box2DWorld); cdecl; external;

{=============================================}
{@! ___Contact Manager___ } 
{=============================================}

procedure Box2DContactManagerAddPair (manager: Box2DContactManager; proxyUserDataA: pointer; proxyUserDataB: pointer); cdecl; external;
procedure Box2DContactManagerFindNewContacts (manager: Box2DContactManager); cdecl; external;
procedure Box2DContactManagerDestroy (manager: Box2DContactManager; c: Box2DContact); cdecl; external;
procedure Box2DContactManagerCollide (manager: Box2DContactManager); cdecl; external;

{=============================================}
{@! ___Contact___ } 
{=============================================}

function Box2DContactGetManifold (contact: Box2DContact): Box2DManifold; cdecl; external;    
procedure Box2DContactGetWorldManifold (contact: Box2DContact; var worldManifold: Box2DWorldManifold); cdecl; external;
function Box2DContactIsTouching (contact: Box2DContact): cbool; cdecl; external;
procedure Box2DContactSetEnabled (contact: Box2DContact; flag: cbool); cdecl; external;
function Box2DContactIsEnabled (contact: Box2DContact): cbool; cdecl; external;
function Box2DContactGetNext (contact: Box2DContact): Box2DContact; cdecl; external;
function Box2DContactGetFixtureA (contact: Box2DContact): Box2DFixture; cdecl; external;
function Box2DContactGetChildIndexA (contact: Box2DContact): int32; cdecl; external;
function Box2DContactGetFixtureB (contact: Box2DContact): Box2DFixture; cdecl; external;
function Box2DContactGetChildIndexB (contact: Box2DContact): int32; cdecl; external;
procedure Box2DContactSetFriction (contact: Box2DContact; friction: float32); cdecl; external;
function Box2DContactGetFriction (contact: Box2DContact): float32; cdecl; external;
procedure Box2DContactResetFriction (contact: Box2DContact); cdecl; external;
procedure Box2DContactSetRestitution (contact: Box2DContact; restitution: float32); cdecl; external;
function Box2DContactGetRestitution (contact: Box2DContact): float32; cdecl; external;
procedure Box2DContactResetRestitution (contact: Box2DContact); cdecl; external;
procedure Box2DContactSetTangentSpeed (contact: Box2DContact; speed: float32); cdecl; external;
function Box2DContactGetTangentSpeed (contact: Box2DContact): float32; cdecl; external;

{=============================================}
{@! ___Memory Utilities___ } 
{=============================================}
    
procedure Box2DRelease (var theClass: Box2DOpaqueClass); cdecl; external;
function Box2DGetCurrentVersion: Box2DVersion; cdecl; external;

implementation

function Box2DColorMake (r: float32; g: float32; b: float32): Box2DColor;
begin
	result.red := r;
  result.green := g;
  result.blue := b;
end;

function Box2DVectorMake (x: float32; y: float32): Box2DVector2;
begin
	result.x := x;
	result.y := y;
end;
	
procedure Box2DVectorShow (vec: Box2DVector2);
begin
	writeln('x: ', vec.x:1:1, ', y: ', vec.y:1:1);
end;	
	
end.