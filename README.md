Box2DC
======

Box2D C API for portability to other languages.

The majority of the API has been wrapped except for some of the lesser used classes, math functions and some C++ structs. If there is any interest in this project or I start using these other features myself I'll get to translating them eventually.

If you are using Box2DC to port to another language you'll want to look at the headers below for the relevant structs and functions. The DebugDraw and WorldCallbackCenter contain a C++ class which is used to dispatch method calls to the C callback functions but you only need to be concerned with the callback functions and related types.

    DebugDraw.h
    WorldCallbackCenter.h
    Box2DC.h

Files remaining to translate (since Box2D 2.3.0):

/Common
    b2BlockAllocator.cpp
    b2Math.cpp (partially)
    b2Timer.cpp
    b2GrowableStack.h
    b2Settings.cpp
    b2StackAllocator.cpp

/Collision
    b2BroadPhase.cpp
    b2CollideCircle.cpp
    b2CollideEdge.cpp
    b2CollidePolygon.cpp
    b2Collision.cpp (partially)
    b2Distance.cpp
    b2DynamicTree.cpp
    b2TimeOfImpact.cpp

/Dynamics
    b2Island.cpp
    b2TimeStep.h

/Dynamics/Contacts
    b2ChainAndCircleContact.cpp
    b2EdgeAndCircleContact.cpp
    b2ChainAndPolygonContact.cpp
    b2EdgeAndPolygonContact.cpp
    b2CircleContact.cpp
    b2PolygonAndCircleContact.cpp
    b2PolygonAndCircleContact.h
    b2PolygonContact.cpp
    b2ContactSolver.cpp

/Rope
    b2Rope.cpp

PLEASE NOTE: I am not a C programmer and there may be errors/oddities to be fixed in the general syntax. Please let me know if anything needs to be changed or contribute to the project.