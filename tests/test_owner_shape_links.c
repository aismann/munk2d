#include "owner_test_compat.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chipmunk/chipmunk_private.h"
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if(!(x))                                                                                   \
		{                                                                                          \
			fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while(0)
#if CP_SHAPE_OWNER_PRUNING
// Exercise the actual private predicate and constructors, not a copied candidate.
// The static library's cpSpace.o is not pulled: these public definitions satisfy
// its references. Match this target's solver feature defines to the library.
#include "../src/cpSpace.c"
static const void *oldQuery(cpShape *shape)
{
	cpBody *body = shape->body;
	return body->shapeList && body->shapeList->next ? body : NULL;
}
static cpShape *sensor(cpBody *body)
{
	cpShape *shape = cpCircleShapeNew(body, .3, cpvzero);
	cpShapeSetSensor(shape, cpTrue);
	return shape;
}
static void member(cpShape *shape)
{
	CHECK(QueryShapeOwner(shape) == oldQuery(shape));
	CHECK(!QueryShapeOwner(shape) || QueryShapeOwner(shape) == ShapeOwner(shape));
}
static void unlinked(cpShape *shape)
{
	CHECK(!shape->next && !shape->prev);
	CHECK(!QueryShapeOwner(shape));
	CHECK(!QueryShapeOwner(shape) || QueryShapeOwner(shape) == oldQuery(shape));
}
#define NB 6
#define NS 64
static void validate(cpBody **bodies, cpShape **shapes)
{
	int seen[NS] = {0};
	for(int b = 0; b < NB; b++)
	{
		cpShape *previous = NULL;
		int count = 0;
		for(cpShape *s = bodies[b]->shapeList; s; s = s->next)
		{
			CHECK(++count <= NS);
			CHECK(s->body == bodies[b]);
			CHECK(s->prev == previous);
			if(previous) CHECK(previous->next == s);
			int i = 0;
			while(i < NS && shapes[i] != s)
				i++;
			CHECK(i < NS);
			CHECK(++seen[i] == 1);
			member(s);
			previous = s;
		}
	}
	for(int i = 0; i < NS; i++)
	{
		if(seen[i])
			CHECK(shapes[i]->space);
		else
			unlinked(shapes[i]);
	}
}
static void model(void)
{
	cpSpace *space = cpSpaceNew();
	cpBody *bodies[NB];
	cpShape *shapes[NS];
	for(int b = 0; b < NB; b++)
	{
		bodies[b] = cpBodyNew(1, 1);
		cpSpaceAddBody(space, bodies[b]);
		cpBodySetPosition(bodies[b], cpv(b * .1, 0));
	}
	for(int i = 0; i < NS; i++)
	{
		shapes[i] = sensor(bodies[i % NB]);
		if(i < NB) cpSpaceAddShape(space, shapes[i]);
	}
	validate(bodies, shapes);
	cpSpaceStep(space, .001);
	for(int step = 0; step < 1000; step++)
	{
		int i = (step * 37) % NS;
		if(shapes[i]->space)
			cpSpaceRemoveShape(space, shapes[i]);
		else
		{
			cpShapeSetBody(shapes[i], bodies[(step / 13) % NB]);
			cpSpaceAddShape(space, shapes[i]);
		}
		if(step % 17 == 0)
		{
			cpBody *body = bodies[(step / 17) % NB];
			cpBodyType old = cpBodyGetType(body),
					   type = old == CP_BODY_TYPE_DYNAMIC  ? CP_BODY_TYPE_STATIC
							  : old == CP_BODY_TYPE_STATIC ? CP_BODY_TYPE_KINEMATIC
														   : CP_BODY_TYPE_DYNAMIC;
			cpBodySetType(body, type);
			if(type == CP_BODY_TYPE_DYNAMIC)
			{
				cpBodySetMass(body, 1);
				cpBodySetMoment(body, 1);
			}
		}
		if(step % 31 == 0)
		{
			cpBBTreeOptimize(space->staticShapes);
			cpBBTreeOptimize(space->dynamicShapes);
		}
		if(step % 19 == 0) cpSpaceReindexShapesForBody(space, bodies[(step / 19) % NB]);
		if(step % 11 == 0) cpSpaceStep(space, .001);
		validate(bodies, shapes);
	}
	for(int i = 0; i < NS; i++)
	{
		if(shapes[i]->space) cpSpaceRemoveShape(space, shapes[i]);
		unlinked(shapes[i]);
		cpShapeFree(shapes[i]);
	}
	for(int b = 0; b < NB; b++)
	{
		CHECK(!bodies[b]->shapeList);
		cpSpaceRemoveBody(space, bodies[b]);
		cpBodyFree(bodies[b]);
	}
	cpSpaceFree(space);
}
static void sleepLateCompound(void)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetSleepTimeThreshold(space, .1);
	cpBody *body = cpBodyNew(1, 1);
	cpSpaceAddBody(space, body);
	cpShape *a = sensor(body), *b = sensor(body);
	cpSpaceAddShape(space, a);
	member(a);
	CHECK(!QueryShapeOwner(a));
	cpSpaceStep(space, .001);
	cpBodySleep(body);
	CHECK(cpBodyIsSleeping(body));
	member(a);
	cpSpaceAddShape(space, b);
	CHECK(!cpBodyIsSleeping(body));
	member(a);
	member(b);
	CHECK(QueryShapeOwner(a) == body && QueryShapeOwner(b) == body);
	cpSpaceStep(space, .001);
	cpSpaceRemoveShape(space, b);
	unlinked(b);
	member(a);
	CHECK(!QueryShapeOwner(a));
	cpShapeFree(b);
	cpBodySleep(body);
	CHECK(cpBodyIsSleeping(body));
	member(a);
	cpBodyActivate(body);
	cpSpaceRemoveShape(space, a);
	cpShapeFree(a);
	cpSpaceRemoveBody(space, body);
	cpBodyFree(body);
	cpSpaceFree(space);
}
static void detachedCrossSpace(void)
{
	cpSpace *a = cpSpaceNew(), *b = cpSpaceNew();
	cpBody *body = cpBodyNew(1, 1);
	cpSpaceAddBody(a, body);
	cpShape *x = sensor(body), *y = sensor(body);
	cpSpaceAddShape(a, x);
	member(x);
	CHECK(!QueryShapeOwner(x));
	cpSpaceRemoveBody(a, body);
	CHECK(!body->space && x->space == a);
	member(x);
	cpSpaceAddBody(b, body);
	cpSpaceAddShape(b, y);
	CHECK(x->space == a && y->space == b);
	member(x);
	member(y);
	CHECK(QueryShapeOwner(x) == body && QueryShapeOwner(y) == body);
	cpSpaceStep(a, .001);
	cpSpaceStep(b, .001);
	cpSpaceRemoveShape(a, x);
	unlinked(x);
	member(y);
	CHECK(!QueryShapeOwner(y));
	cpShapeFree(x);
	cpSpaceRemoveBody(b, body);
	CHECK(!body->space && y->space == b);
	member(y);
	cpSpaceRemoveShape(b, y);
	unlinked(y);
	cpShapeFree(y);
	cpBodyFree(body);
	cpSpaceFree(a);
	cpSpaceFree(b);
}
static cpShape *removing;
static int separateCalls;
static void removedDuringSeparate(cpArbiter *arb, cpSpace *space, void *data)
{
	(void)arb;
	(void)space;
	(void)data;
	if(!removing) return;
	CHECK(!removing->next && !removing->prev);
	CHECK(oldQuery(removing) == removing->body);
	CHECK(!QueryShapeOwner(removing));
	separateCalls++;
}
static void transientRemoval(void)
{
	cpSpace *space = cpSpaceNew();
	cpBody *a = cpBodyNew(1, 1), *b = cpBodyNew(1, 1);
	cpSpaceAddBody(space, a);
	cpSpaceAddBody(space, b);
	cpShape *shapes[4];
	for(int i = 0; i < 4; i++)
	{
		shapes[i] = sensor(i < 3 ? a : b);
		cpSpaceAddShape(space, shapes[i]);
	}
	cpSpaceAddGlobalCollisionHandler(space)->separateFunc = removedDuringSeparate;
	cpSpaceStep(space, .001);
	removing = shapes[0];
	cpSpaceRemoveShape(space, shapes[0]);
	removing = NULL;
	CHECK(separateCalls > 0);
	unlinked(shapes[0]);
	member(shapes[1]);
	member(shapes[2]);
	cpShapeFree(shapes[0]);
	for(int i = 1; i < 4; i++)
	{
		cpSpaceRemoveShape(space, shapes[i]);
		cpShapeFree(shapes[i]);
	}
	cpSpaceRemoveBody(space, a);
	cpSpaceRemoveBody(space, b);
	cpBodyFree(a);
	cpBodyFree(b);
	cpSpaceFree(space);
}
#endif
int main(void)
{
#if CP_SHAPE_OWNER_PRUNING
	model();
	sleepLateCompound();
	detachedCrossSpace();
	transientRemoval();
	puts("Actual shape-local query predicate: listed equivalence, unlinked conservative fallback, "
		 "1000 rebind/type/index operations, late compound/sleep/wake, detached/cross-space "
		 "membership and transient separate callback PASS");
#else
	puts("Shape-local query predicate SKIP: owner pruning compiled out");
#endif
	return 0;
}
