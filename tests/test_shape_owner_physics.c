#include "chipmunk/chipmunk_private.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if(!(x))                                                                                   \
		{                                                                                          \
			fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                   \
			exit(1);                                                                               \
		}                                                                                          \
	} while(0)
static uint64_t digest = 1469598103934665603ULL, randomState = 0xe7b1a927931ULL;
static void hash(const void *p, size_t n)
{
	const unsigned char *s = p;
	while(n--)
	{
		digest ^= *s++;
		digest *= 1099511628211ULL;
	}
}
static void number(double d)
{
	CHECK(isfinite(d));
	hash(&d, sizeof(d));
}
static uint32_t random32(void)
{
	randomState ^= randomState << 13;
	randomState ^= randomState >> 7;
	randomState ^= randomState << 17;
	return (uint32_t)randomState;
}
static double unit(void)
{
	return (double)random32() / 4294967296.0;
}
typedef struct World
{
	cpSpace *space;
	cpBody *bodies[256];
	cpShape *shapes[512];
	cpConstraint *joints[256];
	int nb, ns, nj;
} World;
static World world(void)
{
	World w = {0};
	w.space = cpSpaceNew();
#if CP_SHAPE_OWNER_PRUNING
	CHECK(cpBBTreeOwnersConfigured(w.space->dynamicShapes) &&
		  cpBBTreeOwnersConfigured(w.space->staticShapes));
#endif
	cpSpaceSetIterations(w.space, 20);
	cpSpaceSetCollisionSlop(w.space, .02);
	return w;
}
static cpBody *body(World *w, cpVect p)
{
	CHECK(w->nb < 256);
	cpBody *b = cpBodyNew(1, 1);
	cpBodySetPosition(b, p);
	cpBodySetUserData(b, (void *)(uintptr_t)(w->nb + 1));
	cpSpaceAddBody(w->space, b);
	w->bodies[w->nb++] = b;
	// A tiny sensor makes every fixture body compound without altering response.
	cpShape *sensor = cpCircleShapeNew(b, .001, cpvzero);
	cpShapeSetSensor(sensor, cpTrue);
	cpSpaceAddShape(w->space, sensor);
	w->shapes[w->ns++] = sensor;
	return b;
}
static cpShape *shape(World *w, cpShape *s)
{
	CHECK(w->ns < 512);
	cpShapeSetFriction(s, .7);
	cpSpaceAddShape(w->space, s);
	w->shapes[w->ns++] = s;
	return s;
}
static void joint(World *w, cpConstraint *c)
{
	CHECK(w->nj < 256);
	cpSpaceAddConstraint(w->space, c);
	w->joints[w->nj++] = c;
}
static void destroy(World *w)
{
	for(int i = 0; i < w->nb; i++)
		cpBodyActivate(w->bodies[i]);
	for(int i = 0; i < w->nj; i++)
	{
		cpSpaceRemoveConstraint(w->space, w->joints[i]);
		cpConstraintFree(w->joints[i]);
	}
	for(int i = 0; i < w->ns; i++)
	{
		cpSpaceRemoveShape(w->space, w->shapes[i]);
		cpShapeFree(w->shapes[i]);
	}
	for(int i = 0; i < w->nb; i++)
	{
		cpSpaceRemoveBody(w->space, w->bodies[i]);
		cpBodyFree(w->bodies[i]);
	}
	cpSpaceFree(w->space);
}
static double kinetic(World *w)
{
	double e = 0;
	for(int i = 0; i < w->nb; i++)
	{
		cpBody *b = w->bodies[i];
		e += .5 * b->m * cpvlengthsq(b->v) + .5 * b->i * b->w * b->w;
	}
	CHECK(isfinite(e));
	return e;
}
static double step(World *w, double dt)
{
	cpSpaceStep(w->space, dt);
	for(int i = 0; i < w->nb; i++)
	{
		cpBody *b = w->bodies[i];
		number(b->p.x);
		number(b->p.y);
		number(b->v.x);
		number(b->v.y);
		number(b->a);
		number(b->w);
		int sleep = cpBodyIsSleeping(b);
		hash(&sleep, sizeof(sleep));
		CHECK(fabs(b->p.x) < 10000 && fabs(b->p.y) < 10000);
	}
	double pen = 0;
	int n = w->space->arbiters->num;
	hash(&n, sizeof(n));
	for(int i = 0; i < n; i++)
	{
		cpArbiter *a = w->space->arbiters->arr[i];
		hash(&a->a->hashid, sizeof(a->a->hashid));
		hash(&a->b->hashid, sizeof(a->b->hashid));
		number(a->n.x);
		number(a->n.y);
		for(int j = 0; j < a->count; j++)
		{
			struct cpContact *c = a->contacts + j;
			double d =
				cpvdot(cpvsub(cpvadd(a->body_b->p, c->r2), cpvadd(a->body_a->p, c->r1)), a->n);
			if(-d > pen) pen = -d;
			number(c->jnAcc);
			number(c->jtAcc);
			CHECK(c->jnAcc >= -1e-9);
			CHECK(fabs(c->jtAcc) <= a->u * c->jnAcc + 1e-7);
		}
	}
	return pen;
}
static void bounce(void)
{
	World w = world();
	cpShape *floor =
		shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-10, 0), cpv(10, 0), 0));
	cpShapeSetElasticity(floor, 1);
	cpBody *b = body(&w, cpv(0, 1));
	cpShape *s = shape(&w, cpCircleShapeNew(b, .5, cpvzero));
	cpShapeSetElasticity(s, .8);
	cpBodySetVelocity(b, cpv(0, -4));
	double out = 0;
	for(int i = 0; i < 120; i++)
	{
		step(&w, 1.0 / 240);
		if(b->v.y > 0) out = b->v.y;
	}
	CHECK(fabs(out - 3.2) < .025);
	CHECK(fabs(kinetic(&w) - 5.12) < .1);
	printf("bounce velocity %.9g energy %.9g digest %016llx\n", out, kinetic(&w),
		   (unsigned long long)digest);
	destroy(&w);
}
static void stack(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-30, 0), cpv(30, 0), .1));
	for(int col = 0; col < 4; col++)
		for(int row = 0; row < 8; row++)
		{
			cpBody *b = body(&w, cpv((col - 1.5) * 2, .61 + row * 1.05));
			shape(&w, (col % 2) ? cpCircleShapeNew(b, .5, cpvzero) : cpBoxShapeNew(b, 1, 1, 0));
		}
	double peak = 0, last = 0;
	for(int i = 0; i < 2400; i++)
	{
		last = step(&w, 1.0 / 120);
		if(last > peak) peak = last;
		CHECK(kinetic(&w) < 20000);
	}
	double e = kinetic(&w);
	CHECK(last < .2);
	CHECK(e < 5);
	for(int i = 0; i < w.nb; i++)
		CHECK(w.bodies[i]->p.y > -.5);
	printf("stack final_pen %.9g peak_pen %.9g energy %.9g digest %016llx\n", last, peak, e,
		   (unsigned long long)digest);
	destroy(&w);
}
static void friction(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-30, 0), cpv(30, 0), 0));
	cpBody *b = body(&w, cpv(0, .5));
	shape(&w, cpBoxShapeNew(b, 1, 1, 0));
	cpBodySetVelocity(b, cpv(4, 0));
	for(int i = 0; i < 1200; i++)
		step(&w, 1.0 / 240);
	CHECK(fabs(b->v.x) < .1);
	CHECK(b->p.x > 0 && b->p.x < 5);
	CHECK(kinetic(&w) < .01);
	printf("friction x %.9g speed %.9g digest %016llx\n", b->p.x, cpvlength(b->v),
		   (unsigned long long)digest);
	destroy(&w);
}
static void chain(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	cpBody *previous = cpSpaceGetStaticBody(w.space);
	cpVect prevAnchor = cpv(0, 8);
	for(int i = 0; i < 12; i++)
	{
		cpBody *b = body(&w, cpv((i + 1) * 1.1, 8));
		shape(&w, cpCircleShapeNew(b, .35, cpvzero));
		if(i % 3 == 0) shape(&w, cpBoxShapeNew(b, .5, .3, 0));
		cpConstraint *c = cpPinJointNew(previous, b, prevAnchor, cpvzero);
		cpConstraintSetCollideBodies(c, cpFalse);
		joint(&w, c);
		previous = b;
		prevAnchor = cpvzero;
	}
	double gap = 0;
	for(int i = 0; i < 2400; i++)
	{
		step(&w, 1.0 / 240);
		CHECK(kinetic(&w) < 10000);
		for(int j = 0; j < w.nj; j++)
		{
			cpConstraint *c = w.joints[j];
			double len = cpvdist(cpBodyLocalToWorld(c->a, cpPinJointGetAnchorA(c)),
								 cpBodyLocalToWorld(c->b, cpPinJointGetAnchorB(c)));
			double g = fabs(len - cpPinJointGetDist(c));
			if(g > gap) gap = g;
		}
	}
	CHECK(gap < .15);
	printf("chain max_gap %.9g energy %.9g digest %016llx\n", gap, kinetic(&w),
		   (unsigned long long)digest);
	destroy(&w);
}
static void segments(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-20, 0), cpv(20, 0), .1));
	for(int i = 0; i < 12; i++)
	{
		cpBody *b = body(&w, cpv((i % 4 - 1.5) * 2, 1 + i / 4));
		shape(&w, cpSegmentShapeNew(b, cpv(-.5, 0), cpv(.5, 0), .1));
	}
	for(int i = 0; i < 1200; i++)
	{
		step(&w, 1.0 / 120);
		CHECK(kinetic(&w) < 5000);
	}
	CHECK(kinetic(&w) < 2);
	for(int i = 0; i < w.nb; i++)
		CHECK(w.bodies[i]->p.y > -.5);
	printf("segments energy %.9g digest %016llx\n", kinetic(&w), (unsigned long long)digest);
	destroy(&w);
}
static void sleeping(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	cpSpaceSetSleepTimeThreshold(w.space, .5);
	shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-20, 0), cpv(20, 0), 0));
	cpShape *sensor = shape(&w, cpBoxShapeNew(cpSpaceGetStaticBody(w.space), 10, 10, 0));
	cpShapeSetSensor(sensor, cpTrue);
	for(int i = 0; i < 8; i++)
	{
		cpBody *b = body(&w, cpv(i - 3.5, .5));
		shape(&w, cpCircleShapeNew(b, .5, cpvzero));
	}
	for(int i = 0; i < 900; i++)
		step(&w, 1.0 / 120);
	int asleep = 0;
	for(int i = 0; i < w.nb; i++)
		asleep += cpBodyIsSleeping(w.bodies[i]);
	CHECK(asleep == 8);
	cpBodyApplyImpulseAtWorldPoint(w.bodies[0], cpv(0, 2), cpBodyGetPosition(w.bodies[0]));
	CHECK(!cpBodyIsSleeping(w.bodies[0]));
	for(int i = 0; i < 600; i++)
		step(&w, 1.0 / 120);
	CHECK(kinetic(&w) < 1);
	printf("sleep initial_count %d final_energy %.9g digest %016llx\n", asleep, kinetic(&w),
		   (unsigned long long)digest);
	destroy(&w);
}

static void pointSeen(cpShape *s, cpVect p, cpFloat d, cpVect g, void *data)
{
	(void)p;
	(void)d;
	(void)g;
	World *w = data;
	int found = 0;
	for(int i = 0; i < w->ns; i++)
		if(w->shapes[i] == s) found++;
	CHECK(found == 1);
	int *n = (int *)cpSpaceGetUserData(w->space);
	(*n)++;
}
static void queryOracle(World *w)
{
	for(int k = 0; k < 20; k++)
	{
		cpVect p = cpv(k * .7 - 7, k % 3 - 1);
		int expected = 0, seen = 0;
		for(int i = 0; i < w->ns; i++)
		{
			cpPointQueryInfo info;
			cpShapePointQuery(w->shapes[i], p, &info);
			expected += info.distance < 1;
		}
		cpSpaceSetUserData(w->space, &seen);
		cpSpacePointQuery(w->space, p, 1, CP_SHAPE_FILTER_ALL, pointSeen, w);
		CHECK(seen == expected);
		cpSpaceSetUserData(w->space, NULL);
	}
}
static void pairsOracle(void)
{
	const int fixtures[] = {4, 16, 64};
	for(int sparse = 0; sparse < 2; sparse++)
		for(int f = 0; f < 3; f++)
		{
			World w = world();
			shape(&w,
				  cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-40, 0), cpv(40, 0), .1));
			for(int j = 0; j < 5; j++)
			{
				cpBody *b = body(&w, cpv((j - 2) * .65, .2 + (j % 2) * .3));
				cpBodySetAngle(b, j * .17);
				for(int i = 0; i < fixtures[f]; i++)
				{
					double angle = i * 6.283185307179586 / fixtures[f];
					double r = sparse ? 6.0 : .5;
					cpShape *s =
						shape(&w, cpCircleShapeNew(b, .21, cpv(r * cos(angle), r * sin(angle))));
					cpShapeSetSensor(s, cpTrue);
				}
			}
			for(int i = 0; i < w.ns; i++)
				cpShapeSetSensor(w.shapes[i], cpTrue);
			int expected = 0;
			struct cpContact contacts[2];
			for(int i = 0; i < w.ns; i++)
				for(int j = i + 1; j < w.ns; j++)
					if(w.shapes[i]->body != w.shapes[j]->body)
						expected += cpCollide(w.shapes[i], w.shapes[j], 0, contacts).count > 0;
			for(int k = 0; k < 3; k++)
			{
				step(&w, .001);
				CHECK(cpHashSetCount(w.space->cachedArbiters) == expected);
				CHECK(w.space->arbiters->num == 0);
				queryOracle(&w);
			}
			printf("all-pair/query oracle sparse %d fixtures %d contact_pairs %d PASS\n", sparse,
				   fixtures[f], expected);
			destroy(&w);
		}
}
static void tiles(void)
{
	World w = world();
	cpSpaceSetGravity(w.space, cpv(0, -10));
	shape(&w, cpSegmentShapeNew(cpSpaceGetStaticBody(w.space), cpv(-20, 0), cpv(20, 0), .1));
	for(int j = 0; j < 16; j++)
	{
		cpBody *b = body(&w, cpv((j % 4 - 1.5) * 1.4, .65 + (j / 4) * 1.15));
		for(int k = 0; k < 4; k++)
		{
			cpVect verts[4] = {cpv(-.225, -.225), cpv(.225, -.225), cpv(.225, .225),
							   cpv(-.225, .225)};
			cpTransform t = cpTransformTranslate(cpv((k % 2 - .5) * .5, (k / 2 - .5) * .5));
			shape(&w, cpPolyShapeNew(b, 4, verts, t, 0));
		}
	}
	double pen = 0;
	for(int i = 0; i < 2400; i++)
	{
		pen = step(&w, 1.0 / 120);
		CHECK(kinetic(&w) < 10000);
		if(i % 300 == 0) queryOracle(&w);
	}
	CHECK(pen < .2);
	CHECK(kinetic(&w) < 5);
	for(int i = 0; i < w.nb; i++)
		CHECK(w.bodies[i]->p.y > -.5);
	printf("compound tiles final_pen %.9g energy %.9g digest %016llx\n", pen, kinetic(&w),
		   (unsigned long long)digest);
	destroy(&w);
}
static void transitions(void)
{
	World w = world();
	for(int i = 0; i < 6; i++)
	{
		cpBody *b = body(&w, cpv(i * .4, 0));
		cpShape *s = shape(&w, cpCircleShapeNew(b, .5, cpvzero));
		cpShapeSetSensor(s, cpTrue);
	}
	cpSpaceStep(w.space, .001);
	for(int i = 0; i < 6; i++)
	{
		cpSpaceRemoveShape(w.space, w.shapes[2 * i]);
		cpShapeFree(w.shapes[2 * i]);
	}
	for(int i = 0; i < 6; i++)
		w.shapes[i] = w.shapes[2 * i + 1];
	w.ns = 6;
	cpSpaceStep(w.space, .001);
	queryOracle(&w);
	for(int i = 0; i < 6; i++)
	{
		cpShape *s = shape(&w, cpCircleShapeNew(w.bodies[i], .001, cpvzero));
		cpShapeSetSensor(s, cpTrue);
	}
	cpSpaceStep(w.space, .001);
	queryOracle(&w);
	destroy(&w);
	puts("compound/single-shape/compound mode transitions PASS");
}
static int mutated;
static void reject(cpArbiter *a, cpSpace *s, void *data)
{
	(void)s;
	(void)data;
	mutated++;
	cpArbiterSetProcessCollision(a, cpFalse);
}
static void lifecycle(void)
{
	World w = world();
	cpBody *a = body(&w, cpvzero), *b = body(&w, cpv(.6, 0));
	shape(&w, cpCircleShapeNew(a, 1, cpvzero));
	shape(&w, cpCircleShapeNew(b, 1, cpvzero));
	cpSpaceStep(w.space, .001);
	queryOracle(&w);
	cpSpaceAddGlobalCollisionHandler(w.space)->preSolveFunc = reject;
	cpSpaceStep(w.space, .001);
	CHECK(mutated > 0 && w.space->arbiters->num == 0);
	cpSpaceAddGlobalCollisionHandler(w.space)->preSolveFunc =
		cpCollisionHandlerDoNothing.preSolveFunc;
	cpBodySetPosition(b, cpv(5, 0));
	cpSpaceStep(w.space, .001);
	cpBodySetPosition(b, cpv(.6, 0));
	cpSpaceStep(w.space, .001);
	cpSpaceReindexStatic(w.space);
	cpSpaceReindexShapesForBody(w.space, a);
	queryOracle(&w);
	cpSpaceStep(w.space, .001);
	cpBodySetType(b, CP_BODY_TYPE_STATIC);
	cpSpaceStep(w.space, .001);
	queryOracle(&w);
	cpBodySetType(b, CP_BODY_TYPE_DYNAMIC);
	cpBodySetMass(b, 1);
	cpBodySetMoment(b, 1);
	cpSpaceStep(w.space, .001);
	cpSpaceUseSpatialHash(w.space, 2, 100);
	cpSpaceStep(w.space, .001);
	CHECK(w.space->arbiters->num > 0);
	queryOracle(&w);
	destroy(&w);
	// Repeated removal/reallocation, including address reuse, must not retain proxies.
	w = world();
	for(int round = 0; round < 20; round++)
	{
		cpBody *x = body(&w, cpvzero);
		shape(&w, cpCircleShapeNew(x, .5, cpvzero));
		cpSpaceStep(w.space, .01);
		for(int i = 0; i < w.ns; i++)
		{
			cpSpaceRemoveShape(w.space, w.shapes[i]);
			cpShapeFree(w.shapes[i]);
		}
		w.ns = 0;
		cpSpaceRemoveBody(w.space, x);
		cpBodyFree(x);
		w.nb = 0;
		cpSpaceStep(w.space, .01);
	}
	destroy(&w);
	puts("callback/index/type/reindex/body-address-reuse lifecycle PASS");
}
#if !defined(__EMSCRIPTEN__)
#include "chipmunk/cpHastySpace.h"
static void hasty(void)
{
	cpSpace *s = cpHastySpaceNew();
	cpHastySpaceSetThreads(s, 1);
	cpBody *b[2];
	cpShape *sh[4];
	for(int i = 0; i < 2; i++)
	{
		b[i] = cpBodyNew(1, 1);
		cpBodySetPosition(b[i], cpv(i * .6, 0));
		cpSpaceAddBody(s, b[i]);
		for(int j = 0; j < 2; j++)
		{
			sh[2 * i + j] = cpCircleShapeNew(b[i], .5, cpv(0, j * .2));
			cpSpaceAddShape(s, sh[2 * i + j]);
		}
	}
	cpHastySpaceStep(s, .001);
	CHECK(s->arbiters->num > 0);
	cpSpaceStep(s, .001);
	cpHastySpaceStep(s, .001);
	CHECK(s->arbiters->num > 0);
	for(int i = 0; i < 4; i++)
	{
		cpSpaceRemoveShape(s, sh[i]);
		cpShapeFree(sh[i]);
	}
	for(int i = 0; i < 2; i++)
	{
		cpSpaceRemoveBody(s, b[i]);
		cpBodyFree(b[i]);
	}
	cpHastySpaceFree(s);
	puts("normal/HastySpace query-only restoration PASS");
}
#endif
int main(int argc, char **argv)
{
	int perturb = argc > 1 ? atoi(argv[1]) : 0;
	void *junk[257] = {0};
	if(perturb)
		for(int i = 0; i < 257; i++)
		{
			junk[i] = calloc(1, (size_t)((i * 97 + perturb * 13) % 8000 + 1));
			CHECK(junk[i]);
			if(i % 2 == 0)
			{
				free(junk[i]);
				junk[i] = NULL;
			}
		}
	pairsOracle();
	bounce();
	stack();
	friction();
	chain();
	segments();
	sleeping();
	tiles();
	transitions();
	lifecycle();
#if !defined(__EMSCRIPTEN__)
	hasty();
#endif
	for(int i = 0; i < 257; i++)
		free(junk[i]);
	printf("compound physics / repeatability digest %016llx PASS\n", (unsigned long long)digest);
	return 0;
}
