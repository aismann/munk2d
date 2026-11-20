#include "owner_test_compat.h"
#include "../src/cpBBTree.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "owner_query_reference.h"
typedef struct Object
{
	cpBB bb;
	int id;
} Object;
typedef struct Row
{
	int a, b;
	cpCollisionID id;
} Row;
typedef struct Result
{
	Row rows[64];
	int count, keyCalls, nested;
} Result;
typedef struct Run
{
	cpBBTree *tree;
	Result result;
	int enabled, mutate;
} Run;
static char ownerToken;
static Run *active;
static cpBB bounds(void *p)
{
	return ((Object *)p)->bb;
}
static const void *owner(void *p)
{
	(void)p;
	return &ownerToken;
}
static const void *query(void *p)
{
	(void)p;
	active->result.keyCalls++;
	return active->enabled ? &ownerToken : NULL;
}
static cpCollisionID nestedVisit(void *a, void *b, cpCollisionID id, void *data)
{
	(void)a;
	(void)b;
	(*(int *)data)++;
	return id;
}
static cpCollisionID visit(void *a, void *b, cpCollisionID id, void *data)
{
	Run *run = (Run *)data;
	CHECK(run == active);
	CHECK(run->result.count < 64);
	Object *aa = (Object *)a, *bb = (Object *)b;
	run->result.rows[run->result.count++] = OWNER_TEST_LITERAL(Row){aa->id, bb->id, id};
	if(run->mutate) run->enabled = 1;
	int count = 0, before = run->result.keyCalls;
	cpSpatialIndexQuery((cpSpatialIndex *)run->tree, NULL, cpBBNew(-1, -1, 2, 2), nestedVisit,
						&count);
	CHECK(count == 8);
	CHECK(before == run->result.keyCalls);
	run->result.nested += count;
	return (cpCollisionID)(100 + aa->id * 8 + bb->id);
}
static Result run(int reference, int enabled, int mutate, int cache)
{
	Run state = {0};
	state.enabled = enabled;
	state.mutate = mutate;
	active = &state;
	cpSpatialIndex *index = cpBBTreeNew(bounds, NULL);
	state.tree = (cpBBTree *)index;
	CHECK(cpBBTreeConfigureOwners(index, owner, query));
	CHECK(cpBBTreeCacheOwnerQueries(index, cache));
	Object objects[8];
	Node *nodes[8];
	for(int i = 0; i < 8; i++)
	{
		objects[i] = OWNER_TEST_LITERAL(Object){cpBBNew(0, 0, 1, 1), i};
		nodes[i] = LeafNew(state.tree, objects + i, objects[i].bb);
	}
	Node *a = NodeNew(state.tree, nodes[0], nodes[1]), *b = NodeNew(state.tree, nodes[2], nodes[3]);
	Node *c = NodeNew(state.tree, nodes[4], nodes[5]), *d = NodeNew(state.tree, nodes[6], nodes[7]);
	state.tree->root = NodeNew(state.tree, NodeNew(state.tree, a, b), NodeNew(state.tree, c, d));
	state.tree->stamp = 7;
	nodes[7]->STAMP = 7;
	MarkContext context = {state.tree, NULL, visit, &state};
	if(reference)
		ReferenceMarkLeaf(nodes[7], &context);
	else
		MarkLeaf(nodes[7], &context);
	// Cached-pair callbacks/IDs must remain unchanged on subsequent passes.
	state.tree->stamp++;
	for(int pass = 0; pass < 3; pass++)
	{
		if(reference)
			ReferenceMarkLeaf(nodes[7], &context);
		else
			MarkLeaf(nodes[7], &context);
	}
	Result result = state.result;
	cpSpatialIndexFree(index);
	active = NULL;
	return result;
}
static void compare(int enabled, int mutate, int cache)
{
	Result ref = run(1, enabled, mutate, cache), actual = run(0, enabled, mutate, cache);
	CHECK(ref.count == actual.count);
	CHECK(ref.nested == actual.nested);
	for(int i = 0; i < ref.count; i++)
	{
		CHECK(ref.rows[i].a == actual.rows[i].a);
		CHECK(ref.rows[i].b == actual.rows[i].b);
		CHECK(ref.rows[i].id == actual.rows[i].id);
	}
	CHECK(ref.keyCalls == 3);
	CHECK(actual.keyCalls == (cache ? 1 : 3));
	if(mutate)
		CHECK(actual.count == 4); // one initial callback + three cached-pair callbacks
	else
		CHECK(actual.count == (enabled ? 0 : 28));
}
static void hints(void)
{
	cpSpatialIndex *s = cpBBTreeNew(bounds, NULL), *d = cpBBTreeNew(bounds, s);
	CHECK(cpBBTreeConfigureOwners(d, owner, query));
	CHECK(!((cpBBTree *)s)->cacheOwnerQueries && !((cpBBTree *)d)->cacheOwnerQueries);
	CHECK(cpBBTreeCacheOwnerQueries(s, cpTrue));
	CHECK(((cpBBTree *)s)->cacheOwnerQueries && ((cpBBTree *)d)->cacheOwnerQueries);
	CHECK(cpBBTreeConfigureOwners(d, owner, query));
	CHECK(!((cpBBTree *)s)->cacheOwnerQueries && !((cpBBTree *)d)->cacheOwnerQueries);
	CHECK(cpBBTreeCacheOwnerQueries(d, cpTrue));
	Run state = {0};
	state.tree = (cpBBTree *)d;
	active = &state;
	Object object = {cpBBNew(0, 0, 1, 1), 0};
	cpSpatialIndexInsert(d, &object, 0);
	CHECK(!cpBBTreeCacheOwnerQueries(d, cpFalse));
	cpSpatialIndexFree(d);
	cpSpatialIndexFree(s);
	active = NULL;
}
#endif
int main(void)
{
#if CP_SHAPE_OWNER_PRUNING
	compare(0, 0, 0);
	compare(1, 0, 0);
	compare(0, 1, 0); // generic refresh, including mutable callback
	compare(0, 0, 1);
	compare(1, 0, 1); // explicitly safe per-leaf caching
	hints();
	puts("Owner query differential: stable NULL/nonNULL, generic mutable eligibility, linked hint "
		 "reset, nested public queries, cached IDs and eligibility-call counts PASS");
#else
	puts("Owner query differential SKIP: owner pruning compiled out");
#endif
	return 0;
}
