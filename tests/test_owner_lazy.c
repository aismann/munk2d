#include "owner_test_compat.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "chipmunk/chipmunk_private.h"
static int denyTreeAlloc;
static unsigned long treeMemoryCalls;
static void *testCalloc(size_t n, size_t s)
{
	treeMemoryCalls++;
	return denyTreeAlloc ? NULL : calloc(n, s);
}
static void *testRealloc(void *p, size_t n)
{
	treeMemoryCalls++;
	return denyTreeAlloc ? NULL : realloc(p, n);
}
static void testFree(void *p)
{
	treeMemoryCalls++;
	free(p);
}
#undef cpcalloc
#undef cprealloc
#undef cpfree
#define cpcalloc testCalloc
#define cprealloc testRealloc
#define cpfree testFree
#include "../src/cpBBTree.c"
#undef cpcalloc
#undef cprealloc
#undef cpfree
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
typedef struct Object
{
	cpBB bb;
	int id, owner, where;
} Object;
static char keys[8];
static unsigned long ownerReads;
static int eligible;
static const void *plainKey(Object *o)
{
	return o->owner < 0 ? NULL : keys + o->owner;
}
static cpBB bounds(void *p)
{
	return ((Object *)p)->bb;
}
static const void *owner(void *p)
{
	ownerReads++;
	return plainKey((Object *)p);
}
static const void *query(void *p)
{
	return eligible ? plainKey((Object *)p) : NULL;
}
static int active(cpBBTree *t)
{
	return t->ownersActive;
}
static void validate(cpBBTree *t, int leaves)
{
	Node **stack = (Node **)malloc((size_t)(2 * leaves + 2) * sizeof(*stack));
	CHECK(stack);
	int n = 0, count = 0, internals = 0;
	if(t->root)
	{
		CHECK(!t->root->parent);
		stack[n++] = t->root;
	}
	while(n)
	{
		Node *p = stack[--n];
		if(p->obj)
		{
			count++;
			continue;
		}
		internals++;
		CHECK(p->A->parent == p && p->B->parent == p);
		cpBB bb = cpBBMerge(p->A->bb, p->B->bb);
		CHECK(cpBBContainsBB(p->bb, bb));
		const void *a = p->A->obj ? plainKey((Object *)p->A->obj) : p->A->node.children.owner,
				   *b = p->B->obj ? plainKey((Object *)p->B->obj) : p->B->node.children.owner;
		CHECK(p->node.children.owner == (active(t) ? (a && a == b ? a : NULL) : NULL));
		CHECK(n + 2 <= 2 * leaves + 2);
		stack[n++] = p->B;
		stack[n++] = p->A;
	}
	CHECK(count == leaves);
	CHECK(internals == (leaves ? leaves - 1 : 0));
	free(stack);
}
static Node *chain(cpBBTree *t, Object *o, int count, int offset)
{
	Node *root = NULL;
	for(int i = 0; i < count; i++)
	{
		o[i] = OWNER_TEST_LITERAL(Object){cpBBNew(0, 0, 1, 1), offset + i, 0, 1};
		Node *leaf = LeafNew(t, o + i, o[i].bb);
		root = root ? NodeNew(t, root, leaf) : leaf;
	}
	return root;
}
static void probe(int count)
{
	CHECK(count >= 2);
	cpSpatialIndex *si = cpBBTreeNew(bounds, NULL), *di = cpBBTreeNew(bounds, si);
	cpBBTree *s = (cpBBTree *)si, *d = (cpBBTree *)di;
	CHECK(cpBBTreeConfigureOwners(di, owner, query));
	CHECK(cpBBTreeDeferOwners(si));
	Object *objects = (Object *)calloc((size_t)count, sizeof(*objects));
	CHECK(objects);
	ownerReads = 0;
	eligible = 0;
	int left = count / 2;
	s->root = chain(s, objects, left, 0);
	d->root = chain(d, objects + left, count - left, left);
	unsigned long setupReads = ownerReads;
	validate(s, left);
	validate(d, count - left);
	Object q = {cpBBNew(0, 0, 1, 1), count, 0, 0};
	Node leaf = {0};
	leaf.obj = &q;
	leaf.bb = q.bb;
	MarkContext context = {d, s->root, NULL, NULL};
	unsigned long memoryBefore = treeMemoryCalls, readsBefore = ownerReads;
	eligible = 1;
	denyTreeAlloc = 1;
	MarkLeafQuery(d->root, &leaf, cpTrue, &context);
	denyTreeAlloc = 0;
	CHECK(treeMemoryCalls == memoryBefore);
	CHECK(active(s) && active(d));
	validate(s, left);
	validate(d, count - left);
	unsigned long activationReads = ownerReads - readsBefore;
	CHECK(setupReads == 0);
	CHECK(activationReads ==
		  (unsigned long)((left > 1 ? left : 0) + (count - left > 1 ? count - left : 0) +
						  (count - left == 1 ? 1 : 0)));
	cpSpatialIndexFree(di);
	cpSpatialIndexFree(si);
	free(objects);
}
#define N 96
typedef struct World
{
	cpSpatialIndex *s, *d;
	Object objects[N];
	uint64_t hash, calls;
	int flip;
} World;
static void hash(World *w, const void *p, size_t n)
{
	const unsigned char *b = (const unsigned char *)p;
	while(n--)
	{
		w->hash ^= *b++;
		w->hash *= 1099511628211ULL;
	}
}
static cpCollisionID queryCount(void *a, void *b, cpCollisionID id, void *data)
{
	(void)a;
	(void)b;
	(*(int *)data)++;
	return id;
}
static cpCollisionID visit(void *aa, void *bb, cpCollisionID id, void *data)
{
	World *w = (World *)data;
	Object *a = (Object *)aa, *b = (Object *)bb;
	w->calls++;
	hash(w, &a->id, sizeof(a->id));
	hash(w, &b->id, sizeof(b->id));
	hash(w, &id, sizeof(id));
	if(w->flip) eligible = 1;
	if(w->calls % 257 == 0)
	{
		int count = 0;
		int beforeD = active((cpBBTree *)w->d), beforeS = active((cpBBTree *)w->s);
		cpSpatialIndexQuery(w->d, NULL, cpBBNew(-1e8, -1e8, 1e8, 1e8), queryCount, &count);
		cpSpatialIndexQuery(w->s, NULL, cpBBNew(-1e8, -1e8, 1e8, 1e8), queryCount, &count);
		CHECK(count == N);
		CHECK(active((cpBBTree *)w->d) == beforeD && active((cpBBTree *)w->s) == beforeS);
	}
	return (cpCollisionID)(17 + a->id * N + b->id);
}
static cpSpatialIndex *indexFor(World *w, Object *o)
{
	return o->where == 1 ? w->d : w->s;
}
static void validateWorld(World *w)
{
	int d = 0;
	for(int i = 0; i < N; i++)
		d += w->objects[i].where == 1;
	validate((cpBBTree *)w->d, d);
	validate((cpBBTree *)w->s, N - d);
}
static uint64_t world(int lazy)
{
	World w = {0};
	w.hash = 1469598103934665603ULL;
	w.s = cpBBTreeNew(bounds, NULL);
	w.d = cpBBTreeNew(bounds, w.s);
	CHECK(!cpBBTreeDeferOwners(w.d));
	CHECK(cpBBTreeConfigureOwners(w.d, owner, query));
	if(lazy) CHECK(cpBBTreeDeferOwners(w.s));
	ownerReads = 0;
	eligible = 0;
	for(int i = 0; i < N; i++)
	{
		w.objects[i] =
			OWNER_TEST_LITERAL(Object){cpBBNew(i % 12, i / 12, i % 12 + 4, i / 12 + 4), i, i % 4, i % 3 ? 1 : 2};
		cpSpatialIndexInsert(indexFor(&w, w.objects + i), w.objects + i, i);
	}
	validateWorld(&w);
	if(lazy) CHECK(ownerReads == 0);
	for(int step = 0; step < 64; step++)
	{
		int i = (step * 37) % N;
		Object *o = w.objects + i;
		cpSpatialIndex *old = indexFor(&w, o);
		cpSpatialIndexRemove(old, o, i);
		o->owner = (o->owner + 1) % 4;
		o->where = o->where == 1 ? 2 : 1;
		o->bb = cpBBNew(step % 12, step % 8, step % 12 + 5, step % 8 + 5);
		cpSpatialIndexInsert(indexFor(&w, o), o, i);
		if(step % 8 == 0)
		{
			cpBBTreeOptimize(w.d);
			cpBBTreeOptimize(w.s);
		}
		cpSpatialIndexReindexObject(indexFor(&w, o), o, i);
		cpSpatialIndexReindexQuery(w.d, visit, &w);
		validateWorld(&w);
		if(lazy)
		{
			CHECK(!active((cpBBTree *)w.d) && !active((cpBBTree *)w.s));
			CHECK(ownerReads == 0);
		}
	}
	// First eligible query happens only AFTER a callback flips the generic key.
	// Force fresh moving leaves so both per-subtree refresh and linked bootstrap run.
	for(int i = 0; i < N; i++)
		if(w.objects[i].where == 1)
		{
			Object *o = w.objects + i;
			o->bb = cpBBNew(500 + i % 12, i / 12, 505 + i % 12, i / 12 + 5);
		}
	w.flip = 1;
	cpSpatialIndexReindexQuery(w.d, visit, &w);
	CHECK(eligible);
	CHECK(active((cpBBTree *)w.d) && active((cpBBTree *)w.s));
	validateWorld(&w);
	for(int step = 0; step < 32; step++)
	{
		int i = (step * 29) % N;
		Object *o = w.objects + i;
		cpSpatialIndexRemove(indexFor(&w, o), o, i);
		o->owner = (o->owner + 3) % 4;
		cpSpatialIndexInsert(indexFor(&w, o), o, i);
		if(step % 4 == 0)
		{
			cpBBTreeOptimize(w.s);
			cpBBTreeOptimize(w.d);
		}
		cpSpatialIndexReindexQuery(w.d, visit, &w);
		validateWorld(&w);
	}
	for(int i = 0; i < N; i++)
		cpSpatialIndexRemove(indexFor(&w, w.objects + i), w.objects + i, i);
	CHECK(cpBBTreeDeferOwners(w.d));
	CHECK(!active((cpBBTree *)w.s) && !active((cpBBTree *)w.d));
	unsigned long before = treeMemoryCalls;
	denyTreeAlloc = 1;
	ActivateOwners((cpBBTree *)w.s);
	denyTreeAlloc = 0;
	CHECK(treeMemoryCalls == before);
	CHECK(active((cpBBTree *)w.d) && active((cpBBTree *)w.s));
	CHECK(cpBBTreeDeferOwners(w.s));
	CHECK(cpBBTreeConfigureOwners(w.d, owner, query));
	CHECK(active((cpBBTree *)w.d) && active((cpBBTree *)w.s));
	CHECK(cpBBTreeDeferOwners(w.d));
	eligible = 0;
	ownerReads = 0;
	Object one = {cpBBNew(0, 0, 1, 1), 0, 0, 1};
	cpSpatialIndexInsert(w.d, &one, 0);
	CHECK(ownerReads == 0);
	CHECK(!cpBBTreeDeferOwners(w.d));
	eligible = 1;
	before = treeMemoryCalls;
	denyTreeAlloc = 1;
	ActivateOwners((cpBBTree *)w.d);
	denyTreeAlloc = 0;
	CHECK(treeMemoryCalls == before);
	validate((cpBBTree *)w.d, 1);
	validate((cpBBTree *)w.s, 0);
	cpSpatialIndexFree(w.d);
	cpSpatialIndexFree(w.s);
	return w.hash;
}
#endif
int main(void)
{
#if CP_SHAPE_OWNER_PRUNING
	CHECK(world(0) == world(1));
	probe(2);
	probe(3);
	probe(8192);
	puts("Lazy owner differential dormant/active bounds, pool/optimize/transfers, mutable "
		 "eligibility, linked roots, empty/leaf/deep rebuild, allocation denial, nested queries "
		 "and callback IDs PASS");
#else
	puts("Lazy owner differential SKIP: lazy activation compiled out");
#endif
	return 0;
}
