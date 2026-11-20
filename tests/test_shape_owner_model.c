#include "owner_test_compat.h"
#include "../src/cpBBTree.c"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#define CHECK(x)                                                                                   \
	do                                                                                             \
	{                                                                                              \
		if(!(x))                                                                                   \
		{                                                                                          \
			fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x);                                        \
			exit(1);                                                                               \
		}                                                                                          \
	} while(0)
#define N 96
#define MAX_TRACE 12000
typedef struct Obj
{
	cpBB bb;
	int id, owner, where;
} Obj;
typedef struct Record
{
	int a, b;
	cpCollisionID id;
} Record;
typedef struct Trace
{
	Record rows[MAX_TRACE];
	int n;
} Trace;
static Obj objects[N];
static char keys[8];
static Trace expected, actual;
static uint64_t rng = 0x1e78b61234578ULL, digest = 1469598103934665603ULL;
static uint32_t next(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return (uint32_t)rng;
}
static void hash(const void *p, size_t n)
{
	const unsigned char *s = (const unsigned char *)p;
	while(n--)
	{
		digest ^= *s++;
		digest *= 1099511628211ULL;
	}
}
static cpBB bounds(void *p)
{
	return ((Obj *)p)->bb;
}
static const void *owner(void *p)
{
	Obj *o = (Obj *)p;
	return o->owner >= 0 ? keys + o->owner : NULL;
}
static cpCollisionID token(Obj *a, Obj *b)
{
	int lo = a->id < b->id ? a->id : b->id, hi = a->id > b->id ? a->id : b->id;
	return (cpCollisionID)(97 + 17 * (lo * N + hi));
}
static cpCollisionID visit(Obj *a, Obj *b, cpCollisionID id, Trace *t)
{
	if(a->owner >= 0 && a->owner == b->owner) return id;
	CHECK(t->n < MAX_TRACE);
	CHECK(id == 0 || id == token(a, b));
	t->rows[t->n++] = OWNER_TEST_LITERAL(Record){a->id, b->id, id};
	return token(a, b);
}
static const void *validate(cpBBTree *t, Node *n)
{
	if(!n) return NULL;
	if(NodeIsLeaf(n))
	{
		CHECK(n->obj != NULL);
		return owner(n->obj);
	}
	CHECK(n->A->parent == n && n->B->parent == n);
	const void *a = validate(t, n->A), *b = validate(t, n->B), *key = a && a == b ? a : NULL;
	cpBB bb = cpBBMerge(n->A->bb, n->B->bb);
	CHECK(n->bb.l <= bb.l && n->bb.b <= bb.b && n->bb.r >= bb.r && n->bb.t >= bb.t);
	CHECK(n->node.children.owner == (t->ownerKey ? key : NULL));
	return key;
}
static Node *leaf(cpBBTree *t, Obj *o)
{
	return (Node *)cpHashSetFind(t->leaves, (cpHashValue)o->id, o);
}
static void check(cpSpatialIndex **s, cpSpatialIndex **d)
{
	expected.n = actual.n = 0;
	cpSpatialIndexReindexQuery(d[0], (cpSpatialIndexQueryFunc)visit, &expected);
	cpSpatialIndexReindexQuery(d[1], (cpSpatialIndexQueryFunc)visit, &actual);
	CHECK(expected.n == actual.n);
	for(int i = 0; i < actual.n; i++)
	{
		Record *a = expected.rows + i, *b = actual.rows + i;
		CHECK(a->a == b->a && a->b == b->b && a->id == b->id);
		hash(&b->a, sizeof(b->a));
		hash(&b->b, sizeof(b->b));
		hash(&b->id, sizeof(b->id));
	}
	for(int k = 0; k < 2; k++)
	{
		validate((cpBBTree *)s[k], ((cpBBTree *)s[k])->root);
		validate((cpBBTree *)d[k], ((cpBBTree *)d[k])->root);
	}
	int pairs = 0;
	for(int i = 0; i < N; i++)
		if(objects[i].where)
			for(int j = i + 1; j < N; j++)
				if(objects[j].where && (objects[i].where == 1 || objects[j].where == 1) &&
				   !(objects[i].owner >= 0 && objects[i].owner == objects[j].owner))
				{
					Node *a = leaf((cpBBTree *)(objects[i].where == 1 ? d[1] : s[1]), objects + i),
						 *b = leaf((cpBBTree *)(objects[j].where == 1 ? d[1] : s[1]), objects + j);
					if(cpBBIntersects(a->bb, b->bb)) pairs++;
				}
	CHECK(pairs == actual.n);
}
static void transitions(void)
{
	Obj o[N];
	cpBBTree *t = (cpBBTree *)cpBBTreeNew(bounds, NULL);
	CHECK(cpBBTreeConfigureOwners((cpSpatialIndex *)t, owner, owner));
	for(int i = 0; i < N; i++)
	{
		o[i] = OWNER_TEST_LITERAL(Obj){cpBBNew(i * 2, 0, i * 2 + 1, 1), i, 0, 1};
		cpSpatialIndexInsert((cpSpatialIndex *)t, &o[i], i);
		validate(t, t->root);
	}
	CHECK(t->root->node.children.owner == owner(&o[0]));
	for(int pass = 0; pass < 256; pass++)
	{
		int i = (pass * 37) % N;
		cpSpatialIndexRemove((cpSpatialIndex *)t, &o[i], i);
		validate(t, t->root);
		o[i].owner = 1;
		o[i].bb = cpBBNew(-1000 - pass, -10, 1000 + pass, 10);
		cpSpatialIndexInsert((cpSpatialIndex *)t, &o[i], i);
		validate(t, t->root);
		CHECK(t->root->node.children.owner == NULL);
		cpSpatialIndexRemove((cpSpatialIndex *)t, &o[i], i);
		validate(t, t->root);
		o[i].owner = 0;
		cpSpatialIndexInsert((cpSpatialIndex *)t, &o[i], i);
		validate(t, t->root);
		CHECK(t->root->node.children.owner == owner(&o[0]));
		for(int j = 0; j < N; j++)
		{
			o[j].bb = cpBBNew(j * 2 - pass * 10, -pass - 1, j * 2 + pass * 10 + 1, pass + 2);
			cpSpatialIndexReindexObject((cpSpatialIndex *)t, &o[j], j);
			validate(t, t->root);
		}
		if(pass % 8 == 0)
		{
			cpBBTreeOptimize((cpSpatialIndex *)t);
			validate(t, t->root);
		}
	}
	cpSpatialIndexFree((cpSpatialIndex *)t);
}
int main(int argc, char **argv)
{
	if(argc > 1) rng ^= (uint64_t)strtoul(argv[1], NULL, 0);
	transitions();
#if UINTPTR_MAX == UINT64_MAX && CP_USE_DOUBLES
	CHECK(sizeof(Node) == 72);
	CHECK(sizeof(Pair) == 56);
#endif
	cpSpatialIndex *s[2], *d[2];
	for(int k = 0; k < 2; k++)
	{
		s[k] = cpBBTreeNew(bounds, NULL);
		d[k] = cpBBTreeNew(bounds, s[k]);
	}
	CHECK(cpBBTreeConfigureOwners(d[1], owner, NULL));
	CHECK(cpBBTreeOwnersConfigured(s[1]) && cpBBTreeOwnersConfigured(d[1]));
	for(int i = 0; i < N; i++)
	{
		double x = (int)(next() % 20) - 10, y = (int)(next() % 20) - 10;
		objects[i] = OWNER_TEST_LITERAL(Obj){cpBBNew(x, y, x + 4, y + 4), i, i % 9 == 8 ? -1 : i % 8, i % 4 ? 1 : 2};
		for(int k = 0; k < 2; k++)
			cpSpatialIndexInsert(objects[i].where == 1 ? d[k] : s[k], objects + i, (cpHashValue)i);
	}
	for(int step = 0; step < 800; step++)
	{
		int i = (int)(next() % N), action = (int)(next() % 5);
		Obj *o = objects + i;
		if(action == 0)
		{
			if(o->where)
			{
				for(int k = 0; k < 2; k++)
					cpSpatialIndexRemove(o->where == 1 ? d[k] : s[k], o, (cpHashValue)i);
				o->where = 0;
			}
			else
			{
				o->where = next() % 3 ? 1 : 2;
				for(int k = 0; k < 2; k++)
					cpSpatialIndexInsert(o->where == 1 ? d[k] : s[k], o, (cpHashValue)i);
			}
		}
		else if(action == 1)
		{
			double x = (int)(next() % 24) - 12, y = (int)(next() % 24) - 12;
			o->bb = cpBBNew(x, y, x + 1 + next() % 5, y + 1 + next() % 5);
			if(o->where)
				for(int k = 0; k < 2; k++)
					cpSpatialIndexReindexObject(o->where == 1 ? d[k] : s[k], o, (cpHashValue)i);
		}
		else if(action == 2)
		{
			int old = o->where;
			if(old)
				for(int k = 0; k < 2; k++)
					cpSpatialIndexRemove(old == 1 ? d[k] : s[k], o, (cpHashValue)i);
			o->owner = next() % 9 == 8 ? -1 : (int)(next() % 8);
			if(old)
			{
				o->where = old == 1 ? 2 : 1;
				for(int k = 0; k < 2; k++)
					cpSpatialIndexInsert(o->where == 1 ? d[k] : s[k], o, (cpHashValue)i);
			}
		}
		else if(action == 3)
		{
			for(int k = 0; k < 2; k++)
				cpBBTreeOptimize(d[k]);
		}
		else
		{
			for(int k = 0; k < 2; k++)
				cpBBTreeOptimize(s[k]);
		}
		check(s, d);
	}
	for(int k = 0; k < 2; k++)
	{
		cpSpatialIndexFree(s[k]);
		cpSpatialIndexFree(d[k]);
	}
	printf("Owner model 800 topology operations, filtered callback order/IDs and full fat-pair set "
		   "PASS; Node=%zu Pair=%zu digest=%016llx\n",
		   sizeof(Node), sizeof(Pair), (unsigned long long)digest);
	return 0;
}
