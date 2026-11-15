/* Private topology model plus public cached-ID/order regression trace. */
#ifndef CP_BBTREE_PAIR_SOURCE
#define CP_BBTREE_PAIR_SOURCE "../src/cpBBTree.c"
#endif
#include CP_BBTREE_PAIR_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"pair test %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);}} while(0)
#define LEAVES 32
#define SLOTS 2048
static Node nodes[LEAVES];
static int objects[LEAVES], model[LEAVES][SLOTS], lengths[LEAVES];
typedef struct ModelPair {Pair *pair;int a,b,live;cpCollisionID id;} ModelPair;
static ModelPair slots[SLOTS];
static uint32_t rng=UINT32_C(0x18f375ab);
static uint32_t random32(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static uint64_t digest=UINT64_C(1469598103934665603);
static void hashValue(uint64_t x){digest^=x;digest*=UINT64_C(1099511628211);}
static cpBB emptyBB(void *p){(void)p;return cpBBNew(0,0,1,1);}
static void verifyLists(void)
{
 for(int leaf=0;leaf<LEAVES;leaf++){
#ifdef OWNED_PAIRS
  for(int role=0;role<2;role++){
   Pair *p=role?nodes[leaf].OWNED_PAIRS:nodes[leaf].PAIRS,*prev=NULL;
   for(int j=0;j<lengths[leaf];j++){
    ModelPair *m=&slots[model[leaf][j]];
    if((role?m->b:m->a)!=leaf)continue;
    CHECK(m->live&&p==m->pair);Thread t=role?p->b:p->a;
    CHECK(t.leaf==&nodes[leaf]&&t.prev==prev&&p->id==m->id);
    prev=p;p=t.next;
   }
   CHECK(!p);
  }
#else
  Pair *p=nodes[leaf].PAIRS,*prev=NULL;
  for(int j=0;j<lengths[leaf];j++){
   ModelPair *m=&slots[model[leaf][j]];
   CHECK(m->live&&p==m->pair);Thread t=m->a==leaf?p->a:p->b;
   CHECK(t.leaf==&nodes[leaf]&&t.prev==prev&&p->id==m->id);
   prev=p;p=t.next;
  }
  CHECK(!p);
#endif
 }
}
static void removeModel(int slot)
{
 ModelPair *m=&slots[slot];CHECK(m->live);
 int endpoints[2]={m->a,m->b};
 for(int k=0;k<2;k++){
  int leaf=endpoints[k],j=0;while(j<lengths[leaf]&&model[leaf][j]!=slot)j++;
  CHECK(j<lengths[leaf]);memmove(&model[leaf][j],&model[leaf][j+1],(size_t)(lengths[leaf]-j-1)*sizeof(int));lengths[leaf]--;
 }
 m->live=0;
}
static void clearLeaf(cpBBTree *tree,int leaf)
{
 PairsClear(&nodes[leaf],tree);
 while(lengths[leaf])removeModel(model[leaf][0]);
}
static void insertPair(cpBBTree *tree,int a,int b,cpCollisionID id)
{
 int slot=0;while(slot<SLOTS&&slots[slot].live)slot++;CHECK(slot<SLOTS&&a!=b);
 PairInsert(&nodes[a],&nodes[b],tree);
#ifdef OWNED_PAIRS
 Pair *p=nodes[b].OWNED_PAIRS;
#else
 Pair *p=nodes[b].PAIRS;
#endif
 CHECK(p&&p->a.leaf==&nodes[a]&&p->b.leaf==&nodes[b]&&p->id==0);
 p->id=id;ModelPair next={p,a,b,1,id};slots[slot]=next;
 int endpoints[2]={a,b};for(int k=0;k<2;k++){
  int leaf=endpoints[k];CHECK(lengths[leaf]<SLOTS);
  memmove(&model[leaf][1],&model[leaf][0],(size_t)lengths[leaf]*sizeof(int));model[leaf][0]=slot;lengths[leaf]++;
 }
}
typedef struct Expected {int slots[SLOTS],count,cursor;} Expected;
static cpCollisionID modelCallback(void *a,void *b,cpCollisionID id,void *data)
{
 Expected *e=(Expected *)data;CHECK(e->cursor<e->count);
 ModelPair *m=&slots[e->slots[e->cursor++]];
 CHECK(a==&objects[m->a]&&b==&objects[m->b]&&id==m->id);
 hashValue((uint64_t)m->a);hashValue((uint64_t)m->b);hashValue((uint64_t)id);
 m->id=id^(cpCollisionID)UINT32_C(0x9e3779b9);return m->id;
}
static void checkCallbacks(cpBBTree *tree,int leaf)
{
 Expected e;memset(&e,0,sizeof(e));
 for(int j=0;j<lengths[leaf];j++){int slot=model[leaf][j];if(slots[slot].b==leaf)e.slots[e.count++]=slot;}
 MarkContext context={tree,NULL,modelCallback,&e};MarkLeaf(&nodes[leaf],&context);CHECK(e.cursor==e.count);
}
static void topologyModel(void)
{
 cpBBTree *tree=(cpBBTree *)cpBBTreeNew(emptyBB,NULL);tree->stamp=1;
 for(int i=0;i<LEAVES;i++){objects[i]=i;nodes[i].obj=&objects[i];nodes[i].STAMP=0;}
 cpCollisionID id=1;
 for(int i=0;i<1300;i++){int a=(int)(random32()%LEAVES),b=(a+1+(int)(random32()%(LEAVES-1)))%LEAVES;insertPair(tree,a,b,id++);}
 if(CP_BUFFER_BYTES/sizeof(Pair)<1300)CHECK(tree->allocatedBuffers->num>=2);
 verifyLists();for(int i=0;i<LEAVES;i++)checkCallbacks(tree,i);verifyLists();
 for(int op=0;op<20000;op++){
  int a=(int)(random32()%LEAVES);uint32_t choice=random32()%10;
  if(choice<5){int b=(a+1+(int)(random32()%(LEAVES-1)))%LEAVES;insertPair(tree,a,b,id++);}
  else if(choice<8)clearLeaf(tree,a);
  else checkCallbacks(tree,a);
  verifyLists();
  if(op%97==0){for(int i=0;i<LEAVES;i++)checkCallbacks(tree,i);verifyLists();}
 }
 for(int i=0;i<LEAVES;i++)clearLeaf(tree,i);
 verifyLists();for(int i=0;i<SLOTS;i++)CHECK(!slots[i].live);
 cpSpatialIndexFree((cpSpatialIndex *)tree);
}
typedef struct Object {int id,active;cpBB bb;cpVect velocity;} Object;
static cpBB bounds(void *p){return ((Object *)p)->bb;}
static cpVect velocity(void *p){return ((Object *)p)->velocity;}
static cpCollisionID traceCallback(void *a,void *b,cpCollisionID id,void *unused)
{
 (void)unused;Object *x=(Object *)a,*y=(Object *)b;CHECK(x&&y&&x!=y);
 hashValue((uint64_t)x->id);hashValue((uint64_t)y->id);hashValue((uint64_t)id);
 return (id*(cpCollisionID)1664525u+(cpCollisionID)(x->id*31+y->id+1));
}
static void setBounds(Object *o,int frame)
{
 cpFloat x=(cpFloat)((o->id*13+frame*7)%57)*0.25,y=(cpFloat)((o->id*7+frame*3)%41)*0.25;
 if(o->id==31){x=10000+(cpFloat)frame*2;y=10000;}
 o->bb=cpBBNew(x,y,x+2,y+2);o->velocity=cpv(o->id==31?100:0,o->id==31?-80:0);
}
static void publicTrace(void)
{
 Object moving[32],fixed[16];memset(moving,0,sizeof(moving));memset(fixed,0,sizeof(fixed));
 cpSpatialIndex *stat=cpBBTreeNew(bounds,NULL),*dyn=cpBBTreeNew(bounds,stat);
 cpBBTreeSetVelocityFunc(dyn,velocity);
 for(int i=0;i<16;i++){fixed[i].id=100+i;fixed[i].active=1;setBounds(&fixed[i],0);cpSpatialIndexInsert(stat,&fixed[i],(cpHashValue)(100+i));}
 for(int i=0;i<32;i++){moving[i].id=i;moving[i].active=1;setBounds(&moving[i],0);cpSpatialIndexInsert(dyn,&moving[i],(cpHashValue)i);}
 for(int frame=0;frame<600;frame++){
  for(int k=0;k<4;k++){int i=(frame*5+k*7)%32;if(moving[i].active)setBounds(&moving[i],frame);}
  if(frame%7==0){int i=(frame/7)%32;if(moving[i].active){cpSpatialIndexRemove(dyn,&moving[i],(cpHashValue)i);moving[i].active=0;}else{setBounds(&moving[i],frame);cpSpatialIndexInsert(dyn,&moving[i],(cpHashValue)i);moving[i].active=1;}}
  if(frame%11==0){int i=(frame/11)%16;setBounds(&fixed[i],frame);cpSpatialIndexReindexObject(stat,&fixed[i],(cpHashValue)(100+i));}
  if(frame%13==0){int i=(frame/13)%32;if(moving[i].active)cpSpatialIndexReindexObject(dyn,&moving[i],(cpHashValue)i);}
  if(frame%37==0)cpBBTreeOptimize(dyn);
  cpSpatialIndexReindexQuery(dyn,traceCallback,NULL);
 }
 for(int i=0;i<16;i++)cpSpatialIndexRemove(stat,&fixed[i],(cpHashValue)(100+i));
 cpSpatialIndexReindexQuery(dyn,traceCallback,NULL);
 for(int i=0;i<32;i++)if(moving[i].active)cpSpatialIndexRemove(dyn,&moving[i],(cpHashValue)i);
 CHECK(cpSpatialIndexCount(dyn)==0&&cpSpatialIndexCount(stat)==0);
 cpSpatialIndexFree(dyn);cpSpatialIndexFree(stat);
}
int main(void)
{
 topologyModel();publicTrace();
 printf("PASS pair topology/model, filtered callback order, cached IDs, multiple pools, reuse, both endpoints, static/dynamic remove/reindex/optimize, velocity bounds; trace=%016llx Node=%zu Pair=%zu\n",(unsigned long long)digest,sizeof(Node),sizeof(Pair));
 return 0;
}
