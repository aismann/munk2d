static void ReferenceMarkLeafQuery(Node *subtree, Node *leaf, cpBool left, MarkContext *context)
{
#if CP_SHAPE_OWNER_PRUNING
	cpBBTree *tree = context->tree;
	const void *owner = tree->queryKey ? tree->queryKey(leaf->obj)
									   : (tree->ownerKey ? tree->ownerKey(leaf->obj) : NULL);
#endif
	CP_STACK_INIT(stack);

	CP_STACK_PUSH(stack, subtree);

	while(CP_STACK_NOTEMPTY(stack))
	{
		Node *node = CP_STACK_POP(stack);
#if CP_SHAPE_OWNER_PRUNING
		if(owner && !NodeIsLeaf(node) && node->node.children.owner == owner) continue;
#endif
		if(cpBBIntersects(leaf->bb, node->bb))
		{
			if(NodeIsLeaf(node))
			{
#if CP_SHAPE_OWNER_PRUNING
				if(owner && NodeOwner(tree, node) == owner) continue;
#endif
				if(left)
				{
					PairInsert(leaf, node, context->tree);
				}
				else
				{
					if(node->STAMP < leaf->STAMP) PairInsert(node, leaf, context->tree);
					context->func(leaf->obj, node->obj, 0, context->data);
				}
			}
			else
			{
				CP_STACK_PUSH(stack, node->B);
				CP_STACK_PUSH(stack, node->A);
			}
		}
	}

	CP_STACK_FREE(stack);
}

static void ReferenceMarkLeaf(Node *leaf, MarkContext *context)
{
	cpBBTree *tree = context->tree;
	if(leaf->STAMP == GetMasterTree(tree)->stamp)
	{
		Node *staticRoot = context->staticRoot;
		if(staticRoot) ReferenceMarkLeafQuery(staticRoot, leaf, cpFalse, context);

		for(Node *node = leaf; node->parent; node = node->parent)
		{
			if(node == node->parent->A)
			{
				ReferenceMarkLeafQuery(node->parent->B, leaf, cpTrue, context);
			}
			else
			{
				ReferenceMarkLeafQuery(node->parent->A, leaf, cpFalse, context);
			}
		}
	}
	else
	{
		for(Pair *pair = leaf->OWNED_PAIRS; pair; pair = pair->b.next)
			pair->id = context->func(pair->a.leaf->obj, leaf->obj, pair->id, context->data);
	}
}
