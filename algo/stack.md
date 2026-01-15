栈有两种常见实现方法: 基于数组和基于链表, 两种区别详见 [Array vs Linked List](链表/linked%20list%20vs.%20array.md)

## 数组实现

```c
struct stack_item {
	int val;
} 

struct stack {
	struct stack_item *items;
	size_t top;
	size_t capacity;
};

#define STACK_EMPTY -1
#define stack_is_empty(s) ((s)->top == STACK_EMPTY)
#define stack_is_full(s) ((s)->top == (s)->capacity - 1)
#define stack_size(s) ((s)->top + 1)
#define stack_capacity(s) ((s)->capacity)

bool stack_init(struct stack *s, size_t capacity)
{
	if (capacity <= 0)
		return false;
	s->items = malloc(capacity * sizeof(struct stack_item));	
	assert(s->items != NULL);
	s->top = STACK_EMPTY;
	s->capacity = capacity;
	return true;
}

bool stack_pop(struct stack *s, struct stack_item *item) 
{
	if (stack_is_empty(s)) {
		return false;
	}
	*item = s->items[s->top];
	s->top--;
	return true;
}

bool stack_peek(struct stack *s, struct stack_item *item)
{
	if (stack_is_empty(s)) {
		return false;
	}
	// increase counter of reference, because our stack still holds 
	// the pointer to peeked stack_item.
	*item = s->items[s->top];
	return true;
}

bool stack_push(struct stack *s, struct stack_item* item) 
{
	if (stack_is_full(s))
		return false;
	s->top++;
	s->items[s->top] = *item;
	return true;
}

void stack_resize(struct stack *s, size_t capacity) {
	if (capacity > s->capacity) {
		size_t size = capacity * sizeof(struct stack_item);
		struct stack_item *items = realloc(s->items, size); 
		assert(items != NULL);
		s->items = items;
	}
	s->capacity = capacity;
}
```

## 链表实现

这里使用侵入式链表设计, 

```
stack     top                           bottom
 v         v                              v
header -> node -> node -> node -> ... -> node -> NULL
```

```c
struct stack_item { 
	int val;
	struct stack_item *next;
};

struct stack {
	struct stack_item *top;
};

void stack_init(struct stack *s)
{
	s->top = NULL;
}

/*push:
          last_entry                     first_entry
              v                                v
header -> node(new) -> node -> node -> ... -> node -> NULL
*/

void stack_push(int val, struct stack *s)
{
	assert(s != NULL);
	struct stack_item *it = malloc(sizeof(struct stack_item));

	it->next = s->top;
	it->val = val;
	s->top = it;
	return;
}

/* pop: 
                      last           first
                        v              v
header -- node(out) -> node -> ... -> node -> NULL
*/

int stack_pop(struct stack *s)
{
	assert(list != NULL);
	assert(s->top != NULL);

	struct stack_item *it = s->top;
	int val = it->val;
	s->top = it->next;
	free(it);
	return val;
}
```
