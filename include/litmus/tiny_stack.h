/* 
 * 	tiny_stack.h
 */

#ifndef TINY_STACK
#define TINY_STACK

#define TINY_STACK_SIZE 32

struct tiny_stack {
	void 	*data[TINY_STACK_SIZE];
	int	elements;
};

static inline void t_st_init(struct tiny_stack *st) 
{
	st->elements = -1;
}

static inline int t_st_empty(struct tiny_stack *st)
{
	return (st->elements < 0);
}

static inline void t_st_push(struct tiny_stack *st, void *value) 
{
	if (st->elements > (TINY_STACK_SIZE - 1)) {
		return;
	}
	st->elements += 1;
	st->data[st->elements] = value;
}

static inline void *t_st_pop(struct tiny_stack *st)
{
	void *tmp = NULL;
	if (st->elements < 0)
		return tmp;
	tmp = st->data[st->elements];
	st->elements -= 1;
	return tmp;
}

static inline void *t_st_peek(struct tiny_stack *st)
{
	return (st->elements > -1) ? st->data[st->elements] : NULL;
}

/*
 * tiny stack for only int elements
 */

struct tiny_stack_int {
	int data[TINY_STACK_SIZE];
	int	elements;
};

static inline void t_st_i_init(struct tiny_stack_int *st)
{
	st->elements = -1;
}

static inline int t_st_i_empty(struct tiny_stack_int *st)
{
	return (st->elements < 0);
}

static inline int t_st_i_push(struct tiny_stack_int *st, int value)
{
	if (st->elements > (TINY_STACK_SIZE - 1)) {
		return -1;
	}
	st->elements += 1;
	st->data[st->elements] = value;
	return 0;
}

static inline int t_st_i_pop(struct tiny_stack_int *st, int *value)
{
	if (st->elements < 0)
		return -1;
	*value = st->data[st->elements];
	st->elements -= 1;
	return 0;
}

static inline int t_st_i_peek(struct tiny_stack_int *st, int *value)
{
	if (st->elements < 0)
		return -1;
	*value = st->data[st->elements];
	return 0;
}
#endif
