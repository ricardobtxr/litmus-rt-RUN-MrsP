/* 
 * sched_run.h
 */

#ifndef SCHED_RUN_H
#define SCHED_RUN_H

#include <linux/kernel.h>

//#include <litmus/tiny_heap.h>
#include <litmus/tiny_stack.h>

#define INIT_HORIZON 		0LL
#define MAX_SERVER 			32
#define	MAX_TASK_FOR_SERVER	16
//#define MAX_CPU		8

#define ROUND_ERROR 2000LL
// 140120 - In case of exhaution execute before release even thought it has to execute after budget may not correctly consumed
#define ORDER_ERROR 20000LL

#define circle(node)	node->circled = 1
#define	uncircle(node)	node->circled = 0

static inline lt_t lt_t_min(lt_t a, lt_t b) {
	return (lt_after(a,b) ? b : a);   
}

typedef struct r_node {
	
	int				id;
	int 			circled; // flag indicating the current node is selected for execution
	int				level;
	
	lt_t			rel;
	lt_t			e_dl; // earliest deadline among the child nodes
	
	lt_t			rate[2];
	lt_t			budget; // current updated budget
	lt_t			start_time;
	
	struct r_node* 		sched;
	
	struct hrtimer		timer; // keep track of when the execution budget attached to it is consumed
	int			armed; // if a node selected a child for execution the timer is armed
	
	struct r_node*		parent;
	struct r_node*		l_c; // Left child
	struct r_node*		r_s; // right sibling

	int			is_ZL;
	
	int			color;
} r_node_t;

typedef struct r_tree {

	struct r_node*	root;
	int		level;
	
		
} r_tree_t;

static inline void set_node_global_time(r_node_t *n, lt_t global_time) {
	n->start_time = global_time;
}

static inline int node_budget_exhausted(r_node_t *n, lt_t now) {
	return ((now - n->start_time) >= n->budget);
}

static inline lt_t node_budget_remaining(r_node_t *n) {
	return n->budget;
}

static inline void sched_node_update_budget(r_node_t *n, lt_t now) {
	
	lt_t interval;
	
	if (n->budget == 0) return;
	
	interval = now - n->start_time;
	
	if ((interval > 0) && ((interval /*+ ORDER_ERROR*/) < n->budget))
		n->budget -= interval;
	else
		n->budget = 0;

}

static inline void sched_node_start_exec(r_node_t *n, lt_t now) {
	if (!n) return;
	n->start_time = now;
	circle(n);
}

static inline void sched_node_stop_exec(r_node_t *n, lt_t now) {
	if (!n) return;
	if (!n->is_ZL) 
		sched_node_update_budget(n, now);
	else
		n->is_ZL = 0;
	uncircle(n);
}

static inline r_node_t *get_earliest_node(r_node_t *n) {
	
	r_node_t 	*tmp;
	r_node_t	*e_n;
	lt_t		e_dl;
	
	tmp = n->l_c;

	if (n->sched && (n->sched->budget > 0)) {
		e_n = n->sched;
	} else {		
		while (tmp && (tmp->budget <= 0)) {
			tmp = tmp->r_s;
		}
		e_n = tmp;
	}

	BUG_ON(e_n && (e_n->budget <= 0));
	
	e_dl = (e_n ? e_n->e_dl : INIT_HORIZON);
	
	while (tmp) {
		if((tmp->budget > 0) && lt_after(e_dl, tmp->e_dl)) { 
			e_n = tmp;
			e_dl = tmp->e_dl;
		}
		tmp = tmp->r_s;
	}
	return e_n;
}

static inline lt_t get_earliest_node_deadline(r_node_t *n, lt_t now) {
	
	r_node_t 	*tmp;
	lt_t		e_dl;
	
	if (!n) return INIT_HORIZON;
	tmp = n->l_c;
	if (!tmp) return INIT_HORIZON;
	
	e_dl = INIT_HORIZON;
	while (tmp) {
		if(lt_after(tmp->e_dl, now) && (e_dl == INIT_HORIZON))
			e_dl = tmp->e_dl;
		if(lt_after(tmp->e_dl, now) && lt_after(e_dl, tmp->e_dl))
			e_dl = tmp->e_dl;
		tmp = tmp->r_s;
	}
	return e_dl;
}

static inline r_node_t *_get_earliest_node(r_node_t *n) {
	
	r_node_t 	*tmp;
	r_node_t	*e_n;
	lt_t		e_dl;
	
	e_n = NULL;
	e_dl = LLONG_MAX;
	tmp = n->l_c;
	while (tmp) {
		if(lt_after_eq(e_dl, tmp->e_dl) && (tmp->budget > 0)) { 
			e_n = tmp;
			e_dl = tmp->e_dl;
		}
		tmp = tmp->r_s;
	}
	
	return e_n;
}

static enum hrtimer_restart on_server_budget_event(struct hrtimer *timer);
//static void update_r_node(r_node_t *n, lt_t now);
static void _update_r_node(r_node_t *n, lt_t now);

static inline void INIT_R_NODE(r_node_t *n)
{
	n->id = -1;
	n->circled = 0;
	n->level = -1;
	
	n->rel	= INIT_HORIZON;
	n->e_dl = INIT_HORIZON;
	
	n->rate[0] = 0;
	n->rate[1] = 0;
	n->budget = 0;
	n->start_time = INIT_HORIZON;
	
	n->sched = NULL;
	
	hrtimer_init(&n->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	n->timer.function = on_server_budget_event;
	n->armed = 0;
	
	n->parent = NULL;
	n->l_c = NULL;
	n->r_s = NULL;

	n->is_ZL = 0;
	
	n->color = -1;
}

static inline void INIT_R_TREE(r_tree_t *handle)
{
	handle->root = NULL;
	handle->level = -1;
	
}

static inline int r_tree_empty(r_tree_t *handle)
{
	return (handle->root == NULL);
}

static inline void r_tree_traversal(r_node_t *n, 
					lt_t now, 
					struct tiny_stack *ts)
{		
	
	r_node_t *node;
	
	if (!n) return;
	t_st_init(ts);

	_update_r_node(n, now);
	node = n->l_c;
	while ((!t_st_empty(ts)) || (node != NULL))
	{
		if (node != NULL) {
			_update_r_node(node, now);
			t_st_push(ts, node->r_s);
			node = node->l_c;		
		} else {
			node = t_st_pop(ts);
		}
	}
}

//See http://leetcode.com/2010/10/binary-tree-post-order-traversal.html
//or wikipedia
static inline void r_tree_dealloc(r_node_t *n, struct tiny_stack *ts)
{		
	
	r_node_t *curr, *prev;
	
	if (!n) return;
	t_st_init(ts);
	
	
	t_st_push(ts, n);
	curr = NULL;	
	prev = NULL;
	while (!t_st_empty(ts)) {
		curr = t_st_peek(ts);
		if (!prev || (prev->l_c == curr) || (prev->r_s == curr)) {
			if (curr->l_c)
				t_st_push(ts, curr->l_c);
			else if (curr->r_s)
				t_st_push(ts, curr->r_s);
		}
		else if (curr->l_c == prev) {
			if (curr->r_s)
				t_st_push(ts, curr->r_s);
		} else {
			/*visit curr*/
//			TRACE("Freeing node %d\n", curr->id);
			kfree(curr);
			t_st_pop(ts);
		}
		prev = curr;
	}
}
#endif
