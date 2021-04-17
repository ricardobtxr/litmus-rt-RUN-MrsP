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

#define WANT_OTHER_SCHED_EVENTS

/* RUN SERVER. Maintains the information of the runtime status of a leaf
 * server for the reduction tree.
 *
 * id          : unique identifier
 * cpu         : last cpu that scheduled the server
 * running     : flag to identify if the server is currently scheduled
 * released    : flag to identify a new release inside the server
 *
 * hp_task     : the task that the server executes when running
 * ready_queue : edf ready queue for the ready tasks
 * rn          : pointer to the node structure of the reduction tree
 *
 * rsp_status  : flag that identify whether a task in the server is waiting or
 *               holding a shared resource
 * is_shadow_running_for:
 *               points to the server that is running and waiting for
 *               the resource retained by this server
 * mrsp_ceiling: SRP property indicating the ceiling of the server, i.e.,
 * 				 the maximum mrsp_ceiling of resources which are locked
 * 				 by or waited by the tasks of the server.
 * mrsp_ceiling_stack:
 * 				 stack for storing servers' ceilings for allowing to restore
 * 				 to the previous ceiling when unlocking a resource.
 */
typedef struct run_server {

	int id;
	int cpu;
	int running;
	int released;

	struct task_struct  *hp_task;
	struct bheap        ready_queue;
	struct r_node       *rn;

	int                 task_n;
	struct task_struct  *task_ref[MAX_TASK_FOR_SERVER];

	/*
	 * we are using period as a parameter for defining the ceiling
	 * of a resource, so we will use only par periods and when the helping
	 * mechanism will be activated, we will subtract 1 from the task preemption
	 * level. this variable will receive the dynamic server ceiling property
	 * of MrsP. */
	int						mrsp_ceiling;
	struct tiny_stack_int 	mrsp_ceiling_stack;

} run_server_t;

/* CPU ENTRY. Maintains the runtime status of a cpu
 *
 * cpu         : identifier for a specific cpu
 * resched     : flag to notify the cpu must perform a rechedule. Probably
 *               because a different server must be scheduled.
 * sched       : task currently being scheduled on the cpu
 * s_sched     : server currently being scheduled on the cpu
 */
typedef struct  {

	int cpu;
	int resched;
	struct task_struct  *sched;
	run_server_t        *s_sched;

} cpu_entry_t;

/* RUN RESOURCE. Mantains the state of a shared resource. It is used for
 * the RUN resource sharing protocol.
 *
 * litmus_lock      : structure needed to store litmus_lock_ops
 * id               : unique identifier for the resource
 * actively_waiting : counts the number of servers in the queue that are
 *                    scheduled as 'running' by RUN
 * holding_server   : pointer to the server whose task hold the resource.
 *                    NULL if the resource is free
 * fifo_queue       : structure to remember the order of the requests performed
 *                    by the tasks
 * queue_in_index   : supporting info to implement the fifo queue. It remembers
 *                    the free position where the insertion can be performed
 * queue_out_index  : supporting info to implement the fifo queue. It remembers
 *                    the index of the first queued element
 * next_ticket      : manual implementation for a ticket spinlock
 * actual_ticket    : manual implementation for a ticket spinlock
 * holding_task		: pointer to the task holding the resource.
 *                    NULL if the resource is free
 * mrsp_ceiling		: SRP property indicating the ceiling of the resource per each server
 */
struct run_resource {

	struct litmus_lock litmus_lock;
	int id;
	int actively_waiting;

	run_server_t *holding_server;

	struct task_struct *fifo_queue[MAX_SERVER];
	int          queue_in_index;
	int          queue_out_index;

	atomic_t 	 next_ticket;
	atomic_t 	 actual_ticket;

	struct task_struct *holding_task;
	int			 mrsp_ceiling[MAX_SERVER];
};

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

/*
 * RUN DOMAIN. Maintains the data structures necessary to RUN. It is a global
 * structure.
 *
 * n_cpus    : numbers of available cpus
 * n_servers : numbers of leaf servers for the given taskset
 * ts        : support structure to optimize the rescheduling events on the cpu.
 * domain    : an edf domain. It is used to gather releases of multiple tasks in
 *             a single event
 * rtree     : the reduction tree for the given taskset
 * servers   : list of all the leaf servers
 */
typedef struct {

	int n_cpus;
	int n_servers;

	struct tiny_stack ts;

	rt_domain_t   domain;
	r_tree_t      rtree;
	run_server_t  *servers[MAX_SERVER];

} run_domain_t;

static inline void sched_node_upd_bdgt(r_node_t *n, lt_t now);
static inline void sched_node_start_exec(r_node_t *n, lt_t now);
static inline void sched_node_stop_exec(r_node_t *n, lt_t now);

static inline r_node_t *get_earliest_node(r_node_t *n);
static inline lt_t get_earliest_node_deadline(r_node_t *n, lt_t now);
static inline enum hrtimer_restart on_server_budget_event(struct hrtimer *timer);
static void _update_r_node(r_node_t *n, lt_t now);

static inline void INIT_R_NODE(r_node_t *n);
static inline void INIT_R_TREE(r_tree_t *handle)
{
	handle->root = NULL;
	handle->level = -1;
}

static inline int r_tree_empty(r_tree_t *handle)
{
	return (handle->root == NULL);
}

static inline void r_tree_traversal(r_node_t *n, lt_t now, struct tiny_stack *ts);

//See http://leetcode.com/2010/10/binary-tree-post-order-traversal.html or wikipedia
static inline void r_tree_dealloc(r_node_t *n, struct tiny_stack *ts);

static inline lt_t get_job_deadline(struct task_struct *t, lt_t now);
static inline lt_t get_earliest_deadline(run_server_t *s, lt_t now);

/* Verify if the task is running or not
 * t: 	task to verify if it is running or not
 */
static inline int task_is_running(struct task_struct* t);

/* Verify if the task is busy waiting for the resource
 * t: 	task to verify if it is busy waiting or not
 * r:   resource to verify if task t is waiting for it
 */
static inline int is_busy_waiting_for(struct task_struct* t, struct run_resource* r);

/* Verify if the task which is holding the resource is running or not
 * run_resource: 	resource that is locked by the task which we are asking
 * 					if it is running.
 */
static inline int holding_task_is_running(struct run_resource* r);

/* Define the params for the helping task and the resource holding task
 * which is the one that is being helped by the helping task.
 * helping_task: task that request the resource which is blocked by other task.
 * resource_holding: task which is blocking the requested resource and is not
 * 			running.
 */
static inline void set_helping_task(struct task_struct *helping_task, struct task_struct *resource_holding);

/* resource_holding: task that release the resource which is waited by other task.
 */
static inline void reset_helping_task(struct task_struct *resource_holding);

/* resource_holding: task that release the resource which is waited by other task.
 */
static inline void reset_helping_task_no_requeue(struct task_struct *resource_holding);

/* higher_preemption_level
 * 		  	returns true if first has a higher preemption level
 *          than second.
 * both first and second may be NULL
 */
static inline int higher_preemption_level(struct task_struct* t);

/* Verify if the task needs to help the resource holding task
 * t: 	task to verify
 */
static inline int is_task_need_for_helping(struct task_struct* t);
static inline struct task_struct* set_next_task_check_helping(struct task_struct* next);

static inline int busy_wait_for_resource(struct task_struct  *t, struct run_resource *r, unsigned long flags);

static inline void unqueue_task_preempted(struct task_struct *t);

#endif

