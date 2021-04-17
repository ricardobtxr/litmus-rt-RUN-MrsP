/*
 * litmus/sched_run.c
 *
 * Implementation of the RUN scheduling algorithm.
 *
 */

/* 05/03/2014 - 1)Inserito protocollo risorse protette RUNRSP*/
/* 17/12/2013 - 1)Corretto comportamento esecuzione a ZL*/
/*              2)Definito errore di arrotondamento*/
/* 21/06/2013 - 1)Gestione tardy jobs on completion*/
/* 20/06/2013 - 1)Corretta terminazione anche su albero degenere*/ 
/* 19/06/2013 - 1)Aggiunto entry->resched per reschedule server*/
/* 18/06/2013 - 1)Problema overrun, aggiunto trace on job_completion e */
/* 		flag ->completed=0 in requeue come psn-edf (risolve?)*/
/* 31/05/2013 - 1)Modifica del release con barriera (release_ts)*/
/* 		Uso della coda globale prima del release del sistema*/
/* 31/05/2013 - 1)Pulizia struture dati plugin, tiny stack globale*/
/* ---------------------------------------------------------------------------*/
/* 30/05/2013 - 1)Collassamento dell'albero per rami con branch 1 */
/*		(es. S5 paper) (correzione resched_server)*/
/*		2)Aggiunta istante di release ai nodi per avere il dimensionamento*/
/*		corretto del budget, non basato su litmus_clock() (Full Utilization)*/
/*		(corrette update_deadline_on_tree, on_server_budget, ecc.)*/
/*		3)Corretta terminazione nell'"_update_r_node" (prime righe)*/
/* ---------------------------------------------------------------------------*/

#include <asm/uaccess.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/module.h>

#include <litmus/debug_trace.h>
#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>

#include <litmus/preempt.h>
#include <litmus/budget.h>
#include <litmus/np.h>
#include <litmus/bheap.h>

#include <litmus/sched_run.h>

/*Uncomment this if you want to see all scheduling decisions in the TRACE() log.*/
#define WANT_ALL_SCHED_EVENTS
#define WANT_TIMERS_SCHED_EVENTS
#define WANT_SERVERS_SCHED_EVENTS
#define WANT_ONLY_SCHED_EVENTS

/*Uncomment this if you want to force experiment termination*/
#define TERMINATION_CODE
#define TERMINATION_TIME 3500LL // termination delay in ms

#ifdef CONFIG_LITMUS_LOCKING
#define RUN_RSP
#define WANT_RUNRSP_INFO
#define WANT_RUNRSP_FTRACE
#endif

#ifdef RUN_RSP
#include <litmus/locking.h>
#include <litmus/fdso.h>
#include <linux/types.h>
/* Forward reference to the structure representing a shared resource.
 */
struct run_resource;

/* Define tracing info to be collected by feathertrace
 */
#ifdef WANT_RUNRSP_FTRACE

// Determine lock primitive before starting the busywait
#define RUN_LOCK_START 			1
#define RUN_LOCK_END 			2
// Determine when a task is waiting for the resource
#define RUN_SPINNING_START 		3
#define RUN_SPINNING_END 		4
// Determine when task t is no more running because it is offering
// the cpu to the task holding the resource
#define RUN_IS_SHADOWED_START 		5
#define RUN_IS_SHADOWED_END 		6
// Determine when task t should not be running but is executing
// in place of some task waiting for the resource t is holding
#define RUN_IS_SHADOW_RUNNING_START 	7
#define RUN_IS_SHADOW_RUNNING_END 	8
// Determine unlock primitive
#define RUN_UNLOCK_START 		9
#define RUN_UNLOCK_END 			10

#endif

#endif

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
 * resource    : pointer to the waited/held resource
 */
typedef struct run_server {

	int id;
	int cpu;
	int running;
	int released;

	struct task_struct  *hp_task;
	struct bheap        ready_queue;
	struct r_node       *rn;

	/* -- LIMA */
	int                 task_n;
	struct task_struct  *task_ref[MAX_TASK_FOR_SERVER];
	/* -- LIMA */

#ifdef RUN_RSP
	int                 rsp_status;
	struct run_resource *resource;
	struct run_server   *is_shadow_running_for;
#endif

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

DEFINE_PER_CPU(cpu_entry_t, cpu_entries);

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

run_domain_t  run;


#ifdef RUN_RSP
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
 *                    by the servers
 * queue_in_index   : supporting info to implement the fifo queue. It remembers
 *                    the free position where the insertion can be performed
 * queue_out_index  : supporting info to implement the fifo queue. It remembers
 *                    the index of the first queued element
 * next_ticket      : manual implementation for a ticket spinlock
 * actual_ticket    : manual implementation for a ticket spinlock
 */
struct run_resource {

	struct litmus_lock litmus_lock;
	int id;
	int actively_waiting;

	run_server_t *holding_server;

	run_server_t *fifo_queue[MAX_SERVER];
	int          queue_in_index;
	int          queue_out_index;

	atomic_t next_ticket;
	atomic_t actual_ticket;

};

typedef struct run_resource  run_resource_t;

static int counter_resource = 0;

/* Values to track the status of a task relatively to its requests to shared
 * resources. Since only a task per server can be performing a request at every
 * time instant, these values translate direclty as the status of the server.
 *
 * NO_RESOURCE      : no tasks inside the server requested any resource
 * WAITING_RESOURCE : one task in the server performed a request to a resource.
 *                    The resource is not free and the server (implicitly the
 *                    task) has been queued waiting for the resource to be free
 * HOLDING_RESOURCE : one task in the server is holding a resource
 */
#define NO_RESOURCE      0x00
#define WAITING_RESOURCE 0x01
#define HOLDING_RESOURCE 0x02
#endif

/* Global lock needed perform scheduling operations. A global lock is
 * necessary since multiple events can be fired from different cpus and the
 * RUN algorithm has global data structures (reduction tree).
 * This global lock is also used by the resource sharing protocol, since this
 * protocol needs to change the status of the reduction tree.
 */
raw_spinlock_t slock;

/* Support structures needed to activate the auto termination of RUN
 */
#ifdef TERMINATION_CODE
int tasks_added;
int set_exit_timer;
int exit_mode;
struct hrtimer exit_timer;
#endif

/* Support structure needed to build the reduction tree
 */
r_node_t  *curr_node;

/* Global time reference for the correct start of the plugin
 */
lt_t global_time;

/* Useful define */

#define ONE_MS  1000000LL

/* Adapting to new kernel, Ricardo 
 *
 * #define local_entry       (&__get_cpu_var(cpu_entries)) 
 */
#define local_entry       (this_cpu_ptr(&cpu_entries))

#define	remote_entry(cpu) (&per_cpu(cpu_entries, cpu))

#define get_s_id(task)  task->rt_param.task_params.run_server_id
#define get_s(task)     task->rt_param.server

#define s_from_dom(dom) \
  (container_of((dom), run_server_t, domain))

#define system_started \
  (lt_after(global_time, INIT_HORIZON) && \
   lt_after_eq(litmus_clock(), global_time))

#define run_printk(fmt, arg...) printk(KERN_INFO fmt, ##arg)

#define must_execute(server)                                    \
  ((!(server->rn->level % 2) && !(server->rn->circled)) ||      \
  ((server->rn->level % 2) && (server->rn->circled)))

// Initialize an empty server
static void init_server(
    run_server_t *server,
    int id,
    r_node_t *rn)
{
	server->id       = id;
	server->cpu      = NO_CPU;
	server->running  = 0;
	server->released = 0;
	server->hp_task  = NULL;
	bheap_init(&server->ready_queue);
	server->rn       = rn;
	server->task_n = 0;

#ifdef RUN_RSP
	server->rsp_status = NO_RESOURCE;
	server->resource   = NULL;
	server->is_shadow_running_for = NULL;
#endif
}

// Insert information on a specific node of the reduction tree
static void set_node_data(
    r_node_t *node,
    r_node_t *l_c,
    r_node_t *r_s,
    r_node_t *parent,
    int id,
    lt_t rate_a,
    lt_t rate_b,
    int level)
{
	node->l_c     = l_c;
	node->r_s     = r_s;
	node->parent  = parent;
	node->id      = id;
	node->rate[0] = rate_a;
	node->rate[1] = rate_b;
	node->level   = level;
}

// SYSCALL
// Add a node in the reduction tree. Used while building the reduction tree
asmlinkage long sys_run_add_node(
    int __user  *__id,
    lt_t __user *__rate_a,
    lt_t __user *__rate_b,
    int __user  *__level)
{
	long ret;
	int id;
	lt_t rate_a;
	lt_t rate_b;
	int level;

	r_node_t *node;
	run_server_t *server;

	ret = copy_from_user(&id, __id, sizeof(id));
	if (ret) return ret;
	ret = copy_from_user(&rate_a, __rate_a, sizeof(rate_a));
	if (ret) return ret;
	ret = copy_from_user(&rate_b, __rate_b, sizeof(rate_b));
	if (ret) return ret;
	ret = copy_from_user(&level, __level, sizeof(level));
	if (ret) return ret;

	if (!curr_node) {
		node = kmalloc(sizeof(r_node_t), GFP_ATOMIC);
		t_st_push(&run.ts, node);
		curr_node = node;
		run.rtree.root = node;
		run.rtree.level = level;
		INIT_R_NODE(node);
		set_node_data(node, NULL, NULL, NULL, id, rate_a, rate_b, level);
		node->color = 0;
	} else {
		if (curr_node && curr_node->color == 0) {
			if (id != -1) {
				node = kmalloc(sizeof(r_node_t), GFP_ATOMIC);
				t_st_push(&run.ts, node);
				INIT_R_NODE(node);
				set_node_data(node, NULL, NULL, curr_node, id, rate_a, rate_b, level);
				node->color = 0;
				curr_node->color = 1;
				curr_node->l_c = node;
				curr_node = node;
			} else {
				curr_node->color = 1;
				if (curr_node->l_c == NULL) {
				  server = kmalloc(sizeof(run_server_t), GFP_ATOMIC);
				  init_server(server, curr_node->id, curr_node);
				  #ifdef WANT_ALL_SCHED_EVENTS
				  TRACE("Server %d added (node %d)\n", server->id, curr_node->id);
				  #endif
				  BUG_ON(run.servers[server->id]);
				  run.servers[server->id] = server;
				  run.n_servers += 1;
				}
			}
			return ret;
		}
		if (curr_node && curr_node->color == 1) {
			if (id != -1) {
				node = kmalloc(sizeof(r_node_t), GFP_ATOMIC);
				t_st_push(&run.ts, node);
				INIT_R_NODE(node);
				set_node_data(node, NULL, NULL, curr_node->parent, id, rate_a, rate_b, level);
				node->color = 0;
				curr_node->color = 2;
				curr_node->r_s = node;
				curr_node = node;
			} else {
				curr_node->color = 2;
				curr_node = (r_node_t *)t_st_pop(&run.ts);
				while(curr_node && curr_node->color == 2)
					curr_node = (r_node_t *)t_st_pop(&run.ts);
			}
		}
	}
	return ret;

}

/* Assigns the ready tasks to the rightful owner. Function used when a job of a
 * task is released. Since all release events of the tasks are managed inside
 * the run_domain_t "run->domain" (which is a global EDF domain), it is
 * necessary to insert the released tasks inside the servers that will schedule
 * them. The server is the one decided during the offline phase that produce
 * the reduction tree.
 */
static void run_add_ready(
    struct task_struct *t)
{
	run_server_t *s;

	BUG_ON(bheap_node_in_heap(tsk_rt(t)->heap_node));

	s = get_s(t);
	bheap_insert(edf_ready_order, &s->ready_queue, tsk_rt(t)->heap_node);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t, "run_add_ready: released on server %d\n", s->id);
	#endif

}

/* Extracts the highest priority task from the ready queue with respect to
 * an EDF policy.
 */
static struct task_struct* run_take_ready(
    run_server_t *s)
{
	struct bheap_node* hn = bheap_take(edf_ready_order, &s->ready_queue);
	return (hn) ? bheap2task(hn) : NULL;
}

/* Determines the highest priority task inside the ready queue with respect to
 * an EDF policy.
 */
static struct task_struct* run_peek_ready(
    run_server_t *s)
{
	struct bheap_node* hn = bheap_peek(edf_ready_order, &s->ready_queue);
	return (hn) ? bheap2task(hn) : NULL;
}

/* Requeues the task.
 * If the task has a running job active (that has not completed execution), the
 * task is placed in the ready queue of the right server. The server is the
 * one decided during the offline phase that produce the reduction tree.
 * If the task has a completed job, the task is placed in the release queue
 * of the RUN edf_domain.
 *
 * IMPORTANT: if a job has completed execution, this procedure must be called
 *            after "job_completion". It is important that
 *            "prepare_for_next_period" is already performed on the task.
 */
static noinline void requeue(
    struct task_struct* task)
{

	// sanity check before insertion
	BUG_ON(!task);
	BUG_ON(is_queued(task));

	tsk_rt(task)->completed = 0;

	// if the task has an active not-completed job, requeue it in the server.
	if (is_released(task, litmus_clock())) {
		run_add_ready(task);
	}
	// if the task has a completed job, requeue it in the run edf_domain.
	// We assume that a completed job has already called (possibly indireclty)
	// the "prepare_for_next_period" procedure, this way the completed job is not
	// considered, and the next prepared job will fail the test "is_released".
	else {
		add_release(&run.domain, task);
	}

}

static noinline void init_requeue(
    struct task_struct* task)
{
	// sanity check before insertion
	BUG_ON(!task);
	BUG_ON(is_queued(task));

	tsk_rt(task)->completed = 0;

	if (is_released(task, litmus_clock()))
		__add_ready(&run.domain, task);
	else {
		// it has got to wait
		add_release(&run.domain, task);
	}
}

/* -- LIMA */
static void update_deadline_on_tree(
    r_node_t *n,
    lt_t now,
    lt_t dl)
{
	int      updated;
	r_node_t *tmp;
	lt_t     e_dl;

	tmp = n;

	#ifdef WANT_ALL_SCHED_EVENTS
	if (lt_after_eq(now, dl)) {
		TRACE("WARNING: NOW is after DL (node %d)\n", n->id);
	}
	#endif

	while (tmp) {

		updated = 0;

		e_dl = get_earliest_node_deadline(tmp, now);

		if (lt_after_eq(now, tmp->e_dl)) {

			// This update the release time of a node (previous deadline)
			if (unlikely(tmp->e_dl == INIT_HORIZON)) {
				tmp->rel = now;
			} else {
				tmp->rel = tmp->e_dl;
			}

			if (e_dl == INIT_HORIZON) {
				if (dl == INIT_HORIZON) {
					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE("WARNING: no earliest deadline\n");
					#endif
					tmp->e_dl = now;
				} else {
					tmp->e_dl = dl;
				}
			} else {
				tmp->e_dl = e_dl;
			}
			updated = 1;
		} else {

			if ((tmp->rel == now) && lt_after(tmp->e_dl, e_dl)) {
				tmp->e_dl = e_dl;
				updated = 1;
			}

			#ifdef WANT_ALL_SCHED_EVENTS
			if (tmp->rel != now)
				TRACE("WARNING: now %llu before tmp->edl %llu\n", now, tmp->rel);
			#endif
		}

		if (updated) {
			BUG_ON(tmp->rate[1] == 0);

			if (tmp->e_dl == INIT_HORIZON)
				tmp->budget = 0;
			else
				tmp->budget = (((tmp->e_dl - tmp->rel) * tmp->rate[0]) / tmp->rate[1]);

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("node: %d, updated: rel %llu e_dl %llu budget %llu\n",
					tmp->id, tmp->rel, tmp->e_dl, tmp->budget);
			#endif
		}

		tmp = tmp->parent;
	}
}

/* -- LIMA */

/* Determines whether a preemption should occur inside a server.
 */
static int preemption_needed(
    run_server_t *s)
{
	struct task_struct *a, *b;

	a = s->hp_task;
	b = run_peek_ready(s);

	return edf_higher_prio(b, a);
}


/* Updates the timers of the reduction tree. These timers are used to simulate
 * the execution of non-leaf-nodes. When a timer is fired, it means that a
 * budget exaustion event happend.
 * IMPORTANT : to be called with IRQs off
 */
void node_update_timer(
    r_node_t* n,
    lt_t now)
{
  int ret;
  lt_t when_to_fire;

  if (!n) return;

  if ((n->circled == 0) && (n->sched) && (n->sched->circled)) {
    // If we are here, the node is active

    when_to_fire = n->sched->start_time + n->sched->budget;

#ifdef WANT_TIMERS_SCHED_EVENTS
    TRACE("start %llu, budget %llu, w_t_f %llu\n", n->sched->start_time, n->sched->budget, when_to_fire);
#endif

    if ( lt_after_eq(when_to_fire, n->sched->e_dl) ||
         ((n->sched->e_dl - when_to_fire) < ROUND_ERROR) ) {
      n->sched->is_ZL = 1;

#ifdef WANT_TIMERS_SCHED_EVENTS
      TRACE("node %d timer not armed, child %d at ZL execution\n", n->id, n->sched->id);
#endif

    } else {

#ifdef WANT_TIMERS_SCHED_EVENTS
      TRACE("arming node timer, budget %llu w_t_f %llu\n", n->sched->budget, when_to_fire);
#endif

	hrtimer_start(&n->timer,
		ns_to_ktime(when_to_fire),
		HRTIMER_MODE_ABS_PINNED);

/*	Adapting to litmus-rt-2017.1 based on Linux 4.9.30 - Ricardo
 *      __hrtimer_start_range_ns(&n->timer,
 *          ns_to_ktime(when_to_fire),
 *          0, HRTIMER_MODE_ABS_PINNED, 0);
 */

      n->armed = 1;
    }

  } else if (n->armed) {
    // If the node is inactive and the timer is armed, we have to shut it down

#ifdef WANT_TIMERS_SCHED_EVENTS
    TRACE("cancelling enforcement timer\n");
#endif

      ret = hrtimer_try_to_cancel(&n->timer);

#ifdef WANT_ALL_SCHED_EVENTS
      if (ret == 0 || ret == -1)
        TRACE("WARNING ret == %d\n", ret);
#endif

    n->armed = 0;
  }
}

static void _update_r_node(
    r_node_t *n,
    lt_t now)
{
  r_node_t* tmp;
  int       active, exists, preempt, update, ZL;
  lt_t      w_t_f;

  BUG_ON(!n);

  // Check to determine if the nodes must be terminated. To be able to shut
  // down the whole scheduling algorithm, it is necessary that every task has
  // completed execution. Since a task can execute only if its server has
  // spared budget, and since the budget is replenished at the release of some
  // tasks, if these tasks are already terminated, then no budget is updated
  // and some tasks can never have any chance to execute to terminate. This
  // check prevents this situation.
  if (unlikely(lt_after_eq(now, n->e_dl) && (n->e_dl != INIT_HORIZON))) {

#ifdef WANT_ALL_SCHED_EVENTS
    TRACE("WARNING: now %llu after node %d e_dl %llu\n", now, n->id, n->e_dl);
#endif
    //140120 - Is this necessary?
    return;
  }

  if ((n->level == 0) || (!n->l_c)) return;

#ifdef WANT_ALL_SCHED_EVENTS
  TRACE("Updating node %d\n", n->id);
#endif

  active = (n->circled == 0);

  //Se branching 1 ritorna NULL sempre e aggiorni a mano
  tmp = get_earliest_node(n);

  exists = (tmp != NULL);
  preempt = (exists && (tmp != n->sched));
  update = 0;

  // If a node is active and its child needs to be preempted or if the node
  // is inactive while one of its child is active, we must stop the execution
  // of the child.
  if ((active && preempt) || (!active)) {
    if (n->sched && n->sched->circled) {
      sched_node_stop_exec(n->sched, now);
      update = 1;

#ifdef WANT_ALL_SCHED_EVENTS
      TRACE("Node %d, stopping child %d\n", n->id, n->sched->id);
#endif
    }
  }

  // If the node is active but there is a preemption among its children,
  // we must start the execution of the new child.
  if (active && preempt) {
    n->sched = tmp;
    sched_node_start_exec(n->sched, now);
    update = 1;

#ifdef WANT_ALL_SCHED_EVENTS
    TRACE("Node %d, EARLIEST child %d deadline %llu\n", n->id, n->sched->id, n->sched->e_dl);
#endif
  }

  // If the node is active and the scheduled child is not being preempted,
  // we assure that it executes.
  if (active && exists && !preempt) {
    if (!n->sched->circled) { // non stava eseguendo
      BUG_ON(n->sched->budget <= 0);
      sched_node_start_exec(n->sched, now);
      update = 1;

#ifdef WANT_ALL_SCHED_EVENTS
      TRACE("Node %d, restarting child %d\n", n->id, n->sched->id);
#endif

    } else {

      w_t_f = n->sched->start_time + n->sched->budget;
      ZL = (lt_after_eq(w_t_f, n->sched->e_dl) ||
           ((n->sched->e_dl - w_t_f) < ROUND_ERROR));

      if (!ZL && n->sched->is_ZL) {

#ifdef WANT_ALL_SCHED_EVENTS
        TRACE("!ZL but was in ZL\n");
#endif

        n->sched->is_ZL = 0;
        sched_node_start_exec(n->sched, now);
        update = 1;

      }
    }
  }

  // If the node is active but no child can be scheduled, we assure that
  // the last scheduled child (if any) stops executing.
  if (active && !exists) {

    if (n->sched) {
      BUG_ON(n->sched->budget != 0);

      n->sched->is_ZL = 0;
      uncircle(n->sched);
      n->sched = NULL;
      update = 1;

#ifdef WANT_ALL_SCHED_EVENTS
      TRACE("WARNING node %d, stopping sched\n", n->id);
#endif
    }
  }

  // If some changes happened, we update the timer of the node. The timer
  // simulates the execution or non-execution of the node's budget.
  if (update) {
    node_update_timer(n, now);
  }
}



/* This procedure elaborates the output of the RUN scheduling decision and
 * apply it. Once the reduction tree is updated, this procedure is concerned
 * to understand how to dispatch the servers to the available processors. It
 * is also its duty to free the processors of servers that should not execute
 * and to notify the processors to reschedule their work, since it can be
 * changed.
 *
 * IMPORTANT: must be called with slock
 */
static void run_resched_servers(
    lt_t now)
{

	int s_id, cpu, exists, release_cpu, circled;
	run_server_t *s;
	cpu_entry_t  *entry;

	t_st_init(&run.ts);

	for(cpu = 0; cpu < run.n_cpus; cpu ++) {
		entry = remote_entry(cpu);
		s = entry->s_sched;
		exists = (s != NULL);

		release_cpu = exists && !must_execute(s);

		// If the processor was not running anything, then consider it as a
		// processor that can be used to run something
		if (!exists) {

			t_st_push(&run.ts, entry);

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("Server NULL is releasing the cpu #%d\n", cpu);
			#endif
		}
		#ifdef RUN_RSP
		// It can be the case that the server using the cpu was shadow_running,
		// it was running on behalf of some other server. In this case we want
		// to update the status of the non-executing server, and to treat the
		// shadow_running server as if it was not executing (abiding the original
		// scheduling decision).
		else if (!s->running) {
			/**/BUG_ON (s->rsp_status != HOLDING_RESOURCE);
			/**/BUG_ON (s->is_shadow_running_for == NULL);
			/**/BUG_ON (!s->is_shadow_running_for->running);
			/**/BUG_ON (s->is_shadow_running_for->rsp_status != WAITING_RESOURCE);

			// We manage here the special case of a server that was 'running' but
			// that did not have a cpu (because it was devolved to the server
			// holding the shared resource).
			// If the shadowed server must continuing to be running, we give it its
			// right (origianl) processor. It is necessary to reschedule the cpu
			// since the server is changed.
			if (must_execute(s->is_shadow_running_for)) {
				entry->s_sched = s->is_shadow_running_for;
				///**/sched_trace_action(s->hp_task, 100);
			}
			// Otherwise we consider the cpu as free, because this shadowed server
			// is leaving its cpu. We must update the actively waiting counter.
			else {
				///**/sched_trace_action(s->hp_task, 101);
				s->resource->actively_waiting--;
				s->is_shadow_running_for->running = 0;
				entry->s_sched = NULL;
				t_st_push(&run.ts, entry);
			}
			// In every case, we must resched the cpu (or because of the switched
			// server or because it is now free), and we must update the status
			// of the shadow server, since it is no more shadowing anyone.
			entry->resched = 1;
			#ifdef WANT_RUNRSP_FTRACE
			sched_trace_action(s->hp_task, RUN_IS_SHADOW_RUNNING_END);
			sched_trace_action(s->is_shadow_running_for->hp_task, RUN_IS_SHADOWED_END);
			#endif
			s->is_shadow_running_for = NULL;
		}
		#endif
		// If the server that was being executed by the processor must not be
		// running anymore, remember the cpu as free.
		// We consider the cpu as free (that can receive new work -server- to
		// execute) even if a server is currently being executed, but the server
		// has no more work to do.
		else if (release_cpu) {
			s->running = 0;
			entry->s_sched = NULL;
			entry->resched = 1;
			t_st_push(&run.ts, entry);

			#ifdef RUN_RSP
			// We must consider the consistency of the resource: the counter
			// actively_waiting must be updated.
			if (s->rsp_status == WAITING_RESOURCE) {
				/**/BUG_ON(!s->resource);

				s->resource->actively_waiting--;
				///**/sched_trace_action(s->hp_task, 102);
			}
			#endif

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("Server  %d is releasing the cpu #%d\n", s->id, cpu);
			#endif
		}
	}

	for(s_id = 0; s_id < run.n_servers; s_id++) {

		s = run.servers[s_id];
		circled = must_execute(s);

		#ifdef WANT_ALL_SCHED_EVENTS
		if (s->released && !((s->hp_task) || (!bheap_empty(&s->ready_queue))))
		  TRACE("WARNING: Server  %d\n, release and not active. e_dl: %llu now: %llu\n",
				s->id, s->rn->e_dl, now);
		#endif

		// If a server is decided to not execute, reset possible release events
		// since they will be discarded.
		if (!circled) {
			BUG_ON(s->running);

			s->released = 0;

			#ifdef WANT_SERVERS_SCHED_EVENTS
			TRACE("Server  %d\n", s->id);
			#endif

		}
		// From here on, the server is circled (must be executing)
		else {

			// If a server has work to do, is deemed to be executed and is already
			// running, it means that a processor is being assigned to it.
			// In this case we are concerned if a job has beeing released for
			// the server, in which case remember to notify the relative processor
			// that it must reschedule its workload.
			if (s->running) {
				if (s->released) {
					s->released = 0;
					if (
					#ifdef RUN_RSP
					// We must reschedule only it the server is not holding
					// a resource: otherwise the task waiting for or holding the
					// resource is executing non preemptively.
					s->rsp_status == NO_RESOURCE &&
					#endif
					preemption_needed(s)) {
						entry = remote_entry(s->cpu);
						entry->resched = 1;
    		  		}
    	  		}

				#ifdef WANT_SERVERS_SCHED_EVENTS
				TRACE("Server (%d) cpu #%d\n", s->id, s->cpu);
				#endif

			}
			// Otherwise, if a server has work to perform, is deemed to be executed
			// by RUN but is not already running, we assign the server to a free
			// processor. The free processor is taken from the temporary stack
			// created in the previous step.
			// We also mark the chosen cpu to remember to reschedule its work.
			else {

				if (!s->running) {
        			if (t_st_empty(&run.ts)) {
						#ifdef WANT_SERVERS_SCHED_EVENTS
						TRACE("WARNING: Server (%d) no proc available\n", s->id);
						#endif
          	  	  	}
					else {
						entry = (cpu_entry_t *)t_st_pop(&run.ts);
						s->cpu = entry->cpu;
						s->running = 1;
						s->released = 0;
						entry->s_sched = s;
						entry->resched = 1;

						#ifdef RUN_RSP
						// In case the server is waiting for a resource, we must update
						// the counter actively_waiting.
						if (s->rsp_status == WAITING_RESOURCE) {
							s->resource->actively_waiting++;
							///**/sched_trace_action(s->hp_task, 103);
						}
						#endif

						#ifdef WANT_SERVERS_SCHED_EVENTS
						TRACE("Server (%d) assigned to cpu #%d\n",
							  s->id, entry->cpu);
						#endif
          	  		}
        		}
      		}
    	}
	}

	// Notify the processors marked in the previous step to reschedule their
	// work since something changed: a new server is being executed, a new
	// job of the same server must preempt the previous one, no work needs to
	// be performed.
	// We fire the system call to reschedule only the cpu that needs to
	// be rescheduled
	for(cpu = 0; cpu < run.n_cpus; cpu ++) {
		entry = remote_entry(cpu);

		#ifdef RUN_RSP
		// We must consider the shadow servers. It is a dispatching rules to
		// let execute the server holding the resource if it is not. We can not
		// make this decision before because the state of the counters
		// actively_waiting is being updated in the upper steps.
		// If a server is waiting for a resource held by a server that is
		// 1) not running
		// 2) not executing as a shadow server (perhaps because some other server
		//    already performed this check)
		// Then this server must lend its processor to the holding server.
		if (entry->s_sched &&
			entry->s_sched->rsp_status == WAITING_RESOURCE &&
			!entry->s_sched->resource->holding_server->running &&
			!entry->s_sched->resource->holding_server->is_shadow_running_for) {

			s = entry->s_sched->resource->holding_server;
			s->is_shadow_running_for = entry->s_sched;
			#ifdef WANT_RUNRSP_FTRACE
			sched_trace_action(s->is_shadow_running_for->hp_task, RUN_IS_SHADOWED_START);
			sched_trace_action(s->hp_task, RUN_IS_SHADOW_RUNNING_START);
			#endif
			s->cpu = entry->cpu;
			entry->s_sched = s;
			entry->resched = 1;
			///**/sched_trace_action(s->hp_task, 104);
    	}
    	#endif

    	if (entry->resched) {
			entry->resched = 0;
			litmus_reschedule(cpu);

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("litmus_reschedule(%d) at %llu\n", entry->cpu, litmus_clock());
			#endif
    	}
    }
}

/* -- LIMA */

static lt_t get_job_deadline(
    struct task_struct *t,
    lt_t now)
{
	if (!t) return now;

	if (is_released(t, now))
		return get_deadline(t);
	else
		return get_release(t);
}

static lt_t get_earliest_deadline(run_server_t *s, lt_t now)
{

	int   i;
	lt_t  e_dl, t_dl;

	i = 0;
	e_dl = INIT_HORIZON;

	while(i < s->task_n) {

		t_dl = get_job_deadline(s->task_ref[i], now);

		if (lt_after(t_dl, now) && (e_dl == INIT_HORIZON))
			e_dl = t_dl;

		if(lt_after(t_dl, now) && lt_after(e_dl, t_dl))
			e_dl = t_dl;

		i += 1;
	}

	return e_dl;
}

/* -- LIMA */

/* Manages the releases of jobs. The release of all jobs is managed by the
 * edf_domain inside the RUN global release queue. This procedure is concerned
 * about the assignment of the released jobs to the rightful owner (server).
 * It is also its duty to fire a reschedule event inside each processor
 * since a preemption could be needed.
 *
 * tasks : heap of released tasks. There can be one single event for the
 *         release of multiple tasks.
 */
static void run_release_jobs(
    rt_domain_t *rt,
    struct bheap *tasks)
{
	unsigned long       flags;
	struct task_struct  *t;
	run_server_t        *s;
	int   s_id;
	lt_t  now, e_dl;

	BUG_ON(!system_started);

	// Acquire global lock since we need to update the global data structures.
	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("----------------- RELEASE at %llu\n", litmus_clock());
	#endif

	t = bheap2task(bheap_peek(edf_ready_order, tasks));
	BUG_ON(!t);
	// Get the time instant that fired the event. Since all jobs inside the
	// input parameter bheap "tasks" are relative to the same event, obtaining
	// the time from the first element in the list is equivalent to obtaining it
	// from any of the others.
	now = get_release(t);

	// Assign each released job in the server its task belong to.
	// Mark all servers that received a newly released job. This way we can
	// perform selectively additional management operations.
	while(!bheap_empty(tasks)) {
		t = bheap2task(bheap_take(edf_ready_order, tasks));
		BUG_ON(!t);
		s = get_s(t);
		run_add_ready(t);
		s->released = 1;

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t,"released on server %d\n",s->id);
		#endif
	}

	// Propagates up to the root the (possibly) new earliest deadlines. Necessary
	// also to update the budget of the servers, since a release coincides to a
	// budget replenishment event (deadline == period for fixed rate tasks).
	//
	// Consider all the servers that received at least one newly released job.
	// Since a new job is added the earliest deadline could be changed. If the
	// earliest deadline changes, also the deadline of its parent must be updated.
	// If a server has not received any newly released job, then its earliest
	// deadline has already propagated in the tree, and thus no operations must
	// be performed.
	for(s_id = 0; s_id < run.n_servers; s_id++) {
		s = run.servers[s_id];

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("server %d released %d\n", s->id, s->released);
		#endif

		if (s->released) {

			e_dl = get_earliest_deadline(s, now);

			if (e_dl == INIT_HORIZON) {

				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("WARNING: No earliest_deadline found");
				#endif

				update_deadline_on_tree(s->rn, now, now);
			}
			else {
				update_deadline_on_tree(s->rn, now, e_dl);
			}
		} else {

			#ifdef WANT_ALL_SCHED_EVENTS
			if (s->rn->e_dl != INIT_HORIZON && lt_after(now, s->rn->e_dl)) {
				TRACE("WARNING server %d should complete\n");
			}
			#endif
		}
	}
	// Update the status of the reduction tree with the newly received deadlines.
	r_tree_traversal(run.rtree.root, now, &run.ts);
	// Dispatch the servers that the RUN algorithm deemed to be executing.
	run_resched_servers(now);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("----------------- END -----------------\n");
	#endif

	// Release the global lock.
	raw_spin_unlock_irqrestore(&slock, flags);

}

/* Manage job completion events.
 * Its main duty is to prepare the next job of the task.
 * IMPORTANT: This procedure does not requeue the task in the release queue.
 */
static void job_completion(
    struct task_struct *t,
    int forced)
{
  //lt_t now;

  BUG_ON(!t);

  // Trace job completion event
  sched_trace_task_completion(t, forced);

#ifdef WANT_ALL_SCHED_EVENTS
  TRACE_TASK(t, "job_completion().\n");
#endif

  // Set completion tag
  tsk_rt(t)->completed = 1;
  // Compute the next release and deadline.
  prepare_for_next_period(t);

}

/******** Timer Management *************************/

/* Manages events arising from the budget of servers.
 * Since replenishment event are handled with the release of tasks, this
 * procedure manages only budget exhaustion events.
 *
 * IMPORTANT: to be called with IRQs off
 */
static enum hrtimer_restart on_server_budget_event(
    struct hrtimer *timer)
{

	r_node_t* n = container_of(timer, r_node_t, timer);
	unsigned long flags;
	lt_t now, ts;

	ts = litmus_clock();

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_TIMERS_SCHED_EVENTS
	TRACE("-------------------- BUDGET EXHAUSTED at %llu node %d\n", litmus_clock(), n->id);
	#endif

	BUG_ON(!system_started);
	BUG_ON(!n->sched);

	//It may be affected by rounding problem
	now = n->sched->start_time + n->sched->budget;
	// 140118 - A timer may trigger simultaneously to a release event causing an
	// invalid status of the tree

	if (lt_after(now /*- ROUND_ERROR*/, ts)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("WARNING: exhaustion overlaps release on node %d, now %llu ts %llu\n",
		n->sched->id, now, ts);
		#endif

	} else {
		TS_TREE_START;

		n->sched->budget = 0;
		n->armed = 0;

		// Budget exhaustion events can modify the state of the reduction tree.
		// It is necessary to trigger the scheduling operations of RUN.
		r_tree_traversal(n, now, &run.ts);
		run_resched_servers(now);

		TS_TREE_END;
		#ifdef WANT_TIMERS_SCHED_EVENTS
		TRACE("-------------------- BUDGET END\n");
		#endif
	}

	raw_spin_unlock_irqrestore(&slock, flags);

	return HRTIMER_NORESTART;
}

/***************************************************/

/*
 * Scheduling procedure called from inside a processor. Its decision is based
 * on the leaf server of the reduction tree assigned to the processor. Thus it
 * is indireclty related on the status of the reduction tree.
 */
static struct task_struct* run_schedule(
    struct task_struct *prev)
{
	struct task_struct *just_prev, *next;
	run_server_t       *just_server;
	cpu_entry_t        *entry;
	int out_of_time, sleep, preempt, np, exists, blocks, resched;

	raw_spin_lock(&slock);

	entry = local_entry;

	// If the system has not yet started (before time 0) we manage the situation
	// accordingly (G-EDF).
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {

		BUG_ON(entry->sched && entry->sched != prev);
		BUG_ON(entry->sched && !is_realtime(prev));

		exists  = entry->sched != NULL;
		sleep   = exists && is_completed(entry->sched);

		/* Adapting to new Kernel, Ricardo
		 * blocks  = exists && !is_running(entry->sched);
		 * *** implemented like others plugins */
		blocks      = exists && !is_current_running();

		if (!exists && __next_ready(&run.domain))
			resched = 1;

		if (blocks)
			resched = 1;

		if (sleep && !blocks) {
			job_completion(entry->sched, !sleep);
			resched = 1;
		}

		next = NULL;

		if (resched || !exists) {

			if (entry->sched && !blocks)
				init_requeue(entry->sched);

			next = __take_ready(&run.domain);

		} else
			if (exists)
				next = prev;

		entry->sched = next;

	}
	// If the system is not in its initialization phase, schedule the tasks
	// as RUN commands.
	else {

		// If no server is assigned to the processor, do not execute any
		// realtime task.
		if (!entry->s_sched) {
			next = NULL;
		}
		#ifdef RUN_RSP
		// If the resource sharing protocol is active, we must make sure that
		// the currently highest priority task will not be preempted by a
		// possible release of a task of the same server.
		else if (entry->s_sched->rsp_status != NO_RESOURCE) {
			next = entry->s_sched->hp_task;
		///**/sched_trace_action(next, 105);
		}
		#endif
		// If a server is assigned to the processor, determines which task
		// to execute.
		else {
			// just_server is the server currently assigned to the processor to
			// be scheduled.
			just_server = entry->s_sched;

			// just_prev identifies the task that is running inside the server.
			// If the server changed (becase of RUN scheduling decision),
			// just_prev != prev (from procedure's signature)
			just_prev = just_server->hp_task;

			// Define flags for the status of the last executing task
			exists      = just_prev != NULL;

			/* Adapting to new Kernel, implemented like others plugins, Ricardo
			* 		blocks      = exists && !is_running(just_prev); */
			blocks      = exists && !is_current_running();

			out_of_time = 0;
			np          = exists && is_np(just_prev);
			sleep       = exists && is_completed(just_prev);
			preempt     = preemption_needed(just_server);

			#ifdef WANT_ONLY_SCHED_EVENTS
			if (exists)
				TRACE_TASK(just_prev,
				"blocks:%d out_of_time:%d np:%d sleep:%d preempt:%d "
				"state:%d sig:%d\n",
				blocks, out_of_time, np, sleep, preempt,
				just_prev->state,
				signal_pending(just_prev));
			#endif

			// Understand through the flags if there is the necessity to reschedule
			// the work of the processor (change the running task).

			// If a preemption is needed, we must reschedule the server, since a
			// higher priority task is ready to execute.
			resched = preempt;

			// If the task is blocked, it is not performing any work, let switch it
			if (blocks)
				resched = 1;

			if (np && (out_of_time || preempt || sleep))
				request_exit_np(just_prev);

			if (!np && (out_of_time || sleep) && !blocks) {
				/*This call may call reschedule if task is tardy*/
				job_completion(just_prev, !sleep);
				resched = 1;
			}

			// Determines the next job to execute in the processor.
			next = NULL;

			// If the last scheduled task (or no task at all) from the server must
			// be switched with some other task, let's take this new task.
			// We must also remember to requeue the last task that the server was
			// executing.
			if (resched || !exists) {
				if (just_prev)
					// If the server was executing a task, we have to requeue it. This
					// call place the job in the right queue: the ready queue in the
					// server if the job has not completed, the release queue of the
					// run edf_domain if the job has completed. IMPORTANT: we have already
					// performed "job_completion", so we can call this procedure safely.
					requeue(just_prev);

					next = run_take_ready(just_server);

				}
			// If the highest priority task in the server has not changed, let's
			// select it, which is the last scheduled task.
			else if (exists)
				next = just_prev;

			// We must also update the internal state of the server!
			just_server->hp_task = next;
		}
	}

	if (next) {

		#ifdef WANT_ONLY_SCHED_EVENTS
		TRACE_TASK(next, "run: scheduled at %llu\n", litmus_clock());
		#endif

		tsk_rt(next)->completed = 0;

	} else {
		//TRACE("run: becoming idle at %llu\n", litmus_clock());
	}

	// Notify the processor that the job to schedule has been determined.
	sched_state_task_picked();

	raw_spin_unlock(&slock);

	return next;
}

/* Prepare a task for running in RT mode.
 * Allow to catch the barrier release time.
 */
/* Adapting this function to new Litmus version, Ricardo */
static void run_release_at(
	lt_t start)
{
	// Ricardo, para substituir o parâmetro t da versão antiga desta function
	struct task_struct *t = current;
	unsigned long flags;

	raw_spin_lock_irqsave(&slock, flags);

	if (global_time == INIT_HORIZON) {
		/* Adapting this function to new Litmus version, Ricardo */
		global_time = start;

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("Global time initialized, system will start at %llu\n", global_time);
		#endif
	}

	/* Adapting this function to new Litmus version, Ricardo */
	release_at(t, start);

	trace_litmus_sys_release(&start);

	raw_spin_unlock_irqrestore(&slock, flags);
}

/* Allocate and initialize per-process scheduler state
 * */
static void run_task_new(
    struct task_struct *t,
    int on_rq,
    int running)
{
	unsigned long flags;
	cpu_entry_t   *entry;

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t,"task new, pid:%d\n",t->pid);
	#endif

	#ifdef TERMINATION_CODE
	tasks_added = tasks_added + 1;
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	release_at(t, litmus_clock());

	entry = local_entry;
	if (running) {
		entry->sched = t;
	} else {
		init_requeue(t);
	}

	raw_spin_unlock_irqrestore(&slock, flags);
}

/* Add process to ready queue
 */
static void run_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	lt_t now;

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	BUG_ON(is_queued(task));

	now = litmus_clock();

	if (is_tardy(task, now)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(task, "is_tardy, dl %llu, now %llu\n", get_deadline(task), now );
		#endif

		release_at(task, now);
		sched_trace_task_release(task);
	}

	/* Only add to ready queue if it is not the currently-scheduled
	* task. This could be the case if a task was woken up concurrently
	* on a remote CPU before the executing CPU got around to actually
	* de-scheduling the task, i.e., wake_up() raced with schedule()
	* and won.
	*/
	init_requeue(task);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(task, "will release at %llu\n", get_release(task));
	#endif

	raw_spin_unlock_irqrestore(&slock, flags);
}

static void run_task_exit(
    struct task_struct *t)
{
	run_server_t  *s;
	unsigned long flags;
	lt_t  wtf;
	cpu_entry_t *entry;

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef TERMINATION_CODE
	if (exit_mode) { //Exit mode

		if (is_queued(t)) {
			remove(&run.domain, t);
		}
		entry = local_entry;
		if (entry->sched == t)
			entry->sched = NULL;

	} else { //RUN mode
		s = get_s(t);

		if (is_queued(t)) {
			bheap_delete(edf_ready_order,
			   &s->ready_queue,
			   tsk_rt(t)->heap_node);
		}

		if (s->hp_task == t) {
			s->hp_task = NULL;
			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE_TASK(t, "EXIT: task scheduled\n");
			#endif

			//Timer forces the experiment completion
			if (set_exit_timer) {
				wtf = litmus_clock() + (TERMINATION_TIME * ONE_MS);

				hrtimer_start(&exit_timer,
					ns_to_ktime(wtf),
					HRTIMER_MODE_ABS_PINNED);

				/*	Adapting to litmus-rt-2017.1 based on Linux 4.9.30 - Ricardo
				*        __hrtimer_start_range_ns(&exit_timer, ns_to_ktime(wtf), 0, HRTIMER_MODE_ABS_PINNED, 0);
				*/
				set_exit_timer = 0;
			}

			tasks_added = tasks_added - 1;
			if (tasks_added <= 0) {
				hrtimer_try_to_cancel(&exit_timer);
			}

			wtf = litmus_clock();
			trace_litmus_sys_release(&wtf); // Ricardo, havia um warning neste código

			// If the server of the exiting task is running, we must reschedule since
			// the exiting task has completed its job.
			if (s->running) {
				run_resched_servers(wtf);
			}
		}
	}

	#else

	s = get_s(t);

	if (is_queued(t)) {
		bheap_delete(edf_ready_order,
		   &s->ready_queue,
		   tsk_rt(t)->heap_node);
	}

	if (s->hp_task == t) {
		s->hp_task = NULL;
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t, "EXIT: task scheduled\n");
		#endif

		wtf = litmus_clock();
		trace_litmus_sys_release(&wtf); // Ricardo, havia um warning neste código

		// If the server of the exiting task is running, we must reschedule since
		// the exiting task has completed its job.
		if (s->running) {
			run_resched_servers(wtf);
		}
	}

	#endif

	raw_spin_unlock_irqrestore(&slock, flags);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t, "RIP\n");
	#endif

}

static long run_admit_task(
    struct task_struct *t)
{
	int s_id;
	run_server_t *s;

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t,"sched_run.run_admit_task: run_admit_task();\n");
	#endif

	// No task can be admitted if the system is already running.
	if (system_started) {
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t,"sched_run.run_admit_task: rejected system_started\n");
		#endif

		return -EINVAL;
	}

	s_id = get_s_id(t);
	// Check whether the server that should accomodate the task is valid.
	if ((s_id < 0) || (s_id >= run.n_servers)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t,"sched_run.run_admit_task: rejected, server unknown\n");
		#endif

		return -EINVAL;
	}

	// Error if the server is not initialized.
	if (run.servers[s_id] == NULL) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t,"sched_run.run_admit_task: rejected, server NULL\n");
		#endif

		return -EINVAL;
	}

	s = run.servers[s_id];
	t->rt_param.server = s;

	/* -- LIMA */
	s->task_ref[s->task_n] = t;
	s->task_n += 1;
	/* -- LIMA */

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t,"sched_run.run_admit_task: admitted from server %d\n", s_id);
	#endif

	return 0;
}

/* Deallocate RUN data structures.
 */
static void cleanup_run(void)
{
	int s_id;

	if (!r_tree_empty(&run.rtree)) {
		t_st_init(&run.ts);
		r_tree_dealloc(run.rtree.root, &run.ts);
		INIT_R_TREE(&run.rtree);
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("RUN: Reduction tree correctly deallocate!\n");
		#endif
	}

	for(s_id = 0; s_id < MAX_SERVER; s_id++) {
		if (run.servers[s_id]) {
			kfree(run.servers[s_id]);
			run.servers[s_id] = NULL;
			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("RUN: Server %d correctly deallocated\n", s_id);
			#endif
		}
	}

}

#ifdef TERMINATION_CODE
static enum hrtimer_restart on_exit_experiments(struct hrtimer *timer) {
  int s_id, cpu;
  run_server_t *s;
  cpu_entry_t *entry;
  unsigned long flags;
  struct task_struct *t;

  raw_spin_lock_irqsave(&slock, flags);

  if (tasks_added > 0) {
    //sposta task in coda ready
    for(s_id = 0; s_id < run.n_servers; s_id++) {
      s = run.servers[s_id];
      if (s) {
        if (s->hp_task) {
          add_ready(&run.domain, s->hp_task);
          s->hp_task = NULL;
        }
        while(!bheap_empty(&s->ready_queue)) {
          t = bheap2task(bheap_take(edf_ready_order, &s->ready_queue));
          add_ready(&run.domain, t);
        }
      }
    }
    tasks_added = 0;
    exit_mode = 1;

    for(cpu = 0; cpu < run.n_cpus; cpu ++) {
      entry = remote_entry(cpu);
      if (entry)
        entry->sched = NULL;
    }
    litmus_reschedule_local();
  }

  raw_spin_unlock_irqrestore(&slock, flags);

  return HRTIMER_NORESTART;
}
#endif

static long run_deactivate_plugin(void)
{
	cleanup_run();
	return 0;
}

static long run_activate_plugin(void)
{
	int cpu;
	cpu_entry_t *entry;

	raw_spin_lock_init(&slock);

	cleanup_run();

	run.n_cpus = num_online_cpus();
	run.n_servers = 0; // it will be initialized during the tree construction

	#ifdef TERMINATION_CODE
	tasks_added = 0;
	set_exit_timer = 1;
	exit_mode = 0;
	hrtimer_init(&exit_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	exit_timer.function = on_exit_experiments;
	#endif

	t_st_init(&run.ts);

	for(cpu = 0; cpu < run.n_cpus; cpu++) {
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("run: initializing CPU #%d.\n", cpu);
		#endif
		entry = remote_entry(cpu);
		entry->cpu = cpu;
		entry->s_sched = NULL;
		entry->sched = NULL;
		entry->resched = 0;
	}

	global_time = INIT_HORIZON;
	edf_domain_init(&run.domain, NULL, run_release_jobs);

	INIT_R_TREE(&run.rtree);
	curr_node = NULL;
	t_st_init(&run.ts);

	return 0;
}

#ifdef RUN_RSP
/************* LOCKING PROCEDURES START ******************/

static inline struct run_resource *run_sem_from_lock (
    struct litmus_lock *l)
{
	return container_of(l, struct run_resource, litmus_lock);
}

static inline void perform_busy_wait (
    int      waiting_ticket,
    atomic_t *actual_ticket)
{

	int read_ticket = atomic_read(actual_ticket);
	while (waiting_ticket != read_ticket
		#ifdef TERMINATION_CODE
			&& !exit_mode
		#endif
	) {
		cpu_relax();
		read_ticket = atomic_read(actual_ticket);
	}

	barrier();
}

int lock_run_semaphore(
    struct litmus_lock* l)
{

	struct task_struct  *t = current;
	struct run_resource *r = run_sem_from_lock(l);
	struct run_server   *s = get_s(t);
	cpu_entry_t         *entry;
	int           waiting_ticket;
	unsigned long flags;

	// If the plugin is not yet started or is closing, ignore every
	// lock/unlock request. WORKAROUND usable only for dummy tasks!
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {
		return 0;
	}

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_RUNRSP_FTRACE
	// Tracing for the lock primitive overhead (up to the spinlocking)
	sched_trace_action(t, RUN_LOCK_START);
	#endif

	// No other task in the same server must have requested a shared resource.
	BUG_ON (s->rsp_status != NO_RESOURCE);

	s->resource = r;

	// If the resource is free and no other server is queued, the current
	// request can be immediately satisfied.
	if (r->holding_server == NULL) {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][L] Lock direct: server(%d)\n",
				   r->id, litmus_clock(), s->id);
		#endif

		// Sanity check: no other server must be waiting.
		BUG_ON(r->queue_in_index != r->queue_out_index);

		r->holding_server = s;
		s->rsp_status     = HOLDING_RESOURCE;

		// Now we have the right to use the resource, so we can return.
		// We must anyhow take the ticket, otherwise the other servers will not
		// perform busy wait.
		atomic_inc(&r->next_ticket);

		#ifdef WANT_RUNRSP_FTRACE
		sched_trace_action(t, RUN_LOCK_END);
		#endif

		raw_spin_unlock_irqrestore(&slock, flags);

		return 0;

	}
	// If the resource is occupied we must:
	// 1) update the status of the resource and the server
	// 2) queue the server on the resource
	// 3) take the ticket for the busywait
	// 4) perform busywait
	else {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][L] Spin: server(%d)"
				   " [queued %d, active %d, holding(%d), waitTicket(%d), actualTicket(%d)]\n",
				   r->id,
				   litmus_clock(),
				   s->id,
				   r->queue_in_index>=r->queue_out_index ?
					 r->queue_in_index-r->queue_out_index:
					 r->queue_in_index+MAX_SERVER-r->queue_out_index,
				   r->actively_waiting,
				   r->holding_server->id,
				   atomic_read(&r->next_ticket),
				   atomic_read(&r->actual_ticket)
		);
		#endif

		s->rsp_status = WAITING_RESOURCE;

		r->fifo_queue[r->queue_in_index] = s;
		r->queue_in_index = (r->queue_in_index + 1) % MAX_SERVER;

		// We must take the ticket coherently with the position of the
		// server in the fifo_queue. Then we update the value for the
		// next ticket.
		waiting_ticket = atomic_read(&r->next_ticket);
		atomic_inc(&r->next_ticket);

		// We must give the cpu to the holding task if it is not already executing.
		// It can be the case that the current task belongs to a server that has
		// been suspended: if the task tried to acquire the lock while a scheduling
		// decision was performed, the lock request is being queued in the IRQ and
		// executed independently of the status of the server. So we force to
		// execute the holding server only if we are really running.
		if (s->running) {

			r->actively_waiting++;

			if (!r->holding_server->running &&
				!r->holding_server->is_shadow_running_for) {

				entry = remote_entry(s->cpu);
				entry->s_sched = r->holding_server;
				r->holding_server->is_shadow_running_for = s;

				#ifdef WANT_RUNRSP_FTRACE
				sched_trace_action(r->holding_server->hp_task, RUN_IS_SHADOW_RUNNING_START);
				sched_trace_action(s->hp_task, RUN_IS_SHADOWED_START);
				#endif

				r->holding_server->cpu = s->cpu;

				#ifdef WANT_RUNRSP_INFO
				TRACE_TASK(t, "RESOURCE(%d)[%llu][L] shadowServer(%d)->hiddenServer(%d)\n",
						   r->id, litmus_clock(), r->holding_server->id, s->id);
				#endif

				litmus_reschedule(s->cpu);

			}

		} else {

			#ifdef WANT_RUNRSP_INFO
			TRACE_TASK(t, "RESOUCE(%d)[%llu][L] server(%d) try lock while not running\n",
					   r->id, litmus_clock(), s->id);
			#endif

		}
	}

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_LOCK_END);
	#endif

	/***/TS_LOCK_END
	raw_spin_unlock_irqrestore(&slock, flags);

	// Now we must start the busy wait.
	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_SPINNING_START);
	#endif

	perform_busy_wait(waiting_ticket, &r->actual_ticket);

	/***/TS_LOCK_START
	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_SPINNING_END);
	#endif

	// From here the server is HOLDING_RESOURCE. Some other server must have
	// incremented the actual_ticket when unlocking the resource. That same
	// server must have already changed the state of this server, as if it
	// acquired the resource manually.
	BUG_ON (s->rsp_status != HOLDING_RESOURCE
		#ifdef TERMINATION_CODE
		&& !exit_mode
		#endif
		  );

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][L] Got resource server(%d)\n",
			 r->id, litmus_clock(), s->id);
	#endif

	return 0;
}

static int unlock_run_semaphore(
    struct litmus_lock* l)
{

	struct task_struct  *t = current;
	struct run_resource *r = run_sem_from_lock(l);
	struct run_server   *s = get_s(t);
	unsigned long flags;
	int i;
	int cpu_to_resched_1 = -1, cpu_to_resched_2 = -1;

	// If the plugin is not yet started or is closing, ignore every
	// lock/unlock request. WORKAROUND usable only for dummy tasks!
	// Dummy unlock only if task is not really holding resource,
	// otherwise it could starve tasks already queued!
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {
		return 0;
	}

	// We make sure that the current task is holding the resource.
	BUG_ON (s->rsp_status != HOLDING_RESOURCE);
	BUG_ON (r->holding_server != s);

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_UNLOCK_START);
	#endif

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Unlock: server(%d)\n",
			 r->id, litmus_clock(), s->id);
	#endif

	// Free the resource and reset the state of the server.
	r->holding_server = NULL;
	s->rsp_status = NO_RESOURCE;
	s->resource   = NULL;

	// If the holding server is running, then it already has a dedicated cpu.
	// To restore the 'original RUN scheduling decision' we just have to
	// reschedule the cpu since a release event could have been discarded.
	if (s->running) {
		BUG_ON(s->is_shadow_running_for);
		BUG_ON(remote_entry(s->cpu)->s_sched != s);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] server(%d) continue executing in cpu(%d)\n",
				   r->id, litmus_clock(), s->id, s->cpu);
		#endif

		cpu_to_resched_1 = s->cpu;
	}

	// If the holding server is not running but it is shadow running for some
	// other server, then we must restore the shadowed server inside the cpu
	// and reschedule it since a context switch must happen.
	else if (s->is_shadow_running_for) {
		BUG_ON(remote_entry(s->cpu)->s_sched != s);
		BUG_ON(remote_entry(s->is_shadow_running_for->cpu)->s_sched
			   == s->is_shadow_running_for);
		BUG_ON(s->is_shadow_running_for->cpu != s->cpu);
		BUG_ON(!s->is_shadow_running_for->running);
		BUG_ON(!r->actively_waiting);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Restore shadowed server(%d)\n",
				   r->id, litmus_clock(), s->is_shadow_running_for->id);
		#endif

		#ifdef WANT_RUNRSP_FTRACE
		sched_trace_action(s->hp_task, RUN_IS_SHADOW_RUNNING_END);
		sched_trace_action(s->is_shadow_running_for->hp_task, RUN_IS_SHADOWED_END);
		#endif

		remote_entry(s->cpu)->s_sched = s->is_shadow_running_for;

		// no need to set the cpu to the shadowed server, since it must be the
		// server that offered the cpu to the holding server.
		s->is_shadow_running_for = NULL;
		cpu_to_resched_1 = s->cpu;

	}
	// It can be the case that the holding server is not running nor shadow
	// running for someone else. It happens when the unlock request is performed
	// when a scheduling decision is being made, and the scheduler changed the
	// status of the tree. But since the IRQ has already been made, then it must
	// be evaded. In this case we must do nothing.
	else {
		BUG_ON(remote_entry(s->cpu)->s_sched == s);
		BUG_ON(s->is_shadow_running_for);
		BUG_ON(r->actively_waiting);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] !running & !shadow_running\n",
				   r->id, litmus_clock());
		#endif

	}

	s = NULL;
	// We always must promote the first fifo queued server as new holding server
	// (if any), since that task will be busy waiting.
	if (r->queue_in_index != r->queue_out_index) {
		// Get the next server with respect to fifo order.
		s = r->fifo_queue[r->queue_out_index];

		BUG_ON(s->resource != r);
		BUG_ON(s->rsp_status != WAITING_RESOURCE);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Pass lock to server(%d)\n",
				   r->id, litmus_clock(), s->id);
		#endif

		s->rsp_status = HOLDING_RESOURCE;
		// Update the status of the resource
		r->holding_server = s;
		r->queue_out_index = (r->queue_out_index + 1) % MAX_SERVER;
		// We need to update the counter actively_waiting knowing that the server
		// just removed from the queue could be one of them.
		if (s->running) {
			r->actively_waiting--;
		}

	}

	// We apply the dispatching rule that let execute a possible shadow server.
	// This can happen only if the new holding server (if any) is not running
	// while some other server is actively waiting.
	if (s && !s->running && r->actively_waiting) {
		BUG_ON(s->is_shadow_running_for);
		BUG_ON(s != r->holding_server);

		i = r->queue_out_index;
		while (i != r->queue_in_index) {
			if (r->fifo_queue[i]->running) {

				// We have found a server that is fifo queued and is now
				// wasting cpu by busy waiting. We shadow that server.
				s->is_shadow_running_for = r->fifo_queue[i];

				#ifdef WANT_RUNRSP_FTRACE
				sched_trace_action(s->is_shadow_running_for->hp_task, RUN_IS_SHADOWED_START);
				sched_trace_action(s->hp_task, RUN_IS_SHADOW_RUNNING_START);
				#endif

				// We borrow its cpu for the new holding server and inform the cpu
				// that it must reschedule for the new shadow server.
				s->cpu = r->fifo_queue[i]->cpu;
				remote_entry(s->cpu)->s_sched = s;
				cpu_to_resched_2 = s->cpu;
				break;
			}
			i = (i + 1) % MAX_SERVER;
		}

		BUG_ON(!s->is_shadow_running_for);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] shadowServer(%d)->hiddenServer(%d)\n",
				   r->id, litmus_clock(), s->id, s->is_shadow_running_for->id);
		#endif
	}

	atomic_inc(&r->actual_ticket);

	if (cpu_to_resched_1 != -1) {
		litmus_reschedule(cpu_to_resched_1);
	}
	if (cpu_to_resched_2 != -1 && cpu_to_resched_1 != cpu_to_resched_2) {
		litmus_reschedule(cpu_to_resched_2);
	}

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_UNLOCK_END);
	#endif

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Ticket status: next(%d) actual(%d)\n",
			 r->id, litmus_clock(), atomic_read(&r->next_ticket), atomic_read(&r->actual_ticket));
	#endif

	raw_spin_unlock_irqrestore(&slock, flags);

	return 0;

}

void free_run_semaphore(
    struct litmus_lock* lock)
{
	kfree(run_sem_from_lock(lock));
}

static struct litmus_lock_ops run_rsp_lock_ops = {
	.lock   = lock_run_semaphore,
	.unlock = unlock_run_semaphore,
	.deallocate = free_run_semaphore,
};

static struct litmus_lock *new_run_resource (void)
{
	struct run_resource *sem;

	sem = kmalloc(sizeof(*sem), GFP_KERNEL);
	if (!sem)
		return NULL;

	sem->id = counter_resource++;
	sem->actively_waiting = 0;
	sem->holding_server   = NULL;
	sem->queue_in_index   = 0;
	sem->queue_out_index  = 0;
	sem->litmus_lock.ops = &run_rsp_lock_ops;
	atomic_set(&sem->next_ticket, 0);
	atomic_set(&sem->actual_ticket, 0);

	return &sem->litmus_lock;
}

static long run_allocate_lock(
    struct litmus_lock **lock,
    int type,
    void* __user unused)
{
	long err = -ENXIO;
	if (type == RUN_SEM) {
		*lock = new_run_resource();
		if (*lock)
			err = 0;
		else
			err = -ENOMEM;
	}
	return err;
}

/***************** LOCKING PROCEDURES END *****************/
#endif


/*	Plugin object	*/
static struct sched_plugin run_plugin __cacheline_aligned_in_smp = {
	.plugin_name       = "RUN",
	.task_new          = run_task_new,
	.complete_job      = complete_job,
	.synchronous_release_at = run_release_at,
	.task_exit         = run_task_exit,
	.schedule          = run_schedule,
	.task_wake_up      = run_task_wake_up,
	.admit_task        = run_admit_task,
	.activate_plugin   = run_activate_plugin,
	.deactivate_plugin = run_deactivate_plugin,
#ifdef RUN_RSP
	.allocate_lock     = run_allocate_lock,
#endif
};

static int __init init_run(void)
{
	return register_sched_plugin(&run_plugin);
}

static void clean_run(void)
{
	cleanup_run();
}

module_init(init_run);
module_exit(clean_run);
