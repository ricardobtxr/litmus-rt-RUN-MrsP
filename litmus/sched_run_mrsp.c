/*
 * Implementation of the RUN scheduling algorithm.
 */

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

#include <litmus/sched_run_mrsp.h>

/* For resolving Eclipse parsing errors, it does not affect the compiled code
#ifdef __CDT_PARSER__
#define __user
#define noinline
#define __init
#define __cacheline_aligned_in_smp
#endif // #ifdef __CDT_PARSER__
*/

/*Uncomment this if you want to see all scheduling decisions in the TRACE() log.*/
#define WANT_ALL_SCHED_EVENTS
#define WANT_TIMERS_SCHED_EVENTS
#define WANT_SERVERS_SCHED_EVENTS
#define WANT_ONLY_SCHED_EVENTS
#define WANT_DEBUG_LITMUS_EVENTS


/*Uncomment this if you want to force experiment termination*/
#define TERMINATION_CODE
#define TERMINATION_TIME 1000LL // termination delay in ms

#ifdef CONFIG_LITMUS_LOCKING
#define RUN_MrsP
#define WANT_RUNRSP_INFO
#define WANT_RUNRSP_FTRACE
// #define HELPING_MECHANISM_DISABLED
#endif

#ifdef RUN_MrsP
#include <litmus/locking.h>
#include <litmus/fdso.h>
#include <linux/types.h>

/* Forward reference to the structure representing a shared resource.
 */
struct run_resource;

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

/* Define tracing info to be collected by feathertrace */
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

DEFINE_PER_CPU(cpu_entry_t, cpu_entries);

run_domain_t  run;

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

/* Support structure needed to build the reduction tree */
r_node_t  *curr_node;

/* Global time reference for the correct start of the plugin */
lt_t global_time;

/* Useful define */
#define ONE_MS  1000000LL
#define get_s_id(task)  task->rt_param.task_params.run_server_id
#define get_s(task)     task->rt_param.server
#define local_entry       (this_cpu_ptr(&cpu_entries))
#define	remote_entry(cpu) (&per_cpu(cpu_entries, cpu))

static inline lt_t lt_t_min(lt_t a, lt_t b) {
	return (lt_after(a,b) ? b : a);
}

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
	server->mrsp_ceiling = 0;
	t_st_i_init(&server->mrsp_ceiling_stack);
	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("init_server(server: %d, node: %d)\n", id, rn->id);
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
	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("set_node_data(id: %d, rate_a: %llu, rate_b: %llu, level: %d, parent: %d)\n", id, rate_a, rate_b, level, parent ? parent->id : -1);
	#endif
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

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("sys_run_add_node(%d, %llu, %llu, %d)\n", id, rate_a, rate_b, level);
	#endif

	if (!curr_node) {
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("sys_run_add_node(%d, %d)(!curr_node)\n", id, level);
		#endif
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
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("sys_run_add_node(%d, %d)(curr_node->color == %d)(id != -1)\n", id, level, curr_node->color);
				#endif
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
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("sys_run_add_node(%d, %d)(curr_node->color == %d)(id != -1)\n", id, level, curr_node->color);
				#endif
				node = kmalloc(sizeof(r_node_t), GFP_ATOMIC);
				t_st_push(&run.ts, node);
				INIT_R_NODE(node);
				set_node_data(node, NULL, NULL, curr_node->parent, id, rate_a, rate_b, level);
				node->color = 0;
				curr_node->color = 2;
				curr_node->r_s = node;
				curr_node = node;
			} else {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("sys_run_add_node(%d, %d)(curr_node->color == %d)(id == %d)\n", id, level, curr_node->color, id);
				#endif
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

	#ifdef WANT_DEBUG_LITMUS_EVENTS
	TRACE_TASK(t, "run_add_ready() at %llu\n", litmus_clock());
	#endif

	BUG_ON(bheap_node_in_heap(tsk_rt(t)->heap_node));

	s = get_s(t);
	bheap_insert(edf_ready_order, &s->ready_queue, tsk_rt(t)->heap_node);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t, "run_add_ready: released on server %d at %llu\n", s->id, litmus_clock());
	#endif
}

/* Extracts the highest priority task from the ready queue with respect to
 * an EDF policy.
 */
static struct task_struct* run_take_ready(
    run_server_t *s)
{
	struct bheap_node* hn = bheap_take(edf_ready_order, &s->ready_queue);
	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(current, "run_take_ready(server %d) = %d\n", s->id, (hn) ? bheap2task(hn)->pid : -1);
	#endif
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

	// threat situation when helping is reseted and a reschedule occurs after that
	if (is_queued(task)) {
		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(task, "WRNG the task is already queued. rel=%llu, now=%llu\n", get_release(task), litmus_clock());
		#endif
		tsk_rt(task)->completed = 0;
		return;
	}

	tsk_rt(task)->completed = 0;

	// if the task has an active not-completed job, requeue it in the server.
	if (is_released(task, litmus_clock())) {

		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(task, "Released rel=%llu, ddl=%llu, now=%llu, srv=%d\n", get_release(task), get_deadline(task), litmus_clock(), get_s(task)->id);
		#endif

		run_add_ready(task);

	}
	// if the task has a completed job, requeue it in the run edf_domain.
	// We assume that a completed job has already called (possibly indireclty)
	// the "prepare_for_next_period" procedure, this way the completed job is not
	// considered, and the next prepared job will fail the test "is_released".
	else {

		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(task, "NOT released rel=%llu, ddl=%llu, now=%llu, srv=%d\n", get_release(task), get_deadline(task), litmus_clock(), get_s(task)->id);
		#endif

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

	if (is_released(task, litmus_clock())) {

		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(task, "init_requeue.__add_ready()\n");
		#endif

		__add_ready(&run.domain, task);
	} else {

		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(task, "init_requeue.add_release()\n");
		#endif

		// it has got to wait
		add_release(&run.domain, task);
	}
}

static inline lt_t node_budget_remaining(r_node_t *n) {
	return n->budget;
}

static inline lt_t update_node_budget(r_node_t* tmp) {

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("node(%d) e_dl:%llu, rel:%llu, r0:%llu, r1:%llu\n", tmp->id, tmp->e_dl, tmp->rel, tmp->rate[0], tmp->rate[1]);
	#endif

	return (((tmp->e_dl - tmp->rel) * tmp->rate[0]) / tmp->rate[1]);
}

/* -- LIMA */
static void upd_dl_on_tree(
    r_node_t *n,
    lt_t now,
    lt_t dl)
{
	int      updated;
	r_node_t *tmp;
	lt_t     e_dl;
	lt_t	 old_bdgt;

	tmp = n;

	#ifdef WANT_ALL_SCHED_EVENTS
	if (lt_after_eq(now, dl)) {
		TRACE("WRNG NOW:%llu after DL:%llu node(%d)\n", now, dl, n->id);
	} else {
		TRACE("NOW:%llu before DL:%llu node(%d)\n", now, dl, n->id);
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
					TRACE("WRNG no earliest deadline of node %d set to now: %llu\n", tmp->id, now);
					#endif
					tmp->e_dl = now;
				} else {
					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE("Earliest deadline of node %d: %llu\n", tmp->id, dl);
					#endif
					tmp->e_dl = dl;
				}
			} else {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("Earliest deadline of node %d: %llu\n", tmp->id, e_dl);
				#endif
				tmp->e_dl = e_dl;
			}
			updated = 1;
		} else {

			if ((tmp->rel == now) && lt_after(tmp->e_dl, e_dl)) {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("Earliest deadline of node %d:%llu now:%llu rel:%llu\n", tmp->id, e_dl, now, tmp->rel);
				#endif
				tmp->e_dl = e_dl;
				updated = 1;
			}

			#ifdef WANT_ALL_SCHED_EVENTS
			if (tmp->rel != now)
				TRACE("WRNG now %llu before tmp->edl %llu\n", now, tmp->rel);
			#endif
		}

		if (updated) {
			BUG_ON(tmp->rate[1] == 0);

			old_bdgt = tmp->budget;
			if (tmp->e_dl == INIT_HORIZON)
				tmp->budget = 0;
			else
				tmp->budget = update_node_budget(tmp);

			if (tmp->budget > (tmp->e_dl - now)) {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("WRNG node(%d) bgt:%llu > dl:%llu - now:%llu [%llu]\n", tmp->id, tmp->budget, tmp->e_dl, now, litmus_clock());
				#endif
			}

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("n:%d upd:%llu rel:%llu dl:%llu bgt:%llu o_bgt:%llu\n",
					tmp->id, now, tmp->rel, tmp->e_dl, tmp->budget, old_bdgt);
			#endif
		}

		tmp = tmp->parent;
	}
}

/* Determines whether a preemption should occur inside a server.
 * If server's hp_task is helping a resource holder, then we must
 * compare the inherited preemption level of the resource holder
 * with the preemption level of the task in the top of the ready queue.
 */
static inline int preemption_needed(
    run_server_t *s)
{
	struct task_struct *a, *b;

	a = s->hp_task;
	b = run_peek_ready(s);

	#ifdef WANT_ALL_SCHED_EVENTS
	if (higher_preemption_level(b) && edf_higher_prio(b, a))
		TRACE_TASK(b, "preemption needed for %d\n", a?a->pid:-1);
	else
		TRACE("preemption NOT needed: tasks %d and %d, server %d, CPU#%d, HPL(b)=%d, EHP(b, a)=%d \n",
				b?b->pid:-1, a?a->pid:-1, s->id, s->cpu, higher_preemption_level(b), edf_higher_prio(b, a));
	#endif

	return (higher_preemption_level(b) && edf_higher_prio(b, a));
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
		TRACE("start(%d) %llu, budget %llu, w_t_f %llu, now:%llu\n",
				n->sched->id, n->sched->start_time, n->sched->budget, when_to_fire, litmus_clock());
		#endif

		if (lt_after_eq(when_to_fire, n->sched->e_dl) ||
				((n->sched->e_dl - when_to_fire) < ROUND_ERROR)) {

			n->sched->is_ZL = 1;

			#ifdef WANT_TIMERS_SCHED_EVENTS
			TRACE("WRNG n:%d timer NOT armed, child %d at ZL execution, now:%llu\n", n->id, n->sched->id, litmus_clock());
			#endif

		} else {

			#ifdef WANT_TIMERS_SCHED_EVENTS
			TRACE("arming n(%d) timer budget:%llu w_t_f:%llu, now:%llu\n", n->sched->id, n->sched->budget, when_to_fire, litmus_clock());
			#endif

			hrtimer_start(&n->timer,
			ns_to_ktime(when_to_fire),
			HRTIMER_MODE_ABS_PINNED);

			n->armed = 1;
		}

	} else if (n->armed) {
		// If the node is inactive and the timer is armed, we have to shut it down

		#ifdef WANT_TIMERS_SCHED_EVENTS
		TRACE("cancelling enforcement timer now:%llu\n", litmus_clock());
		#endif

		ret = hrtimer_try_to_cancel(&n->timer);

		#ifdef WANT_ALL_SCHED_EVENTS
		if (ret == 0 || ret == -1)
		TRACE("WRNG ret == %d\n", ret);
		#endif

		n->armed = 0;
	}
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

static inline void sched_node_upd_bdgt(r_node_t *n, lt_t now) {

	lt_t interval;

	#ifdef WANT_OTHER_SCHED_EVENTS
	TRACE("Node(%d) upd bgt at %llu, bgt:%llu, start:%llu, now:%llu\n", n->id, now, n->budget, n->start_time, litmus_clock());
	#endif

	if (n->budget == 0) return;

	interval = now - n->start_time;

	#ifdef WANT_OTHER_SCHED_EVENTS
	TRACE("Node(%d) upd bgt at %llu bgt:%llu intvl:%llu, now:%llu\n", n->id, now, n->budget, interval, litmus_clock());
	#endif

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
		sched_node_upd_bdgt(n, now);
	else
		n->is_ZL = 0;
	uncircle(n);
}

static inline r_node_t *get_earliest_node(r_node_t *n)
{
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

static inline lt_t get_earliest_deadline(run_server_t *s, lt_t now)
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

static inline lt_t get_job_deadline(
    struct task_struct *t,
    lt_t now)
{
	if (!t) return now;

	if (is_released(t, now))
		return get_deadline(t);
	else
		return get_release(t);
}

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
			kfree(curr);
			t_st_pop(ts);
		}
		prev = curr;
	}
}

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
		if (n->sched)
			TRACE("WRNG now:%llu after n(%d)->e_dl:%llu strt:%llu bdgt:%llu\n", now, n->id, n->e_dl, n->sched->start_time, n->sched->budget);
		else
			TRACE("WRNG now:%llu after n(%d)->e_dl:%llu n->sched=NULL\n", now, n->id, n->e_dl);
		#endif
		//140120 - Is this necessary?
		return;
	}

	if ((n->level == 0) || (!n->l_c)) return;

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("Updating node(%d) at %llu, n->e_dl:%llu now:%llu\n", n->id, now, n->e_dl, litmus_clock());
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
			TRACE("Node %d, stopping child %d, active:%d, prmpt:%d, now:%llu\n", n->id, n->sched->id, active, preempt, litmus_clock());
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
		TRACE("Node %d, EARLIEST child %d deadline %llu, now:%llu\n", n->id, n->sched->id, n->sched->e_dl, litmus_clock());
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
			TRACE("Node %d, restarting child %d, now:%llu\n", n->id, n->sched->id, litmus_clock());
			#endif

		} else {

			w_t_f = n->sched->start_time + n->sched->budget;
			ZL = (lt_after_eq(w_t_f, n->sched->e_dl) ||
			   ((n->sched->e_dl - w_t_f) < ROUND_ERROR));

			if (!ZL && n->sched->is_ZL) {

				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("!ZL but was in ZL, node %d, child %d, now:%llu\n", n->id, n->sched->id, litmus_clock());
				#endif

				n->sched->is_ZL = 0;
				sched_node_start_exec(n->sched, now);
				update = 1;

			} else if (ZL || n->sched->is_ZL) {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("WRNG ZL child:%d w_t_f:%llu e_dl:%llu bgt:%llu now:%llu\n",
						n->sched->id, w_t_f, n->sched->e_dl, n->sched->budget, litmus_clock());
				#endif
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
			TRACE("WRNG node %d, stopping sched. now:%llu\n", n->id, litmus_clock());
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

	int s_id, cpu, exists, release_cpu, circled, i;
	run_server_t 	*s, *server_aux;
	struct task_struct 	*resource_holding;
	struct task_struct 	*hp_task, *help_candidate;
	cpu_entry_t  	*entry;
	struct run_resource 	*r;

	#ifdef WANT_DEBUG_LITMUS_EVENTS
	TRACE("Servers rescheduled at %llu, now:%llu\n", now, litmus_clock());
	#endif

	t_st_init(&run.ts);

	for(cpu = 0; cpu < run.n_cpus; cpu ++) {
		entry = remote_entry(cpu);
		s = entry->s_sched;
		exists = (s != NULL);

		release_cpu = exists && !must_execute(s);

		#ifdef WANT_ALL_SCHED_EVENTS
		if (s && s->rn)
			TRACE("Server(%d) nodelevel:%d circled:%d CPU #%d. Rel_CPU?%d now:%llu\n",
					s->id, s->rn->level, s->rn->circled, cpu, release_cpu, litmus_clock());
		#endif

		// If the processor was not running anything, then consider it as a
		// processor that can be used to run something
		if (!exists) {
			t_st_push(&run.ts, entry);

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("Server NULL is releasing the cpu #%d now:%llu\n", cpu, litmus_clock());
			#endif
		}
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

			hp_task = s->hp_task;
			if (hp_task != NULL) {

				resource_holding = tsk_rt_parms(hp_task)->helping_task;

				/* If the resource holding task is being helped by server's high
				 * priority task, then we need to reset the helping parameters. */
				if (resource_holding != NULL) {
					BUG_ON (tsk_rt_parms(resource_holding)->helper_task != hp_task);
					BUG_ON (tsk_rt_parms(hp_task)->mrsp_status != WAITING_RESOURCE);
					reset_helping_task(resource_holding);
					entry = remote_entry(get_s(resource_holding)->cpu);
					entry->resched = 1;
				}

				// We must consider the consistency of the resource: the counter
				// actively_waiting must be updated.
				if (tsk_rt_parms(hp_task)->mrsp_status == WAITING_RESOURCE) {
					r = tsk_rt_parms(hp_task)->resource;
					BUG_ON (!r);
					r->actively_waiting--;

					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE("RESOURCE(%d) decreased r->actively_waiting = %d\n", r->id, r->actively_waiting);
					#endif

					///**/sched_trace_action(hp_task, 102);
				} else if (tsk_rt_parms(hp_task)->mrsp_status == HOLDING_RESOURCE) {

					r = tsk_rt_parms(hp_task)->resource;
					BUG_ON (!r);

					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE("RESOURCE(%d) task(%d) need helping\n", r->id, hp_task->pid);
					#endif

					i = r->queue_out_index;
					while (i != r->queue_in_index) {
						help_candidate = r->fifo_queue[i];
						server_aux = get_s(help_candidate);
						if (server_aux->running) {
							entry = remote_entry(server_aux->cpu);
							entry->resched = 1;
							#ifdef WANT_ALL_SCHED_EVENTS
							TRACE("RESOURCE(%d) helping candidate(%d) at CPU #%d, server %d\n",
									r->id, help_candidate->pid, server_aux->cpu, server_aux->id);
							#endif
						}
						i = (i + 1) % MAX_SERVER;
					}
				}
			}
			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("Server %d is releasing the cpu #%d, now:%llu\n", s->id, cpu, litmus_clock());
			#endif
		}
	}

	for(s_id = 0; s_id < run.n_servers; s_id++) {

		s = run.servers[s_id];
		circled = must_execute(s);

		#ifdef WANT_ALL_SCHED_EVENTS
		if (s->released && !((s->hp_task) || (!bheap_empty(&s->ready_queue))))
			TRACE("WRNG Server %d released/not active. e_dl:%llu at %llu, now:%llu\n",
					s->id, s->rn->e_dl, now, litmus_clock());
		#endif

		// If a server is decided to not execute, reset possible release events
		// since they will be discarded.
		if (!circled) {
			BUG_ON(s->running);
			s->released = 0;

			#ifdef WANT_SERVERS_SCHED_EVENTS
			TRACE("Server %d not selected for executing, now:%llu\n", s->id, litmus_clock());
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

				#ifdef WANT_SERVERS_SCHED_EVENTS
				TRACE("Server (%d) running on cpu #%d, now:%llu\n", s->id, s->cpu, litmus_clock());
				#endif

				if (s->released) {
					s->released = 0;
					entry = remote_entry(s->cpu);
					if (preemption_needed(s)) {
						entry->resched = 1;
    		  		}
    	  		}
			}
			// Otherwise, if a server has work to perform, is deemed to be executed
			// by RUN but is not already running, we assign the server to a free
			// processor. The free processor is taken from the temporary stack
			// created in the previous step.
			// We also mark the chosen cpu to remember to reschedule its work.
			else {
				if (t_st_empty(&run.ts)) {
					#ifdef WANT_SERVERS_SCHED_EVENTS
					TRACE("WRNG Server (%d) no proc available, now:%llu\n", s->id, litmus_clock());
					#endif
				}
				else {
					entry = (cpu_entry_t *)t_st_pop(&run.ts);
					s->cpu = entry->cpu;
					s->running = 1;
					s->released = 0;
					entry->s_sched = s;
					entry->resched = 1;

					// In case the server's hp task is waiting for a resource,
					// we must update the counter actively_waiting.
					hp_task = s->hp_task;
					if (hp_task && (tsk_rt_parms(hp_task)->mrsp_status == WAITING_RESOURCE)) {
						r = tsk_rt_parms(hp_task)->resource;
						BUG_ON(!r);
						r->actively_waiting++;

						///**/sched_trace_action(hp_task, 103);

						#ifdef WANT_ALL_SCHED_EVENTS
						TRACE("RESOURCE(%d) increased r->actively_waiting = %d\n", r->id, r->actively_waiting);
						#endif
					}

					#ifdef WANT_SERVERS_SCHED_EVENTS
					TRACE("Server (%d) assigned to cpu #%d, now:%llu\n", s->id, entry->cpu, litmus_clock());
					#endif
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
    	if (entry->resched) {
			entry->resched = 0;
			litmus_reschedule(cpu);
			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE("Reschedule CPU %d at %llu\n", cpu, litmus_clock());
			#endif
    	}
    }
}

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
	int   qt_jobs_released = 0;

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
		qt_jobs_released++;

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t, "REL s:%d nd:%d rel:%llu dl:%llu C:%llu T:%llu\n", s->id, s->rn->id, get_release(t), get_deadline(t), get_exec_cost(t), get_rt_period(t));
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
		TRACE("server(%d) released?%d bdgt:%llu node:%d\n", s->id, s->released, s->rn->budget, s->rn->id);
		#endif

		if (s->released) {

			e_dl = get_earliest_deadline(s, now);

			if (e_dl == INIT_HORIZON) {

				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE("WRNG No earliest_deadline found for server(%d) bdgt:%llu\n", s->id, s->rn->budget);
				#endif

				upd_dl_on_tree(s->rn, now, now);
			}
			else {
				upd_dl_on_tree(s->rn, now, e_dl);
			}
		} else {

			#ifdef WANT_ALL_SCHED_EVENTS
			if (s->rn->e_dl != INIT_HORIZON && lt_after(now, s->rn->e_dl)) {
				TRACE("WRNG server(%d) should complete. now:%llu e_dl:%llu bdgt:%llu\n", s->id, now, s->rn->e_dl, s->rn->budget);
			}
			#endif
		}
	}

	// Update the status of the reduction tree with the newly received deadlines.
	r_tree_traversal(run.rtree.root, now, &run.ts);

	// Dispatch the servers that the RUN algorithm deemed to be executing.
	run_resched_servers(now);

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("jobs released: %d. ----------------- END -----------------\n", qt_jobs_released);
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
	TRACE("BUDGET EXHAUSTED at %llu node %d bdgt:%llu\n", litmus_clock(), n->id, n->budget);
	#endif

	BUG_ON(!system_started);
	BUG_ON(!n->sched);

	//It may be affected by rounding problem
	now = n->sched->start_time + n->sched->budget;
	// 140118 - A timer may trigger simultaneously to a release event causing an
	// invalid status of the tree

	if (lt_after(now /*- ROUND_ERROR*/, ts)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("WRNG exhaustion overlaps release on node %d, now %llu ts %llu\n", n->sched->id, now, ts);
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

/* Set the task that must execute according to the helping mechanism
 * If the resource holding is not executing, set parameters for helping
 * and define the next task to be the resource holder. Do not change
 * the server's hp_task parameter, which continue to be the high priority
 * task of the server.
 */
static inline struct task_struct* set_next_task_check_helping(
		struct task_struct* next)
{
	struct run_resource *r = NULL;
	struct task_struct* resource_holder;
	struct task_struct* helper_task;
	cpu_entry_t  	*entry;
	run_server_t       *help_server=NULL;

	#ifdef HELPING_MECHANISM_DISABLED
	return;
	#endif

	if (!next) return NULL;
	if (!is_realtime(next)) return next;

	entry = local_entry;
	resource_holder = tsk_rt_parms(next)->helping_task;
	helper_task = tsk_rt_parms(next)->helper_task;

	/* If the resource holder is being helped by the selected task, then select the resource holder for execution */
	if (resource_holder) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(next, "task is helping %d.\n", resource_holder->pid);
		#endif

		tsk_rt_parms(resource_holder)->cpu = tsk_rt_parms(next)->cpu;
		next = resource_holder;

	/* If the resource holder is not being helped, then configure the helping mechanism */
	} else if (is_task_need_for_helping(next)) {

		r = tsk_rt_parms(next)->resource;
		set_helping_task(next, r->holding_task);

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(next, "task will help the task %d.\n", r->holding_task ? r->holding_task->pid : -1);
		#endif

		next = r->holding_task;

	} else if (helper_task) {

		help_server = get_s(helper_task);

		/* Avoid next task from being helped and running at its server simultaneously */
		if (help_server->running) {

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE_TASK(next, "task is being helped by %d. Return NULL for execution\n", helper_task->pid);
			#endif

			next = NULL;

		}
	}

	return next;
}

/***************************************************/

/*
 * Scheduling procedure called from inside a processor. Its decision is based
 * on the leaf server of the reduction tree assigned to the processor. Thus it
 * is indirectly related on the status of the reduction tree.
 * The prev parameter indicates the task which was assigned to the processor
 * before the schedule function is called.
 */
static struct task_struct* run_schedule(
    struct task_struct *prev)
{
	struct task_struct *hp_task=NULL, *next=NULL, *helping_task=NULL;
	run_server_t       *just_server=NULL;
	run_server_t       *help_server=NULL;
	run_server_t       *prev_server=NULL;
	cpu_entry_t        *entry=NULL;
	int sleep, preempt, np, exists, blocks, resched, out_of_time, prev_is_requeued, srv_changed;

	prev_is_requeued = 0;

	raw_spin_lock(&slock);

	entry = local_entry;

	helping_task = tsk_rt_parms(prev)->helping_task;

	// If the system has not yet started (before time 0) we manage the situation
	// accordingly (G-EDF).
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		//TRACE_TASK(prev, "!system_started || exit_mode\n");
		#endif

		BUG_ON(entry->sched && entry->sched != prev);
		BUG_ON(entry->sched && !is_realtime(prev));

		exists  = entry->sched != NULL;
		sleep   = exists && is_completed(entry->sched);
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
	// If the system is not in its initialization phase, schedule the tasks as RUN commands.
	else {

		// If no server is assigned to the processor, do not execute any realtime task.
		if (!entry->s_sched) {

			#ifdef WANT_ALL_SCHED_EVENTS
			TRACE_TASK(prev, "!entry->s_sched\n");
			#endif

			/* If the previous task was helping another task, then reset
			 * its configuration of the helping mechanism. */
			if (prev && helping_task) {
				reset_helping_task(helping_task);
				help_server = get_s(helping_task);
				if (help_server->running) {
					litmus_reschedule(help_server->cpu);
				}
			}

			next = NULL;
		}
		// If a server is assigned to the processor, determines which task to execute.
		else {

			// just_server is the server currently assigned to the processor to be scheduled.
			just_server = entry->s_sched;
			prev_server = get_s(prev);

			// just_prev identifies the task that is running inside the server.
			// If the server changed (becase of RUN scheduling decision),
			// hp_task != prev (from procedure's signature)
			hp_task = just_server->hp_task;

			// Define flags for the status of the last executing task
			exists = (prev != NULL);

			if (exists && is_realtime(prev) && budget_exhausted(prev)) {
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE_TASK(prev, "OUT_OF_TIME server:%d, exec:%llu, cost:%llu, node_bud:%llu\n",
						prev_server->id,
						get_exec_time(prev),
						get_exec_cost(prev),
						node_budget_remaining(prev_server->rn));
				#endif
			}

			out_of_time = 0; // no out of time, no budget enforcement

			/* current executing task exists and is not running */
			blocks = exists && !is_current_running();

			/* current executing task exists and is non-preemptive */
			np = exists && is_np(prev);

			/* current executing task exists and its parameter is_completed = TRUE */
			sleep = exists && is_completed(prev);

			/* Verify if the task at the top of the server's ready tasks queue
			 * is different of the hp_task of the server and there are conditions
			 * for preemption to occur */
			preempt = preemption_needed(just_server);

			// always preempt the prev task when the scheduled server is changing
			srv_changed = 0;
			if (just_server && (!prev_server || prev_server != just_server)) {
				srv_changed = 1;
				#ifdef WANT_ALL_SCHED_EVENTS
				TRACE_TASK(prev, "prev_server:%d just_server:%d now:%llu\n", prev_server?prev_server->id:0,
						just_server->id, litmus_clock());
				#endif
			}

			// Understand through the flags if there is the necessity to reschedule
			// the work of the processor (change the running task).
			#ifdef WANT_ONLY_SCHED_EVENTS
			if (exists && is_realtime(prev)) {
				TRACE_TASK(prev, "blk:%d t_out:%d np:%d sl:%d prmpt:%d st:%d sg:%d srv:%d cp:%d nw:%llu\n",
					blocks, out_of_time, np, sleep, preempt, prev->state,
					signal_pending(prev), prev_server->id, prev_server->cpu, litmus_clock());
			} else if (exists)
					TRACE_TASK(prev, "non RT task blk:%d sl:%d prmpt:%d nw:%llu\n", blocks, sleep, preempt, litmus_clock());
			#endif

			// If a preemption is needed, we must reschedule the server, since a
			// higher priority task is ready to execute.
			resched = preempt;

			// If the task is blocked, it is not performing any work, let switch it
			if (blocks)
				resched = 1;

			if (srv_changed)
				resched = 1;

			/* If the task is set to non preemptive, but there is a need to preempt it
			 * or it is completed, then is requested the end of the non preemptive section */
			if (np && (out_of_time || preempt || sleep || srv_changed))
				request_exit_np(prev);

			if (!np && (out_of_time || sleep) && !blocks) {
				/* This call may call reschedule if task is tardy */
				job_completion(prev, !sleep);
				resched = 1;
				if (prev && is_realtime(prev)) {
					get_s(prev)->hp_task = NULL;
				}
			}

			// Determines the next job to execute in the processor.
			next = NULL;

			// If the last scheduled task (or no task at all) from the server must
			// be switched with some other task, let's take this new task.
			// We must also remember to requeue the last task that the server was
			// executing.
			if (resched || !exists) {
				if (prev && is_realtime(prev)) {
					// If the server was executing a task, we have to requeue it. This
					// call place the job in the right queue: the ready queue in the
					// server if the job has not completed, the release queue of the
					// run edf_domain if the job has completed. IMPORTANT: we have already
					// performed "job_completion", so we can call this procedure safely.
					if (!blocks) {

						// If the task selected to executing must be preempted and it was
						// helping another task, then the helping parameters must be unset.
						if (helping_task) {
							reset_helping_task(helping_task);
							help_server = get_s(helping_task);
							if (help_server->running && entry->cpu != help_server->cpu) {
								litmus_reschedule(help_server->cpu);
							}
						} else {
							if (tsk_rt_parms(prev)->helper_task) {
								reset_helping_task(prev);
								help_server = get_s(prev);
								if (help_server->running && entry->cpu != help_server->cpu) {
									litmus_reschedule(help_server->cpu);
								}
							} else {
								requeue(prev);
							}
						}

						prev_is_requeued = 1;

						#ifdef WANT_ALL_SCHED_EVENTS
						TRACE_TASK(prev, "requeue(%d); hp_task=%d;\n", (prev ? prev->pid : 0), (hp_task ? hp_task->pid : 0));
						#endif
					}
				}

				// Maintain SRP properties when the server is changed
				if (!preempt && srv_changed && hp_task) {
					next = hp_task;
					unqueue_task_preempted(hp_task);
					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE_TASK(prev, "after unqueue_task_preempted(), next=%d;\n", (next ? next->pid : 0));
					#endif
				} else {
					next = run_take_ready(just_server);
					#ifdef WANT_ALL_SCHED_EVENTS
					TRACE_TASK(prev, "after run_take_ready(), next=%d;\n", (next ? next->pid : 0));
					#endif
				}

				// We must also update the internal state of the server!
				just_server->hp_task = next;

			}
			// If the highest priority task in the server has not changed, let's
			// select it, which is the last scheduled task.
			else if (exists)
				next = hp_task;
			// We must also update the internal state of the server!
			else
				just_server->hp_task = next;

			/* Set the task that must execute according to the helping mechanism
			 * If the resource holding is not executing, set parameters for helping
			 * and define the next task to be the resource holder. Do not change
			 * the server's hp_task parameter, which continue to be the high priority
			 * task of the server.
			 */
			next = set_next_task_check_helping(next);

		}
	}

	if (next) {
		tsk_rt(next)->completed = 0;
		#ifdef WANT_ONLY_SCHED_EVENTS
		TRACE_TASK(next, "run: scheduled at %llu\n", litmus_clock());
		#endif
	}
	else {
	}

	// Notify the processor that the job to schedule has been determined.
	sched_state_task_picked();
	raw_spin_unlock(&slock);
	return next;
}

/* Prepare a task for running in RT mode.
 * Allow to catch the barrier release time.
 */
static void run_release_at(
	lt_t start)
{
	struct task_struct *t = current;
	unsigned long flags;

	raw_spin_lock_irqsave(&slock, flags);

	if (global_time == INIT_HORIZON) {
		global_time = start;

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE("Global time initialized, system will start at %llu, now:%llu\n", global_time, litmus_clock());
		#endif
	}

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
	TRACE_TASK(task, "wake_up at %llu\n", now);
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	BUG_ON(is_queued(task));

	now = litmus_clock();

	if (is_tardy(task, now)) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(task, "is_tardy, dl %llu, now %llu\n", get_deadline(task), now);
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
	TRACE_TASK(task, "will release at %llu, now %llu, difference %llu \n", get_release(task), now, get_release(task)-now);
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

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(t, "run_task_exit()\n");
	#endif

	#ifdef TERMINATION_CODE
	if (exit_mode) { //Exit mode

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t, "run_task_exit().if(exit_mode)//TERMINATION_CODE\n");
		#endif

		if (is_queued(t)) {
			remove(&run.domain, t);
		}
		entry = local_entry;
		if (entry->sched == t)
			entry->sched = NULL;

	} else { //RUN mode

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t, "run_task_exit().elseif(!exit_mode)\n");
		#endif

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

				set_exit_timer = 0;
			}

			tasks_added = tasks_added - 1;
			if (tasks_added <= 0) {
				hrtimer_try_to_cancel(&exit_timer);
			}

			wtf = litmus_clock();
			trace_litmus_sys_release(&wtf);

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
		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t, "run_task_exit().if(is_queued(t))\n");
		#endif

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
		trace_litmus_sys_release(&wtf);

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

/*
 * Check if process is correctly configured
 */
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

	// Error if the task's period is not greather to 0
	if (tsk_rt_parms(t)->period <= 0) {

		#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(t,"sched_run.run_admit_task: rejected, task's period is not greather to 0\n");
		#endif

		return -EINVAL;
	}

	s = run.servers[s_id];
	t->rt_param.server = s;
	//tsk_rt_parms(t)->budget_policy = QUANTUM_ENFORCEMENT;
	tsk_rt_parms(t)->mrsp_ceiling = tsk_rt_parms(t)->period;
	tsk_rt_parms(t)->mrsp_status = NO_RESOURCE;
	tsk_rt_parms(t)->helper_task = NULL;
	tsk_rt_parms(t)->helping_task = NULL;
	tsk_rt_parms(t)->waiting_ticket = -1;

	s->task_ref[s->task_n] = t;
	s->task_n += 1;

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

	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("on_exit_experiments()\n");
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	if (tasks_added > 0) {
		for(s_id = 0; s_id < run.n_servers; s_id++) {
			s = run.servers[s_id];
			if (s) {
				if (s->hp_task) {
					if (!bheap_node_in_heap(tsk_rt(s->hp_task)->heap_node))
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
	#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("run_deactivate_plugin()\n");
	#endif
	cleanup_run();
	return 0;
}

static long run_activate_plugin(void)
{
	int cpu;
	cpu_entry_t *entry;

	raw_spin_lock_init(&slock);

	cleanup_run();

	counter_resource = 0;
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

static inline struct run_resource* run_sem_from_lock (
    struct litmus_lock *l)
{
	return container_of(l, struct run_resource, litmus_lock);
}

/* Perform a busy wait loop
 */
static inline int perform_busy_wait (
    int      waiting_ticket,
    atomic_t *actual_ticket,
	struct task_struct  *t,
	struct run_resource *r
	)
{
	int read_ticket;

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(current, "BEFORE wait\n");
	#endif

	read_ticket = atomic_read(actual_ticket);
	while (waiting_ticket != read_ticket
		#ifdef TERMINATION_CODE
			&& !exit_mode
		#endif
	) {
		cpu_relax();
		read_ticket = atomic_read(actual_ticket);
	}

	barrier();

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(current, "AFTER wait\n");
	#endif

	return 1;

}

/* This function is called when a task locks a resource. It receives
 * a pointer to the litmus lock struct and from this struct is possible
 * to get the run resource semaphore.
 * This function sets the parameters of the requesting task and the
 * requested resource. If the resource is already locked, the current
 * task can do one of these options:
 * - Busy wait
 * - Help the resource holder task which is locking the requested resource
 *   but not executing
 * If the requested resource is not locked, the task get the resource.
 */
int lock_run_semaphore(
    struct litmus_lock* l)
{

	struct task_struct  *t = current;
	struct run_resource *r = run_sem_from_lock(l);
	run_server_t   		*s = get_s(t);
	unsigned long 		flags;

	/* If the plugin is not yet started or is closing, ignore every
	 * lock/unlock request. WORKAROUND usable only for dummy tasks! */
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {
		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d) lock_run_semaphore().exit_mode\n", r->id);
		#endif
		return 1;
	}

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d) r->ceil[%d]=%d s->ceil=%d t->ceil=%d t->period=%d\n",
			r->id, s->id, r->mrsp_ceiling[s->id], s->mrsp_ceiling, tsk_rt_parms(t)->mrsp_ceiling, tsk_rt_parms(t)->period);
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_LOCK_START);
	#endif

	/* The MrsP ceiling property of the resource for the server must be defined previously */
	BUG_ON (r->mrsp_ceiling[s->id] == 0);

	/* The ceiling of the resource cannot be lower (i.e., bigger numeral) than the task's period */
	BUG_ON (r->mrsp_ceiling[s->id] > tsk_rt_parms(t)->period);

	tsk_rt_parms(t)->resource = r;

	/* Raises the MrsP ceiling of the task to the MrsP ceiling of the resource */
	tsk_rt_parms(t)->mrsp_ceiling = r->mrsp_ceiling[s->id];

	/* For restoring later to the previous ceiling */
	t_st_i_push(&s->mrsp_ceiling_stack, s->mrsp_ceiling);

	/* Raises the server's MrsP ceiling if necessary. */
	if (s->mrsp_ceiling > r->mrsp_ceiling[s->id] || s->mrsp_ceiling == 0)
		s->mrsp_ceiling = r->mrsp_ceiling[s->id];

	BUG_ON (r->holding_task == NULL && r->holding_server != NULL);
	BUG_ON (r->holding_task != NULL && r->holding_server == NULL);

	// If the resource is free and no other task is queued, the current
	// request can be immediately satisfied.
	if (r->holding_server == NULL) {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][L] LOCK server:%d task:%d cpu:%d\n",
				   r->id, litmus_clock(), s->id, t->pid, s->cpu);
		#endif

		/* Sanity check: no other task must be waiting. */
		BUG_ON(r->queue_in_index != r->queue_out_index);

		r->holding_server = s;
		r->holding_task   = t;
		tsk_rt_parms(t)->mrsp_status = HOLDING_RESOURCE;

		/* Now we have the right to use the resource, so we can return.
		 * We must anyhow take the ticket, otherwise the other tasks will not
		 * perform busy wait. */
		atomic_inc(&r->next_ticket);

		#ifdef WANT_RUNRSP_FTRACE
		sched_trace_action(t, RUN_LOCK_END);
		#endif

		raw_spin_unlock_irqrestore(&slock, flags);

		return 1;
	}
	// If the resource is occupied we must:
	// 1) update the status of the resource and the task
	// 2) queue the task on the resource
	// 3) take the ticket for the busywait
	// 4) perform busywait
	else {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][L] SPIN server:%d cpu:%d [queued:%d wait:%d hold:%d wtkt:%d atkt:%d]\n",
				   r->id, litmus_clock(), s->id, s->cpu,
				   r->queue_in_index>=r->queue_out_index ?
					 r->queue_in_index-r->queue_out_index:
					 r->queue_in_index+MAX_SERVER-r->queue_out_index,
				   r->actively_waiting, r->holding_task->pid,
				   atomic_read(&r->next_ticket), atomic_read(&r->actual_ticket)
		);
		#endif

		tsk_rt_parms(t)->mrsp_status = WAITING_RESOURCE;
		r->fifo_queue[r->queue_in_index] = t;
		r->queue_in_index = (r->queue_in_index + 1) % MAX_SERVER;

		// We must take the ticket coherently with the position of the
		// server in the fifo_queue. Then we update the value for the next ticket.
		tsk_rt_parms(t)->waiting_ticket = atomic_read(&r->next_ticket);
		atomic_inc(&r->next_ticket);

		// We must give the cpu to the holding task if it is not already executing.
		// It can be the case that the current task belongs to a server that has
		// been suspended: if the task tried to acquire the lock while a scheduling
		// decision was performed, the lock request is being queued in the IRQ and
		// executed independently of the status of the server. So we force to
		// execute the holding task only if we are really running.
		if (s->running) {

			r->actively_waiting++;

			/* If the task that is holding the resource is not running, then the
			 * helping mechanism will be triggered. */
			if (!holding_task_is_running(r)) {
				#ifdef WANT_RUNRSP_INFO
				TRACE_TASK(t, "RESOUCE(%d)[%llu][L] holding task is NOT running\n", r->id, litmus_clock(), t->pid, s->id);
				#endif
				set_helping_task(t, r->holding_task);
				litmus_reschedule(s->cpu);
			} else {
				#ifdef WANT_RUNRSP_INFO
				TRACE_TASK(t, "RESOUCE(%d)[%llu][L] holding task is running\n", r->id, litmus_clock(), t->pid, s->id);
				#endif
			}
		} else {
			#ifdef WANT_RUNRSP_INFO
			TRACE_TASK(t, "RESOUCE(%d)[%llu][L] task(%d) of server(%d) try lock while not running\n",
					   r->id, litmus_clock(), t->pid, s->id);
			#endif
		}
	}

	// Now we must start the busy wait.
	busy_wait_for_resource(t, r, flags);

	// From here the server is HOLDING_RESOURCE. Some other server must have
	// incremented the actual_ticket when unlocking the resource. That same
	// server must have already changed the state of this server, as if it
	// acquired the resource manually.
	BUG_ON (tsk_rt_parms(t)->mrsp_status != HOLDING_RESOURCE
		#ifdef TERMINATION_CODE
		&& !exit_mode
		#endif
		  );

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][L] Task got resource\n", r->id, litmus_clock());
	#endif

	return 1;
}

/* Perform a busy wait loop
 * IMPORTANT: must be called with slock
 */
static inline int busy_wait_for_resource(
		struct task_struct  *t,
		struct run_resource *r,
		unsigned long 		flags)
{
	int waiting_ticket = tsk_rt_parms(t)->waiting_ticket;
	int result = 1;

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_LOCK_END);
	#endif

	/**/TS_LOCK_END
	raw_spin_unlock_irqrestore(&slock, flags);

	// Now we must start the busy wait.
	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_SPINNING_START);
	#endif

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][L] Perform busy wait(%d)\n", r->id, litmus_clock(), t->pid);
	#endif

	perform_busy_wait(waiting_ticket, &r->actual_ticket, t, r);

	/***/TS_LOCK_START
	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_SPINNING_END);
	#endif

	return result;

}

/* This function is called when a task releases a resource.
 * It resets the locking parameters of the task and the resource.
 * If the task which releases the resource is being helped,
 * then the helper locking parameters will be reset and the
 * helper server will be set to be rescheduled. If not,
 * the server of the task which releases the resource will
 * be set to be rescheduled.
 * The first task on the resource's queue (if exists) will be
 * configured to lock the resource.
 */
static int unlock_run_semaphore(
    struct litmus_lock* l)
{

	struct task_struct  *t = current;
	struct task_struct  *helper = NULL;
	struct task_struct  *new_holder = NULL;
	struct run_resource *r = run_sem_from_lock(l);
	run_server_t		*s = get_s(t);
	unsigned long 		 flags;
	int 				 i;
	int 				 cpu_to_resched_1 = -1;
	int					 cpu_to_resched_2 = -1;
	int					 cpu_to_resched_3 = -1;

	// If the plugin is not yet started or is closing, ignore every
	// lock/unlock request. WORKAROUND usable only for dummy tasks!
	// Dummy unlock only if task is not really holding resource,
	// otherwise it could starve tasks already queued!
	if (unlikely(!system_started
		#ifdef TERMINATION_CODE
		|| exit_mode
		#endif
	)) {
		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "unlock_run_semaphore(%d).exit_mode at %llu\n", t->pid, litmus_clock());
		#endif
		return 0;
	}

	// We make sure that the current task/server is holding the resource.
	BUG_ON (tsk_rt_parms(t)->mrsp_status != HOLDING_RESOURCE);
	BUG_ON (r->holding_task != t);
	BUG_ON (r->holding_server != s);
	BUG_ON (tsk_rt_parms(t)->resource == NULL);

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_UNLOCK_START);
	#endif

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][U] UNLOCK t->ceil=%d, t->period=%d, s->ceil=%d\n",
			 r->id, litmus_clock(), tsk_rt_parms(t)->mrsp_ceiling, tsk_rt_parms(t)->period, s->mrsp_ceiling);
	#endif

	// Free the resource and reset the state of the task.
	r->holding_server = NULL;
	r->holding_task = NULL;
	tsk_rt_parms(t)->mrsp_status = NO_RESOURCE;
	tsk_rt_parms(t)->resource = NULL;
	tsk_rt_parms(t)->mrsp_ceiling = tsk_rt_parms(t)->period;

	// Restoring to the previous server's ceiling
	t_st_i_pop(&s->mrsp_ceiling_stack, &s->mrsp_ceiling);

	helper = tsk_rt_parms(t)->helper_task;

	/* If the current task was being helped, then we must restore it to
	 * the CPU of its server and reschedule the CPU of the helper task
	 * and the CPU of the current task. */
	if (helper) {
		BUG_ON(r->actively_waiting == 0);
		reset_helping_task(t);
		cpu_to_resched_1 = get_s(helper)->cpu;
		cpu_to_resched_2 = get_s(t)->cpu;
	}
	// If the holding server is running, then it already has a dedicated cpu.
	// To restore the 'original RUN scheduling decision' we just have to
	// reschedule the cpu since a release event could have been discarded.
	else if (s->running) {
		BUG_ON(remote_entry(s->cpu)->s_sched != s);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] server(%d) continue executing in cpu(%d)\n",
				   r->id, litmus_clock(), s->id, s->cpu);
		#endif

		cpu_to_resched_1 = s->cpu;
	}
	// It can be the case that the holding server is not running nor shadow
	// running for someone else. It happens when the unlock request is performed
	// when a scheduling decision is being made, and the scheduler changed the
	// status of the tree. But since the IRQ has already been made, then it must
	// be evaded. In this case we must do nothing.
	else {
		BUG_ON(remote_entry(s->cpu)->s_sched == s);
		//BUG_ON(r->actively_waiting); *** Ricardo ***
		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] !running && !being_helped\n",
				   r->id, litmus_clock());
		#endif
	}

	// We always must promote the first fifo queued task as new holding task
	// (if any), since that task will be busy waiting.
	if (r->queue_in_index != r->queue_out_index) {
		// Get the next task with respect to fifo order.
		new_holder = r->fifo_queue[r->queue_out_index];

		BUG_ON(tsk_rt_parms(new_holder)->resource != r);
		BUG_ON(tsk_rt_parms(new_holder)->mrsp_status != WAITING_RESOURCE);

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Pass lock to task(%d)\n",
				   r->id, litmus_clock(), new_holder->pid);
		#endif

		tsk_rt_parms(new_holder)->mrsp_status = HOLDING_RESOURCE;

		// Update the status of the resource
		r->holding_server = get_s(new_holder);
		r->holding_task = new_holder;
		r->queue_out_index = (r->queue_out_index + 1) % MAX_SERVER;

		// We need to update the counter actively_waiting knowing that the task
		// just removed from the queue could be one of them. // *** Ricardo ***
		if (task_is_running(new_holder)) {
			r->actively_waiting--;
			BUG_ON(r->actively_waiting < 0);
		}
	}

	// We apply the dispatching rule that let execute a possible shadow task.
	// This can happen only if the new holding task (if any) is not running
	// while some other task is actively waiting.
	if (new_holder && !task_is_running(new_holder) && r->actively_waiting) {

		BUG_ON(tsk_rt_parms(new_holder)->helper_task);
		helper = NULL;
		i = r->queue_out_index;
		while (i != r->queue_in_index) {
			helper = r->fifo_queue[i];
			if (is_busy_waiting_for(helper, r)) {

				#ifdef WANT_RUNRSP_INFO
				TRACE_TASK(helper, "RESOUCE(%d)[%llu][L] holding task is NOT running\n",
						r->id, litmus_clock());
				#endif

				// We have found a task that is queued and is now
				// wasting CPU by busy waiting. We shadow that task.
				set_helping_task(helper, new_holder);

				/* The CPU which the helper task is running must be rescheduled
				 * for the resource holding task execute by the helping mechanism */
				cpu_to_resched_3 = get_s(helper)->cpu;

				break;
			}
			i = (i + 1) % MAX_SERVER;
		}

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] helper(%d)->new_holder(%d)\n",
				   r->id, litmus_clock(), helper ? helper->pid : -1, new_holder ? new_holder->pid : -1);
		#endif
	}

	atomic_inc(&r->actual_ticket);

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(t, RUN_UNLOCK_END);
	#endif

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu][U] Ticket status: next(%d) actual(%d)\n",
			 r->id, litmus_clock(), atomic_read(&r->next_ticket), atomic_read(&r->actual_ticket));
	#endif

	if (cpu_to_resched_1 != -1) {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] cpu_to_resched:%d from server (%d)\n",
				   r->id, litmus_clock(), cpu_to_resched_1, s->id);
		#endif

		litmus_reschedule(cpu_to_resched_1);
	}
	if (cpu_to_resched_2 != -1
			&& cpu_to_resched_1 != cpu_to_resched_2) {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] cpu_to_resched:%d from server (%d)\n",
				   r->id, litmus_clock(), cpu_to_resched_2, s->id);
		#endif

		litmus_reschedule(cpu_to_resched_2);
	}

	if (cpu_to_resched_3 != -1
			&& cpu_to_resched_1 != cpu_to_resched_3
			&& cpu_to_resched_2 != cpu_to_resched_3) {

		#ifdef WANT_RUNRSP_INFO
		TRACE_TASK(t, "RESOURCE(%d)[%llu][U] cpu_to_resched:%d from server (%d)\n",
				   r->id, litmus_clock(), cpu_to_resched_3, s->id);
		#endif

		litmus_reschedule(cpu_to_resched_3);
	}

	raw_spin_unlock_irqrestore(&slock, flags);

	return 0;
}

void free_run_semaphore(
    struct litmus_lock* lock)
{
	#ifdef WANT_RUNRSP_INFO
	TRACE("free_run_semaphore()\n");
	#endif

	kfree(run_sem_from_lock(lock));
}

/*
 * Verify task/resource configurations and set the MrsP Ceiling for the resource
 */
static int open_run_semaphore(
		struct litmus_lock* l,
		void* __user arg)
{
	struct run_resource *r = run_sem_from_lock(l);
	struct task_struct  *t = current;
	struct run_server   *s = get_s(t);
	int err = 0;
	unsigned long 		 flags;

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(t, "RESOURCE(%d)[%llu] OPEN, server:%d, T:%llu\n", r->id, litmus_clock(), s->id, get_rt_period(t));
	#endif

	raw_spin_lock_irqsave(&slock, flags);

	if (!is_realtime(t))
		return -EPERM;

	if (r->holding_task != NULL)
		err = -EBUSY;

	if (err == 0)
		/* If the resource's ceiling is not defined yet or if the period of the task
		 * is smaller than the resource's ceiling, set the ceiling of the resource
		 * for the task to be equals to the task's period.
		 */
		if ((r->mrsp_ceiling[s->id] == 0) || (tsk_rt_parms(t)->period < r->mrsp_ceiling[s->id]))
			r->mrsp_ceiling[s->id] = tsk_rt_parms(t)->period;

	raw_spin_unlock_irqrestore(&slock, flags);

	return err;
}

/*
 * If a task is holding the resource, calls unlock
 */
static int close_run_semaphore(struct litmus_lock* l)
{
	struct run_resource *r = run_sem_from_lock(l);
	int err = 0;
	unsigned long 		 flags;

	raw_spin_lock_irqsave(&slock, flags);

	#ifdef WANT_RUNRSP_INFO
	TRACE("RESOURCE(%d)[%llu] CLOSE\n", r->id, litmus_clock());
	#endif

	if (r->holding_task == current)
		unlock_run_semaphore(l);

	raw_spin_unlock_irqrestore(&slock, flags);

	return err;
}

/*	Plugin object for locking	*/
static struct litmus_lock_ops run_rsp_lock_ops = {
	.open   = open_run_semaphore,
	.close  = close_run_semaphore,
	.lock   = lock_run_semaphore,
	.unlock = unlock_run_semaphore,
	.deallocate = free_run_semaphore,
};

static struct litmus_lock* new_run_resource (void)
{
	struct run_resource *sem;
	int i;

	sem = kmalloc(sizeof(*sem), GFP_KERNEL);
	if (!sem)
		return NULL;

	sem->id = counter_resource++;
	sem->actively_waiting = 0;
	sem->holding_server   = NULL;
	sem->holding_task	  = NULL;
	sem->queue_in_index   = 0;
	sem->queue_out_index  = 0;
	sem->litmus_lock.ops = &run_rsp_lock_ops;

	for (i=0; i<MAX_SERVER; i++)
		sem->mrsp_ceiling[i] = 0;

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

static void run_task_block(struct task_struct *t)
{
	#ifdef WANT_DEBUG_LITMUS_EVENTS
	TRACE_TASK(t, "task blocked at [%llu]\n", litmus_clock());
	#endif
	BUG_ON(!is_realtime(t));
}

/*	Plugin object for scheduling	*/
static struct sched_plugin run_plugin __cacheline_aligned_in_smp = {
	.plugin_name       = "RUN",
	.task_new          = run_task_new,
	.synchronous_release_at = run_release_at,
	.task_exit         = run_task_exit,
	.schedule          = run_schedule,
	.task_wake_up      = run_task_wake_up,
	.admit_task        = run_admit_task,
	.activate_plugin   = run_activate_plugin,
	.deactivate_plugin = run_deactivate_plugin,
	.allocate_lock     = run_allocate_lock,
	.task_block		   = run_task_block,
	.complete_job      = complete_job
};

static int __init init_run(void)
{
	return register_sched_plugin(&run_plugin);
}

static void clean_run(void)
{
	cleanup_run();
}

static inline int task_is_running(
		struct task_struct* t)
{
	run_server_t* s;
	cpu_entry_t* entry;
	int is_running = 0;
	if (t) {
		s = get_s(t);
		if (s->running && (s->hp_task == t)) {
			entry = remote_entry(s->cpu);
			is_running = ((entry->sched == t) || (entry->s_sched == s));
			if (!is_running) {
				is_running = (tsk_rt_parms(t)->helper_task != NULL);
			}
		} else {
			is_running = (tsk_rt_parms(t)->helper_task != NULL);
		}
	}
	return is_running;
}

static inline int is_busy_waiting_for(
		struct task_struct* t,
		struct run_resource* r)
{
	run_server_t* s;
	cpu_entry_t* entry;
	int is_busy_waiting = 0;
	if (t) {
		s = get_s(t);
		entry = remote_entry(s->cpu);
		if (t->state == TASK_RUNNING &&
				tsk_rt_parms(t)->mrsp_status == WAITING_RESOURCE &&
				tsk_rt_parms(t)->resource == r &&
				s->running &&
				entry->sched == t) {

			is_busy_waiting = 1;
		}
	}
	return is_busy_waiting;
}

/* Undo the configuration of the helping mechanism.
 */
static inline void reset_helping_task(
		struct task_struct *resource_holder)
{
	struct task_struct *helper;

	#ifdef HELPING_MECHANISM_DISABLED
	return;
	#endif

	if (!resource_holder) return;

	helper = tsk_rt_parms(resource_holder)->helper_task;
	if (!helper) return;

	reset_helping_task_no_requeue(resource_holder);

	requeue(resource_holder);
	requeue(helper);

}

/* Undo the configuration of the helping mechanism.
 */
static inline void reset_helping_task_no_requeue(
		struct task_struct *resource_holder)
{
	struct task_struct *helper;
	int r_id;

	#ifdef HELPING_MECHANISM_DISABLED
	return;
	#endif

	if (!resource_holder) return;

	helper = tsk_rt_parms(resource_holder)->helper_task;
	if (!helper) return;

	r_id = tsk_rt_parms(resource_holder)->resource ? ((struct run_resource*)tsk_rt_parms(resource_holder)->resource)->id : -1;

	tsk_rt_parms(resource_holder)->helper_task = NULL;
	tsk_rt_parms(resource_holder)->mrsp_ceiling = tsk_rt_parms(resource_holder)->period;
	tsk_rt_parms(resource_holder)->cpu = get_s(resource_holder)->cpu;

	tsk_rt_parms(helper)->helping_task = NULL;

	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(helper, "RESOURCE(%d)[%llu][H] Reset helping: holder(%d)\n",
			r_id, litmus_clock(), resource_holder->pid);
	TRACE_TASK(resource_holder, "RESOURCE(%d)[%llu][H] Reset helping: helper(%d)\n", r_id, litmus_clock(), helper->pid);
	#endif

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(resource_holder, RUN_IS_SHADOW_RUNNING_END);
	sched_trace_action(helper, RUN_IS_SHADOWED_END);
	#endif
}

static inline int holding_task_is_running(
		struct run_resource* r)
{
	#ifdef HELPING_MECHANISM_DISABLED
	return 1;
	#endif

	return task_is_running(r->holding_task);
}

static inline void unqueue_task_preempted(
		struct task_struct *t)
{
	if (is_queued(t) && is_released(t, litmus_clock())) {

		#ifdef WANT_DEBUG_LITMUS_EVENTS
		TRACE_TASK(t, "unqueue ready task at [%llu]\n", litmus_clock());
		#endif

		bheap_delete(edf_ready_order,
		   &(get_s(t))->ready_queue,
		   tsk_rt(t)->heap_node);
	}
}


/* Configure the helping mechanism.
 */
static inline void set_helping_task(
		struct task_struct *helper,
		struct task_struct *resource_holder)
{
	struct run_resource *r;

	#ifdef HELPING_MECHANISM_DISABLED
	return;
	#endif

	if (task_is_running(resource_holder))
		return;

	tsk_rt_parms(resource_holder)->helper_task = helper;
	tsk_rt_parms(resource_holder)->mrsp_ceiling = tsk_rt_parms(helper)->period - 1;
	tsk_rt_parms(resource_holder)->cpu = tsk_rt_parms(helper)->cpu;

	tsk_rt_parms(helper)->helping_task = resource_holder;

	unqueue_task_preempted(resource_holder);

	#ifdef WANT_RUNRSP_FTRACE
	sched_trace_action(resource_holder, RUN_IS_SHADOW_RUNNING_START);
	sched_trace_action(helper, RUN_IS_SHADOWED_START);
	#endif
	r = tsk_rt_parms(resource_holder)->resource;
	#ifdef WANT_RUNRSP_INFO
	TRACE_TASK(helper, "RESOURCE(%d)[%llu][H] Set helping: holder(%d) helper(%d)\n",
			r->id, litmus_clock(), resource_holder->pid, helper->pid);
	TRACE_TASK(resource_holder, "RESOURCE(%d)[H] holder period:%llu ceiling:%llu cpu:%d\n",
			r->id, tsk_rt_parms(resource_holder)->period, tsk_rt_parms(resource_holder)->mrsp_ceiling,
			tsk_rt_parms(resource_holder)->cpu);
	#endif
}

static inline int higher_preemption_level(
		struct task_struct* t)
{
	if (!t)
		return 0;

	if (get_s(t)->mrsp_ceiling == 0)
		return 1;

	return (tsk_rt_parms(t)->mrsp_ceiling < get_s(t)->mrsp_ceiling);
}

static inline int is_task_need_for_helping(struct task_struct* t)
{
	int is_help_needed = 0;
	struct run_resource *r;

	#ifdef HELPING_MECHANISM_DISABLED
		return 0;
	#endif

	if (!t)
		return 0;

	r = tsk_rt_parms(t)->resource;
	if (r) {
		if (r->holding_task && r->holding_task != t)
			is_help_needed = !holding_task_is_running(r);
	}
	return is_help_needed;
}

module_init(init_run);
module_exit(clean_run);
