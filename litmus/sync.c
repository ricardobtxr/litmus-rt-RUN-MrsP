/* litmus/sync.c - Support for synchronous and asynchronous task system releases.
 *
 *
 */

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/completion.h>

#include <litmus/litmus.h>
#include <litmus/sched_plugin.h>
#include <litmus/jobs.h>

#include <litmus/sched_trace.h>
#include <litmus/debug_trace.h>

#define TRACE_TASK_SYSTEM_CALL

struct ts_release_wait {
	struct list_head list;
	struct completion completion;
	lt_t ts_release_time;
};

#define DECLARE_TS_RELEASE_WAIT(symb)					\
	struct ts_release_wait symb =					\
	{								\
		LIST_HEAD_INIT(symb.list),				\
		COMPLETION_INITIALIZER_ONSTACK(symb.completion),	\
		0							\
	}

static LIST_HEAD(task_release_list);
static DEFINE_MUTEX(task_release_lock);

static long  do_wait_for_ts_release(void)
{
	DECLARE_TS_RELEASE_WAIT(wait);

	long ret = -ERESTARTSYS;

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(current, "do_wait_for_ts_release()\n");
	#endif

	if (mutex_lock_interruptible(&task_release_lock))
		goto out;

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(current, "do_wait_for_ts_release() after mutex_lock_interruptible\n");
	#endif

	list_add(&wait.list, &task_release_list);

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(current, "do_wait_for_ts_release() after list_add\n");
	#endif

	mutex_unlock(&task_release_lock);

	/* We are enqueued, now we wait for someone to wake us up. */
	ret = wait_for_completion_interruptible(&wait.completion);

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(current, "do_wait_for_ts_release() after wait_for_completion_interruptible, ret=%d\n", ret);
	#endif

	if (!ret) {
		/* Completion succeeded, setup release time. */
		ret = litmus->wait_for_release_at(
			wait.ts_release_time + get_rt_phase(current));

		#ifdef TRACE_TASK_SYSTEM_CALL
		TRACE_TASK(current, "do_wait_for_ts_release() after wait_for_release_at, ret=%d\n", ret);
		#endif

	} else {
		/* We were interrupted, must cleanup list. */
		mutex_lock(&task_release_lock);
		if (!wait.completion.done)
			list_del(&wait.list);
		mutex_unlock(&task_release_lock);

		#ifdef TRACE_TASK_SYSTEM_CALL
		TRACE_TASK(current, "do_wait_for_ts_release() after list_del\n");
		#endif
	}

out:
	return ret;
}

int  count_tasks_waiting_for_release(void)
{
	int task_count = 0;
	struct list_head *pos;

	mutex_lock(&task_release_lock);

	list_for_each(pos, &task_release_list) {
		task_count++;
	}

	mutex_unlock(&task_release_lock);


	return task_count;
}

static long  do_release_ts(lt_t start)
{
	long  task_count = 0;

	struct list_head	*pos, *safe;
	struct ts_release_wait	*wait;

	if (mutex_lock_interruptible(&task_release_lock)) {
		task_count = -ERESTARTSYS;
		goto out;
	}

	TRACE("<<<<<< synchronous task system release >>>>>>\n");
	sched_trace_sys_release(&start);
	litmus->synchronous_release_at(start);

	task_count = 0;
	list_for_each_safe(pos, safe, &task_release_list) {
		wait = (struct ts_release_wait*)
			list_entry(pos, struct ts_release_wait, list);

		task_count++;
		wait->ts_release_time = start;
		complete(&wait->completion);
	}

	/* clear stale list */
	INIT_LIST_HEAD(&task_release_list);

	mutex_unlock(&task_release_lock);

out:
	return task_count;
}


asmlinkage long  sys_wait_for_ts_release(void)
{
	long ret = -EPERM;
	struct task_struct *t = current;

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(t, "sys_wait_for_ts_release()\n");
	#endif

	if (is_realtime(t))
		ret = do_wait_for_ts_release();

	#ifdef TRACE_TASK_SYSTEM_CALL
	TRACE_TASK(t, "sys_wait_for_ts_release(), ret=%d\n", ret);
	#endif

	return ret;
}

#define ONE_MS 1000000ULL
#define ONE_SECOND (ONE_MS * 1000)

asmlinkage long sys_release_ts(lt_t __user *__when)
{
	long ret;
	lt_t start_time;
	lt_t now;

	/* FIXME: check capabilities... */

	ret = copy_from_user(&start_time, __when, sizeof(start_time));
	if (ret == 0) {
		now = litmus_clock();
		if (lt_before(start_time, now))
			start_time = now + ONE_SECOND;
		ret = do_release_ts(start_time);
	}

	return ret;
}
