
#include "threads.h"
#include <assert.h>
#include <string.h>


int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg) { return pthread_create(t,NULL,start_routine,arg); }
void de265_thread_join(de265_thread* t) { pthread_join(*t,NULL); }
void de265_thread_destroy(de265_thread* t) { }
void de265_mutex_init(de265_mutex* m) { pthread_mutex_init(m,NULL); }
void de265_mutex_destroy(de265_mutex* m) { pthread_mutex_destroy(m); }
void de265_mutex_lock(de265_mutex* m) { pthread_mutex_lock(m); }
void de265_mutex_unlock(de265_mutex* m) { pthread_mutex_unlock(m); }
void de265_cond_init(de265_cond* c) { pthread_cond_init(c,NULL); }
void de265_cond_destroy(de265_cond* c) { pthread_cond_destroy(c); }
void de265_cond_broadcast(de265_cond* c) { pthread_cond_broadcast(c); }
void de265_cond_wait(de265_cond* c,de265_mutex* m) { pthread_cond_wait(c,m); }
void de265_cond_signal(de265_cond* c) { pthread_cond_signal(c); }


static void* worker_thread(void* pool_ptr)
{
  thread_pool* pool = (thread_pool*)pool_ptr;

  while(true) {
    de265_mutex_lock(&pool->mutex);

    // wait until we can pick a task or until the pool has been stopped

    int available_task;

    for (;;) {
      // find an unblocked task

      available_task = -1;
      for (int i=0;i<pool->num_tasks;i++) {
        if (pool->tasks[i].num_blockers==0) {
          available_task=i;
          break;
        }
      }


      // end waiting if thread-pool has been stopped or we have a task to execute

      if (pool->stopped || available_task>=0) {
        break;
      }

      de265_cond_wait(&pool->cond_var, &pool->mutex);
    }

    // if the pool was shut down, end the execution

    if (pool->stopped) {
      de265_mutex_unlock(&pool->mutex);
      return NULL;
    }


    // get a task

    thread_task task = pool->tasks[available_task];
    pool->num_tasks--;

    if (available_task<pool->num_tasks) {
      memmove(&pool->tasks[available_task],
              &pool->tasks[available_task+1],
              (pool->num_tasks-available_task)*sizeof(thread_task));
    }

    pool->num_threads_working++;

    de265_mutex_unlock(&pool->mutex);


    // execute the task

    if (task.work_routine != NULL) {
      task.work_routine( &task.data );
    }


    // end processing and check if this was the last task to be processed

    de265_mutex_lock(&pool->mutex);

    pool->num_threads_working--;

    if (pool->num_tasks==0 && pool->num_threads_working==0) {
      de265_cond_broadcast(&pool->finished_cond);
    }

    printf("processed task %d at CTB %d;%d\n", task.task_cmd,
           task.data.task_ctb.ctb_x,
           task.data.task_ctb.ctb_y);

    de265_mutex_unlock(&pool->mutex);
  }
  return NULL;
}


de265_error start_thread_pool(thread_pool* pool, int num_threads)
{
  assert(num_threads <= MAX_THREADS);

  pool->num_tasks = 0;
  pool->num_threads = 0; // will be increased below
  pool->num_threads_working = 0;
  pool->stopped = false;

  de265_mutex_init(&pool->mutex);
  de265_cond_init(&pool->cond_var);
  de265_cond_init(&pool->finished_cond);

  // start worker threads

  for (int i=0; i<num_threads; i++) {
    int ret = de265_thread_create(&pool->thread[i], worker_thread, pool);
    if (ret != 0) {
      // cerr << "pthread_create() failed: " << ret << endl;
      return DE265_ERROR_CANNOT_START_THREADPOOL;
    }

    pool->num_threads++;
  }

  return DE265_OK;
}


void flush_thread_pool(thread_pool* pool)
{
  de265_mutex_lock(&pool->mutex);
  if (pool->num_tasks>0) {
    de265_cond_wait(&pool->finished_cond, &pool->mutex);
  }
  de265_mutex_unlock(&pool->mutex);
}


void stop_thread_pool(thread_pool* pool)
{
  de265_mutex_lock(&pool->mutex);
  pool->stopped = true;
  de265_mutex_unlock(&pool->mutex);

  de265_cond_broadcast(&pool->cond_var);

  for (int i=0;i<pool->num_threads;i++) {
    de265_thread_join(&pool->thread[i]);
    de265_thread_destroy(&pool->thread[i]);
  }

  de265_mutex_destroy(&pool->mutex);
  de265_cond_destroy(&pool->cond_var);
  de265_cond_destroy(&pool->finished_cond);
}


void   add_task(thread_pool* pool, const thread_task* task)
{
  de265_mutex_lock(&pool->mutex);
  if (!pool->stopped) {
    assert(pool->num_tasks < MAX_THREAD_TASKS);
    pool->tasks[pool->num_tasks] = *task;
    pool->num_tasks++;

    // wake up one thread

    if (task->num_blockers==0) {
      de265_cond_signal(&pool->cond_var);
    }
  }
  de265_mutex_unlock(&pool->mutex);
}


void   deblock_task(thread_pool* pool, int task_id)
{
  de265_mutex_lock(&pool->mutex);
  for (int i=0;i<pool->num_tasks;i++) {
    if (pool->tasks[i].task_id == task_id) {
      assert(pool->tasks[i].num_blockers > 0);

      // deblock task by one

      pool->tasks[i].num_blockers--;

      // if task can be executed now, wake up one thread

      if (pool->tasks[i].num_blockers==0) {
        de265_cond_signal(&pool->cond_var);
      }
      break;
    }
  }
  de265_mutex_unlock(&pool->mutex);
}

