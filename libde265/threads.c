
#include "threads.h"
#include <assert.h>
#include <string.h>

#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#else
# include <alloca.h>
#endif


#ifndef _WIN32
// #include <intrin.h>

#define THREAD_RESULT       void*
#define THREAD_PARAM        void*

int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg) { return pthread_create(t,NULL,start_routine,arg); }
void de265_thread_join(de265_thread t) { pthread_join(t,NULL); }
void de265_thread_destroy(de265_thread* t) { }
void de265_mutex_init(de265_mutex* m) { pthread_mutex_init(m,NULL); }
void de265_mutex_destroy(de265_mutex* m) { pthread_mutex_destroy(m); }
void de265_mutex_lock(de265_mutex* m) { pthread_mutex_lock(m); }
void de265_mutex_unlock(de265_mutex* m) { pthread_mutex_unlock(m); }
void de265_cond_init(de265_cond* c) { pthread_cond_init(c,NULL); }
void de265_cond_destroy(de265_cond* c) { pthread_cond_destroy(c); }
void de265_cond_broadcast(de265_cond* c,de265_mutex* m) { pthread_cond_broadcast(c); }
void de265_cond_wait(de265_cond* c,de265_mutex* m) { pthread_cond_wait(c,m); }
void de265_cond_signal(de265_cond* c) { pthread_cond_signal(c); }
#else  // _WIN32

#define THREAD_RESULT       DWORD WINAPI
#define THREAD_PARAM        LPVOID

int  de265_thread_create(de265_thread* t, LPTHREAD_START_ROUTINE start_routine, void *arg) {
    HANDLE handle = CreateThread(NULL, 0, start_routine, arg, 0, NULL);
    if (handle == NULL) {
        return -1;
    }
    *t = handle;
    return 0;
}
void de265_thread_join(de265_thread t) { WaitForSingleObject(t, INFINITE); }
void de265_thread_destroy(de265_thread* t) { CloseHandle(*t); *t = NULL; }
void de265_mutex_init(de265_mutex* m) { *m = CreateMutex(NULL, FALSE, NULL); }
void de265_mutex_destroy(de265_mutex* m) { CloseHandle(*m); }
void de265_mutex_lock(de265_mutex* m) { WaitForSingleObject(*m, INFINITE); }
void de265_mutex_unlock(de265_mutex* m) { ReleaseMutex(*m); }
void de265_cond_init(de265_cond* c) { win32_cond_init(c); }
void de265_cond_destroy(de265_cond* c) { win32_cond_destroy(c); }
void de265_cond_broadcast(de265_cond* c,de265_mutex* m)
{
  de265_mutex_lock(m);
  win32_cond_broadcast(c);
  de265_mutex_unlock(m);
}
void de265_cond_wait(de265_cond* c,de265_mutex* m) { win32_cond_wait(c,m); }
void de265_cond_signal(de265_cond* c) { win32_cond_signal(c); }
#endif // _WIN32

#include "libde265/decctx.h"

const char* line="--------------------------------------------------";
void printblks(const thread_pool* pool)
{
  int w = pool->tasks[0].data.task_ctb.ctx->current_sps->PicWidthInCtbsY;
  int h = pool->tasks[0].data.task_ctb.ctx->current_sps->PicHeightInCtbsY;

  printf("active threads: %d  queue len: %d\n",pool->num_threads_working,pool->num_tasks);

  char *const p = (char *)alloca(w * h * sizeof(char));
  assert(p != NULL);
  memset(p,' ',w*h);

  for (int i=0;i<pool->num_tasks;i++) {
    int b = 0; //pool->tasks[i].num_blockers;
    int x = pool->tasks[i].data.task_ctb.ctb_x;
    int y = pool->tasks[i].data.task_ctb.ctb_y;
    p[y*w+x] = b+'0';
  }

  for (int i=0;i<pool->num_threads_working;i++) {
    int x = pool->ctbx[i];
    int y = pool->ctby[i];
    p[y*w+x] = '*';
  }

  printf("+%s+\n",line+50-w);
  for (int y=0;y<h;y++)
    {
      printf("|");
      for (int x=0;x<w;x++)
        {
          printf("%c",p[x+y*w]);
        }
      printf("|\n");
    }
  printf("+%s+\n",line+50-w);
}


static THREAD_RESULT worker_thread(THREAD_PARAM pool_ptr)
{
  thread_pool* pool = (thread_pool*)pool_ptr;


  de265_mutex_lock(&pool->mutex);

  while(true) {

    // wait until we can pick a task or until the pool has been stopped

    for (;;) {
      // end waiting if thread-pool has been stopped or we have a task to execute

      if (pool->stopped || pool->num_tasks>0) {
        break;
      }

      //printf("going idle\n");
      de265_cond_wait(&pool->cond_var, &pool->mutex);
    }

    // if the pool was shut down, end the execution

    if (pool->stopped) {
      de265_mutex_unlock(&pool->mutex);
      return NULL;
    }

    if (0)
      {
        printf("%03d [%d]: ",pool->num_tasks,pool->num_threads_working);

        for (int i=0;i<pool->num_tasks;i++) {
          printf("%d%c%d ",
                 pool->tasks[i].data.task_ctb.ctb_x,
                 i==0 ? 'X' :
                 '*',
                 pool->tasks[i].data.task_ctb.ctb_y);
        }

        printf("\n");
      }


    // get a task

    thread_task task = pool->tasks[0];
    pool->num_tasks--;

    if (pool->num_tasks>0) {
      if (1) {
        memmove(&pool->tasks[0],
                &pool->tasks[1],
                pool->num_tasks*sizeof(thread_task));
      }
      else {
        pool->tasks[0] = pool->tasks[pool->num_tasks];
      }
    }

    
    //pool->ctbx[pool->num_threads_working] = task.data.task_ctb.ctb_x;
    //pool->ctby[pool->num_threads_working] = task.data.task_ctb.ctb_y;

    //pool->num_threads_working++;

    //printblks(pool);

    de265_mutex_unlock(&pool->mutex);


    // execute the task

    if (task.work_routine != NULL) {
      task.work_routine( &task.data );
    }


    // end processing and check if this was the last task to be processed

    de265_mutex_lock(&pool->mutex);

    /* num-1 because: if at last pos, we do not need to shift */
    /*
    for (int i=0;i<pool->num_threads_working -1;i++)
      {
        if (pool->ctbx[i] == task.data.task_ctb.ctb_x &&
            pool->ctby[i] == task.data.task_ctb.ctb_y) {
          pool->ctbx[i] = pool->ctbx[pool->num_threads_working-1];
          pool->ctby[i] = pool->ctby[pool->num_threads_working-1];
          break;
        }
      }
    */

    //pool->num_threads_working--;
#ifndef _WIN32
    int pending = __sync_sub_and_fetch(&pool->tasks_pending, 1);
#else
    int pending = InterlockedDecrement((volatile long*)(&pool->tasks_pending));
#endif


    /*
      printf("%03d [%d]: ",pool->num_tasks,pool->num_threads_working);

      for (int i=0;i<pool->num_tasks;i++) {
      printf("%d%c%d ",
      pool->tasks[i].data.task_ctb.ctb_x,
      pool->tasks[i].num_blockers==0 ? '*':'.',
      pool->tasks[i].data.task_ctb.ctb_y);
      }

      printf("\n");
    */

    //printf("finish %d;%d\n",task.data.task_ctb.ctb_x, task.data.task_ctb.ctb_y);
    //printblks(pool);

    if (pending==0) {
      de265_cond_broadcast(&pool->finished_cond, &pool->mutex);
    }

    /*
      printf("processed task %d at CTB %d;%d\n", task.task_cmd,
      task.data.task_ctb.ctb_x,
      task.data.task_ctb.ctb_y);
    */

  }
  de265_mutex_unlock(&pool->mutex);

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
  while (pool->tasks_pending>0) {
    de265_cond_wait(&pool->finished_cond, &pool->mutex);
  }
  de265_mutex_unlock(&pool->mutex);

  /*
  printf("----------------------------------------------------------------------------------------------------\n");
  */
}


void stop_thread_pool(thread_pool* pool)
{
  de265_mutex_lock(&pool->mutex);
  pool->stopped = true;
  de265_mutex_unlock(&pool->mutex);

  de265_cond_broadcast(&pool->cond_var, &pool->mutex);

  for (int i=0;i<pool->num_threads;i++) {
    de265_thread_join(pool->thread[i]);
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

    de265_cond_signal(&pool->cond_var);
  }
  de265_mutex_unlock(&pool->mutex);
}


void   decrement_tasks_pending(thread_pool* pool)
{
  //de265_mutex_lock(&pool->mutex);
  //pool->tasks_pending--;
  //de265_mutex_unlock(&pool->mutex);

#ifndef _WIN32
    __sync_sub_and_fetch(&pool->tasks_pending, 1);
#else
    InterlockedDecrement((volatile long*)(&pool->tasks_pending));
#endif
}


/*
bool deblock_task(thread_pool* pool, int task_id)
{
  bool found=false;

  de265_mutex_lock(&pool->mutex);
  for (int i=0;i<pool->num_tasks;i++) {
    if (pool->tasks[i].task_id == task_id) {
      assert(pool->tasks[i].num_blockers > 0);

      // deblock task by one

      pool->tasks[i].num_blockers--;

      //printf("remaining blockers = %d\n",pool->tasks[i].num_blockers);

      // if task can be executed now, wake up one thread

      if (pool->tasks[i].num_blockers==0) {
        de265_cond_signal(&pool->cond_var);
      }

      found = true;
      break;
    }
  }
  de265_mutex_unlock(&pool->mutex);

  return found;
}
*/


