/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "threads.h"
#include <assert.h>
#include <string.h>

#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#elif defined(HAVE_ALLOCA_H)
# include <alloca.h>
#endif

#include <stdio.h>


#ifndef _WIN32
// #include <intrin.h>

int  de265_thread_create(de265_thread_primitive* t, void *(*start_routine) (void *), void *arg)
{ return pthread_create(t,NULL,start_routine,arg); }
void de265_thread_join(de265_thread_primitive t) { pthread_join(t,NULL); }
void de265_thread_destroy(de265_thread_primitive* t) { }
void de265_mutex_init(de265_mutex_primitive* m) {
  pthread_mutexattr_t attr;

  pthread_mutexattr_init(&attr);
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  pthread_mutex_init(m,&attr);
}
void de265_mutex_destroy(de265_mutex_primitive* m) { pthread_mutex_destroy(m); }
void de265_mutex_lock(de265_mutex_primitive* m) { pthread_mutex_lock(m); }
void de265_mutex_unlock(de265_mutex_primitive* m) { pthread_mutex_unlock(m); }
void de265_cond_init(de265_cond_primitive* c) { pthread_cond_init(c,NULL); }
void de265_cond_destroy(de265_cond_primitive* c) { pthread_cond_destroy(c); }
void de265_cond_broadcast(de265_cond_primitive* c,de265_mutex_primitive* m)
{ pthread_cond_broadcast(c); }
void de265_cond_wait(de265_cond_primitive* c,de265_mutex_primitive* m) { pthread_cond_wait(c,m); }
void de265_cond_signal(de265_cond_primitive* c) { pthread_cond_signal(c); }
#else  // _WIN32

int  de265_thread_create(de265_thread_primitive* t, LPTHREAD_START_ROUTINE start_routine, void *arg) {
    HANDLE handle = CreateThread(NULL, 0, start_routine, arg, 0, NULL);
    if (handle == NULL) {
        return -1;
    }
    *t = handle;
    return 0;
}
void de265_thread_join(de265_thread_primitive t) { WaitForSingleObject(t, INFINITE); }
void de265_thread_destroy(de265_thread_primitive* t) { CloseHandle(*t); *t = NULL; }
void de265_mutex_init(de265_mutex_primitive* m) { *m = CreateMutex(NULL, FALSE, NULL); }
void de265_mutex_destroy(de265_mutex_primitive* m) { CloseHandle(*m); }
void de265_mutex_lock(de265_mutex_primitive* m) { WaitForSingleObject(*m, INFINITE); }
void de265_mutex_unlock(de265_mutex_primitive* m) { ReleaseMutex(*m); }
void de265_cond_init(de265_cond_primitive* c) { win32_cond_init(c); }
void de265_cond_destroy(de265_cond_primitive* c) { win32_cond_destroy(c); }
void de265_cond_broadcast(de265_cond_primitive* c,de265_mutex_primitive* m)
{
  // TODO: why do we lock the mutex here? The mutex should already be locked outside...
  de265_mutex_lock(m);
  win32_cond_broadcast(c);
  de265_mutex_unlock(m);
}
void de265_cond_wait(de265_cond_primitive* c,de265_mutex_primitive* m) { win32_cond_wait(c,m); }
void de265_cond_signal(de265_cond_primitive* c) { win32_cond_signal(c); }
#endif // _WIN32




de265_thread::de265_thread()
{
  m_running = false;
}


de265_thread::~de265_thread()
{
}

void de265_thread::start()
{
  assert(!m_running);

  m_stop_request = false;
  de265_thread_create(&m_thread, start_thread_main, this);
  m_running = true;
}

void de265_thread::stop()
{
  m_mutex.lock();
  m_stop_request = true;
  m_mutex.unlock();
}

void de265_thread::join()
{
  assert(m_running);
  de265_thread_join(m_thread);
  de265_thread_destroy(&m_thread);
  m_running = false;
}

bool de265_thread::should_stop() const
{
  bool req;

  {
    //printf("thread LOCK\n");
    lock_guard lock(m_mutex);
    req = m_stop_request;
    //printf("thread UNLOCK\n");
  }

  return req;
}

bool de265_thread::running() const
{
  return m_running;
}

THREAD_RESULT de265_thread::start_thread_main(THREAD_PARAM me)
{
  de265_thread* thread = (de265_thread*)me;
  thread->run();
  return 0;
}

//virtual void run() = 0;


de265_progress_lock::de265_progress_lock()
{
  initialized = true;
  destroyed = false;
  mProgress = 0;
}

de265_progress_lock::~de265_progress_lock()
{
  initialized = false;
  destroyed = true;
}

void de265_progress_lock::wait_for_progress(int progress)
{
  if (mProgress >= progress) {
    return;
  }

  mutex.lock();
  while (mProgress < progress) {
#if D_MT
    printf("waiting for %s, progress %d<%d\n",
           get_name(), mProgress, progress);
#endif
    cond.wait(mutex);
  }

#if D_MT
  printf("continuing for %s, progress %d>=%d\n",
         get_name(), mProgress, progress);
#endif

  mutex.unlock();
}

void de265_progress_lock::set_progress(int progress)
{
  assert(initialized);

  mutex.lock();

  if (progress>mProgress) {
    //printf("set progress %d\n",progress);

    mProgress = progress;

    cond.broadcast(mutex);
  }

  mutex.unlock();
}

void de265_progress_lock::increase_progress(int progress)
{
  assert(initialized);

  mutex.lock();

  mProgress += progress;
  cond.broadcast(mutex);

  mutex.unlock();
}

int  de265_progress_lock::get_progress() const
{
  if (!initialized) {
    printf("destroyed: %d\n", destroyed);
  }
  assert(initialized);

  mutex.lock(); // need lock, otherwise helgrind complains
  int progress = mProgress;
  mutex.unlock();

  return progress;
}



void thread_task::wait_until_finished() const
{
  m_mutex.lock();
  while (!m_finished) {
    m_cond_finished.wait(m_mutex);
  }
  m_mutex.unlock();
}


void thread_task::mark_finished()
{
  m_mutex.lock();
  m_finished = true;
  m_cond_finished.broadcast(m_mutex);
  m_mutex.unlock();
}



#include "libde265/decctx.h"

#if 0
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
#endif


THREAD_RESULT thread_pool::main_loop_thread(THREAD_PARAM pool_ptr)
{
  thread_pool* pool = (thread_pool*)pool_ptr;

  pool->worker_thread_main_loop();

  return (THREAD_RESULT)NULL;
}



void thread_pool::worker_thread_main_loop()
{
  m_mutex.lock();

  while(true) {

    // wait until we can pick a task or until the pool has been stopped

    for (;;) {
      // end waiting if thread-pool has been stopped or we have a task to execute

      if (m_stopped || m_tasks.size()>0) {
        break;
      }

      //printf("going idle\n");
      m_cond_var.wait(m_mutex);
    }

    // if the pool was shut down, end the execution

    if (m_stopped) {
      m_mutex.unlock();
      return;
    }


    // get a task

    thread_task_ptr task = m_tasks.front();
    m_tasks.pop_front();

    m_num_threads_working++;

    //printblks(pool);

    m_mutex.unlock();


    // execute the task

    //printf("start task: %s\n",task->name().c_str());
    task->work();
    task->mark_finished();
    //printf("end task: %s\n",task->name().c_str());

    // end processing and check if this was the last task to be processed

    m_mutex.lock();

    m_num_threads_working--;
  }

  m_mutex.unlock();
}


de265_error thread_pool::start(int num_threads)
{
  de265_error err = DE265_OK;

  // limit number of threads to maximum

  if (num_threads > MAX_THREADS) {
    num_threads = MAX_THREADS;
    err = DE265_WARNING_NUMBER_OF_THREADS_LIMITED_TO_MAXIMUM;
  }

  m_num_threads = 0; // will be increased below

  m_mutex.lock();
  m_num_threads_working = 0;
  m_stopped = false;
  m_mutex.unlock();

  // start worker threads

  for (int i=0; i<num_threads; i++) {
    int ret = de265_thread_create(&m_thread[i],
                                  main_loop_thread,
                                  this);
    if (ret != 0) {
      // cerr << "pthread_create() failed: " << ret << endl;
      return DE265_ERROR_CANNOT_START_THREADPOOL;
    }

    m_num_threads++;
  }

  return err;
}


void thread_pool::stop()
{
  m_mutex.lock();
  m_stopped = true;

  m_cond_var.broadcast(m_mutex);
  m_mutex.unlock();

  for (int i=0;i<m_num_threads;i++) {
    de265_thread_join(m_thread[i]);
    de265_thread_destroy(&m_thread[i]);
  }
}


void thread_pool::add_task(thread_task_ptr task)
{
  m_mutex.lock();

  //assert(!m_stopped);

  if (true) { // !m_stopped) {

    m_tasks.push_back(task);

    // wake up one thread

    m_cond_var.signal();
  }
  m_mutex.unlock();
}

extern inline int de265_sync_sub_and_fetch(de265_sync_int* cnt, int n);
extern inline int de265_sync_add_and_fetch(de265_sync_int* cnt, int n);
