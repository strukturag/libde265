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


#ifndef _WIN32
// #include <intrin.h>

#define THREAD_RESULT_TYPE  void*
#define THREAD_PARAM_TYPE   void*
#define THREAD_CALLING_CONVENTION

#include <stdio.h>

int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg) { return pthread_create(t,NULL,start_routine,arg); }
void de265_thread_join(de265_thread t) { pthread_join(t,NULL); }
void de265_thread_destroy(de265_thread* t) { }
#else  // _WIN32

#define THREAD_RESULT_TYPE    DWORD
#define THREAD_CALLING_CONVENTION WINAPI
#define THREAD_PARAM_TYPE        LPVOID

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
#endif // _WIN32


de265_progress_lock::de265_progress_lock()
{
  mProgress = 0;
}

de265_progress_lock::~de265_progress_lock()
{
}

void de265_progress_lock::wait_for_progress(int progress)
{
  if (mProgress >= progress) {
    return;
  }

  std::unique_lock<std::mutex> lock(mutex);
  while (mProgress < progress) {
    cond.wait(lock);
  }
}

void de265_progress_lock::set_progress(int progress)
{
  std::unique_lock<std::mutex> lock(mutex);

  if (progress>mProgress) {
    mProgress = progress;

    cond.notify_all();
  }
}

void de265_progress_lock::increase_progress(int progress)
{
  std::unique_lock<std::mutex> lock(mutex);

  mProgress += progress;
  cond.notify_all();
}

int  de265_progress_lock::get_progress() const
{
  return mProgress;
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


static THREAD_RESULT_TYPE THREAD_CALLING_CONVENTION worker_thread(THREAD_PARAM_TYPE pool_ptr)
{
  thread_pool* pool = (thread_pool*)pool_ptr;


  while(true) {

    thread_task* task = nullptr;

    {
      std::unique_lock<std::mutex> lock(pool->mutex);

      // wait until we can pick a task or until the pool has been stopped

      for (;;) {
        // end waiting if thread-pool has been stopped or we have a task to execute

        if (pool->stopped || pool->tasks.size()>0) {
          break;
        }

        //printf("going idle\n");
        pool->cond_var.wait(lock);
      }

      // if the pool was shut down, end the execution

      if (pool->stopped) {
        return (THREAD_RESULT_TYPE)0;
      }


      // get a task

      task = pool->tasks.front();
      pool->tasks.pop_front();

      pool->num_threads_working++;

      //printblks(pool);
    }

    // execute the task

    task->work();

    // end processing and check if this was the last task to be processed

    // TODO: the num_threads_working can probably be an atomic integer
    std::unique_lock<std::mutex> lock(pool->mutex);

    pool->num_threads_working--;
  }

  return (THREAD_RESULT_TYPE)0;
}


de265_error start_thread_pool(thread_pool* pool, int num_threads)
{
  de265_error err = DE265_OK;

  // limit number of threads to maximum

  if (num_threads > MAX_THREADS) {
    num_threads = MAX_THREADS;
    err = DE265_WARNING_NUMBER_OF_THREADS_LIMITED_TO_MAXIMUM;
  }

  pool->num_threads = 0; // will be increased below

  {
    std::unique_lock<std::mutex> lock(pool->mutex);

    pool->num_threads_working = 0;
    pool->stopped = false;
  }

  // start worker threads

  for (int i=0; i<num_threads; i++) {
    int ret = de265_thread_create(&pool->thread[i], worker_thread, pool);
    if (ret != 0) {
      // cerr << "pthread_create() failed: " << ret << endl;
      return DE265_ERROR_CANNOT_START_THREADPOOL;
    }

    pool->num_threads++;
  }

  return err;
}


void stop_thread_pool(thread_pool* pool)
{
  {
    std::unique_lock<std::mutex> lock(pool->mutex);
    pool->stopped = true;
  }

  pool->cond_var.notify_all();

  for (int i=0;i<pool->num_threads;i++) {
    de265_thread_join(pool->thread[i]);
    de265_thread_destroy(&pool->thread[i]);
  }
}


void   add_task(thread_pool* pool, thread_task* task)
{
  std::unique_lock<std::mutex> lock(pool->mutex);

  if (!pool->stopped) {

    pool->tasks.push_back(task);

    // wake up one thread

    pool->cond_var.notify_one();
  }
}
