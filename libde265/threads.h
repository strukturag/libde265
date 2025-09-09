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

#ifndef DE265_THREADS_H
#define DE265_THREADS_H

#include "libde265/de265.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <deque>
#include <string>
#include <atomic>

#ifndef _WIN32
#include <pthread.h>

typedef pthread_t        de265_thread;

#else // _WIN32
#if !defined(NOMINMAX)
#define NOMINMAX 1
#endif
#include <windows.h>
#include "../extra/win32cond.h"
#if _MSC_VER > 1310
#include <intrin.h>
#endif

typedef HANDLE              de265_thread;
#endif  // _WIN32

#include <mutex>
#include <condition_variable>
//#include <thread>

//typedef pthread_t        de265_thread;

#ifndef _WIN32
int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg);
#else
int  de265_thread_create(de265_thread* t, LPTHREAD_START_ROUTINE start_routine, void *arg);
#endif
void de265_thread_join(de265_thread t);
void de265_thread_destroy(de265_thread* t);

class de265_progress_lock
{
public:
  de265_progress_lock();
  ~de265_progress_lock();

  void wait_for_progress(int progress);
  void set_progress(int progress);
  void increase_progress(int progress);
  int  get_progress() const;
  void reset(int value=0) { mProgress=value; }

private:
  int mProgress;

  // private data

  std::mutex mutex;
  std::condition_variable cond;
};



class thread_task
{
public:
  thread_task() : state(Queued) { }
  virtual ~thread_task() { }

  enum { Queued, Running, Blocked, Finished } state;

  virtual void work() = 0;

  virtual std::string name() const { return "noname"; }
};


#define MAX_THREADS 32

/* TODO NOTE: When unblocking a task, we have to check first
   if there are threads waiting because of the run-count limit.
   If there are higher-priority tasks, those should be run instead
   of the just unblocked task.
 */

class thread_pool
{
 public:
  bool stopped;

  std::deque<thread_task*> tasks;  // we are not the owner

  de265_thread thread[MAX_THREADS];
  int num_threads;

  int num_threads_working;

  int ctbx[MAX_THREADS]; // the CTB the thread is working on
  int ctby[MAX_THREADS];

  std::mutex  mutex;
  std::condition_variable  cond_var;
};


de265_error start_thread_pool(thread_pool* pool, int num_threads);
void        stop_thread_pool(thread_pool* pool); // do not process remaining tasks

void        add_task(thread_pool* pool, thread_task* task); // TOCO: can make thread_task const

#endif
