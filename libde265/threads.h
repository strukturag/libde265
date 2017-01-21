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

#ifdef HAVE_CONFIG_H
#include "config.h"  // NOLINT(build/include)
#endif

#include "libde265/de265.h"
#include "libde265/util.h"

#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#endif

#include <deque>
#include <string>
#include <memory>

#ifndef _WIN32
#  include <pthread.h>

#define THREAD_RESULT       void*
#define THREAD_PARAM        void*

typedef pthread_t        de265_thread_primitive;
typedef pthread_mutex_t  de265_mutex_primitive;
typedef pthread_cond_t   de265_cond_primitive;

#else // _WIN32
#  include <windows.h>
#  include "../extra/win32cond.h"
#  if _MSC_VER > 1310
#    include <intrin.h>
#  else
extern "C"
{
   LONG  __cdecl _InterlockedExchangeAdd(long volatile *Addend, LONG Value);
}
#    pragma intrinsic (_InterlockedExchangeAdd)
#    define InterlockedExchangeAdd _InterlockedExchangeAdd
#  endif

#define THREAD_RESULT       DWORD WINAPI
#define THREAD_PARAM        LPVOID

typedef HANDLE              de265_thread_primitive;
typedef HANDLE              de265_mutex_primitive;
typedef win32_cond_t        de265_cond_primitive;
#endif  // _WIN32

#ifndef _WIN32
int  de265_thread_create(de265_thread_primitive* t, void *(*start_routine) (void *), void *arg);
#else
int  de265_thread_create(de265_thread_primitive* t,LPTHREAD_START_ROUTINE start_routine, void *arg);
#endif
void de265_thread_join(de265_thread_primitive t);
void de265_thread_destroy(de265_thread_primitive* t);
void de265_mutex_init(de265_mutex_primitive* m);
void de265_mutex_destroy(de265_mutex_primitive* m);
void de265_mutex_lock(de265_mutex_primitive* m);
void de265_mutex_unlock(de265_mutex_primitive* m);
void de265_cond_init(de265_cond_primitive* c);
void de265_cond_destroy(de265_cond_primitive* c);
void de265_cond_broadcast(de265_cond_primitive* c, de265_mutex_primitive* m);
void de265_cond_wait(de265_cond_primitive* c,de265_mutex_primitive* m);
//bool de265_cond_timedwait(de265_cond_primitive* c,de265_mutex_primitive* m, int msecs);
void de265_cond_signal(de265_cond_primitive* c);

typedef volatile long de265_sync_int;

inline int de265_sync_sub_and_fetch(de265_sync_int* cnt, int n)
{
#ifdef _WIN64
  return _InterlockedAdd(cnt, -n);
#elif _WIN32
  return _InterlockedExchangeAdd(cnt, -n) - n;
#else
  return __sync_sub_and_fetch(cnt, n);
#endif
}

inline int de265_sync_add_and_fetch(de265_sync_int* cnt, int n)
{
#ifdef _WIN64
  return _InterlockedAdd(cnt, n);
#elif _WIN32
  return _InterlockedExchangeAdd(cnt, n) + n;
#else
  return __sync_add_and_fetch(cnt, n);
#endif
}


class de265_mutex
{
 public:
  de265_mutex() { de265_mutex_init(&m_mutex); }
  ~de265_mutex() { de265_mutex_destroy(&m_mutex); }

  void lock() { de265_mutex_lock(&m_mutex); }
  void unlock() { de265_mutex_unlock(&m_mutex); }

 private:
  de265_mutex_primitive m_mutex;

  friend class de265_cond;
};


class lock_guard
{
 public:
 lock_guard(de265_mutex& m) : m_mutex(m) { m_mutex.lock(); }
  ~lock_guard() { m_mutex.unlock(); }

 private:
  de265_mutex& m_mutex;
};


class de265_cond
{
 public:
  de265_cond() { de265_cond_init(&m_cond); }
  ~de265_cond() { de265_cond_destroy(&m_cond); }

  void broadcast(de265_mutex& mutex) { de265_cond_broadcast(&m_cond, &mutex.m_mutex); }
  void wait(de265_mutex& mutex) { de265_cond_wait(&m_cond, &mutex.m_mutex); }
  //bool timedwait(de265_mutex& mutex, int msecs) { return de265_cond_timedwait(&m_cond, &mutex.m_mutex, msecs); }
  void signal() { de265_cond_signal(&m_cond); }

 private:
  de265_cond_primitive m_cond;
};


class de265_easy_cond_class
{
 public:
  void lock_mutex() { m_mutex.lock(); }
  void unlock_mutex() { m_mutex.unlock(); }

  void wait() { m_cond.wait(m_mutex); }
  void signal() { m_cond.signal(); }
  void broadcast() { m_cond.broadcast(m_mutex); }

 private:
  de265_mutex m_mutex;
  de265_cond  m_cond;
};


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

  void set_name(const std::string& n) { name=n; }
  const char* get_name() const { return name.c_str(); }

private:
  bool initialized; // debug only
  bool destroyed; // debug only

  int mProgress;

  std::string name;

  // private data

  mutable de265_mutex mutex;
  mutable de265_cond  cond;
};



class de265_thread
{
 public:
  de265_thread();
  virtual ~de265_thread();

  void start();
  void stop();
  void join();
  bool running() const;

  virtual void run() = 0;

  bool should_stop() const;

 private:
  de265_thread_primitive m_thread;
  bool m_running;
  bool m_stop_request;

  mutable de265_mutex m_mutex;

  static THREAD_RESULT start_thread_main(THREAD_PARAM me);
};

typedef std::shared_ptr<de265_thread> de265_thread_ptr;


class thread_task
{
public:
 thread_task() : m_finished(false) {
    //printf("-------------------- %p %p\n", &m_mutex, this);
  }
  virtual ~thread_task() { }

  virtual void work() = 0;

  bool finished() const {
    //printf("finished / lock\n");
    m_mutex.lock();
    bool finished = m_finished;
    m_mutex.unlock();
    //printf("finished / unlock\n");

    return finished;
  }
  void wait_until_finished() const;

  virtual std::string name() const { return "noname"; }
  virtual void debug_dump() const { }

 private:
  mutable de265_mutex m_mutex;
  mutable de265_cond  m_cond_finished;

  bool m_finished;

  void mark_finished(); // only called by thread_pool

  friend class thread_pool;
};


typedef std::shared_ptr<thread_task> thread_task_ptr;


#define MAX_THREADS 32

/* TODO NOTE: When unblocking a task, we have to check first
   if there are threads waiting because of the run-count limit.
   If there are higher-priority tasks, those should be run instead
   of the just unblocked task.
 */

/* TODO: blocked threads should not count towards the max thread limit
 */
class thread_pool
{
 public:
  thread_pool() { m_stopped=true; }

  de265_error start(int num_threads);
  void stop();

  void reset(); // remove all pending tasks

  void add_task(thread_task_ptr task);

  void debug_list_tasks() const;

 private:
  bool m_stopped;

  std::deque<thread_task_ptr> m_tasks;

  de265_thread_primitive m_thread[MAX_THREADS];
  int m_num_threads;

  int m_num_threads_working;

  de265_mutex  m_mutex;
  de265_cond   m_cond_var;

  static THREAD_RESULT main_loop_thread(THREAD_PARAM pool_ptr);

  // main loop for each worker thread
  void worker_thread_main_loop();
};

#endif
