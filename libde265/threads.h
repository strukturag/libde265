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

#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#endif

#ifndef _WIN32
#include <pthread.h>

typedef pthread_t        de265_thread;
typedef pthread_mutex_t  de265_mutex;
typedef pthread_cond_t   de265_cond;

#else // _WIN32
#include <windows.h>
#include "../extra/win32cond.h"
#include <intrin.h>

typedef HANDLE              de265_thread;
typedef HANDLE              de265_mutex;
typedef win32_cond_t        de265_cond;
#endif  // _WIN32

#ifndef _WIN32
int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg);
#else
int  de265_thread_create(de265_thread* t, LPTHREAD_START_ROUTINE start_routine, void *arg);
#endif
void de265_thread_join(de265_thread t);
void de265_thread_destroy(de265_thread* t);
void de265_mutex_init(de265_mutex* m);
void de265_mutex_destroy(de265_mutex* m);
void de265_mutex_lock(de265_mutex* m);
void de265_mutex_unlock(de265_mutex* m);
void de265_cond_init(de265_cond* c);
void de265_cond_destroy(de265_cond* c);
void de265_cond_broadcast(de265_cond* c, de265_mutex* m);
void de265_cond_wait(de265_cond* c,de265_mutex* m);
void de265_cond_signal(de265_cond* c);

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


typedef struct de265_progress_lock
{
  int progress;

  // private data

  de265_mutex mutex;
  de265_cond  cond;
} de265_progress_lock;

void de265_progress_lock_init(de265_progress_lock* lock);
void de265_progress_lock_destroy(de265_progress_lock* lock);
int  de265_wait_for_progress(de265_progress_lock* lock, int progress);
void de265_announce_progress(de265_progress_lock* lock, int progress);



enum thread_task_ctb_init_type { INIT_RESET, INIT_COPY, INIT_NONE };

struct thread_task_ctb
{
  int ctb_x, ctb_y;
  struct decoder_context* ctx;
  struct thread_context* tctx;
  struct slice_segment_header* shdr;

  enum thread_task_ctb_init_type CABAC_init;
};

struct thread_task_ctb_row
{
  int thread_context_id;
  bool initCABAC;
  struct decoder_context* ctx;
};

struct thread_task_deblock
{
  struct de265_image* img;
  int first;  // stripe row
  int last;
  int ctb_x,ctb_y;
  bool vertical;
};

enum thread_task_id {
  THREAD_TASK_SYNTAX_DECODE_CTB,
  THREAD_TASK_DEBLOCK,
  THREAD_TASK_DECODE_CTB_ROW,
  THREAD_TASK_DECODE_SLICE_SEGMENT,
  //THREAD_TASK_PIXEL_DECODE_CTB,
  //THREAD_TASK_POSTPROC_CTB
};

typedef struct
{
  int task_id;
  enum thread_task_id task_cmd;

  void (*work_routine)(void* data);

  union {
    struct thread_task_ctb task_ctb;
    struct thread_task_ctb_row task_ctb_row;
    struct thread_task_deblock task_deblock;
  } data;
} thread_task;


#define MAX_THREAD_TASKS 1024
#define MAX_THREADS 32

typedef struct thread_pool
{
  bool stopped;

  thread_task tasks[MAX_THREAD_TASKS];
  int num_tasks;

  de265_thread thread[MAX_THREADS];
  int num_threads;

  int num_threads_working;
  //long tasks_pending;

  int ctbx[MAX_THREADS]; // the CTB the thread is working on
  int ctby[MAX_THREADS];

  de265_mutex  mutex;
  de265_cond   cond_var;
} thread_pool;


de265_error start_thread_pool(thread_pool* pool, int num_threads);
void        stop_thread_pool(thread_pool* pool); // do not process remaining tasks

void        add_task(thread_pool* pool, const thread_task* task);

#endif
