/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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
#include "config.h"

#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#endif

#ifndef WIN32
#include <pthread.h>

typedef pthread_t        de265_thread;
typedef pthread_mutex_t  de265_mutex;
typedef pthread_cond_t   de265_cond;

#else // WIN32
#include <windows.h>
#include <win32cond.h>

typedef HANDLE              de265_thread;
typedef HANDLE              de265_mutex;
typedef win32_cond_t        de265_cond;
#endif  // WIN32

int  de265_thread_create(de265_thread* t, void *(*start_routine) (void *), void *arg);
void de265_thread_join(de265_thread t);
void de265_thread_destroy(de265_thread* t);
void de265_mutex_init(de265_mutex* m);
void de265_mutex_destroy(de265_mutex* m);
void de265_mutex_lock(de265_mutex* m);
void de265_mutex_unlock(de265_mutex* m);
void de265_cond_init(de265_cond* c);
void de265_cond_destroy(de265_cond* c);
void de265_cond_broadcast(de265_cond* c);
void de265_cond_wait(de265_cond* c,de265_mutex* m);
void de265_cond_signal(de265_cond* c);

enum thread_task_ctb_init_type { INIT_RESET, INIT_COPY, INIT_NONE };

struct thread_task_ctb
{
  int ctb_x, ctb_y;
  struct decoder_context* ctx;
  struct thread_context* tctx;
  struct slice_segment_header* shdr;

  enum thread_task_ctb_init_type CABAC_init;
};

enum thread_task_id {
  THREAD_TASK_SYNTAX_DECODE_CTB,
  THREAD_TASK_PIXEL_DECODE_CTB
  //THREAD_TASK_POSTPROC_CTB
};

typedef struct
{
  int task_id;
  enum thread_task_id task_cmd;

  void (*work_routine)(void* data);

  union {
    struct thread_task_ctb task_ctb;
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

  long tasks_pending;

  int ctbx[MAX_THREADS]; // the CTB the thread is working on
  int ctby[MAX_THREADS];

  de265_mutex  mutex;
  de265_cond   cond_var;
  de265_cond   finished_cond;
} thread_pool;


de265_error start_thread_pool(thread_pool* pool, int num_threads);
void        flush_thread_pool(thread_pool* pool);  // process pool until no more tasks
void        stop_thread_pool(thread_pool* pool); // do not process remaining tasks

void   add_task(thread_pool* pool, const thread_task* task);
void   decrement_tasks_pending(thread_pool* pool);

//bool   deblock_task(thread_pool* pool, int task_id); // returns false if task does not exist

#endif
