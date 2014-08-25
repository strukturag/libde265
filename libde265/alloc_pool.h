/*
 * H.265 video codec.
 * Copyright (c) 2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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

#ifndef ALLOC_POOL_H
#define ALLOC_POOL_H



template <class T> class alloc_pool
{
public:
  alloc_pool() {
    mBlkSize = BLKSIZE;
  }

  ~alloc_pool() {
    for (int i=0;i<mem.size();i++) {
      delete[] mem[i].data;
    }
  }


  void set_blk_size(int blkSize) { mBlkSize=blkSize; }

  void free_all() {
    for (int i=0;i<mem.size();i++) {
      mem[i].nUsed=0;
    }

    freelist.clear();
  }

  T* get_new(int n=1) {
    if (n==1 && !freelist.empty()) {
      T* t = freelist.back();
      freelist.pop_back();
      return t;
    }

    if (mem.empty() || mem.back().nUsed + n > mem.back().size) {
      range r;
      r.data = new T[mBlkSize];
      r.size = mBlkSize;
      r.nUsed= 0;
      mem.push_back(r);
    }

    range& r = mem.back();

    assert(r.nUsed + n <= r.size);

    T* t = r.data + r.nUsed;
    r.nUsed += n;

    return t;
  }

  void free(T* t) {
    freelist.push_back(t);
  }

 private:
  static const int BLKSIZE = 128;
  int mBlkSize;

  struct range {
    T* data;
    int size;
    int nUsed;
  };

  std::vector<range> mem;
  std::vector<T*> freelist;
};

#endif
