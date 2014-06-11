#!/usr/bin/python
"""
H.265 video codec.
Copyright (c) 2014 struktur AG, Joachim Bauch <bauch@struktur.de>

This file is part of libde265.

libde265 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libde265 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with libde265.  If not, see <http://www.gnu.org/licenses/>.
"""
import glob
try:
    import multiprocessing
except ImportError:
    multiprocessing = None
import os
import subprocess
import sys

CPU_COUNT = multiprocessing is not None and multiprocessing.cpu_count() or 2
if CPU_COUNT > 2:
    THREAD_COUNT = str(CPU_COUNT / 2)
else:
    THREAD_COUNT = '2'

PROCESS_COUNT = min(4, CPU_COUNT)

DEFAULT_ROOT = '/var/lib/libde265-teststreams'

def decode_file(filename):
    print filename
    cmd = ['./dec265/dec265', '-q', '-c', '-t', THREAD_COUNT, filename]
    res = subprocess.call(cmd)
    assert res == 0, cmd

class BaseProcessor(object):

    def __init__(self, filenames):
        self.filenames = filenames

    def process(self, filename):
        decode_file(filename)

    def run(self):
        for filename in sorted(self.filenames):
            self.process(filename)

    def cancel(self):
        pass

if multiprocessing is not None:

    class MultiprocessingProcessor(BaseProcessor):

        def __init__(self, *args, **kw):
            super(MultiprocessingProcessor, self).__init__(*args, **kw)
            self.pool = multiprocessing.Pool(PROCESS_COUNT)
            self.pending_jobs = []

        def process(self, *args, **kw):
            job = self.pool.apply_async(decode_file, args, kw)
            self.pending_jobs.append(job)
            self._check_pending_jobs()

        def _check_pending_jobs(self):
            try:
                job = self.pending_jobs[0]
            except IndexError:
                return
            
            try:
                result = job.get(timeout=0)
            except multiprocessing.TimeoutError:
                # result not available yet
                return
            except:
                self.pending_jobs.pop(0)
                raise
            else:
                self.pending_jobs.pop(0)

        def run(self):
            try:
                super(MultiprocessingProcessor, self).run()
            except KeyboardInterrupt:
                self.pool.terminate()
            else: 
                self.pool.close()
                while self.pending_jobs:
                    self._check_pending_jobs()
            self.pool.join()

    def cancel(self):
        super(MultiprocessingProcessor, self).cancel()
        self.pool.terminate()
        self.pool.join()

    ProcessorClass = MultiprocessingProcessor
else:
    ProcessorClass = BaseProcessor

def main():
    if len(sys.argv) > 1:
        root = sys.argv[1]
        if not os.path.isdir(root):
            root = DEFAULT_ROOT
    else:
        root = DEFAULT_ROOT
    
    filenames = glob.glob(os.path.join(root, '*.bin'))
    print 'Processing %d streams in %s' % (len(filenames), root)
    print 'Using %d processes with %s threads each' % (PROCESS_COUNT, THREAD_COUNT)
    
    processor = ProcessorClass(filenames)
    try:
        processor.run()
    except KeyboardInterrupt:
        processor.cancel()

if __name__ == '__main__':
    main()
