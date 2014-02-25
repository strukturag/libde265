#!/usr/bin/python
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
            self.pool = multiprocessing.Pool()
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
    
    processor = ProcessorClass(filenames)
    try:
        processor.run()
    except KeyboardInterrupt:
        processor.cancel()

if __name__ == '__main__':
    main()
