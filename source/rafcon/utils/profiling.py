import os
import sys
import time

if sys.version_info >= (3, ):
    import tracemalloc


def memory_profiling(args):
    memory_profiling_path = args['memory_profiling_path']
    memory_profiling_interval = int(args['memory_profiling_interval'])
    memory_profiling_print = args['memory_profiling_print']
    while not args['stop']:
        snapshot = tracemalloc.take_snapshot()
        time.sleep(memory_profiling_interval)
        with open(os.path.join(memory_profiling_path, 'memory_profiling.log'), 'a') as file:
            file.write('[ Top 10 Differences ]\n')
            if memory_profiling_print:
                print('[ Top 10 Differences ]\n')
            for result in tracemalloc.take_snapshot().compare_to(snapshot, 'lineno')[:10]:
                file.write(str(result) + '\n')
                if memory_profiling_print:
                    print(result)
            file.write('\n\n\n')
