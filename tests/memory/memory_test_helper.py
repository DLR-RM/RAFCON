# Python Lib Imports
import json
import threading
import time
import pytest
import sys
import os
from os.path import join

# IMPORTANT: Do not import any rafcon specific modules at the top of a test file
# if pytest collects the tests it will import those modules
# by importing it will already create objects
# these created objects will then create problems in multi-threaded gtkmvc Observers
# as for those Observers it is relevant which thread created which objects!

# Memory Profiling Tool Imports
import gc


class MemoryTestHelper:
    def __init__(self, key, iterations=100):
        """
        Initialize rafcon memory test class

        Parameters:
        key (string): To specify which state machine is going to be tested
        number_iterations: How many times the state machine will be run

       """
        self.key = key
        self.number_iterations = iterations

        memory_test_dir = os.path.dirname(os.path.abspath(__file__))
        state_machines_dir = join(memory_test_dir, "state_machines")

        self.paths_to_be_tested = {
            "sequential": join(state_machines_dir, "99_bottles_of_beer"),
            "concurrent": join(state_machines_dir, "barrier_concurrency_state_machine"),
            "big_data": join(state_machines_dir, "state_machine_passing_big_date_via_data_flows"),
        }

        self.total_memory = []
        self.uncollected_object_count = 0

    @pytest.mark.skipif(sys.version_info < (3, 4), reason="requires python3.4 or higher")
    def _display_top(self, snapshot, key_type='lineno', limit=10):
        """
        Displays the top lines of code using the most memory and total memory usage at the time of the memory snapshot
        Can be used to optimize memory usage of the program by indicating where most of it happens

        Parameters:
        snapshot (snapshot): tracemalloc memory snapshot object
        key_type (str):
        limit (int): Maximum number of lines of code to be displayed

       """
        import tracemalloc
        # filter_traces does the following:
        # Create a new snapshot instance with a filtered traces sequence,
        # filters is a list of DomainFilter and Filter instances.
        # If filters is an empty list, return a new Snapshot instance with a copy of the traces.
        snapshot = self._filter_snapshot(snapshot)
        top_stats = snapshot.statistics(key_type)
        for stat in top_stats[:limit]:
            print(stat)
        print(
            "------------------------------------------------------------------------------------")
        total = sum(stat.size for stat in top_stats)
        print("Total allocated size: %.1f KiB" % (total / 1024))

    def _run_iteration(self, sm):
        """
        Start state machine in rafcon core

        Parameters:
        sm (state_machine): rafcon state machine

       """
        from rafcon.core import start as core_start
        core_start.start_state_machine(sm)
        # if wait_for_state_machine_finished is executed too quickly
        # then it returns true without the state machine being executed yet
        # thus, wait until state machine is started properly
        # => do not use: core_start.wait_for_state_machine_finished(sm)
        from rafcon.core.states.state import StateExecutionStatus
        while sm.root_state.state_execution_status is StateExecutionStatus.INACTIVE:
            time.sleep(0.01)
        sm.join()

    @pytest.mark.skipif(sys.version_info < (3, 4), reason="requires python3.4 or higher")
    def _get_total_size(self, current_snapshot, key_type='lineno'):
        """
        Get total memory usage at snapshot

        Parameters:
        current_snapshot (snapshot): tracemalloc memory snapshot

        Returns:
        int:Total memory used in Kb

       """
        import tracemalloc
        top_stats = current_snapshot.statistics(key_type)
        total = sum(stat.size for stat in top_stats)
        return total / 1024

    @pytest.mark.skipif(sys.version_info < (3, 4), reason="requires python3.4 or higher")
    def _filter_snapshot(self, snapshot):
        """
        Filter memory snapshot to ignore memory usage data of imports, tracemalloc and unknown files

        Parameters:
        snapshot (snapshot): tracemalloc memory snapshot

        Returns:
        snapshot:Returning filtered snapshot

       """
        import tracemalloc
        return snapshot.filter_traces((
            tracemalloc.Filter(False, "<frozen importlib._bootstrap>"),
            tracemalloc.Filter(False, tracemalloc.__file__),
            tracemalloc.Filter(False, "<unknown>"),
        ))

    def _save_to_json(self):
        """Saves collected memory usage per iteration to file in the JSON format"""
        # Save collected memory data to file
        save_to_json = {
            "state_machine_index": self.key,
            "data": self.total_memory,
            "uncollected": self.uncollected_object_count
        }
        with open("tests/memory/data/" + self.key + "_" + 'memory_data.txt', 'w') as outfile:
            json.dump(save_to_json, outfile)
        outfile.close()

    @pytest.mark.skipif(sys.version_info < (3, 4), reason="requires python3.4 or higher")
    def run(self, leak_threshold=1024, running_leak_threshold=4096, save_to_json=False, assert_during_execution=False):
        """
        Run memory test by opening the test state machine, initializing tracemalloc, starting state machine and taking
        snapshots at each one, then filtering snapshots, checking for uncollectable objects in memory and saving
        total memory usage of the iteration.
        If there are uncollectable objects then the test fails.
        or if the memory usage increases above the defined threshold, then the memory test fails.

        Parameters:
        leak threshold (int): Maximum amount of memory usage fluctuation allowed before saying there is a memory leak

        Returns:
        dict:Returning test results

       """
        import tracemalloc
        # Choose State Machine to be Tested and Open It
        from rafcon.core import start as core_start
        sm = core_start.open_state_machine(self.paths_to_be_tested[self.key])

        # Test
        # Initialize Tracemalloc
        tracemalloc.start()

        def memory_assertion(stop, pre_run_snapshot_size, threshold):
            while not stop[0]:
                assert max(self._get_total_size(self._filter_snapshot(tracemalloc.take_snapshot())) - pre_run_snapshot_size, 0) < threshold
                time.sleep(0.1)

        # Run Test
        for i in range(self.number_iterations):
            print("\n\n\n\n\n\n\n\n\n\n------------------------------\n")
            print("Iteration number: ", i, "\n")
            print("------------------------------\n")
            if assert_during_execution:
                memory_assertion_thread_stop = [False]
                memory_assertion_thread = threading.Thread(target=memory_assertion,
                                                           args=(memory_assertion_thread_stop,
                                                                 self._get_total_size(self._filter_snapshot(tracemalloc.take_snapshot())),
                                                                 running_leak_threshold))
                memory_assertion_thread.start()
            self._run_iteration(sm)
            if assert_during_execution:
                memory_assertion_thread_stop[0] = True
                memory_assertion_thread.join()
            current_snapshot = self._filter_snapshot(tracemalloc.take_snapshot())
            self.total_memory.append(self._get_total_size(current_snapshot))
            print(self._display_top(current_snapshot), "\n")

            # Get gc stats and update uncollectable object count
            gc_stats = gc.get_stats()
            for j in range(len(gc_stats)):
                if gc_stats[j]['uncollectable'] >= 0:
                    self.uncollected_object_count = self.uncollected_object_count + gc_stats[j]['uncollectable']

            # Print stats of python's gc and objects in memory
            print("\nStats of gc: ", gc_stats, "\n")

            # Clear tracemalloc traces
            tracemalloc.clear_traces()

        print("\nTotal Memory: ", self.total_memory)
        print("Total Uncollected Objects: ", self.uncollected_object_count, "\n")

        # Stop Tracemalloc
        tracemalloc.stop()
        if save_to_json:
            self._save_to_json()

        total_leak = self.total_memory[-1] - self.total_memory[0]
        if total_leak < 0:
            total_leak = 0

        test_results = {"uncollectable": self.uncollected_object_count, "leak": total_leak}

        print("Total memory leak: ", total_leak)
        print("Number of uncollectable objects: ", self.uncollected_object_count)

        print("\n\n\n=================== Asserting Results ===================\n\n\n")

        assert (test_results["uncollectable"] == 0)
        assert (test_results["leak"] < leak_threshold)

        return test_results
