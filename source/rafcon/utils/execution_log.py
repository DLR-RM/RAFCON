# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>

from future.utils import string_types, native_str
from builtins import range
from builtins import str
import shelve
import json
import pickle

from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


def log_to_raw_structure(execution_history_items):
    """
    :param dict execution_history_items: history items, in the simplest case
           directly the opened shelve log file
    :return: start_item, the StateMachineStartItem of the log file
             previous, a dict mapping history_item_id --> history_item_id of previous history item
             next_, a dict mapping history_item_id --> history_item_id of the next history item (except if
                    next item is a concurrent execution branch)
             concurrent, a dict mapping history_item_id --> []list of concurrent next history_item_ids
                         (if present)
             grouped, a dict mapping run_id --> []list of history items with this run_id
    :rtype: tuple
    """
    previous = {}
    next_ = {}
    concurrent = {}
    grouped_by_run_id = {}
    start_item = None

    for k,v in execution_history_items.items():
        if v['item_type'] == 'StateMachineStartItem':
            start_item = v
        else:
            # connect the item to its predecessor
            prev_item_id = native_str(v['prev_history_item_id'])

            if prev_item_id in execution_history_items:
                ## should always be the case except if shelve is broken/missing data

                previous[k] = prev_item_id
                if execution_history_items[prev_item_id]['item_type'] == 'ConcurrencyItem' and \
                   execution_history_items[k]['item_type'] != 'ReturnItem':
                    # this is not a return  item, thus this 'previous' relationship of this
                    # item must be a call item of one of the concurrent branches of
                    # the concurrency state
                    if prev_item_id in concurrent:
                        concurrent[prev_item_id].append(k)
                    else:
                        concurrent[prev_item_id] = [k]
                else:
                    # this is a logical 'next' relationship
                    next_[prev_item_id] = k
            else:
                logger.warning('HistoryItem is referring to a non-existing previous history item, HistoryItem was %s' % str(v))

        rid = v['run_id']
        if rid in grouped_by_run_id:
            grouped_by_run_id[rid].append(v)
        else:
            grouped_by_run_id[rid] = [v]

    return start_item, previous, next_, concurrent, grouped_by_run_id


def log_to_collapsed_structure(execution_history_items, throw_on_pickle_error=True,
                               include_erroneous_data_ports=False, full_next=False):
    """
    Collapsed structure means that all history items belonging to the same state execution are
    merged together into one object (e.g. CallItem and ReturnItem of an ExecutionState). This
    is based on the log structure in which all Items which belong together have the same run_id.
    The collapsed items hold input as well as output data (direct and scoped), and the outcome
    the state execution.
    :param dict execution_history_items: history items, in the simplest case
           directly the opened shelve log file
    :param bool throw_on_pickle_error: flag if an error is thrown if an object cannot be un-pickled
    :param bool include_erroneous_data_ports: flag if to include erroneous data ports
    :param bool full_next: flag to indicate if the next relationship has also to be created at the end
           of container states
    :return: start_item, the StateMachineStartItem of the log file
             next_, a dict mapping run_id --> run_id of the next executed state on the same
                    hierarchy level
             concurrent, a dict mapping run_id --> []list of run_ids of the concurrent next
                         executed states (if present)
             hierarchy, a dict mapping run_id --> run_id of the next executed state on the
                        deeper hierarchy level (the start state within that HierarchyState)
             items, a dict mapping run_id --> collapsed representation of the execution of
                    the state with that run_id
    :rtype: tuple
    """

    # for debugging purposes
    # execution_history_items_dict = dict()
    # for k, v in execution_history_items.items():
    #     execution_history_items_dict[k] = v

    start_item, previous, next_, concurrent, grouped = log_to_raw_structure(execution_history_items)

    start_item = None
    collapsed_next = {}
    collapsed_concurrent ={}
    collapsed_hierarchy = {}
    collapsed_items = {}

    # single state executions are not supported
    if len(next_) == 0 or len(next_) == 1:
        for rid, gitems in grouped.items():
            if gitems[0]['item_type'] == 'StateMachineStartItem':
                item = gitems[0]
                execution_item = {}
                ## add base properties will throw if not existing
                for l in ['description', 'path_by_name', 'state_name', 'run_id', 'state_type',
                          'path', 'timestamp', 'root_state_storage_id', 'state_machine_version',
                          'used_rafcon_version', 'creation_time', 'last_update', 'os_environment']:
                    try:
                        execution_item[l] = item[l]
                    except KeyError:
                        logger.warning("Key {} not in history start item".format(str(l)))

                ## add extended properties (added in later rafcon versions),
                ## will add default value if not existing instead
                for l, default in [('semantic_data', {}),
                                     ('is_library', None),
                                     ('library_state_name', None),
                                     ('library_name', None),
                                     ('library_path', None)]:
                    execution_item[l] = item.get(l, default)

                start_item = execution_item
        return start_item, collapsed_next, collapsed_concurrent, collapsed_hierarchy, collapsed_items

    # build collapsed items
    for rid, gitems in grouped.items():
        if gitems[0]['item_type'] == 'StateMachineStartItem':
            item = gitems[0]
            execution_item = {}
            ## add base properties will throw if not existing
            for l in ['description', 'path_by_name', 'state_name', 'run_id', 'state_type',
                      'path', 'timestamp', 'root_state_storage_id', 'state_machine_version',
                      'used_rafcon_version', 'creation_time', 'last_update', 'os_environment']:
                try:
                    execution_item[l] = item[l]
                except KeyError:
                    logger.warning("Key {} not in history start item".format(str(l)))

            ## add extended properties (added in later rafcon versions),
            ## will add default value if not existing instead
            for l, default in [('semantic_data', {}),
                                 ('is_library', None),
                                 ('library_state_name', None),
                                 ('library_name', None),
                                 ('library_path', None)]:
                execution_item[l] = item.get(l, default)

            start_item = execution_item

            collapsed_next[rid] = execution_history_items[next_[gitems[0]['history_item_id']]]['run_id']
            collapsed_items[rid] = execution_item
        elif gitems[0]['state_type'] == 'ExecutionState' or \
             gitems[0]['state_type'] == 'HierarchyState' or \
             gitems[0]['state_type'] == 'LibraryState' or \
             'Concurrency' in gitems[0]['state_type']:

            # for item in gitems:
            #     if item["description"] is not None:
            #         print(item["item_type"], item["call_type"], item["state_type"], item["state_name"])
            #     print(item["description"])

            # select call and return items for this state
            try:
                call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and \
                                    gitems[i]['call_type'] == 'EXECUTE' \
                                    for i in range(len(gitems))].index(True)]
            except ValueError:
                # fall back to container call, should only happen for root state
                try:
                    call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and \
                                        gitems[i]['call_type'] == 'CONTAINER' \
                                        for i in range(len(gitems))].index(True)]
                except ValueError:
                    logger.warning('Could not find a CallItem in run_id group %s\nThere will probably be log information missing on this execution branch!' % str(rid))
                    ## create dummy returnitem with the properties referenced later in this code
                    call_item = dict(description=None,
                                     history_item_id=None,
                                     path_by_name=None,
                                     state_name=None,
                                     run_id=None,
                                     state_type=None,
                                     path=None,
                                     timestamp=None,
                                     input_output_data={},
                                     scoped_data={})

            try:
                return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and \
                                      gitems[i]['call_type'] == 'EXECUTE' \
                                      for i in range(len(gitems))].index(True)]
            except ValueError:
                # fall back to container call, should only happen for root state
                try:
                    return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and \
                                          gitems[i]['call_type'] == 'CONTAINER' \
                                          for i in range(len(gitems))].index(True)]
                except ValueError:
                    logger.warning('Could not find a ReturnItem in run_id group %s\nThere will probably be log information missing on this execution branch!' % str(rid))
                    ## create dummy returnitem with the properties referenced later in this code
                    return_item = dict(history_item_id=None,
                                       outcome_name=None,
                                       outcome_id=None,
                                       timestamp=None,
                                       input_output_data={},
                                       scoped_data={})

            # next item (on same hierarchy level) is always after return item
            if return_item['history_item_id'] in next_:
                # no next relationship at the end of containers
                if execution_history_items[next_[return_item['history_item_id']]]['state_type'] == 'HierarchyState' and execution_history_items[next_[return_item['history_item_id']]]['item_type'] == 'ReturnItem' and execution_history_items[next_[return_item['history_item_id']]]['call_type'] == 'CONTAINER':
                    if full_next:
                        collapsed_next[rid] = execution_history_items[next_[return_item['history_item_id']]]['run_id']
                    else:
                        pass
                else:
                    collapsed_next[rid] = execution_history_items[next_[return_item['history_item_id']]]['run_id']

            # treat hierarchy level
            if call_item['history_item_id'] in previous:
                if execution_history_items[previous[call_item['history_item_id']]]['state_type'] == 'HierarchyState' and execution_history_items[previous[call_item['history_item_id']]]['item_type'] == 'CallItem':
                    prev_rid = execution_history_items[previous[call_item['history_item_id']]]['run_id']
                    collapsed_hierarchy[prev_rid] = rid

                # treat concurrency level
                if execution_history_items[previous[call_item['history_item_id']]]['item_type'] == 'ConcurrencyItem':
                    prev_rid = execution_history_items[previous[call_item['history_item_id']]]['run_id']
                    if prev_rid in collapsed_concurrent:
                        collapsed_concurrent[prev_rid].append(rid)
                    else:
                        collapsed_concurrent[prev_rid] = [rid]

            # assemble grouped item
            execution_item = {}
            # add base properties will throw if not existing
            for l in ['description', 'path_by_name', 'state_name', 'run_id', 'state_type', 'path']:
                execution_item[l] = call_item[l]

            # add extended properties (added in later rafcon versions),
            # will add default value if not existing instead
            for l, default in [('semantic_data', {}),
                                 ('is_library', None),
                                 ('library_state_name', None),
                                 ('library_name', None),
                                 ('library_path', None)]:
                execution_item[l] = return_item.get(l, default)

            for l in ['outcome_name', 'outcome_id']:
                execution_item[l] = return_item[l]
            for l in ['timestamp']:
                execution_item[l+'_call'] = call_item[l]
                execution_item[l+'_return'] = return_item[l]

            def unpickle_data(data_dict):
                r = dict()
                # support backward compatibility
                if isinstance(data_dict, string_types):  # formerly data dict was a json string
                    r = json.loads(data_dict)
                else:
                    for k, v in data_dict.items():
                        if not k.startswith('!'):  # ! indicates storage error
                            try:
                                r[k] = pickle.loads(v)
                            except Exception as e:
                                if throw_on_pickle_error:
                                    raise
                                elif include_erroneous_data_ports:
                                    r['!' + k] = (str(e), v)
                                else:
                                    pass  # ignore
                        elif include_erroneous_data_ports:
                            r[k] = v

                return r

            execution_item['data_ins'] = unpickle_data(call_item['input_output_data'])
            execution_item['data_outs'] = unpickle_data(return_item['input_output_data'])
            execution_item['scoped_data_ins'] = unpickle_data(call_item['scoped_data'])
            execution_item['scoped_data_outs'] = unpickle_data(return_item['scoped_data'])
            # backward compatibility
            if isinstance(execution_item['semantic_data'], Vividict):
                execution_item['semantic_data'] = execution_item['semantic_data']
            else:
                execution_item['semantic_data'] = unpickle_data(execution_item['semantic_data'])

            collapsed_items[rid] = execution_item

    return start_item, collapsed_next, collapsed_concurrent, collapsed_hierarchy, collapsed_items


def log_to_DataFrame(execution_history_items, data_in_columns=[], data_out_columns=[], scoped_in_columns=[],
                     scoped_out_columns=[], semantic_data_columns=[], throw_on_pickle_error=True):
    """
    Returns all collapsed items in a table-like structure (pandas.DataFrame) with one row per executed 
    state and a set of properties resp. columns (e.g. state_name, outcome, run_id) for this state.
    The data flow (data_in/out, scoped_data_in/out, semantic_data) is omitted from this table
    representation by default, as the different states have different data in-/out-port, scoped_data-
    ports and semantic_data defined. However, you can ask specific data-/scoped_data-ports and semantic
    data to be exported as table column, given they are primitive-valued, by including the port / key
    names in the *_selected-parameters. These table-columns will obviously only be well-defined for
    states having this kind of port-name-/semantic-key and otherwise will contain a None-like value,
    indicating missing data.

    The available data per execution item (row in the table) can be printed using pandas.DataFrame.columns.
    """
    try:
        import pandas as pd
    except ImportError:
        raise ImportError("The Python package 'pandas' is required for log_to_DataFrame.")

    start, next_, concurrency, hierarchy, gitems = log_to_collapsed_structure(
        execution_history_items, throw_on_pickle_error=throw_on_pickle_error)
    gitems.pop(start['run_id'])
    if len(gitems) == 0:
        return pd.DataFrame()

    # remove columns which are not generic over all states (basically the
    # data flow stuff)
    df_keys = list(list(gitems.values())[0].keys())
    df_keys.remove('data_ins')
    df_keys.remove('data_outs')
    df_keys.remove('scoped_data_ins')
    df_keys.remove('scoped_data_outs')
    df_keys.remove('semantic_data')
    df_keys.sort()

    df_items = []

    for rid, item in gitems.items():
        row_data = [item[k] for k in df_keys]

        for key, selected_columns in [('data_ins', data_in_columns),
                                      ('data_outs', data_out_columns),
                                      ('scoped_data_ins', scoped_in_columns),
                                      ('scoped_data_outs', scoped_out_columns),
                                      ('semantic_data', semantic_data_columns)]:
            for column_key in selected_columns:
                row_data.append(item[key].get(column_key, None))
        df_items.append(row_data)

    for key, selected_columns in [('data_ins', data_in_columns),
                                  ('data_outs', data_out_columns),
                                  ('scoped_data_ins', scoped_in_columns),
                                  ('scoped_data_outs', scoped_out_columns),
                                  ('semantic_data', semantic_data_columns)]:
        df_keys.extend([key + '__' + s for s in selected_columns])
    df = pd.DataFrame(df_items, columns=df_keys)
    # convert epoch to datetime
    df.timestamp_call = pd.to_datetime(df.timestamp_call, unit='s')
    df.timestamp_return = pd.to_datetime(df.timestamp_return, unit='s')

    # use call timestamp as index
    df_timed = df.set_index(df.timestamp_call)
    df_timed.sort_index(inplace=True)

    return df_timed


def log_to_ganttplot(execution_history_items):
    """
    Example how to use the DataFrame representation
    """
    import matplotlib.pyplot as plt
    import matplotlib.dates as dates
    import numpy as np

    d = log_to_DataFrame(execution_history_items)

    # de-duplicate states and make mapping from state to idx
    unique_states, idx = np.unique(d.path_by_name, return_index=True)
    ordered_unique_states = np.array(d.path_by_name)[np.sort(idx)]
    name2idx = {k: i for i, k in enumerate(ordered_unique_states)}

    calldate = dates.date2num(d.timestamp_call.dt.to_pydatetime())
    returndate = dates.date2num(d.timestamp_return.dt.to_pydatetime())

    state2color = {'HierarchyState': 'k',
                   'ExecutionState': 'g',
                   'BarrierConcurrencyState': 'y',
                   'PreemptiveConcurrencyState': 'y'}

    fig, ax = plt.subplots(1, 1)
    ax.barh(bottom=[name2idx[k] for k in d.path_by_name], width=returndate-calldate,
            left=calldate, align='center', color=[state2color[s] for s in d.state_type], lw=0.0)
    plt.yticks(list(range(len(ordered_unique_states))), ordered_unique_states)
