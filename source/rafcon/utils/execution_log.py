import json

def log_to_raw_structure(execution_history_items):
    """
    param: execution_history_items is a dict, in the simplest directly the opened shelve log file
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
            prev_item_id = v['prev_history_item_id']
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

        rid = v['run_id']
        if rid in grouped_by_run_id:
            grouped_by_run_id[rid].append(v)
        else:
            grouped_by_run_id[rid] = [v]
    return start_item, previous, next_, concurrent, grouped_by_run_id

def log_to_collapsed_structure(execution_history_items):
    start_item, previous, next_, concurrent, grouped = log_to_raw_structure(execution_history_items)

    start_item = None
    collapsed_next = {}
    collapsed_concurrent ={}
    collapsed_hierarchy = {}
    collapsed_items = {}
    # build collapsed items
    for rid, gitems in grouped.items():
        if gitems[0]['item_type'] == 'StateMachineStartItem':
            execution_item = {}
            execution_item['item_type'] = 'StateMachineStartItem'
            execution_item['state_type'] = 'StateMachineStartState'
            execution_item['state_name'] = 'Start'
            execution_item['run_id'] = gitems[0]['run_id']
            start_item = execution_item
            collapsed_next[rid] = execution_history_items[next_[gitems[0]['history_item_id']]]['run_id']
            collapsed_items[rid] = execution_item
        elif gitems[0]['state_type'] == 'ExecutionState' or \
             gitems[0]['state_type'] == 'HierarchyState' or \
             'Concurrency' in gitems[0]['state_type']:

            # select call and return items for this state
            try:
                call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and \
                                    gitems[i]['call_type'] == 'EXECUTE' \
                                    for i in range(len(gitems))].index(True)]
            except ValueError:
                # fall back to container call
                call_item = gitems[[gitems[i]['item_type'] == 'CallItem' and \
                                    gitems[i]['call_type'] == 'CONTAINER' \
                                    for i in range(len(gitems))].index(True)]


            try:
                return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and \
                                      gitems[i]['call_type'] == 'EXECUTE' \
                                      for i in range(len(gitems))].index(True)]
            except ValueError:
                # fall back to container call
                return_item = gitems[[gitems[i]['item_type'] == 'ReturnItem' and \
                                      gitems[i]['call_type'] == 'CONTAINER' \
                                      for i in range(len(gitems))].index(True)]

            # next item (on same hierarchy level) is always after return item
            if return_item['history_item_id'] in next_:
                # no next relationship at the end of containers
                if execution_history_items[next_[return_item['history_item_id']]]['state_type'] == 'HierarchyState' and execution_history_items[next_[return_item['history_item_id']]]['item_type'] == 'ReturnItem':
                    pass
                else:
                    collapsed_next[rid] = execution_history_items[next_[return_item['history_item_id']]]['run_id']

            # treat hierarchy level 
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
            for l in ['description', 'path_by_name', 'state_name', 'run_id', 'state_type', 'path']:
                execution_item[l] = call_item[l]
            for l in ['outcome_name', 'outcome_id']:
                execution_item[l] = return_item[l]
            for l in ['timestamp']:
                execution_item[l+'_call'] = call_item[l]
                execution_item[l+'_return'] = return_item[l]


            execution_item['data_ins'] = json.loads(call_item['input_output_data'])
            execution_item['data_outs'] = json.loads(return_item['input_output_data'])

            execution_item['scoped_data_ins'] = {}
            for k, v in json.loads(call_item['scoped_data']).items():
                if k.startswith('error'):
                    pass
                execution_item['scoped_data_ins'][v['name']] = v['value']
            execution_item['scoped_data_outs'] = {}
            for k, v in json.loads(return_item['scoped_data']).items():
                if k.startswith('error'):
                    pass
                execution_item['scoped_data_outs'][v['name']] = v['value']

            collapsed_items[rid] = execution_item

    return start_item, collapsed_next, collapsed_concurrent, collapsed_hierarchy, collapsed_items

