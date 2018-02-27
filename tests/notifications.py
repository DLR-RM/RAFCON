from os.path import join
from graphviz import Digraph
from collections import OrderedDict

from testing_utils import RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE

# notification debugging
debugging_enabled = False
notification_graph_to_render = None
nodes = {}
edges = OrderedDict()
existing_dot_nodes_to_colors = dict()
dot_node_sequence_number = None
filter_self_references = True


def node_id(node):
    return str(id(node))


def node_name(node):
    return "{class_name} ({id})".format(class_name=node.__class__.__name__, id=hex(id(node)))


def enable_debugging():
    global debugging_enabled, notification_graph_to_render, dot_node_sequence_number, existing_dot_nodes_to_colors
    global nodes, edges
    existing_dot_nodes_to_colors = dict()
    debugging_enabled = True
    # does not work as all edges with the same source and endpoint are merged
    # dot_graph = Digraph(comment='Our fancy debugging graph', graph_attr={"concentrate": "true"})
    # does not change anything
    # dot_graph = Digraph(comment='Our fancy debugging graph', graph_attr={"labelfloat": "true"})
    # "ortho" does not work for all engines, for others it does not do anything
    # dot_graph = Digraph(comment='Our fancy debugging graph', graph_attr={"splines": "compound"})
    # dot_graph = Digraph(comment='Our fancy debugging graph', graph_attr={"splines": "compound", "overlap": "false"})
    notification_graph_to_render = Digraph(name='notification_graph_to_render')
    dot_node_sequence_number = 0
    nodes = {}
    edges = OrderedDict()


def show_debug_graph(print_to_console=False, open_text_file=True, render_graph=True):
    global notification_graph_to_render
    assert isinstance(notification_graph_to_render, Digraph)
    print "started rendering graph ... "
    notification_graph_to_render.engine = 'circo'  # nice
    # dot_graph.engine = 'fdp'  # nice, slowest
    # dot_graph.engine = 'neato'
    # dot_graph.engine = 'twopi'
    # dot_graph.engine = 'dot'  # default
    # 'dotty', 'lefty' and 'sfdp' # does not exist yet in this version
    if render_graph:
        notification_graph_to_render.render(
            join(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, 'notification_output.gv'), view=True)

    with open(join(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, 'notification_print_out.txt'), "w") as text_file:
        # unfortunately printing a graph like this mixes nodes and edges
        # text_file.write(str(full_notification_graph_to_print_out.source))
        text_file.write("{}\n".format("#" * 50))
        text_file.write("number of nodes: {}; number of edges: {}\n".format(len(nodes), len(edges)))
        text_file.write("{}\n".format("#" * 50))
        text_file.write("{}\n".format("#" * 30))
        text_file.write("nodes\n")
        text_file.write("{}\n".format("#" * 30))
        for node_name in nodes.itervalues():
            text_file.write(node_name+"\n-----\n")
        text_file.write("{}\n".format("#" * 30))
        text_file.write("edges\n")
        text_file.write("{}\n".format("#" * 30))
        for sequence_number, edge_info in edges.iteritems():
            source_node_id = edge_info['source_node_id']
            source_node_name = edge_info['source_node_name']
            target_node_id = edge_info['target_node_id']
            target_node_name = edge_info['target_node_name']
            if 'method_name' in edge_info['info']:  # before/after
                method_name = edge_info['info']['method_name']
                args = ", ".join([str(arg) for arg in edge_info['info']['args'][1:]])
                before_after = "before" if "before" in edge_info['info'] else "after"
                change = "{method}({args}), {before_after}".format(method=method_name, args=args, before_after=before_after)
            elif 'assign' in edge_info['info']:  # assign
                prop_name = edge_info['info']['prop_name']
                value = edge_info['info']['new']
                change = "{prop_name}={value}".format(prop_name=prop_name, value=value)
            else:  # signal
                signal_name = edge_info['info']['prop_name']
                arg = edge_info['info']['arg']
                change = "{signal}({arg})".format(signal=signal_name, arg=arg)
            text_file.write("{number}: {source_node}.{prop_name}[{change}] => {target_node}.{callback}".format(
                number=sequence_number,
                source_node=source_node_name,
                target_node=target_node_name,
                prop_name=edge_info['prop_name'],
                change=change,
                callback=edge_info['callback'].__func__.func_name
            ))
            text_file.write("\n-----\n")

    if open_text_file:
        import subprocess
        path = join(RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, 'notification_print_out.txt')
        subprocess.Popen("/usr/bin/gedit %s" % (path), shell=True)

    if print_to_console:
        print(full_notification_graph_to_print_out.source)

    print "finished rendering graph"


def disable_debugging():
    global debugging_enabled, notification_graph_to_render, dot_node_sequence_number, existing_dot_nodes_to_colors
    global full_notification_graph_to_print_out
    existing_dot_nodes_to_colors = None
    debugging_enabled = False
    notification_graph_to_render = None
    full_notification_graph_to_print_out = None
    dot_node_sequence_number = None


def feed_debugging_graph(observable, observer, callback, *args, **kwargs):
    global debugging_enabled, notification_graph_to_render, existing_dot_nodes_to_colors, \
        dot_node_sequence_number, filter_self_references, nodes, edges

    if debugging_enabled:
        model, prop_name, info = args
        source_node_id = node_id(observable)
        target_node_id = node_id(observer)

        import random
        color = "#%06x" % random.randint(0, 0xFFFFFF)

        source_node_name = node_name(observable)
        target_node_name = node_name(observer)

        if source_node_id not in existing_dot_nodes_to_colors.keys():
            existing_dot_nodes_to_colors[source_node_id] = color
            notification_graph_to_render.node(source_node_id, source_node_name)
            nodes[source_node_id] = source_node_name
        if target_node_id not in existing_dot_nodes_to_colors.keys():
            existing_dot_nodes_to_colors[target_node_id] = color
            notification_graph_to_render.node(target_node_id, target_node_name)
            nodes[target_node_id] = target_node_name

        # for routing edges over dedicated nodes
        # problem: does not scale
        # node_label = str(dot_node_sequence_number) + "\n" + str(callback)

        # tried out: xlabel, style="invis"

        edges[dot_node_sequence_number] = {
            'source_node_id': source_node_id,
            'source_node_name': source_node_name,
            'target_node_id': target_node_id,
            'target_node_name': target_node_name,
            'model': model,
            'prop_name': prop_name,
            'callback': callback,
            'info': info,
        }

        if not filter_self_references or source_node_id != target_node_id:
            # info on edges
            notification_graph_to_render.edge(source_node_id, target_node_id,
                                              headlabel="_" + str(dot_node_sequence_number) + "_",
                                              # label="_"+str(dot_node_sequence_number)+"_",
                                              # decorate only works for normal labels
                                              # decorate="true",
                                              labeldistance="5",
                                              labelfontsize="6",
                                              fontcolor=existing_dot_nodes_to_colors[source_node_id],
                                              color=existing_dot_nodes_to_colors[source_node_id])
            # info on nodes: does not scale with many edges
            # dot_graph.node(str(dot_node_sequence_number), node_label)
            # dot_graph.edge(str(source_node_id), str(dot_node_sequence_number),
            #                color=existing_dot_nodes_to_colors[source_node_id])
            # dot_graph.edge(str(dot_node_sequence_number), str(target_node_id),
            #                color=existing_dot_nodes_to_colors[source_node_id])
        dot_node_sequence_number += 1
