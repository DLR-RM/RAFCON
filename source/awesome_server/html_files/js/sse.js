var active_marked_states = [];
var highlighted_states = [];
var previous_active_states = [];
var previous_highlighted_states = [];
var id_list = {};

function sse() {
    if (!!window.EventSource) {
        var eventList = document.getElementById("eventlist");
        var sm_selector = document.getElementById("sm_selector");
        var source = new EventSource('/state_machine_event_source');
        source.onmessage = function(e) {
        	
        	if (e.lastEventId == 1) {
        		new_connection_event(e);
        		return;
        	}
        	
            data_lines = e.data.split("\n");
            console.log(e);
            console.log(data_lines);
            if (eventList) {
            	var newElement = document.createElement("li");
				
            	newElement.innerHTML = "message: " + data_lines[0];
            	eventList.appendChild(newElement);
            	eventList.scrollTop = eventList.scrollTop + 50;
            }
            if (!data_lines[0].startsWith('-')){
            	new_active_states = data_lines[0].split("/");
            	for (i in new_active_states) {
            		state = new_active_states[i];
            		if (i == new_active_states.length - 1 && highlighted_states.indexOf(state) == -1) {
            			highlighted_states.push(state);
            			return;
            		}
            		else if (active_marked_states.indexOf(state) == -1 && highlighted_states.indexOf(state) == -1) {
            			active_marked_states.push(state);
            		}
            	}
            }
            else if (data_lines[0].startsWith('-')) {
            	handle_active_states();
            }
        };
        source.addEventListener("execution", function(e) {
        	data_lines = e.data.split("\n");
        	if (data_lines[0] == "STOPPED") {
        		for (state_id in id_list) {
	        		deactivate_state(id_list[state_id]);
	        		deactivate_parent_state(id_list[state_id]);
	        	}
	        	active_marked_states = [];
				highlighted_states = [];
				previous_active_states = [];
				previous_highlighted_states = [];
        	}
        }, false);
        source.addEventListener("registration", function(e) {
        	var data = JSON.parse(e.data); 
            var newElement = document.createElement("li");
            newElement.innerHTML = "New Statemachine registered: " + data.msg;
            if (eventList) {
            	eventList.appendChild(newElement);
            	eventList.scrollTop = eventList.scrollTop + 50;	
            }

            var newOption = document.createElement("option");

            var addr = data.ip + ":" + data.port; 
            newOption.value = addr;

            if (check_selector_options(data.msg)) {
                newOption.text = data.msg + ":" + data.port;
            } else {
                newOption.text = data.msg;
            }
            sm_selector.appendChild(newOption);

            if(sm_selector[0].value == "none") {
                sm_selector.removeChild(sm_selector[0]);
            }

            alert("New Statemachine registered: " + data.msg);
        }, false);
        source.addEventListener("new_state", function(e) {
        	var data = JSON.parse(e.data);
        	
        	var position = { x: parseFloat(data.pos_x), y: parseFloat(data.pos_y) };
        	var size = { width: parseFloat(data.width), height: parseFloat(data.height) };
        		
        	var outcomes = [];
        		
        	for (i in data.outcomes) {
        		outcomes.push(data.outcomes[i]);
        	}

        	if (data.parent_id == 'none') {
        		viewer_state_id = add_new_state(position, size, data.state_name, parseInt(data.hierarchy_level), outcomes);
        		id_list[data.state_id] = viewer_state_id;
        	} else {
        		container_id = id_list[data.parent_id];
        		viewer_state_id = add_new_state_to_container_state(container_id, position, size, data.state_name, parseInt(data.hierarchy_level), outcomes);
        		id_list[data.state_id] = viewer_state_id;
        	}
        	
        	paper.scaleContentToFit();
        	graph_scale = V(paper.viewport).scale().sx;
        }, false);
        source.addEventListener("new_connection", function (e) {
        	// data_lines = e.data.split("\n");
        	// var waypoints = [];
        	// 
			// container_id = id_list[data_lines[0]];
        	// source_id = id_list[data_lines[2]];
        	// target_id = id_list[data_lines[4]];
        	// 
        	// container = graph.getCell(container_id);
        	// cont_pos_x = container["attributes"]["position"].x;
        	// cont_pos_y = container["attributes"]["position"].y;
        	// 
        	// if (data_lines.length > 5) {
        	// 	for (var i = 5; i < data_lines.length; i += 2) {
        	// 		wpx = parseFloat(data_lines[i]);
        	// 		wpy = parseFloat(data_lines[i+1]);
        	// 		waypoints.push({ x: wpx + cont_pos_x, y: wpy + cont_pos_y });
        	// 	}
        	// }
        	
        	new_connection_event(e);
        	
        	
        }, false);
        source.onerror = function(e) {
            alert("Server closed connection.");
            source.close();
        };
        source.onstoprequest = function(e) {
        	console.log("test");
        };
    }
}

function new_connection_event(e) {
	var data = JSON.parse(e.data);
	
	var waypoints = [];
	
	container_id = id_list[data.container_state_id];
	source_id = id_list[data.from_state];
	target_id = id_list[data.to_state];
	
	container = graph.getCell(container_id);
	cont_pos_x = container["attributes"]["position"].x;
	cont_pos_y = container["attributes"]["position"].y;
	
	for (i in data.waypoints) {
		wpx = data.waypoints[i][0];
		wpy = data.waypoints[i][1];
		waypoints.push({ x: wpx + cont_pos_x, y: wpy + cont_pos_y });
	}
	
	connect(source_id, data.from_outcome, target_id, data.to_outcome, waypoints);
}

function check_selector_options(title) {
    var sm_selector = document.getElementById("sm_selector");
    for (i = 0; i < sm_selector.length; i++){
        if (sm_selector[i].text == title){
            return true;
        }
    }
    return false;
}

function handle_active_states() {
	
	for (i in active_marked_states) {
		state = active_marked_states[i];
		if (previous_active_states.indexOf(state) == -1) {
			state_id = id_list[state];
			activate_parent_state(state_id);
		}
	}
	for (i in previous_active_states) {
		state = previous_active_states[i];
		if (active_marked_states.indexOf(state) == -1) {
			state_id = id_list[state];
			deactivate_parent_state(state_id);
		}
	}
	
	for (i in highlighted_states) {
		state = highlighted_states[i];
		if (previous_highlighted_states.indexOf(state) == -1) {
			state_id = id_list[state];
			activate_state(state_id);
		}
	}
	for (i in previous_highlighted_states) {
		state = previous_highlighted_states[i];
		if (highlighted_states.indexOf(state) == -1) {
			state_id = id_list[state];
			deactivate_state(state_id);
		}
	}
	
	previous_active_states = active_marked_states;
	active_marked_states = [];
	
	previous_highlighted_states = highlighted_states;
	highlighted_states = [];
}
