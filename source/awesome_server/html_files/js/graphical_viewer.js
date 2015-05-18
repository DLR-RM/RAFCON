var graph_scale = 1;
var graph;
var paper;

function setup_paper() {
	
	graph = new joint.dia.Graph;
	paper = new joint.dia.Paper({
		el: $('#graphical_viewer'),
		width: window.innerWidth,
		height: window.innerHeight - 100,
		gridSize: 1,
		model: graph,
		defaultLink: new joint.dia.Link({
			attrs: { '.marker-target': { d: 'M 10 0 L 0 5 L 10 10 z'} }
		})
	});
	paper.$el.css('pointer-events', 'none');
	
	/* mouse translate */
	var last_position = {};
	var current_translation = { x: 0, y: 0};
	paper.scale(1, 1);
	
	window.onmousemove = function(evt) {
		if (evt.buttons == 4 || evt.buttons == 1) {
			if (typeof(last_position.x) != 'undefined') {
				var deltaX = last_position.x - evt.clientX,
					deltaY = last_position.y - evt.clientY;
				
				paper.viewport.transform.baseVal[0].matrix.e -= deltaX;
				paper.viewport.transform.baseVal[0].matrix.f -= deltaY;
				
				current_translation.x = paper.viewport.transform.baseVal[0].matrix.e;
				current_translation.y = paper.viewport.transform.baseVal[0].matrix.f;
			}
			
			last_position = {
				x: evt.clientX,
				y: evt.clientY
			};
		}
		else {
			last_position = {};
		}
	};
	
	/* window resize */
	window.addEventListener('resize', resize_paper, false);
	
	function resize_paper() {
		paper.setDimensions(window.innerWidth, window.innerHeight - 100);
	}
	
	/* add mouse scoll */
	
	register_mouse_scroll(paper);
	
	function offset_to_local_point(offset_x, offset_y, paper) {
		var svg_point = paper.svg.createSVGPoint();
		svg_point.x = offset_x;
		svg_point.y = offset_y;
		var offset_transformed = svg_point.matrixTransform(paper.viewport.getCTM().inverse());
		return offset_transformed;
	}
	
	function register_mouse_scroll(paper) {
		if (window.addEventListener) {
			window.addEventListener("mousewheel", function(event) {
				scale_content(paper, event);
			}, false);
			window.addEventListener("DOMMouseScroll", function(event) {
				scale_content(paper, event);
			}, false);
		}
	}
	
	function scale_content(paper, event) {
		if (event.detail < 0) graph_scale += 0.1;
		else {
			graph_scale -= 0.1;
			if (graph_scale < 0.1) {
				graph_scale = 0.1;
				return;
			}
		}
		var p = offset_to_local_point(event.pageX, event.pageY, paper);
		paper.setOrigin(current_translation.x, current_translation.y);
		paper.scale(graph_scale, graph_scale);
		current_translation.x = paper.viewport.transform.baseVal[0].matrix.e;
		current_translation.y = paper.viewport.transform.baseVal[0].matrix.f;
	}
}

function create_state(position, size, name, hierarchy_level, outcomes) {
	var state = new joint.shapes.devs.Model({
		position: { x: position.x, y: position.y},
		size: { width: size.width, height: size.height },
		inPorts: [''],
		outPorts: outcomes,
		attrs: {
			'.label': { text: name, 'ref-x': .5, 'ref-y': .1, style: {'font-size': 72.0 / hierarchy_level} },
			'.port-label': { style: {'font-size': 72.0 / hierarchy_level} },
			'.port-body': { r: 30 / hierarchy_level }
		}
	});
	
	return state;
}

function add_new_state(position, size, name, hierarchy_level, outcomes) {
	var state = create_state(position, size, name, hierarchy_level, outcomes);
	graph.addCell(state);
	return state.id;
}

function add_new_state_to_container_state(container_id, position, size, name, hierarchy_level, outcomes) {
	container_cell = graph.getCell(container_id);
	abs_position = { x: 0, y: 0};
	abs_position.x = position.x + container_cell.get('position').x;
	abs_position.y = position.y + container_cell.get('position').y;
	state = create_state(abs_position, size, name, hierarchy_level, outcomes);
	container_cell.embed(state);
	graph.addCell(state);
	return state.id;
}

function connect(source_id, source_port, target_id, target_port, waypoints) {
	source = graph.getCell(source_id);
	target = graph.getCell(target_id);
	var link = new joint.dia.Link({
		source: { id: source_id, selector: source.getPortSelector(source_port) },
		target: { id: target_id, selector: target.getPortSelector(target_port) },
		vertices: waypoints,
		attrs: {
        '.marker-target': {
            fill: '#333333',
            d: 'M 10 0 L 0 5 L 10 10 z'
        }
    }
	});
	link.addTo(graph).reparent();
}

function activate_state(state_id) {
	state = graph.getCell(state_id);
	state = paper.findViewByModel(state);
	V(state.el).addClass('active-state');
}

function activate_parent_state(state_id) {
	state = graph.getCell(state_id);
	state = paper.findViewByModel(state);
	V(state.el).addClass('active-parent-state');
}

function deactivate_state(state_id) {
	state = graph.getCell(state_id);
	state = paper.findViewByModel(state);
	if (V(state.el).hasClass('active-state')) {
		V(state.el).removeClass('active-state');
	}
}

function deactivate_parent_state(state_id) {
	state = graph.getCell(state_id);
	state = paper.findViewByModel(state);
	if (V(state.el).hasClass('active-parent-state')) {
		V(state.el).removeClass('active-parent-state');
	}
}
