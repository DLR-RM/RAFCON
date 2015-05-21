if (typeof exports === 'object') {

    var joint = {
        util: require('../src/core').util,
        shapes: {
            basic: require('./joint.shapes.basic')
        },
        dia: {
            ElementView: require('../src/joint.dia.element').ElementView,
            Link: require('../src/joint.dia.link').Link
        }
    };
    var _ = require('lodash');
}

joint.shapes.my_test = {};

joint.shapes.my_test.State = joint.shapes.basic.Generic.extend(_.extend({}, joint.shapes.basic.PortsModelInterface, {

    markup: '<g class="rotatable"><g class="scalable"><rect class="body"/></g><text class="label"/><g class="inPorts"/><g class="outPorts"/></g>',
    portMarkup: '<g class="port port<%= id %>"><rect class="port-body"/><rect class="port-inner-body"/><text class="port-label"/></g>',

    defaults: joint.util.deepSupplement({

        type: 'my_test.State',
        size: { width: 1, height: 1 },
        
        inPorts: [],
        outPorts: [],

        attrs: {
            '.': { magnet: false },
            '.body': {
                width: 450, height: 250,
                stroke: '#000000'
            },
            '.port-body': {
                width: 20, height: 20,
                magnet: true,
                stroke: '#000000'
            },
            '.port-inner-body': {
            	magnet: true,
            	ref: '.port-body',
            	'ref-x': .5,
            	'ref-y': .5,
            	'x-alignment': 'middle',
            	'y-alignment': 'middle',
            	'ref-width': .75,
            	'ref-height': .75
            },
            text: {
                'pointer-events': 'none'
            },
            '.label': { text: 'Model', 'ref-x': .5, 'ref-y': 10, ref: '.body', 'text-anchor': 'middle', fill: '#000000' },
            '.inPorts .port-label': { x:-15, dy: 4, 'text-anchor': 'end', fill: '#000000' },
            '.outPorts .port-body': { x: -25, stroke: '#ff0000', fill: '#000000' },
            '.outPorts .port-inner-body': { x: -25, fill: '#ff0000'},
            '.outPorts .port-label':{ x: 15, dy: 4, fill: '#000000' }
        }

    }, joint.shapes.basic.Generic.prototype.defaults),

    getPortAttrs: function(portName, index, total, selector, type) {
        var attrs = {};

        var portClass = 'port' + index;
        var portSelector = selector + '>.' + portClass;
        var portLabelSelector = portSelector + '>.port-label';
        var portBodySelector = portSelector + '>.port-body';

        attrs[portLabelSelector] = { text: portName };
        attrs[portBodySelector] = { port: { id: portName || _.uniqueId(type) , type: type } };
        attrs[portSelector] = { ref: '.body', 'ref-y': (index + 0.5) * (1 / total) };

        if (selector === '.outPorts') { attrs[portSelector]['ref-dx'] = 0; }

        return attrs;
    }
}));


joint.shapes.my_test.OutcomePort = joint.shapes.basic.Generic.extend(_.extend({}, joint.shapes.basic.PortsModelInterface, {

    markup: '<g class="rotatable"><g class="scalable"><rect class="body"/></g><text class="label"/><g class="inPorts"/><g class="outPorts"/></g>',
    portMarkup: '<g class="port port<%= id %>"><rect class="port-body"/><rect class="port-inner-body"/></g>',

    defaults: joint.util.deepSupplement({

        type: 'my_test.OutcomePort',
        size: { width: 1, height: 1 },
        
        inPorts: [],
        outPorts: [],

        attrs: {
            '.': { magnet: false },
            '.body': {
                width: 150, height: 30,
                fill: '#50555f'
            },
            '.port-body': {
                width: 20, height: 20,
                magnet: true, 
                stroke: '#ffffff',
                fill: '#000000'
            },
            '.port-inner-body': {
            	magnet: true,
            	ref: '.port-body',
            	fill: '#ffffff',
            	'ref-x': .5,
            	'ref-y': .5,
            	'x-alignment': 'middle',
            	'y-alignment': 'middle',
            	'ref-width': .75,
            	'ref-height': .75
            },
            text: {
                'pointer-events': 'none'
            },
            '.label': { text: 'Model', 'ref-x': .5, 'ref-y': .5, ref: '.body', 'text-anchor': 'middle', 'y-alignment': 'middle', fill: '#ffffff' },
            '.inPorts .port-body': { x: 5 },
            '.outPorts .port-body': { x: -25 },
            '.outPorts .port-inner-body': { x: -30 }
        }

    }, joint.shapes.basic.Generic.prototype.defaults),

    getPortAttrs: function(portName, index, total, selector, type) {
        var attrs = {};

        var portClass = 'port' + index;
        var portSelector = selector + '>.' + portClass;
        var portBodySelector = portSelector + '>.port-body';

        attrs[portBodySelector] = { port: { id: portName || _.uniqueId(type) , type: type } };
        attrs[portSelector] = { 
        		ref: '.body',
            	'ref-y': .5,
            	'y-alignment': 'middle',
            	'ref-width': .75,
            	'ref-height': .75 };

        if (selector === '.outPorts') { attrs[portSelector]['ref-dx'] = 0; }

        return attrs;
    }
}));

joint.shapes.my_test.StateView = joint.dia.ElementView.extend(joint.shapes.basic.PortsViewInterface);
joint.shapes.my_test.OutcomePortView = joint.dia.ElementView.extend(joint.shapes.basic.PortsViewInterface);

if (typeof exports === 'object') {

    module.exports = joint.shapes.my_test;
}




function create_outcome(name, color, position, size, font_size) {
	var wrap_name = joint.util.breakText(name, { width: size.width - 50 }, { 'font-size': font_size });
	
	var outcome = new joint.shapes.my_test.OutcomePort({
		size: { width: size.width, height: size.height },
		position: { x: position.x, y: position.y},
		inPorts: ['in'],
		outPorts: ['out'],
		attrs: {
			'.label': { text: wrap_name, style: { 'font-size': font_size } },
			'.port-body': { stroke: color },
            '.port-inner-body': { fill: color }
		}
	});
	graph.addCell(outcome);
	return outcome;
}

function draw_my_test_state() {
	var state = new joint.shapes.my_test.State({
		position: { x: 100, y: 100},
		size: { width: 450, height: 200 },
		inPorts: ['in1'],
		outPorts: ['out1', 'out2'],
		attrs: {
			'.label': { text: 'my_test_state', 'ref-x': .5, 'ref-y': .1, style: {'font-size': 20.0} }
		}
	});
	graph.addCell(state);
	
	var error_outcome = create_outcome('Error', '#fe2626', { x: 100, y: 400 }, { width: 150, height: 30 }, 20);
	var preemtion_outcome = create_outcome('Preempted', '#2696ff', { x: 100, y: 430 }, { width: 150, height: 30 }, 20);
	var success_outcome = create_outcome('Success', '#ffffff', { x: 100, y: 460 }, { width: 150, height: 30 }, 20);
}
