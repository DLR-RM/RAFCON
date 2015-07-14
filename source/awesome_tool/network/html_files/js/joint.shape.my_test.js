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
	var error_outcome = create_outcome('Error', '#fe2626', { x: 100, y: 100 }, { width: 150, height: 30 }, 20);
	var preemtion_outcome = create_outcome('Preempted', '#2696ff', { x: 100, y: 130 }, { width: 150, height: 30 }, 20);
	var success_outcome = create_outcome('Success', '#ffffff', { x: 100, y: 160 }, { width: 150, height: 30 }, 20);
}
