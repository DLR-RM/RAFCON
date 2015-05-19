function send_command(cmd) {
    host = "http://" + document.location.host;

    var sm_selector = document.getElementById("sm_selector");
    addr = sm_selector[sm_selector.selectedIndex].value;
    
    if (addr == "none") {
    	return;	
    }

    $.ajax({type:"POST", url: host, data: {"command": cmd, "addr": addr}, success: function() {}});
}

function send_state_id(sm_state_id, viewer_state_id) {
	host = "http://" + document.location.host;
	
	$.ajax({type:"POST", url: host, data: {"sm_state_id": sm_state_id, "viewer_state_id": viewer_state_id}, success: function() {}});
}
