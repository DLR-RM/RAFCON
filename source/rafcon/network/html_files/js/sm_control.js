function send_command(cmd) {
    host = "http://" + document.location.host;

    var sm_selector = document.getElementById("sm_selector");
    addr = sm_selector[sm_selector.selectedIndex].value;
    selected_sm = sm_selector[sm_selector.selectedIndex].text;
    
    if (addr == "none") {
    	return;	
    }
    
    if (cmd == "reload_sm") {
	    requested_reload_sm = true;
    }

    $.ajax({type:"POST", url: host, data: {"command": cmd, "sm_name": selected_sm, "addr": addr}, success: function() {}});
}

function send_reload_command() {
	send_command("reload_sm");
	send_command("resend_active_states");
}

function send_state_id(sm_state_id, viewer_state_id) {
	host = "http://" + document.location.host;
	
	$.ajax({type:"POST", url: host, data: {"sm_state_id": sm_state_id, "viewer_state_id": viewer_state_id}, success: function() {}});
}
