function send_command(cmd) {
    host = "http://" + document.location.host
    $.ajax({type:"POST", url: host, data: {"command": cmd}, success: function() {}});
}