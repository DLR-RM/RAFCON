function send_command(cmd) {
    host = "http://" + document.location.host

    var sm_selector = document.getElementById("sm_selector");
    addr = sm_selector[sm_selector.selectedIndex].value;

    $.ajax({type:"POST", url: host, data: {"command": cmd, "addr": addr}, success: function() {}});
}