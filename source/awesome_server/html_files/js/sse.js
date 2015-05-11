function sse() {
    if (!!window.EventSource) {
        var eventList = document.getElementById("eventlist");
        var sm_selector = document.getElementById("sm_selector");
        var source = new EventSource('/my_event_source');
        source.onmessage = function(e) {
            data_lines = e.data.split("\n")
            var newElement = document.createElement("li");

            newElement.innerHTML = "message: " + data_lines[0];
            eventList.appendChild(newElement);
        }
        source.addEventListener("registration", function(e) {
            data_lines = e.data.split("\n")
            var newElement = document.createElement("li");
            newElement.innerHTML = "New Statemachine registered: " + data_lines[0];
            eventList.appendChild(newElement);

            var newOption = document.createElement("option");

            addr = data_lines[1] + ":" + data_lines[2];
            newOption.value = addr;

            if (check_selector_options(data_lines[0])) {
                newOption.text = data_lines[0] + ":" + data_lines[2];
            } else {
                newOption.text = data_lines[0];
            }
            sm_selector.appendChild(newOption);

            if(sm_selector[0].value == "none") {
                sm_selector.removeChild(sm_selector[0]);
            }

            alert("New Statemachine registered: " + e.data);
        }, false);
        source.onerror = function(e) {
            alert("Server closed connection.");
            source.close();
        };
    }
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