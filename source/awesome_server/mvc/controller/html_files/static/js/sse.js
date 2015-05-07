function sse() {
    if (!!window.EventSource) {
        var eventList = document.getElementById("eventlist");
        var source = new EventSource('/my_event_source');
        source.onmessage = function(e) {
            var newElement = document.createElement("li");

            newElement.innerHTML = "message: " + e.data;
            eventList.appendChild(newElement);
        }
        source.onerror = function(e) {
            alert("Server closed connection.");
            source.close();
        };
    }
}