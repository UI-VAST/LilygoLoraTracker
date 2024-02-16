var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var dataIndex = 0; // Index to keep track of data points
// Init web socket when the page loads
window.addEventListener('load', onload);


function onload(event) {
    initWebSocket();
}

function sendCutDown(){
    websocket.send("cd");
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

// When websocket is established, call the getReadings() function
function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
    console.log(event.data);
    var myObj = JSON.parse(event.data);
    var keys = Object.keys(myObj);

    for (var i = 0; i < keys.length; i++){
        var key = keys[i];
        document.getElementById(key).innerHTML = myObj[key];
    }

    //console.log(myObj["rssi"]);
    
    //addData(rssiChart, dataIndex, parseInt(myObj["rssi"]));
    //removeOldData(rssiChart);
}


var rssiData = {
    labels: [],
    datasets: [{
        label: 'RSSI',
        backgroundColor: 'rgba(75, 192, 192, 0.2)',
        borderColor: 'rgba(75, 192, 192, 1)',
        data: [],
        borderWidth: 1
    }]
};

// Initialize Chart.js
const ctx = document.getElementById('rssiChart').getContext('2d');
const rssiChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'RSSI',
            data: [],
            backgroundColor: 'rgba(54, 162, 235, 0.2)',
            borderColor: 'rgba(54, 162, 235, 1)',
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: false
            }
        }
    }
});



// Function to add data to the chart
function addData(chart, label, data) {
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(data);
    });
    chart.update();
}

// Function to remove oldest data point if more than 30 points exist
function removeOldData(chart) {
    if (chart.data.labels.length > 60) {
        chart.data.labels.shift();
        chart.data.datasets.forEach((dataset) => {
            dataset.data.shift();
        });
    }
}



