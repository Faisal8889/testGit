var dataArray = [];
var dataBalance=[];

var defaultZoomTime = 1*60*60*1000; // 1 hour
var minZoom = -4; // 1 minutes 30 seconds
var maxZoom = 8; // ~ 8.4 months

var zoomLevel = 0;
var viewportEndTime = new Date();
var viewportStartTime = new Date();


getState();
loadCSV(); // Download the CSV data, load Google Charts, parse the data, and draw the chart


/*
Structure:

    loadCSV
        callback:
        parseCSV
        load Google Charts (anonymous)
            callback:
            updateViewport
                displayDate
                drawChart
*/

/*
               |                    CHART                    |
               |                  VIEW PORT                  |
invisible      |                   visible                   |      invisible
---------------|---------------------------------------------|--------------->  time
       viewportStartTime                              viewportEndTime

               |______________viewportWidthTime______________|

viewportWidthTime = 1 hour * 2^zoomLevel = viewportEndTime - viewportStartTime
*/

function getState() {
	var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            state = this.responseText;
			document.getElementById("state_div").innerHTML = "State:" + (state);
        }
    };
    xmlhttp.open("GET", "/state", true);
    xmlhttp.send();
}


function loadCSV() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            dataArray = parseCSV(this.responseText);
            google.charts.load('current', { 'packages': ['line', 'corechart'] });
            google.charts.setOnLoadCallback(updateViewport);
        }
    };
    xmlhttp.open("GET", "/temp.csv", true);
    xmlhttp.send();
    var loadingdiv = document.getElementById("loading");
    loadingdiv.style.visibility = "visible";
}

function parseCSV(string) {
    var array = [];
    var lines = string.split("\n");
    for (var i = 0; i < lines.length-1; i++) {
        var data = lines[i].split(",");
        var cells =[];
        cells[0] = new Date(parseInt(data[0]) * 1000);
		
		

		for (var j = 1; j < data.length-3; j++) {
		    cells[j] = parseFloat(data[j]/10000);
			var packNO = parseInt((j-1)/12 +25);
		    dataBalance[j-1] = data[packNO] >> ((j-1) % 12) & 1;
		}
		
        array.push(cells);
    }
	
		var balanceString = "Balancing cells \n";
	for (var i = 1; i < dataBalance.length; i++) {
		if (dataBalance[i]) {
			balanceString+= (i+1) + ", ";
		}

	}
	document.getElementById("balance_div").innerHTML = balanceString.slice(0, -1);
	
	
    return array;
}

function drawChart() {
    var data = new google.visualization.DataTable();
    data.addColumn('datetime', 'UNIX');
	
	for (var i = 1; i < dataArray[0].length; i++) {
		data.addColumn('number', 'cell' + i);
	}

    for (var i = 1; i < dataArray.length; i++) {
		 data.addRow(dataArray[i]);
	}

    var options = {
        curveType: 'function',

        height: 600,

        legend: { position: 'none' },

        hAxis: {
            viewWindow: {
                min: viewportStartTime,
                max: viewportEndTime
            },
            gridlines: {
                count: -1,
                units: {
                    days: { format: ['MMM dd'] },
                    hours: { format: ['HH:mm', 'ha'] },
                }
            },
            minorGridlines: {
                units: {
                    hours: { format: ['hh:mm:ss a', 'ha'] },
                    minutes: { format: ['HH:mm a Z', ':mm'] }
                }
            }
        },
        vAxis: {
            title: "Voltage (V)" 
        }
    };

    var chart = new google.visualization.LineChart(document.getElementById('chart_div'));

    chart.draw(data, options);

    var dateselectdiv = document.getElementById("dateselect");
    dateselectdiv.style.visibility = "visible";

    var loadingdiv = document.getElementById("loading");
    loadingdiv.style.visibility = "hidden";
}

function displayDate() { // Display the start and end date on the page
    var dateDiv = document.getElementById("date");

    var endDay = viewportEndTime.getDate();
    var endMonth = viewportEndTime.getMonth();
    var startDay = viewportStartTime.getDate();
    var startMonth = viewportStartTime.getMonth()
    if (endDay == startDay && endMonth == startMonth) {
        dateDiv.textContent = (endDay).toString() + "/" + (endMonth + 1).toString();
    } else {
        dateDiv.textContent = (startDay).toString() + "/" + (startMonth + 1).toString() + " - " + (endDay).toString() + "/" + (endMonth + 1).toString();
    }
}

document.getElementById("prev").onclick = function() {
    viewportEndTime = new Date(viewportEndTime.getTime() - getViewportWidthTime()/3); // move the viewport to the left for one third of its width (e.g. if the viewport width is 3 days, move one day back in time)
    updateViewport();
}
document.getElementById("next").onclick = function() {
    viewportEndTime = new Date(viewportEndTime.getTime() + getViewportWidthTime()/3); // move the viewport to the right for one third of its width (e.g. if the viewport width is 3 days, move one day into the future)
    updateViewport();
}

document.getElementById("zoomout").onclick = function() {
    zoomLevel += 1; // increment the zoom level (zoom out)
    if(zoomLevel > maxZoom) zoomLevel = maxZoom;
    else updateViewport();
}
document.getElementById("zoomin").onclick = function() {
    zoomLevel -= 1; // decrement the zoom level (zoom in)
    if(zoomLevel < minZoom) zoomLevel = minZoom;
    else updateViewport();
}

document.getElementById("reset").onclick = function() {
    viewportEndTime = new Date(); // the end time of the viewport is the current time
    zoomLevel = 0; // reset the zoom level to the default (one day)
    updateViewport();
}
document.getElementById("refresh").onclick = function() {
    viewportEndTime = new Date(); // the end time of the viewport is the current time
    loadCSV(); // download the latest data and re-draw the chart
	getState();
}

document.getElementById("monitor").onclick = function() {
   var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        getState();
    };
    xmlhttp.open("GET", "/monitor", true);
    xmlhttp.send();
}

document.getElementById("balance").onclick = function() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        getState();
    };
    xmlhttp.open("GET", "/charging", true);
    xmlhttp.send();
}

document.body.onresize = drawChart;

function updateViewport() {
    viewportStartTime = new Date(viewportEndTime.getTime() - getViewportWidthTime());
    displayDate();
    drawChart();
}
function getViewportWidthTime() {
    return defaultZoomTime*(2**zoomLevel); // exponential relation between zoom level and zoom time span
                                           // every time you zoom, you double or halve the time scale
}

