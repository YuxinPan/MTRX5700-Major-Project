<!DOCTYPE html><html><head> 

<meta charset="utf-8" /><title>Robot Tracker</title><meta name="viewport" content="width=device-width, initial-scale=1.0"><meta name="keywords" content="Yuxin Pan"><meta name="description" content="Yuxin Pan course project."><meta name="author" content="Yuxin Pan"><link rel="shortcut icon" href="https://www.panyuxin.com/favicon.ico">
    <link rel="stylesheet" href="https://www.panyuxin.com/assets/css/bootstrap.min.css" >
    <!--<link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/font-awesome/4.7.0/css/font-awesome.min.css" >-->
<link rel="stylesheet" href="font-awesome.min.css" >

<script src="https://www.panyuxin.com/assets/js/jquery.min.js"></script>
<script src="Chart.js"></script>
</head>
<body>
<canvas id="myChart" width="100px" height="40px"></canvas>
<span id="in"></span>
<script>
function loadDoc() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (xhttp.readyState == 4 && xhttp.status == 200) {
      //document.getElementById("cookie").innerHTML = xhttp.responseText;
    }
  };
  xhttp.open("GET", "io.php?act=reset", true);
  xhttp.send();
  window.location.href = "./";

}

function loadDoc2() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (xhttp.readyState == 4 && xhttp.status == 200) {
      //document.getElementById("cookie").innerHTML = xhttp.responseText;
    }
  };
  xhttp.open("GET", "io.php?act=retract", true);
  xhttp.send();
  //window.location.href = "./";

}

function loadDoc3() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (xhttp.readyState == 4 && xhttp.status == 200) {
      //document.getElementById("cookie").innerHTML = xhttp.responseText;
    }
  };
  xhttp.open("GET", "io.php?act=release", true);
  xhttp.send();
  //window.location.href = "./";

}


var ctx = document.getElementById("myChart");
var data1='{"time":[],"voltage":[]} ';
var data1Old='';
var distance=0.0,interDist=0.0,speed= 0.0,updateInterval=0.5;

plot = function() {


var client = new XMLHttpRequest();
client.open('GET', 'data.json?t='+ Date.now());
client.onreadystatechange = function() {
  data1=client.responseText;
}
client.send();

//document.getElementById("in").innerHTML = data1;
//while (data1==''){}
obj = JSON.parse(data1);

if (data1Old==data1) return;
data1Old=data1;



if (obj.length>1){

    interDist= Math.sqrt(Math.pow(parseFloat(obj[obj.length-1].x)-parseFloat(obj[obj.length-2].x),2)+Math.pow(parseFloat(obj[obj.length-1].y)-parseFloat(obj[obj.length-2].y),2)).toFixed(2);
    speed = updateInterval*interDist;
    interDist =0.0;
    for (var i = 1; i < obj.length; i++) {
        interDist= interDist+Math.sqrt(Math.pow(parseFloat(obj[i].x)-parseFloat(obj[i-1].x),2)+Math.pow(parseFloat(obj[i].y)-parseFloat(obj[i-1].y),2));
//interDist= Math.sqrt(Math.pow(obj[i].x-obj[i-1].x,2)+Math.pow(obj[i].y-obj[i-1].y,2));
        //distance = distance+interDist;
        console.log(interDist)
    }
    interDist=interDist.toFixed(2);

    
}

//sumEnergy=2+updateInterval*tempN;
//sumEnergy=0;
//for (var i = 0; i < obj.voltage.length; i++) {
//    sumEnergy=sumEnergy+obj.voltage[i]*updateInterval;
//}
//tempN=tempN.toFixed(3);
//sumEnergy=sumEnergy.toFixed(1);
document.getElementById("power").innerHTML = speed.toString(); 
document.getElementById("energy").innerHTML = interDist.toString(); 




var data = {
    labels: obj.time,
    //type: 'scatter',
    datasets: [
        {
            label: "Robot Position (m)",
            fill: false,
            lineTension: 0.1,
            backgroundColor: "rgba(75,192,192,0.4)",
            borderColor: "rgba(75,192,192,1)",
            borderCapStyle: 'butt',
            borderDash: [],
            borderDashOffset: 0.0,
            //borderWidth: 40,
            borderJoinStyle: 'miter',
            pointBorderColor: "rgba(75,192,192,1)",
            pointBackgroundColor: "#fff",
            pointBorderWidth: 8,
            pointHoverRadius: 10,
            pointHoverBackgroundColor: "rgba(75,192,192,1)",
            pointHoverBorderColor: "rgba(220,220,220,1)",
            pointHoverBorderWidth: 2,
            pointRadius: 1,
            pointHitRadius: 10,
            //data: obj.voltage,
data: obj,
//data: [{
//y:obj.voltage,
//x:obj.time,
// }],


//            data: obj.time,
//            data.y: obj.voltage,
//         data: [{
//            x: 0,
//            y: 5
//         }, {
//            x: 5,
//            y: 10
//         }, {
//            x: 8,
//            y: 5
//         }, {
//            x: 15,
//            y: 0
//         }],

            spanGaps: false,
            showLine: false
        }
    ]
};        












var myLineChart = new Chart(ctx, {
    type: 'scatter',
    data: data,
    options: {
animation : false, 

//legend: {
 //           display: true,
  //          fontSize: 16
   //         },


title: {
            display: true,
            text: 'Robot Tracker',
            fontSize: 26
        },

scales: {

xAxes: [{
    ticks: {
        autoSkip: true,
        maxTicksLimit: 20
    }
}],
            yAxes: [{
                ticks: {
                    beginAtZero:true
                }
            }]
        }
    }

});};

setInterval(plot, 3000);
</script>  <br /><br /><br /><center> <p style="font-size:20px">
<i class="fa fa-cog fa-spin fa-fw"></i>
<span>Robot Speed: </span><span id="power"></span> m/s<br />
<i class="fa fa-cog fa-spin fa-fw"></i>
<span>Total Distance: </span><span id="energy"></span> m <br /><br /></font>
<!--
<input type="button" class="btn btn-info" value="Start Robot" onclick="loadDoc2();">
<input type="button" class="btn btn-info" value="Stop Robot" onclick="loadDoc3();"> 
-->
</center>      
<!--
<script src="https://cdnjs.cloudflare.com/ajax/libs/tether/1.4.0/js/tether.min.js"></script>
<script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-alpha.6/js/bootstrap.min.js"></script>
-->
</body></html>