
// main simulation script; one separate script for each scenario;
// and one separate html page where only the respective sim script is loaded

//#############################################################
// Initial settings
//#############################################################

// global appearance settings

// !!! HEINEOUS slowdown bug
// if size of background != size of background image (800x800)
// everything slower by factor of 2-3 and jerky !!!
// define canvas size in html file;
// take everything from canvas at init*() time !!!

// also see "slowdown" keyword(s) below


// graphical settings

var hasChanged=true; // window dimensions have changed (responsive design)

var drawBackground=false; // if false, default unicolor background
var drawRoad=true; // if false, only vehicles are drawn

var vmin=0; // min speed for speed colormap (drawn in red)
var vmax=100/3.6; // max speed for speed colormap (drawn in blue-violet)




// physical geometry settings

var sizePhys=290;    //responsive design
var center_xPhys=145;
var center_yPhys=-155; // ypixel downwards=> physical center <0

var roadRadius=120; // 90 change scaleInit in gui.js correspondingly
var roadLen=roadRadius*2*Math.PI;
var nLanes=1;
var laneWidth=10;

// specification of vehicle and traffic  properties

var car_length=7; // car length in m
var car_width=6; // car width in m
var truck_length=15; // trucks
var truck_width=7;

// initial parameter settings (!! transfer def to GUI if variable in sliders!)

var MOBIL_bSafe=4;
var MOBIL_bSafeMax=17;
var MOBIL_bThr=0.4;
var MOBIL_bBiasRight_car=0.05;
var MOBIL_bBiasRight_truck=0.2;
var dt_LC=4; // duration of a lane change

// simulation initial conditions settings
//(initial values and range of user-ctrl var in gui.js)

var speedInit=20; // m/s
var speedInitPerturb=13;
var relPosPerturb=0.8;
// densityInit etc in gui.js


var truckFracToleratedMismatch=0.02; // open system: need tolerance, otherwise sudden changes

// simulation initial conditions settings
//(initial values and range of user-ctrl var in gui.js)

// densityInit in gui.js



//####################################################################
// image file settings
//####################################################################



//var car_srcFile='figs/carSmall2.png'; // ringNewOrig.js
var car_srcFile='figs/blackCarCropped.gif';
var truck_srcFile='figs/truck1Small.png';
var obstacle_srcFile='figs/obstacleImg.png';
var road1lane_srcFile='figs/oneLaneRoadRealisticCropped.png';
var road2lanes_srcFile='figs/twoLanesRoadRealisticCropped.png';
var road3lanes_srcFile='figs/threeLanesRoadRealisticCropped.png';

// Notice: set drawBackground=false if no bg wanted
 var background_srcFile='figs/backgroundGrass.jpg';


var scaleFactorImg=roadRadius/280; // [pixels/m]



//#################################
// Global graphics specification
//#################################

var canvas;
var ctx;  // graphics context

var background;



//###############################################################
// physical (m) road and vehicles  specification
//###############################################################


var longModelCar;
var longModelTruck;
var LCModelCar;
var LCModelTruck;
updateModels();

var isRing=1;  // 0: false; 1: true
var roadID=1;
var mainroad=new road(roadID, roadLen, nLanes, densityInit, speedInit,
		      truckFracInit, isRing);

var veh=mainroad.veh;  // should be not needed in final stage=>onramp.js

// initial perturbation of the vehicles
// first actual action apart from constructors
// (can be seen as extension of the constructor)

var iveh=Math.floor(relPosPerturb*mainroad.veh.length);
iveh=Math.max(0, Math.min(iveh,mainroad.veh.length));
mainroad.veh[iveh].speed=speedInitPerturb;


//############################################
// run-time specification and functions
//############################################

var time=0;
var itime=0;
var fps=30; // frames per second
var dt=timewarp/fps;

//############################################
// global data
//############################################
var realtimeAvgSpeedData = [];
var realtimeMinSpeedData = [];
var realtimeMaxSpeedData = [];
var realtimeJamLenData = [];
var graphDataDomain = [{min: 0, max: 70}, {min: 0, max: 1400}];
var realtimeGraphWidth = 1200;
var realtimeXDataLength = 1000;
var xShift = 20;
var xAxisScale = [];
var yAxisScale = [];
var count = 0;
var titleOfGraph = ['Speed of Vehicles (mph)', 'Traffic Jam Length (feet)'];

function getActualWidth() {
    // console.log("canvas width="+canvas.width);
    // console.log("canvas height="+canvas.height);
    return canvas.width < canvas.height ? (canvas.width + 100) : (canvas.height + 100);
}

// var densityRange = [20, 40, 60, 80, 100];
// var accRange = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
// var decRange = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
// var writeToFile = false;
// var densityIndex = 0;
// var accIndex = 0;
// var decIndex = 0;
// var jsonExport = [];

//############################################
function updateRing(){
//############################################

    // update times

    time += dt; // dt depends on timewarp slider (fps=const)
    itime++;
    //console.log("does Math.tanh exist?");
    //console.log("Math.tanh(5)=",Math.tanh(5));

    // transfer effects from slider interaction to the vehicles and models:
    // modelparam sliders (updateModelsOfAllVehicles), density, truckFrac sliders

    mainroad.updateModelsOfAllVehicles(longModelCar,longModelTruck,
				       LCModelCar,LCModelTruck);
    mainroad.updateTruckFrac(truckFrac, truckFracToleratedMismatch);
    mainroad.updateDensity(density);

    // do central simulation update of vehicles

    mainroad.updateLastLCtimes(dt);
    mainroad.calcAccelerations();
    mainroad.changeLanes();
    mainroad.updateSpeedPositions();
    //mainroad.writeVehicles();

    //#####################################################
    // Calculate vehicle stats
    // lane
    // speed: parseFloat(this.veh[i].speed,10).toFixed(1)
    // acc: parseFloat(this.veh[i].acc,10).toFixed(1)
    // iLead: index of leading vehicle
    // iLag: index of lagging vehicle
    //
    // Don't need to consider iLeadRight/Left iLagRight/Left in Lane = 1 case
    //#####################################################

    // calculate initial density => which setting won't result in traffic jam
    var initDistance = this.roadLen/mainroad.nveh;
    // calculate avg speed, calculate traffic jam length
    var avgSpeed = 0;
    var jamCarCount = 0;
    var minSpeed = 30;
    var maxSpeed = 0;
    for (var i=0; i<this.veh.length; i++) {
        avgSpeed += this.veh[i].speed;
        if(this.veh[i].speed < 0.1) {
            jamCarCount += 1;
        }
        if(this.veh[i].speed < minSpeed) {
            minSpeed = this.veh[i].speed;
        }
        if(this.veh[i].speed > maxSpeed) {
            maxSpeed = this.veh[i].speed;
        }
    }
    avgSpeed /= this.veh.length;

    // console.log("initDistance = "+parseFloat(initDistance).toFixed(1)+ " m"
    //         +"  average speed = "+parseFloat(avgSpeed).toFixed(1)+ " m/s"
    //         +"  jam length = "+parseFloat(jamCarCount*car_length).toFixed(1) + " m"
    //         +"");
    var round = 100;
    realtimeAvgSpeedData.push(Math.round(avgSpeed * round) / round);
    realtimeJamLenData.push(jamCarCount*car_length);
    minSpeed = Math.round(minSpeed * round) / round;
    maxSpeed = Math.round(maxSpeed * round) / round;
    realtimeMinSpeedData.push(minSpeed);
    realtimeMaxSpeedData.push(maxSpeed);
    drawLine(0, realtimeAvgSpeedData);
    drawLine(1, realtimeJamLenData);

    // ############################################
    // Below code is for generating simulation data
    // ############################################
    // count++;
    // // set WriteToFile true to extract data
    // if (writeToFile) {
    //     if(count == 500) {
    //         // Store data, clean data, add index
    //         var avgLast10Speed = 0;
    //         var avgLast10JamLength = 0;
    //         var sampleLength = 4;
    //         for(var i=realtimeAvgSpeedData.length - 1; i >= (realtimeAvgSpeedData.length - sampleLength); i--) {
    //             avgLast10Speed += realtimeAvgSpeedData[i];
    //             avgLast10JamLength += realtimeJamLenData[i];
    //         }
    //         avgLast10Speed /= sampleLength;
    //         avgLast10JamLength /= sampleLength;
    //         avgLast10Speed = Math.round(avgLast10Speed * round) / round;
    //         avgLast10JamLength = Math.round(avgLast10JamLength * round) / round;
    //         jsonExport.push({
    //             "density": densityRange[densityIndex],
    //             "acceleration": accRange[accIndex],
    //             "deceleration": decRange[decIndex],
    //             "avgSpeed": avgLast10Speed,
    //             "jamLength": avgLast10JamLength,
    //             "minSpeed": minSpeed,
    //             "maxSpeed": maxSpeed,
    //             "avgSpeedAll": realtimeAvgSpeedData,
    //             "jamLengthAll": realtimeJamLenData
    //         });
    //         var key = densityRange[densityIndex] + "," + accRange[accIndex] + "," + decRange[decIndex];
    //         console.log("add key: "+key);
    //         count = 0;
    //         realtimeAvgSpeedData = [];
    //         realtimeJamLenData = [];
    //
    //         decIndex ++;
    //         if(decIndex >= decRange.length) {
    //             decIndex = 0;
    //             accIndex ++;
    //             if(accIndex >= accRange.length) {
    //                 accIndex = 0;
    //                 densityIndex ++;
    //                 if(densityIndex >= densityRange.length) {
    //                     saveText( JSON.stringify(jsonExport), "traffic_data_500_all.json" );
    //                     writeToFile = false;
    //                     return;
    //                 }
    //             }
    //         }
    //
    //         densityInit = densityRange[densityIndex] * 0.001;
    //         IDM_aInit = accRange[accIndex];
    //         IDM_bInit = decRange[decIndex];
    //         density = densityInit;
    //         IDM_a = IDM_aInit;
    //         IDM_b = IDM_bInit;
    //         IDM_v0 = IDM_v0Init;
    //         IDM_T = IDM_TInit;
    //         IDM_s0 = IDM_s0Init;
    //         clearInterval(myRun);
    //         ctx.clearRect(0, 0, canvas.width, canvas.height);
    //         updateModels();
    //
    //         // Initialize the vehicles position
    //         mainroad.veh=[];
    //         for(var i=0; i<mainroad.nveh; i++){
    //
    //           // position trucks mainly on the right lane nLanes-1
    //
    //         var lane=i%mainroad.nLanes; // left: 0; right: nLanes-1
    //         var truckFracRight=Math.min(mainroad.nLanes*truckFracInit,1);
    //         var truckFracRest=(mainroad.nLanes*truckFracInit>1)
    //             ? ((mainroad.nLanes*truckFracInit-1)/(mainroad.nLanes-1)) : 0;
    //         var truckFrac=(lane==mainroad.nLanes-1) ? truckFracRight : truckFracRest;
    //         var r=Math.random();
    //         var vehType=(Math.random()<truckFrac) ? "truck" : "car";
    //         var vehLength=(vehType == "car") ? car_length:truck_length;
    //         var vehWidth=(vehType == "car") ? car_width:truck_width;
    //
    //           // actually construct vehicles
    //
    //         mainroad.veh[i]=new vehicle(vehLength, vehWidth,
    //         			 (mainroad.nveh-i-1)*mainroad.roadLen/(mainroad.nveh+1),
    //         			 i%mainroad.nLanes, 0.8*speedInit,vehType);
    //
    //         mainroad.veh[i].longModel=(vehType == "car")
    //             ? longModelCar : longModelTruck;
    //         mainroad.veh[i].LCModel=(vehType == "car")
    //             ? LCModelCar : LCModelTruck;
    //         }
    //         veh = mainroad.veh;
    //         iveh = Math.floor(relPosPerturb*mainroad.veh.length);
    //         iveh = Math.max(0, Math.min(iveh,mainroad.veh.length));
    //         mainroad.veh[iveh].speed = speedInitPerturb;
    //         myRun = init();
    //     }
    //
    // } else {
        // drawLine(0, realtimeAvgSpeedData);
        // drawLine(1, realtimeJamLenData);
    // }

    // Add sudden brake
    // if(count == 300) {
    //     this.veh[0].speed = 0;
    // }
}

function saveText(text, filename){
    var a = document.createElement('a');
    a.setAttribute('href', 'data:text/plain;charset=utf-u,'+encodeURIComponent(text));
    a.setAttribute('download', filename);
    a.click();
}
//##################################################
// Graph
//##################################################

// initialize realtime graph
function initGraph() {
    var graphHeight = canvas.height;
    d3.select("body").select("#temporal_graph_0")
        .style("width", (screen.width - getActualWidth()) + "px")
        .style("height", (graphHeight / 2) + "px")
        .style("margin-left", (getActualWidth() - 100) + "px");

    d3.select("body").select("#temporal_graph_1")
        .style("width", (screen.width - getActualWidth()) + "px")
        .style("height", (graphHeight / 2) + "px")
        .style("margin-left", (getActualWidth() - 100) + "px");

    // FIXME: check if axis already exist
    if(d3.select("body").select("#temporal_graph_0").select("svg").node() == null) {
        initAxis(0);
        initAxis(1);
    }

}
function initAxis (graphID) {
    var graphHeight = canvas.height;
    var yAxisHeight = canvas.height / 2 - 200;
    realtimeGraphWidth = screen.width - getActualWidth() - 100;

    xAxisScale[graphID] = d3.scale.linear().domain([0, realtimeXDataLength]).range([0, realtimeGraphWidth]);
    yAxisScale[graphID] = d3.scale.linear().domain([graphDataDomain[graphID].min, graphDataDomain[graphID].max]).range([yAxisHeight, 20]);

    var graph = d3.select("body").select("#temporal_graph_"+graphID).append("svg:svg")
            .style("padding-left", 40)
            .style("padding-top", 40)
            .style("width", (screen.width - getActualWidth()) + "px")
            .style("height", (yAxisHeight + 50) + "px");
    //TODO: draw xy-Axis
    // create xAxis
	var xAxis = d3.svg.axis().scale(xAxisScale[graphID]).tickFormat("");
	// Add the x-axis.
	graph.append("svg:g")
	      .attr("class", "x axis")
	      .attr("transform", "translate("+ xShift +"," + yAxisHeight + ")")
	      .call(xAxis);

	// create left yAxis
	var yAxisLeft = d3.svg.axis().scale(yAxisScale[graphID]).orient("left");
	// Add the y-axis to the left
	graph.append("svg:g")
	      .attr("class", "y axis")
	      .attr("transform", "translate("+ xShift +","+ 0 +")")
	      .call(yAxisLeft);

    graph.append("text")
        .attr("class", "title"+graphID)
		.attr("x", realtimeGraphWidth / 2 )
        .attr("y", 0)
        .style("text-anchor", "middle")
        .text(titleOfGraph[graphID]);

    if (graphID == 0) { // add legend
        graph.append("text")
            .attr("class", "legendBlue")
            .attr("x", realtimeGraphWidth / 2 )
            .attr("y", 20)
            .style("fill", "blue")
            .style("text-anchor", "middle")
            .text("average");
        graph.append("text")
            .attr("class", "legendGreen")
            .attr("x", realtimeGraphWidth / 2 - 80)
            .attr("y", 20)
            .style("fill", "green")
            .style("text-anchor", "middle")
            .text("maximum");
        graph.append("text")
            .attr("class", "legendOrange")
            .attr("x", realtimeGraphWidth / 2 + 80)
            .attr("y", 20)
            .style("fill", "orange")
            .style("text-anchor", "middle")
            .text("minimum");
    }
}

function updateGraph(graphID) {
    // TODO: update graph size
    var graphHeight = canvas.height;
    var yAxisHeight = canvas.height / 2 - 200;
    realtimeGraphWidth = screen.width - getActualWidth() - 100;
    // var yOffset = graphID * yAxisHeight + 20;

    d3.select("body").select("#temporal_graph_"+graphID)
                .style("width", (screen.width - getActualWidth()) + "px")
                .style("height", (graphHeight / 2 - 50) + "px")
                .style("margin-left", (getActualWidth() - 100) + "px");

    var graph = d3.select("body").select("#temporal_graph_"+graphID).select("svg")
                .style("padding-left", 40)
                .style("padding-top", 40)
                .style("width", (screen.width - getActualWidth()) + "px")
                .style("height", (yAxisHeight + 50) + "px");

    xAxisScale[graphID] = d3.scale.linear().domain([0, realtimeXDataLength]).range([0, realtimeGraphWidth]);
    yAxisScale[graphID] = d3.scale.linear().domain([graphDataDomain[graphID].min, graphDataDomain[graphID].max]).range([yAxisHeight, 0]);

    var xAxis = d3.svg.axis().scale(xAxisScale[graphID]).tickFormat("");
	graph.select(".x.axis")
	      .attr("transform", "translate("+ xShift +"," + yAxisHeight  + ")")
          .call(xAxis);

	var yAxisLeft = d3.svg.axis().scale(yAxisScale[graphID]).orient("left");
	graph.select(".y.axis")
	      .attr("transform", "translate("+ xShift +","+ 0 +")")
          .call(yAxisLeft);

    graph.select(".title"+graphID)
        .attr("x", realtimeGraphWidth / 2 );
    if (graphID == 0) { // add legend
        graph.select(".legendBlue")
            .attr("x", realtimeGraphWidth / 2)
            .attr("y", 20);
        graph.select(".legendGreen")
            .attr("x", realtimeGraphWidth / 2 - 80)
            .attr("y", 20);
        graph.select(".legendOrange")
            .attr("x", realtimeGraphWidth / 2 + 80)
            .attr("y", 20);
    }

    var path = graph.append("path")
          .attr("class", "data-line"+graphID);
        //   .attr("d", line(data));
    if(graphID == 0) {
        graph.append("path")
            .attr("class", "data-line"+graphID+"min")
            .style("stroke", "orange")
            .style("stroke-width", "1")
            .style("fill", "none");
        graph.append("path")
            .attr("class", "data-line"+graphID+"max")
            .style("stroke", "green")
            .style("stroke-width", "1")
            .style("fill", "none");
    }
}

function drawLine(graphID, data) {

    // drawRealTimeGraph
    // create a line function that can convert data[] into x and y points
    var line = d3.svg.line()
		// assign the X function to plot our line as we wish
	.x(function(d,i) {
		// verbose logging to show what's actually being done
		// console.log('Plotting X value for data point: ' + d + ' using index: ' + i + ' to be at: ' + xAxisScale(i) + ' using our xScale.');
		// return the X coordinate where we want to plot this datapoint
		return xAxisScale[graphID](i) + xShift;
	})
	.y(function(d) {
		// verbose logging to show what's actually being done
		// console.log('Plotting Y value for data point: ' + d + ' to be at: ' + yAxisScale(d) + " using our yScale.");
		// return the Y coordinate where we want to plot this datapoint
        // 1 m/s = 2.23694 mph
        if(graphID == 0) {
            return yAxisScale[graphID](d * 2.23694); // m/s to mph
        } else {
            return yAxisScale[graphID](d * 3.28084); // meters to feet
        }

	})

    // var graph = d3.select("body").select("#temporal_graph").select(".realtime-graph");
    var graph = d3.select("body").select("#temporal_graph_"+graphID).select("svg")
    // pop the old data point off the front
    if (data.length > realtimeXDataLength) {
        data.shift();
        var path = graph.select("path.data-line"+graphID)
            // .attr("class", "data-line"+graphID)
            .attr("d", line(data));
        if(graphID == 0) {
            realtimeMinSpeedData.shift();
            graph.select("path.data-line"+graphID+"min")
                .attr("d", line(realtimeMinSpeedData));
            realtimeMaxSpeedData.shift();
            graph.select("path.data-line"+graphID+"max")
                .attr("d", line(realtimeMaxSpeedData));
        }
    } else {
        var path = graph.select("path.data-line"+graphID)
            // .attr("class", "data-line"+graphID)
            .attr("d", line(data));
        if(graphID == 0) {
            graph.select("path.data-line"+graphID+"min")
                .attr("d", line(realtimeMinSpeedData));
            graph.select("path.data-line"+graphID+"max")
                .attr("d", line(realtimeMaxSpeedData));
        }
    }

}

//##################################################
function drawRing() {
//##################################################

    // resize drawing region if browser's dim has changed (responsive design)
    // canvas_resize(canvas,aspectRatio)

    hasChanged=canvas_resize(canvas,0.96);
    if(hasChanged){
        console.log(" new canvas size ",canvas.width,"x",canvas.height);
        updateGraph(0);
        updateGraph(1);
    }

    // (0) reposition physical x center coordinate as response
    // to viewport size (changes)

    var aspectRatio=canvas.width/canvas.height;
    //!!center_xPhys=0.5*sizePhys*Math.max(aspectRatio,1.);


    // (1) define road geometry as parametric functions of arclength u
    // (physical coordinates!)

    function traj_x(u){
        return center_xPhys + roadRadius*Math.cos(u/roadRadius);
    }

    function traj_y(u){
        return center_yPhys + roadRadius*Math.sin(u/roadRadius);
    }

    // update heading of all vehicles rel. to road axis
    // (for some reason, strange rotations at beginning)

    mainroad.updateOrientation();



    // (2) reset transform matrix and draw background
    // (only needed if no explicit road drawn)
    // sloppy at first drawing.
    // Remind running engine at increasing time spans...

    ctx.setTransform(1,0,0,1,0,0);
    if(drawBackground){
	if(hasChanged||(itime<=1) || false || (!drawRoad)){
            ctx.drawImage(background,0,0,canvas.width,canvas.height);
	}
    }

    // (3) draw ring road
    // (always drawn; changedGeometry only triggers building a new lookup table)

    var changedGeometry=hasChanged||(itime<=1);
    mainroad.draw(roadImg,scale,traj_x,traj_y,laneWidth,changedGeometry);


    // (4) draw vehicles

    mainroad.drawVehicles(carImg,truckImg,obstacleImg,scale,traj_x,traj_y,
			  laneWidth, vmin, vmax);


    // draw some running-time vars

    ctx.setTransform(1,0,0,1,0,0);
    var textsize=0.02*Math.min(canvas.width,canvas.height); // 2vw;

    ctx.font=textsize+'px Arial';

    var timeStr="Time="+Math.round(10*time)/10;
    var timeStr_xlb=textsize;
    var timeStr_ylb=2*textsize;
    var timeStr_width=7*textsize;
    var timeStr_height=1.2*textsize;
    ctx.fillStyle="rgb(255,255,255)";
    ctx.fillRect(timeStr_xlb,timeStr_ylb-timeStr_height,timeStr_width,timeStr_height);
    ctx.fillStyle="rgb(0,0,0)";
    ctx.fillText(timeStr, timeStr_xlb+0.2*textsize, timeStr_ylb-0.2*textsize);

    var scaleStr="scale="+Math.round(10*scale)/10;
    var scaleStr_xlb=9*textsize;
    var scaleStr_ylb=2*textsize;
    var scaleStr_width=7*textsize;
    var scaleStr_height=1.2*textsize;
    ctx.fillStyle="rgb(255,255,255)";
    ctx.fillRect(scaleStr_xlb,scaleStr_ylb-scaleStr_height,scaleStr_width,scaleStr_height);
    ctx.fillStyle="rgb(0,0,0)";
    ctx.fillText(scaleStr, scaleStr_xlb+0.2*textsize, scaleStr_ylb-0.2*textsize);

/*
    var timewStr="timewarp="+Math.round(10*timewarp)/10;
    var timewStr_xlb=16*textsize;
    var timewStr_ylb=2*textsize;
    var timewStr_width=7*textsize;
    var timewStr_height=1.2*textsize;
    ctx.fillStyle="rgb(255,255,255)";
    ctx.fillRect(timewStr_xlb,timewStr_ylb-timewStr_height,timewStr_width,timewStr_height);
    ctx.fillStyle="rgb(0,0,0)";
    ctx.fillText(timewStr, timewStr_xlb+0.2*textsize, timewStr_ylb-0.2*textsize);

    var densStr="density="+Math.round(10000*density)/10;
    var densStr_xlb=24*textsize;
    var densStr_ylb=2*textsize;
    var densStr_width=7*textsize;
    var densStr_height=1.2*textsize;
    ctx.fillStyle="rgb(255,255,255)";
    ctx.fillRect(densStr_xlb,densStr_ylb-densStr_height,densStr_width,densStr_height);
    ctx.fillStyle="rgb(0,0,0)";
    ctx.fillText(densStr, densStr_xlb+0.2*textsize, densStr_ylb-0.2*textsize);


    var genVarStr="truckFrac="+Math.round(100*truckFrac)+"\%";
    var genVarStr_xlb=32*textsize;
    var genVarStr_ylb=2*textsize;
    var genVarStr_width=7.2*textsize;
    var genVarStr_height=1.2*textsize;
    ctx.fillStyle="rgb(255,255,255)";
    ctx.fillRect(genVarStr_xlb,genVarStr_ylb-genVarStr_height,
		 genVarStr_width,genVarStr_height);
    ctx.fillStyle="rgb(0,0,0)";
    ctx.fillText(genVarStr, genVarStr_xlb+0.2*textsize,
		 genVarStr_ylb-0.2*textsize);
    */

    // (6) draw the speed colormap

    drawColormap(scale*(center_xPhys-0.03*roadRadius),
                -scale*(center_yPhys+0.40*roadRadius),
		 scale*50, scale*50,
		 vmin,vmax,0,100/3.6);


    // revert to neutral transformation at the end!
    ctx.setTransform(1,0,0,1,0,0);

}


//############################################
// initialization function of the simulation thread
// THIS function does all the things; everything else only functions
// ultimately called by init()
// activation of init:
// (i) automatically when loading the simulation ("var myRun=init();" below)
// (ii) when pressing the start button defined in onramp_gui.js ("myRun=init();")
// "var ..." Actually does something;
// function keyword [function fname(..)] defines only
//############################################

// in init() definition no "var" (otherwise local to init())
// "init()" since first scenario=onramp.
// Called at the end of this script and in *_gui.js (corresp. sim button)

function init() {
    canvas = document.getElementById("canvas_ring"); // "canvas_ring" defined in ring.html
    ctx = canvas.getContext("2d");

    // init background image
    // and width and height of the sim window
    // and centering of the road image

    // !! although background.naturalWidth etc gives correct real width,
    // setting this dimension as width does NOT resolve slowdown bug.
    // direct setting  (width=800 ...) does not resolve it as well.
    // ONLY heritage from html (width = canvas.width ...) does it!!!

    background = new Image();
    background.src = background_srcFile;
    //background.onload = function() {
    //  console.log("image size of background:"+this.width +'x'+ this.height);
    //}

    //console.log("image size of background:"+background.naturalWidth);

    width = canvas.width;
    height = canvas.height;
    //width = background.naturalWidth
    //height = background.naturalHeight
    //width=800;
    //height=800;

    center_x=0.50*width*scaleFactorImg;
    center_y=0.48*height*scaleFactorImg;


	// init background image

    background = new Image();
    background.src =background_srcFile;

	// init vehicle image(s)

    carImg = new Image();
    carImg.src = car_srcFile;
    truckImg = new Image();
    truckImg.src = truck_srcFile;
    obstacleImg = new Image();  // only pro forma (no obstacles here)
    obstacleImg.src = obstacle_srcFile;

	// init road image(s)

    roadImg = new Image();
    roadImg.src=(nLanes==1)
	? road1lane_srcFile
	: (nLanes==2) ? road2lanes_srcFile
	: road3lanes_srcFile;


    // apply externally functions of mouseMove events  to initialize sliders settings

    change_timewarpSliderPos(timewarp);
    change_densitySliderPos(density);
    //change_scaleSliderPos(scale);
    change_truckFracSliderPos(truckFrac);

    change_IDM_v0SliderPos(IDM_v0);
    change_IDM_TSliderPos(IDM_T);
    change_IDM_s0SliderPos(IDM_s0);
    change_IDM_aSliderPos(IDM_a);
    change_IDM_bSliderPos(IDM_b);

    // TODO: initialize graph
    initGraph();

    // starts simulation thread "main_loop" (defined below)
    // with update time interval 1000/fps milliseconds
    // thread starts with "var myRun=init();" or "myRun=init();" (below)
    // thread stops with "clearInterval(myRun);"

    return setInterval(main_loop, 1000/fps);
} // end init()


//##################################################
// Running function of the sim thread (triggered by setInterval)
//##################################################

function main_loop() {
    drawRing();
    updateRing();
    //mainroad.writeVehicles(); // for debugging
}


//##################################################
// Actual start of the simulation thread
// (also started from gui.js "Ring Road" button)
// everything w/o function keyword [function f(..)]" actually does something, not only def
//##################################################
// init() ends with return setInterval(main_loop,1000/fps);


 var myRun=init(); //if start with ring road: init, starts thread "main_loop"
// var myRun; // starts with empty canvas; can be started with "start" button
// init(); //[w/o var]: starts as well but not controllable by start/stop button (no ref)
// myRun=init(); // selber Effekt wie "var myRun=init();"
// (aber einmal "var"=guter Stil, geht aber implizit auch ohne: Def erstes Mal, dann ref)
