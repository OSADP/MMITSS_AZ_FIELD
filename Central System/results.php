
<?php
    /* Database Name */
    $dbname = 'performance';

    /* Database User Name and Passowrd */
    $username = 'harsha';
    $password = '123';
    
    
	
	//$detector_num = 1;
    try 
    {
      /* Establish the database connection */
      $conn = new PDO("mysql:host=localhost;dbname=$dbname", $username, $password);
      $conn->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
		  //Fetching the data coming from the form using "quote" to avoid illegal characters
		  $detector_num = $_POST["detnumber"] ;
		  $detector_num = $conn->quote($detector_num);

    /* -----------------------select all the volume data from the table Detector ----------------------*/
		$det_result = $conn->query("SELECT time_stamp, Volume FROM Daily_Det WHERE idDetector=$detector_num");

		  $det_x = array();
		  $det_y = array();
		  
			/* Extract the information from $result */
			foreach($det_result as $det_r) 
			{
			  // the following line will be used to get the values
			  $det_x[] = (int) $det_r['time_stamp']; 
			  $det_y[] = (int) $det_r['Volume'];
			}
		// convert data into JSON format
		$det_x_jsonTable = json_encode($det_x);
		$det_y_jsonTable = json_encode($det_y);
		//echo $det_x_jsonTable;
		//echo $det_y_jsonTable;
		
	/* -----------------------select all the performance data from the table Measures ----------------------*/
		$perf_result = $conn->query("SELECT * FROM Measures WHERE idMeasures=301");
		  
		    //---------------------------- Individual Vehicle Observation Table------------------------------------
		    $perf_rows = array();
		    $perf_table = array();
		    $perf_table['cols'] = array(
			// Labels for your chart, these represent the column titles. Be careful about data types and formats!
			array('label' => 'Temporary Vehicle ID', 'type' => 'number'),
			array('label' => 'Travel Time (s)', 'type' => 'number'),
			array('label' => 'Delay (s)', 'type' => 'number'),
			array('label' => 'Number of Stops', 'type' => 'number'),
			);
			/* Extract the information from $result */
			foreach($perf_result as $perf_r) 
			{
			  $perf_temp = array();
			  // the following line will be used to get the values
			  $perf_temp[] = array('v' => (int) $perf_r['vehid']);
			  $perf_temp[] = array('v' => (float) $perf_r['TT']); 
			  $perf_temp[] = array('v' => (float) $perf_r['Delay']);
			  $perf_temp[] = array('v' => (float) $perf_r['Stops']); 
			  
			  $perf_rows[] = array('c' => $perf_temp);
			}
			$perf_table['rows'] = $perf_rows;

			// convert data into JSON format
			$perf_jsonTable = json_encode($perf_table);
			//echo $jsonTable;
			
			//----------------------------Individual Distance Traveled Chart----------------------------
		$distance_result = $conn->query("SELECT * FROM Measures WHERE idMeasures=301");
			$distance_x = array();
		    $distance_y = array();
		    
			/* Extract the information from $result */
			foreach($distance_result as $distance_r) 
			{
			  // the following line will be used to get the values
			  $distance_x[] = (int) $distance_r['vehid'];
			  $distance_y[] = (float) $distance_r['Distance'];
			}

			// convert data into JSON format
			$distance_x_jsonTable = json_encode($distance_x);
			$distance_y_jsonTable = json_encode($distance_y);
			//echo $jsonTable;
			
	/* --------------------------------------Select all the Average Performance Measures Table----------------------------*/
		$avg_result = $conn->query("SELECT * FROM Avg_Measures WHERE (rsuid=301 and idCounter<5)");
			$avg_rows = array();
			/* Extract the information from $result */
			foreach($avg_result as $avg_r) 
			{
			  // the following line will be used to get the values
			  $avg_rows[] = (float) $avg_r['Avg_TT']; 
			}
			// convert data into JSON format
			$avg_jsonTable = json_encode($avg_rows);
			//echo $avg_rows[0];
			
	/* --------------------------------------Select the Average TT for Radar Diagram----------------------------*/
		$radar_result = $conn->query("SELECT * FROM Avg_Measures WHERE rsuid=301 ");
			$radar_rows = array();
			/* Extract the information from $result */
			foreach($radar_result as $radar_r) 
			{
			  $radar_TT_rows[] = (float) $radar_r['Avg_TT'];
			  $radar_Delay_rows[] = (float) $radar_r['Avg_Delay'];
			  $radar_Stops_rows[] = (float) $radar_r['Avg_Stops'];
			}
			// convert data into JSON format
			$radar_TT_jsonTable = json_encode($radar_TT_rows);		
			$radar_Delay_jsonTable = json_encode($radar_Delay_rows);
			$radar_Stops_jsonTable = json_encode($radar_Stops_rows);			
    } 
    catch(PDOException $e) 
    {
        echo 'ERROR: ' . $e->getMessage();
    }
?>


<html>
  <head>
	<title>MMITSS WebServer</title>
	<meta name="viewport" content="width=device-width, initial-scale=1">
	<link href="results.css" type="text/css" rel="stylesheet" />
	<!--Load the Ajax API-->
	<script type="text/javascript" src="https://www.google.com/jsapi"></script>
	<script type="text/javascript" src="http://ajax.googleapis.com/ajax/libs/jquery/1.8.2/jquery.min.js"></script>
	<script src="Chart.js"></script>
	
	<script type="text/javascript">

// Load the Visualization API and the piechart package.
	google.load('visualization', '1', {'packages':['corechart']});
	// Set a callback to run when the Google Visualization API is loaded.
	
	//Drawing the Line Diagram is using chart.js which is a bit different from the Google Chart API
		var lineChartData = {
		labels: <?=$det_x_jsonTable?>,
		datasets: [
		{
		label: "Volume Data",
		fillColor: "rgba(220,220,220,0.2)", //Color of filling
		strokeColor: "rgba(0,0,255,1)", //Color of the line
		pointColor: "rgba(0,0,0,1)", //Color of Points
		pointStrokeColor: "#fff",
		pointHighlightFill: "#fff",
		pointHighlightStroke: "rgba(220,220,220,1)",
		data: <?=$det_y_jsonTable?> 
		}
		]};	
		
	//Drawing the Pie Diagram is using chart.js which is a bit different from the Google Chart AP
		var pieChartData = [
			{
				value: 9,
				color:"#F7464A",
				highlight: "#FF5A5E",
				label: "Passenger Vehicle"
			},
			{
				value: 2,
				color: "#46BFBD",
				highlight: "#5AD3D1",
				label: "Truck"
			},
			{
				value: 1,
				color: "#FDB45C",
				highlight: "#FFC870",
				label: "Pedestrian"
			},
			{
				value: 3,
				color: "#41733D",
				highlight: "#579951",
				label: "Transit"
			}
		];
	
		
	//Drawing the Bar Diagram is using chart.js which is a bit different from the Google Chart API
		var barChartData = {
		labels: <?=$distance_x_jsonTable?>,
		datasets: [
		{
		label: "Distance Data",
		fillColor: "rgba(0,0,255,0.8)", //Color of filling
		strokeColor: "rgba(0,0,255,1)", //Color of the line
		pointColor: "rgba(0,0,0,1)", //Color of Points
		pointStrokeColor: "#fff",
		pointHighlightFill: "#fff",
		pointHighlightStroke: "rgba(220,220,220,1)",
		data: <?=$distance_y_jsonTable?> 
		}
		]};
	
	//Drawing the Polar Area Diagram is using chart.js which is a bit different from the Google Chart API
		var polarChartData = [
			{
				value: <?=$avg_rows[0]?>,
				color:"#F7464A",
				highlight: "#FF5A5E",
				label: "Northbound"
			},
			{
				value: <?=$avg_rows[1]?>,
				color: "#46BFBD",
				highlight: "#5AD3D1",
				label: "Eastbound"
			},
			{
				value: <?=$avg_rows[2]?>,
				color: "#FDB45C",
				highlight: "#FFC870",
				label: "Southbound"
			},
			{
				value: <?=$avg_rows[3]?>,
				color: "#41733D",
				highlight: "#579951",
				label: "Westbound"
			}
		];
	
	google.load('visualization', '1', {'packages':['geochart']});
	google.setOnLoadCallback(GeoChart);
		function GeoChart()
		{
			var data = google.visualization.arrayToDataTable([
				['lat','long','Area','Travel Time (s)'],
				[33.842941, -112.135206,'Daisy-Gavilan',55], //Gavilan
				[33.843234, -112.131538, 'Daisy-Dedication',30], //Dedication
				[33.846938, -112.122123, 'Daisy-Meridian',31], //Meridian
				[33.850371, -112.116563, 'Daisy-Hastings',38], //Hastings
				[33.856539, -112.114215, 'Dasiy-Memorial',49], //Memorial
				[33.859896, -112.111972, 'Daisy-Anthem',61], //Anthem
				[33.424394, -111.927924, 'ASU Campus',89], //ASU Campus
				[33.354204, -112.628796, 'MC-Highway 85',54] //MC85
			]);
			var options = {
				sizeAxis: { minValue:0 , maxValue:120 },
				region: 'US-AZ',
				resolution: 'metros',
				displayMode: 'markers',
				colorAxis: {minValue:30, maxValue:60, colors: ['green','red']},
				enableRegionInteractivity: 'true',
				magnifyingGlass: {enable:true, zoomFactor:50.0},
				keeoAspectRatio: 'true'
			};
			var chart = new google.visualization.GeoChart(document.getElementById('chart_geo'));
			chart.draw(data,options);	
		}
	
	google.load('visualization', '1', {'packages':['table']});
	google.setOnLoadCallback(VehObsTable);
		function VehObsTable()
		{
			var data = new google.visualization.DataTable(<?=$perf_jsonTable?>);
			var options = {
			   showRowNumber: true,
			   allowHtml: true,
			   width: 500,
			   
			   page: 'enable'	  
			};
		  // Instantiate and draw our chart, passing in some options.
		  // Do not forget to check your div ID
		  var table = new google.visualization.Table(document.getElementById('table_ind'));
		  table.draw(data, options);		
		}
		
	//Drawing the Radar Diagram is using chart.js which is a bit different from the Google Chart API
		var radarChartData = {
		labels: ["NB", "NBRT", "EBLT", "EB", "EBRT", "SBLT", "SB", "SBRT", "WBLT", "WB", "WBRT", "NBLT"],
		datasets: [
		{
		label: "Avg. Travel Time (s)",
		fillColor: "rgba(220,220,220,0.2)", //Color of filling
		strokeColor: "rgba(0,0,255,1)", //Color of the line
		pointColor: "rgba(0,0,0,1)", //Color of Points
		pointStrokeColor: "#fff",
		pointHighlightFill: "#fff",
		pointHighlightStroke: "rgba(220,220,220,1)",
		data: <?=$radar_TT_jsonTable?> 
		}
		]};
		
	//Drawing the Radar Diagram is using chart.js which is a bit different from the Google Chart API
		var radarDelayData = {
		labels: ["NB", "NBRT", "EBLT", "EB", "EBRT", "SBLT", "SB", "SBRT", "WBLT", "WB", "WBRT", "NBLT"],
		datasets: [
		{
		label: "Avg. Delay (s)",
		fillColor: "rgba(220,220,220,0.2)", //Color of filling
		strokeColor: "rgba(0,0,255,1)", //Color of the line
		pointColor: "rgba(0,0,0,1)", //Color of Points
		pointStrokeColor: "#fff",
		pointHighlightFill: "#fff",
		pointHighlightStroke: "rgba(220,220,220,1)",
		data: <?=$radar_Delay_jsonTable?> 
		}
		]};
		
	//Drawing the Bar Diagram for Number of Stops is using chart.js which is a bit different from the Google Chart API
		var barStopsData = {
		labels: ["NB", "NBRT", "EBLT", "EB", "EBRT", "SBLT", "SB", "SBRT", "WBLT", "WB", "WBRT", "NBLT"],
		datasets: [
		{
		label: "Number of Stops Data",
		fillColor: "rgba(0,0,255,0.8)", //Color of filling
		strokeColor: "rgba(0,0,255,1)", //Color of the line
		pointColor: "rgba(0,0,0,1)", //Color of Points
		pointStrokeColor: "#fff",
		pointHighlightFill: "#fff",
		pointHighlightStroke: "rgba(220,220,220,1)",
		data: <?=$radar_Stops_jsonTable?> 
		}
		]};
		 
		
	//This parts loads up the page with different diagrams! window.onload accepts only one value
		window.onload = function()
		{
			window.myRadar = new Chart(document.getElementById("radar").getContext("2d")).Radar(radarChartData, {
			responsive: true,
			pointLabelFontStyle: "bold"
			});
			
			window.myRadar = new Chart(document.getElementById("radarDelay").getContext("2d")).Radar(radarDelayData, {
			responsive: true,
			pointLabelFontStyle: "bold"
			});
			
			window.myLine = new Chart(document.getElementById("line").getContext("2d")).Line(lineChartData, {
			responsive: true,
			pointLabelFontStyle: "bold"
			});
			
			window.myBar = new Chart(document.getElementById("bar").getContext("2d")).Bar(barChartData, {
			responsive: true,
			pointLabelFontStyle: "bold"
			});
			
			window.myBar = new Chart(document.getElementById("barStops").getContext("2d")).Bar(barStopsData, {
			responsive: true,
			pointLabelFontStyle: "bold"
			});
			
			//window.myPie = new Chart(document.getElementById("pie").getContext("2d")).Doughnut(pieChartData, {
			//animateScale:true
			//});
			
			window.myPolar = new Chart(document.getElementById("polar").getContext("2d")).PolarArea(polarChartData, {
			animateScale:true
			});
		}
		
		
		

	
	</script>
  </head>

  <body>
  
	<center> <H1> Multi-Modal Intelligent Traffic Signal Systems </H1> </center>  
	<br/>
	<div id="cv">
		<img src="./Pics/AZCV.jpg" alt="UALogo"/>
	</div>
	<center><div id="logo">

		<img src="./Pics/banner.jpg" alt="UALogo"/>
	</div></center>
	<hr size="10" noshade>
	
	<!--this is the div that will hold the charts and tables-->
	<div id="overall">
		<!--this is the div that will hold the charts and tables-->
	
		<!--<div id="table_ind"></div>-->
		
		<div id="chart_distance">
		<fieldset>
		<legend>Distance Traveled (m)</legend>
			<canvas id="bar" height="250" width="350"></canvas>
		</fieldset>		
		</div>

		<!--<div id="chart_geo"></div>-->
		<div id="chart_det">
		<fieldset>
		<legend>Volume Data (Vehicles Per Hour)</legend>
			<canvas id="line" height="250" width="350"></canvas>
		</fieldset>
		</div>
		
		<div id="chart_avg">
		<fieldset>
		<legend>Major Movement TT Estimation (s)</legend>
			<canvas id="polar" height="250" width="350"></canvas>
		</fieldset>
		</div>
		
		<div id="chart_radarDelay">
		<fieldset>
		<legend>Average Delay (s)</legend>
			<canvas id="radarDelay" height="250" width="350"></canvas>
		</fieldset>
		</div>
		
		<!--<div id="chart_mode">
			<canvas id="pie" height="250" width="450"></canvas>
		</div>-->
		
		<div id="chart_stops">
		<fieldset>
		<legend>Average Number of Stops</legend>
			<canvas id="barStops" height="250" width="350"></canvas>
		</fieldset>
		</div>		
	
		<div id="chart_radar">
		<fieldset>
		<legend>Average Travel Time (s)</legend>
			<canvas id="radar" height="250" width="350"></canvas>
		</fieldset>
		</div>
	</div>
  </body>
</html>



<?php
/*
$db = new PDO("mysql:dbname=Performance; host=localhost","root","Be@rD0wn!");
$rows = $db->query("SELECT * FROM Detector WHERE idDetector=1;");

foreach ($rows as $row)
{
	?>
	<p>The volume of Detector <?= $row["idDetector"]?> at time <?= $row["time_stamp"]?> is: <?=$row["Volume"]?></p>	
	<?php
}
*/
?>
