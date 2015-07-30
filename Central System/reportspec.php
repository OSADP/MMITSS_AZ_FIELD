<!DOCTYPE html>
<head>
		<title>ReportSpec</title>
		<link href="reportspec.css" type="text/css" rel="stylesheet" />
		<meta http-equiv="Content-type" content="text/html; charset=utf-8" />
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
	
	<form action="results.php" method="post">
		<div id="overall">
		<fieldset>
			<div id="Infrstructure">  
			<fieldset>
				<legend>Infrastructure Report:</legend>
				<strong>Report Type: </strong><br/>
							&nbsp; &nbsp; &nbsp; &nbsp;
							<select name="reporttype" multiple="multiple">
								<option value="Detector" selected="selected">Detectors Report</option>
								<option value="Signal">Signal Report</option>
								<option value="RSE">RSE Status Report</option>
								<option value="Cabinet">Cabinet Health Report</option>
								<option value="Doc">MC DOT Documentation</option>
							</select><br/><br/>
				<strong>Detector Data: </strong><br/>
							&nbsp; &nbsp; &nbsp; &nbsp;
							<input type="radio" name="vol/occ" value="V" checked="checked"/> Volume 
							<input type="radio" name="vol/occ" value="O" /> Occupancy <br/> 
				<strong>Detector Number: </strong><br/> 	
								&nbsp; &nbsp; &nbsp; &nbsp;
								<select name="detnumber">
										<option value="1">Detector#1: SB Right Lane</option>
										<option value="2">Detector#2: SB Left Lane</option>
										<option value="6">Detector#6: WB Right Lane</option>
										<option value="7">Detector#7: WB Center Lane</option>
										<option value="8">Detector#8: WB Left Lane</option>
										<option value="22">Detector#22: EB Right Lane</option>
										<option value="23">Detector#23: EB Center Lane</option>
										<option value="24">Detector#24: EB Left Lane</option>
									</select><br/><br/>
				<strong>Signal Measures:    </strong><br/>
									&nbsp; &nbsp; &nbsp; &nbsp;
									<input type="checkbox" name="signal" value="AOR" checked="checked"/> Arrival on Red
									<input type="checkbox" name="signal" value="AOG" /> Arrival on Green<br/>&nbsp; &nbsp; &nbsp; &nbsp;
									<input type="checkbox" name="signal" value="TSD" /> Time-Space Diagram
									<input type="checkbox" name="signal" value="split" /> Split Monitor<br/>&nbsp; &nbsp; &nbsp; &nbsp; 
									<input type="checkbox" name="signal" value="TPD" /> Time-Phase Diagram
									<input type="checkbox" name="signal" value="TMC" /> Turning Movement Counts

			</fieldset>
			</div>
			
			<div id="CV">  
			<fieldset>
				<legend>Connected Vehicle Report:</legend>
				<strong>Report Type: </strong><br/>
							&nbsp; &nbsp; &nbsp; &nbsp;
							<select name="reporttype" multiple="multiple">
								<option value="ind" selected="selected">Movement Observation Report</option>
								<option value="app_tt">Approach Travel Time</option>
								<option value="app_delay">Approach Delay</option>
								<option value="mode">Mode Distribution Report</option>
								<option value="corridor">Corridor Monitor Report</option>
							</select><br/><br/>
				<strong>Polar Chart: </strong><br/>
							&nbsp; &nbsp; &nbsp; &nbsp;
							<input type="radio" name="polar" value="M" checked="checked"/> Major Movements 
							<input type="radio" name="polar" value="m" /> Minor Movements <br/> 
				<strong>Performance Observation by: </strong><br/> 	
								&nbsp; &nbsp; &nbsp; &nbsp;
								<select name="perf">
										<option value="mode">Mode</option>
										<option value="movement">Movement</option>
									</select><br/><br/>
				<strong>Performance Metrics:    </strong><br/>
									&nbsp; &nbsp; &nbsp; &nbsp;
									<input type="checkbox" name="cvm" value="tt" checked="checked"/> Travel Time
									<input type="checkbox" name="cvm" value="delay" /> Delay
									<input type="checkbox" name="cvm" value="distance" /> Distance Traveled <br/>&nbsp; &nbsp; &nbsp; &nbsp;
									<input type="checkbox" name="cvm" value="stops" /> Number of Stops
									<input type="checkbox" name="cvm" value="throughput" /> Vehicle Throughput<br/>&nbsp; &nbsp; &nbsp; &nbsp; 
									<input type="checkbox" name="cvm" value="QLE" /> Queue Length
									<input type="checkbox" name="cvm" value="MPR" /> Market Penetration Rate

			</fieldset>
			</div>		

	</fieldset>
			<div class="submission">
			<input type="submit" name="submit" value="Show metrics" /> <br/>
			<input type="reset" /></div>
	</div>
	</form>
	
</body>
</html>
