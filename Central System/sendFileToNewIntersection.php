<?php

echo 'Summary:'."</br>\n";


if ($_POST['GavilanPeakConfig']=="GavilanPeak")  
{
	echo 'Intersection  Name:  '.$_POST['GavilanPeakConfig'];
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['GavilanCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['GavilanCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['GavilanOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['GavilanSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['GavilanTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['GavilanCycle']) && isset($_POST['GavilanCoordWeight'])&& isset($_POST['GavilanOffset'])&& isset($_POST['GavilanSplit'])&& isset($_POST['GavilanTruckTransitWeight'])) {
		$data = $_POST['GavilanCoordWeight'] . '-' . $_POST['GavilanCycle'] . '-' . $_POST['GavilanOffset']. '-' . $_POST['GavilanSplit']. '-' . $_POST['GavilanTruckTransitWeight']. "\n";
		$ret = file_put_contents('myfile.txt',"");
		$ret = file_put_contents('myfile.txt', $data, FILE_APPEND | LOCK_EX);
		if($ret === false) {
			die('There was an error writing this file');
		}
		else {
			echo "\n $ret bytes written to file";
		}
	}
	else {
	   die('no post data to process');
	}

	$connection = ssh2_connect('10.254.56.35', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}


if ($_POST['DedicationConfig']=="Dedication")
{
	echo 'Intersection Name:  '.$_POST['DedicationConfig'];
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['DedicationCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['DedicationCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['DedicationOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['DedicationSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['DedicationTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['DedicationCycle']) && isset($_POST['DedicationCoordWeight'])&& isset($_POST['DedicationOffset'])&& isset($_POST['DedicationSplit'])&& isset($_POST['DedicationTruckTransitWeight'])) {
		$data = $_POST['DedicationCoordWeight'] . '-' . $_POST['DedicationCycle'] . '-' . $_POST['DedicationOffset']. '-' . $_POST['DedicationSplit']. '-' . $_POST['DedicationTruckTransitWeight']. "\n";
		$ret = file_put_contents('myfile.txt',"");
		$ret = file_put_contents('myfile.txt', $data, FILE_APPEND | LOCK_EX);
		if($ret === false) {
			die('There was an error writing this file');
		}
		else {
			echo "\n $ret bytes written to file";
		}
	}
	else {
	   die('no post data to process');
	}

	$connection = ssh2_connect('10.254.56.34', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";

}

?>
<html>
<body>
<H2> <p> File Transfer Complete! </p> </H2\>
 <a   href="intersectionsMap.html"> <H5>  << Back </H5>  </a> 
</body>
</html> 