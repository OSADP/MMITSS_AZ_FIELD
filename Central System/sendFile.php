<?php

echo 'Summary:'."</br>\n";

if (isset($_POST['gavilanConfig']))  
{
	echo 'Intersection  Name:  Gavilan Peak';
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

	$connection = ssh2_connect('10.254.56.29', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}
if (isset($_POST['dedicationConfig']))  
{
	echo 'Intersection  Name:  Dedication';
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

	$connection = ssh2_connect('10.254.56.30', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}
if (isset($_POST['meridianConfig']))  
{
	echo 'Intersection  Name:  Meridian';
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['MeridianCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['MeridianCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['MeridianOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['MeridianSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['MeridianTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['MeridianCycle']) && isset($_POST['MeridianCoordWeight'])&& isset($_POST['MeridianOffset'])&& isset($_POST['MeridianSplit'])&& isset($_POST['MeridianTruckTransitWeight'])) {
		$data = $_POST['MeridianCoordWeight'] . '-' . $_POST['MeridianCycle'] . '-' . $_POST['MeridianOffset']. '-' . $_POST['MeridianSplit']. '-' . $_POST['MeridianTruckTransitWeight']. "\n";
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

	$connection = ssh2_connect('10.254.56.31', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}
if (isset($_POST['hastingConfig']))  
{
	echo 'Intersection  Name:  Hasting';
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['HastingCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['HastingCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['HastingOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['HastingSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['HastingTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['HastingCycle']) && isset($_POST['HastingCoordWeight'])&& isset($_POST['HastingOffset'])&& isset($_POST['HastingSplit'])&& isset($_POST['HastingTruckTransitWeight'])) {
		$data = $_POST['HastingCoordWeight'] . '-' . $_POST['HastingCycle'] . '-' . $_POST['HastingOffset']. '-' . $_POST['HastingSplit']. '-' . $_POST['HastingTruckTransitWeight']. "\n";
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

	$connection = ssh2_connect('10.254.56.32', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}

if (isset($_POST['memorialConfig']))  
{
	echo 'Intersection  Name:  Memorial';
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['MemorialCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['MemorialCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['MemorialOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['MemorialSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['MemorialTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['MemorialCycle']) && isset($_POST['MemorialCoordWeight'])&& isset($_POST['MemorialOffset'])&& isset($_POST['MemorialSplit'])&& isset($_POST['MemorialTruckTransitWeight'])) {
		$data = $_POST['MemorialCoordWeight'] . '-' . $_POST['MemorialCycle'] . '-' . $_POST['MemorialOffset']. '-' . $_POST['MemorialSplit']. '-' . $_POST['MemorialTruckTransitWeight']. "\n";
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

	$connection = ssh2_connect('10.254.56.33', 22);
	if (!$connection) die('Connection failed');
	ssh2_auth_password($connection, 'root', 'gems');

	ssh2_scp_send($connection, 'myfile.txt', '/nojournal/bin/configurationManager.txt', 0644);
	echo "<br/>\n";
	echo '*************************';
	echo "<br/>\n";
}


if (isset($_POST['anthemConfig']))  
{
	echo 'Intersection  Name:  Anthem';
	echo "<br/>\n";
	echo 'Coordination Weight:  '.$_POST['AnthemCoordWeight'];
	echo "<br/>\n";
	echo 'Cycle:  '.$_POST['AnthemCycle'];
	echo "<br/>\n";
	echo 'Offset:  '.$_POST['AnthemOffset'];
	echo "<br/>\n";
	echo 'Split:  '.$_POST['AnthemSplit'];
	echo "<br/>\n";
	echo 'Truck Weight vs Transit:  '.$_POST['AnthemTruckTransitWeight'];
	echo "<br/>\n";
	

	if(isset($_POST['AnthemCycle']) && isset($_POST['AnthemCoordWeight'])&& isset($_POST['AnthemOffset'])&& isset($_POST['AnthemSplit'])&& isset($_POST['AnthemTruckTransitWeight'])) {
		$data = $_POST['AnthemCoordWeight'] . '-' . $_POST['AnthemCycle'] . '-' . $_POST['AnthemOffset']. '-' . $_POST['AnthemSplit']. '-' . $_POST['AnthemTruckTransitWeight']. "\n";
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