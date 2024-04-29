<?php
	// GET THE BASH SCRIPT NAME
	$scriptname = $_GET['scriptname'];

	// GET THE (x,y,z,yaw) VALUES OF THE NEW SETPOINT
	// > These will be strings
	$x_new = $_GET['x'];
	$y_new = $_GET['y'];
	$z_new = $_GET['z'];
	$yaw_new = $_GET['yaw'];


	// Check that the new setpoint values are numerical
	if ( ! (is_numeric($x_new)) )
	{
		echo "x = $x_new, is not a numeric value.";
		exit();
	}
	if ( ! (is_numeric($y_new)) )
	{
		echo "y = $y_new, is not a numeric value.";
		exit();
	}
	if ( ! (is_numeric($z_new)) )
	{
		echo "z = $z_new, is not a numeric value.";
		exit();
	}
	if ( ! (is_numeric($yaw_new)) )
	{
		echo "yaw = $yaw_new, is not a numeric value.";
		exit();
	}

	// ONLY EXECUTE "SCRIPT NAMES" WITH AN EXACT MATCH
	// For the CONTROL tab:
	// GET SETPOINT
	if ($scriptname == "rosSetSetpointDefault") {
		$output = shell_exec("./bashscripts/rosSetNewSetpoint_forAgent.sh default $x_new $y_new $z_new $yaw_new");
	}
	elseif ($scriptname == "rosSetSetpointStudent") {
		$output = shell_exec("./bashscripts/rosSetNewSetpoint_forAgent.sh student $x_new $y_new $z_new $yaw_new");
	}


	echo "$output";
?>
