<?php
	// GET THE BASH SCRIPT NAME
	$scriptname = $_GET['scriptname'];

	// ONLY EXECUTE "SCRIPT NAMES" WITH AN EXACT MATCH
	//
	// > For the MASTER
	if ($scriptname == "master")
	{
		// GET THE VALUES OF THE EMULATE MOCAP FLAG
		// > This will be a string
		$emulate_mocap = strtolower( $_GET['emulatemocap'] );
		// Check that the new setpoint values are numerical
		if (in_array($emulate_mocap, array("true", "1", "yes"), true))
		{
			$emulate_mocap = "true";
		}
		elseif (in_array($emulate_mocap, array("false", "0", "no"), true))
		{
			$emulate_mocap = "false";
		}
		else
		{
			echo "emulate mocap flag = $emulate_mocap, is not a boolean value.";
			exit();
		}
		// Call the bash script for launching the master
		$output = shell_exec("./bashscripts/launchRosMaster.sh $emulate_mocap");
	}
	//
	// > For the AGENT
	elseif ($scriptname == "agent")
	{
		// GET THE VALUES OF THE EMULATE MOCAP FLAG
		// > This will be a string
		$emulate_crazyradio = strtolower( $_GET['emulatecrazyradio'] );
		// Check that the new setpoint values are numerical
		if (in_array($emulate_crazyradio, array("true", "1", "yes"), true))
		{
			$emulate_crazyradio = "true";
		}
		elseif (in_array($emulate_crazyradio, array("false", "0", "no"), true))
		{
			$emulate_crazyradio = "false";
		}
		else
		{
			echo "emulate crazyradio flag = $emulate_crazyradio, is not a boolean value.";
			exit();
		}
		// Call the bash script for launching the agent
		$output = shell_exec("./bashscripts/launchRosAgent.sh $emulate_crazyradio");
	}
	else
	{
		$output = "launch name = $scriptname is not a valid option";
	}

	echo "$output";
?>
