<?php
	// Get the ID string that specifies
	// which file this is within the
	// dfall-system
	$dfall_system_file_id = $_GET['dfallfileid'];

	// Convert the file id to all lower case
	$file_id_lower = strtolower( $dfall_system_file_id );

	// Check if the uploaded file has an error:
	if ( 0 < $_FILES['file']['error'] )
	{
		// Echo the error message
		echo "Upload ERROR: error message = " . $_FILES['file']['error'] . "<br>";
	}
	// Otherwise, process the uploaded file:
	else
	{
		// Specify the dfall-system base path
		// to keep the following code cleaner
		$dfall_system_base_path = "/home/www-share/dfall/dfall-system/dfall_ws/src/dfall_pkg/";

		//echo "dfall system file id = " . $dfall_system_file_id;

		// Specify the upload location based
		// on the "dfall_system_file_id"
		if ($file_id_lower == "studentcontrollerservicecpp")
		{
			$upload_location = $dfall_system_base_path . "src/nodes/StudentControllerService.cpp";
		}
		elseif ($file_id_lower == "studentcontrolleryaml")
		{
			$upload_location = $dfall_system_base_path . "param/StudentController.yaml";
		}
		else
		{
			// Echo that the file id is NOT recognised
			echo "ERROR: file id NOT recognised, file id provided = " . $dfall_system_file_id . "<br>";
			// Specify a default upload location
			$upload_location = "uploads/unknown.txt";
		}

		// Move the uploaded file to the specified location
		move_uploaded_file($_FILES['file']['tmp_name'], $upload_location );
		
		// Echo success
		echo "Upload SUCCESSFUL.<br>";
		// Echo the from and to paths
		//echo " Moved " . $_FILES['file']['tmp_name'] . " to " . $upload_location;
		// Echo the to path
		echo "Upload location =  " . $upload_location;

	}
?>