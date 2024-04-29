function setSetpointViaRosMessage_outputLabelID(controllerName, inputBaseID, labelID)
{
	// Convert the "controllerName" to a heading string
	var scriptname_for_php = "";
	if (controllerName == "default")
	{
		scriptname_for_php = scriptname_for_php + "rosSetSetpointDefault";
	}
	else if (controllerName == "student")
	{
		scriptname_for_php = scriptname_for_php + "rosSetSetpointStudent";
	}

	// Get the (x,y,z,yaw) values of the new setpoint
	var inputID_forX   = inputBaseID + "X";
	var inputID_forY   = inputBaseID + "Y";
	var inputID_forZ   = inputBaseID + "Z";
	var inputID_forYaw = inputBaseID + "Yaw";
	x_new   = document.getElementById(inputID_forX).value;
	y_new   = document.getElementById(inputID_forY).value;
	z_new   = document.getElementById(inputID_forZ).value;
	yaw_new = document.getElementById(inputID_forYaw).value / 57.29578;


	// Set the label to be sending
	if(labelID){document.getElementById(labelID).innerHTML = "sending...";}
	
	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();
	// Add the function to be run when the response is recieved
	xmlhttp.onreadystatechange = function()
	{
		if (this.readyState == 4 && this.status == 200)
		{
			// Construct and display the appropriate information
			var base_string = "";
			if(labelID){document.getElementById(labelID).innerHTML = base_string + this.responseText;}
			//if(labelID){document.getElementById(labelID).innerHTML = base_string + "sent";}

		}
		else
		{
			// Construct and display the appropriate information
			var base_string = "";
			var display_message = getDisplayMessageForXMLHttpRequest(this);
			if (this.readyState == 4)
			{
				if(labelID){document.getElementById(labelID).innerHTML = base_string + display_message;}
			}
			else
			{
				//if(labelID){document.getElementById(labelID).innerHTML = base_string + display_message;}
			}
		}
	};
	xmlhttp.open("GET", "callBashScript_setNewSetpoint.php?scriptname=" + scriptname_for_php + "&x=" + x_new + "&y=" + y_new + "&z=" + z_new + "&yaw=" + yaw_new, true);
	xmlhttp.send();
}