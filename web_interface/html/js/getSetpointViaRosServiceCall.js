function getSetpointViaRosServiceCall_outputLabelID(controllerName, labelID)
{
	// Convert the "controllerName" to a heading string
	var scriptname_for_php = "";
	if (controllerName == "default")
	{
		scriptname_for_php = scriptname_for_php + "rosGetSetpointDefault";
	}
	else if (controllerName == "student")
	{
		scriptname_for_php = scriptname_for_php + "rosGetSetpointStudent";
	}

	// Set the label to be sending
	if(labelID){document.getElementById(labelID).innerHTML = "requesting...";}
	
	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();
	// Add the function to be run when the response is recieved
	xmlhttp.onreadystatechange = function()
	{
		if (this.readyState == 4 && this.status == 200)
		{
			// Get the response into a string
			var response_as_string = this.responseText;
			//var response_as_struct = JSON.parse(response_as_string);

			// Check that the string is not empty
			if (response_as_string)
			{
				// NOTE: use https://regex101.com/ to get an explaination
				// of the regular expressions used

				// Get the x value
				//var regex_for_x = new RegExp("(x: )([-+]?\d*\.?\d*)");
				var regex_for_x = /(x: )([-+]?\d*\.?\d*)/;
				var found_for_x = response_as_string.match(regex_for_x);
				var x_value_as_float  = parseFloat( found_for_x[2] );
				var x_value_as_string = x_value_as_float.toFixed(3);

				// Construct and display the appropriate information
				var base_string = "received x= ";
				//if(labelID){document.getElementById(labelID).innerHTML = base_string + response_as_string;}
				if(labelID){document.getElementById(labelID).innerHTML = base_string + x_value_as_string;}
			}
			else
			{
				// Construct and display the appropriate information
				var base_string = "received empty response";
				if(labelID){document.getElementById(labelID).innerHTML = base_string;}
			}

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
	xmlhttp.open("GET", "callBashScript.php?scriptname=" + scriptname_for_php, true);
	xmlhttp.send();
}