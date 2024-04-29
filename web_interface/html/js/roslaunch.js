function roslaunch_outputLabelID_clearOtherLabels_clickOtherButtons(launchName, labelID, otherLabels, otherButtons)
{
	// Convert the "controllerName" to a heading string
	var scriptname_for_php = "";
	if (launchName == "master")
	{
		scriptname_for_php = "master";
	}
	else if (launchName == "agent")
	{
		scriptname_for_php = "agent";
	}
	else
	{
		return;
	}

	// Get the booleans for emulation
	emulated_mocap      = document.getElementById("checkboxEmulateMocap").checked;
	emulated_crazyradio = document.getElementById("checkboxEmulateCrazyRadio").checked;

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
			// Click the "other buttons" as requested
			if (otherButtons)
			{
				if ( typeof(otherButtons) == "string" )
				{
					document.getElementById(otherButtons).click();
				}
				else if (otherButtons.constructor === Array)
				{
					for (otherButtonID of otherButtons)
					{
						if (otherButtonID)
						{
							if ( typeof(otherButtonID) == "string" )
							{
								document.getElementById(otherButtonID).click();
							}
						}
					}
				}
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
	if (launchName == "master")
	{
		xmlhttp.open("GET", "callBashScript_roslaunch.php?scriptname=" + scriptname_for_php + "&emulatemocap=" + emulated_mocap, true);
		xmlhttp.send();
	}
	else if (launchName == "agent")
	{
		xmlhttp.open("GET", "callBashScript_roslaunch.php?scriptname=" + scriptname_for_php + "&emulatecrazyradio=" + emulated_crazyradio, true);
		xmlhttp.send();
	}
	else
	{
		if(labelID){document.getElementById(labelID).innerHTML = "ERROR: launch name = " + launchName + " is not a valid option";}
	}
	

	// Clear the other labels as requested
	if (otherLabels)
	{
		if ( typeof(otherLabels) == "string" )
		{
			document.getElementById(otherLabels).innerHTML = "";
		}
		else if (otherLabels.constructor === Array)
		{
			for (otherLabelID of otherLabels)
			{
				if (otherLabelID)
				{
					if ( typeof(otherLabelID) == "string" )
					{
						document.getElementById(otherLabelID).innerHTML = "";
					}
				}
			}
		}
	}
}