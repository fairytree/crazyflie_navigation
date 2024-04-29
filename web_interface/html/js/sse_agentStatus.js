let agentStatusEventSource;

window.onload = function() {
	document.getElementById("checkboxTopBarStatus").checked = false;

	initializeFileInputs();
};

function checkboxTopBarStatus_changed()
{
	if ( document.getElementById("checkboxTopBarStatus").checked )
	{
		severSentAgentStatus_start();
	}
	else
	{
		severSentAgentStatus_stop();
	}
}


function severSentAgentStatus_start()
{
	// Check is server sent events are supported
	if (typeof(EventSource) == "undefined")
	{
		// Server-sent events are NOT supported
		// Display this to the user
		document.getElementById("agentStatusDebuggingDiv").innerHTML = "Server Sent Events are not supported by this browser.";
		return;
	}

	// If the code makes it to here, then
	// server sent events are supported

	// Create the "EventSource" variable
	agentStatusEventSource = new EventSource("sse_agentStatus.php");

	// Handle the "onopen" event
	agentStatusEventSource.onopen = function(eventSource)
	{
		// DEBUGGING: print out
		//document.getElementById("agentStatusDebuggingDiv").innerHTML += "Event: on open <br>";
	};

	// Handle the "onerror" event
	agentStatusEventSource.onerror = function(eventSource)
	{
		// DEBUGGING: print out
		document.getElementById("agentStatusDebuggingDiv").innerHTML += "Event: on error <br>";

		// Check is the event is still connecting
		if (this.readyState == EventSource.CONNECTING)
		{
			// DEBUGGING: print out
			document.getElementById("agentStatusDebuggingDiv").innerHTML += "Reconnecting (readyState = " + this.readyState + ")... <br>";
		}
		else
		{
			// DEBUGGING: print out
			document.getElementById("agentStatusDebuggingDiv").innerHTML += "Error has occured. <br>";
		}
	};

	// Handle the "message" event
	//agentStatusEventSource.onmessage = function(eventSource)
	agentStatusEventSource.addEventListener("message", function(eventSource)
	{
		// DEBUGGING: print out the whole message
		//document.getElementById("agentStatusDebuggingDiv").innerHTML += "Event: message, data: " + eventSource.data + " <br>";
	});

	// Handle the "ping" event
	agentStatusEventSource.addEventListener("agentstatus", function(eventSource)
	{
		// Extract "agent found" entry in the json
		agent_found = JSON.parse(eventSource.data).agentfound;

		// If the agent is found
		if ( agent_found === "true" )
		{
			// Parse the status dictionary from the JSON
			status_dict = JSON.parse(eventSource.data).statusjson;
			
			// Call the fuctions to update the status icons
			// > For the Radio status:		
			updateStatusIconRadio( status_dict.crazyradiostatus );
			// > For the Battery level:
			updateStatusIconBattery( status_dict.batterylevel );
			// > For the Flying state:
			updateStatusIconFlyingState( status_dict.flyingstate );
			// > For the Instant controller:
			updateStatusForInstantController( status_dict.instantcontroller );

			// Call the function to update the "measurement,
			// setpoint, error" tables
			// > For the Default controller:
			updateDefaultMseTable( status_dict.stateestimate , status_dict.setpointdefault );
			// > For the Student controller:
			updateStudentMseTable( status_dict.stateestimate , status_dict.setpointstudent );

			// Call the function to update the "debug values" table
			updateStudentDebugValuesTable( status_dict.debugvaluesstudent );

			// DEBUGGING: print out the whole message
			//document.getElementById("agentStatusDebuggingDiv").innerHTML += "Event: agent status, data: " + eventSource.data + " <br>";
		}
		// Otherwise, the agent is NOT found
		else
		{
			// So set all the icons accordingly
			// > For the Radio status:		
			updateStatusIconRadio( "disconnected" );
			// > For the Battery level:
			updateStatusIconBattery( "unavailable" );
			// > For the Flying state:
			updateStatusIconFlyingState( "unavailable" );
			// > For the Instant controller:
			updateStatusForInstantController( "none" );

			// Call the function to clear the "measurement,
			// setpoint, error" tables
			// > For the Default controller:
			clearDefaultMseTable();
			// > For the Student controller:
			clearStudentMseTable();

			// Call the function to clear the "debug values" table
			clearStudentDebugValuesTable();

			// DEBUGGING: print out the whole message
			//document.getElementById("agentStatusDebuggingDiv").innerHTML += "Event: agent status, data: " + eventSource.data + " <br>";
		}
	});

} // END OF: function severSentAgentStatus_start()





function severSentAgentStatus_stop()
{
	// Check is server sent events are supported
	if (typeof(EventSource) == "undefined")
	{
		// Server-sent events are NOT supported
		// Display this to the user
		document.getElementById("agentStatusDebuggingDiv").innerHTML = "Server Sent Events are not supported by this browser.";
		return;
	}

	// If the code makes it to here, then
	// server sent events are supported

	// Close the "EventSource" variable
	agentStatusEventSource.close();

	// DEBUGGING: print out
	//document.getElementById("agentStatusDebuggingDiv").innerHTML += "Closed. <br>";

	// So set all the icons accordingly
	// > For the Radio status:		
	updateStatusIconRadio( "disconnected" );
	// > For the Battery level:
	updateStatusIconBattery( "unavailable" );
	// > For the Flying state:
	updateStatusIconFlyingState( "unavailable" );
	// > For the Instant controller:
	updateStatusForInstantController( "none" );

	// Call the function to clear the "measurement,
	// setpoint, error" tables
	// > For the Default controller:
	clearDefaultMseTable();
	// > For the Student controller:
	clearStudentMseTable();

	// Call the function to clear the "debug values" table
	clearStudentDebugValuesTable();

} // END OF: function severSentAgentStatus_stop()



function updateStatusIconRadio( status_string )
{
	if ( typeof status_string !== "string" )
	{
		// Default to disconnected
		status_string = "disconnected";
	}

	switch ( status_string.toLowerCase() )
	{
		case "connected":
			document.getElementById("radio-icon").src = "img/rf_connected.png";
			break;
		case "connecting":
			document.getElementById("radio-icon").src = "img/rf_connecting.png";
			break;
		case "disconnected":
			document.getElementById("radio-icon").src = "img/rf_disconnected.png";
			break;
		default:
			document.getElementById("radio-icon").src = "img/rf_disconnected.png";
	}

} // END OF: "function updateStatusIconRadio( status_string )"



function updateStatusIconBattery( level_string )
{
	if ( typeof level_string !== "string" )
	{
		// Default to unavailable
		level_string = "unavailable";
	}

	switch ( level_string.toLowerCase() )
	{
		case "000":
			document.getElementById("battery-icon").src = "img/battery_empty.png";
			break;
		case "010":
			document.getElementById("battery-icon").src = "img/battery_20.png";
			break;
		case "020":
			document.getElementById("battery-icon").src = "img/battery_20.png";
			break;
		case "030":
			document.getElementById("battery-icon").src = "img/battery_40.png";
			break;
		case "040":
			document.getElementById("battery-icon").src = "img/battery_40.png";
			break;
		case "050":
			document.getElementById("battery-icon").src = "img/battery_60.png";
			break;
		case "060":
			document.getElementById("battery-icon").src = "img/battery_60.png";
			break;
		case "070":
			document.getElementById("battery-icon").src = "img/battery_80.png";
			break;
		case "080":
			document.getElementById("battery-icon").src = "img/battery_80.png";
			break;
		case "090":
			document.getElementById("battery-icon").src = "img/battery_full.png";
			break;
		case "100":
			document.getElementById("battery-icon").src = "img/battery_full.png";
			break;
		case "unknown":
			document.getElementById("battery-icon").src = "img/battery_unknown.png";
			break;
		case "unavailable":
			document.getElementById("battery-icon").src = "img/battery_unavailable.png";
			break;
		default:
			document.getElementById("battery-icon").src = "img/battery_unavailable.png";
	}

} // END OF: "function updateStatusIconBattery( level_string )"



function updateStatusIconFlyingState( state_string )
{
	if ( typeof state_string !== "string" )
	{
		// Default to unavailable
		state_string = "unavailable";
	}

	switch ( state_string.toLowerCase() )
	{
		case "motorsoff":
			document.getElementById("flying-state-icon").src = "img/flying_state_off.png";
			break;
		case "takeoff":
			document.getElementById("flying-state-icon").src = "img/flying_state_enabling.png";
			break;
		case "flying":
			document.getElementById("flying-state-icon").src = "img/flying_state_flying.png";
			break;
		case "land":
			document.getElementById("flying-state-icon").src = "img/flying_state_disabling.png";
			break;
		case "unavailable":
			document.getElementById("flying-state-icon").src = "img/flying_state_unavailable.png";
			break;
		default:
			document.getElementById("flying-state-icon").src = "img/flying_state_unavailable.png";
	}

} // END OF: "function updateStatusIconRadio( status_string )"



function updateStatusForInstantController( controller_string )
{
	if ( typeof controller_string !== "string" )
	{
		// Default to unavailable
		controller_string = "none";
	}

	// Colours:
	// #c43c35    Dark Red
	// #f44336    Light Red
	// #57a957    Green

	// Set all border colours to red
	document.getElementById("control-tab-panel-default").style.borderColor = "#c43c35";
	document.getElementById("control-tab-panel-student").style.borderColor = "#c43c35";

	switch ( controller_string.toLowerCase() )
	{
		case "default":
			document.getElementById("control-tab-panel-default").style.borderColor = "#57a957";
			break;
		case "student":
			document.getElementById("control-tab-panel-student").style.borderColor = "#57a957";
			break;
	}

} // END OF: "function updateStatusIconRadio( status_string )"


function updateDefaultMseTable( measurement_dict , setpoint_dict )
{
	if ( (typeof measurement_dict !== "object") || (typeof setpoint_dict !== "object") )
	{
		// Call function to set fields to blank
		clearDefaultMseTable();
		return;
	}

	document.getElementById("default-measurement-x").innerHTML     = measurement_dict.x.toFixed(3);
	document.getElementById("default-measurement-y").innerHTML     = measurement_dict.y.toFixed(3);
	document.getElementById("default-measurement-z").innerHTML     = measurement_dict.z.toFixed(3);
	document.getElementById("default-measurement-yaw").innerHTML   = measurement_dict.yaw.toFixed(3);
	document.getElementById("default-measurement-pitch").innerHTML = measurement_dict.pitch.toFixed(1);
	document.getElementById("default-measurement-roll").innerHTML  = measurement_dict.roll.toFixed(1);

	document.getElementById("default-setpoint-x").innerHTML     = setpoint_dict.x.toFixed(3);
	document.getElementById("default-setpoint-y").innerHTML     = setpoint_dict.y.toFixed(3);
	document.getElementById("default-setpoint-z").innerHTML     = setpoint_dict.z.toFixed(3);
	document.getElementById("default-setpoint-yaw").innerHTML   = setpoint_dict.yaw.toFixed(1);

	document.getElementById("default-error-x").innerHTML     = (measurement_dict.x  -setpoint_dict.x  ).toFixed(3);
	document.getElementById("default-error-y").innerHTML     = (measurement_dict.y  -setpoint_dict.y  ).toFixed(3);
	document.getElementById("default-error-z").innerHTML     = (measurement_dict.z  -setpoint_dict.z  ).toFixed(3);
	document.getElementById("default-error-yaw").innerHTML   = (measurement_dict.yaw-setpoint_dict.yaw).toFixed(1);

} // END OF: "function updateDefaultMseTable( measurement_dict , setpoint_dict )"

function clearDefaultMseTable()
{
	document.getElementById("default-measurement-x").innerHTML     = "xx.xx";
	document.getElementById("default-measurement-y").innerHTML     = "xx.xx";
	document.getElementById("default-measurement-z").innerHTML     = "xx.xx";
	document.getElementById("default-measurement-yaw").innerHTML   = "xx.xx";
	document.getElementById("default-measurement-pitch").innerHTML = "xx.xx";
	document.getElementById("default-measurement-roll").innerHTML  = "xx.xx";

	document.getElementById("default-setpoint-x").innerHTML     = "xx.xx";
	document.getElementById("default-setpoint-y").innerHTML     = "xx.xx";
	document.getElementById("default-setpoint-z").innerHTML     = "xx.xx";
	document.getElementById("default-setpoint-yaw").innerHTML   = "xx.xx";

	document.getElementById("default-error-x").innerHTML     = "xx.xx";
	document.getElementById("default-error-y").innerHTML     = "xx.xx";
	document.getElementById("default-error-z").innerHTML     = "xx.xx";
	document.getElementById("default-error-yaw").innerHTML   = "xx.xx";

} // END OF: "function clearDefaultMseTable()"



function updateStudentMseTable( measurement_dict , setpoint_dict )
{
	if ( (typeof measurement_dict !== "object") || (typeof setpoint_dict !== "object") )
	{
		// Call function to set fields to blank
		clearStudentMseTable();
		return;
	}

	document.getElementById("student-measurement-x").innerHTML     = measurement_dict.x.toFixed(3);
	document.getElementById("student-measurement-y").innerHTML     = measurement_dict.y.toFixed(3);
	document.getElementById("student-measurement-z").innerHTML     = measurement_dict.z.toFixed(3);
	document.getElementById("student-measurement-yaw").innerHTML   = measurement_dict.yaw.toFixed(3);
	document.getElementById("student-measurement-pitch").innerHTML = measurement_dict.pitch.toFixed(1);
	document.getElementById("student-measurement-roll").innerHTML  = measurement_dict.roll.toFixed(1);

	document.getElementById("student-setpoint-x").innerHTML     = setpoint_dict.x.toFixed(3);
	document.getElementById("student-setpoint-y").innerHTML     = setpoint_dict.y.toFixed(3);
	document.getElementById("student-setpoint-z").innerHTML     = setpoint_dict.z.toFixed(3);
	document.getElementById("student-setpoint-yaw").innerHTML   = setpoint_dict.yaw.toFixed(1);

	document.getElementById("student-error-x").innerHTML     = (measurement_dict.x  -setpoint_dict.x  ).toFixed(3);
	document.getElementById("student-error-y").innerHTML     = (measurement_dict.y  -setpoint_dict.y  ).toFixed(3);
	document.getElementById("student-error-z").innerHTML     = (measurement_dict.z  -setpoint_dict.z  ).toFixed(3);
	document.getElementById("student-error-yaw").innerHTML   = (measurement_dict.yaw-setpoint_dict.yaw).toFixed(1);

} // END OF: "function updateStudentMseTable( measurement_dict , setpoint_dict )"

function clearStudentMseTable()
{
	document.getElementById("student-measurement-x").innerHTML     = "xx.xx";
	document.getElementById("student-measurement-y").innerHTML     = "xx.xx";
	document.getElementById("student-measurement-z").innerHTML     = "xx.xx";
	document.getElementById("student-measurement-yaw").innerHTML   = "xx.xx";
	document.getElementById("student-measurement-pitch").innerHTML = "xx.xx";
	document.getElementById("student-measurement-roll").innerHTML  = "xx.xx";

	document.getElementById("student-setpoint-x").innerHTML     = "xx.xx";
	document.getElementById("student-setpoint-y").innerHTML     = "xx.xx";
	document.getElementById("student-setpoint-z").innerHTML     = "xx.xx";
	document.getElementById("student-setpoint-yaw").innerHTML   = "xx.xx";

	document.getElementById("student-error-x").innerHTML     = "xx.xx";
	document.getElementById("student-error-y").innerHTML     = "xx.xx";
	document.getElementById("student-error-z").innerHTML     = "xx.xx";
	document.getElementById("student-error-yaw").innerHTML   = "xx.xx";

} // END OF: "function clearStudentMseTable()"




function updateStudentDebugValuesTable( values_dict )
{
	if (typeof values_dict !== "object")
	{
		// Call function to set fields to blank
		clearStudentDebugValuesTable();
		return;
	}

	document.getElementById("debug-value1").innerHTML  = values_dict.value1.toFixed(3);
	document.getElementById("debug-value2").innerHTML  = values_dict.value2.toFixed(3);
	document.getElementById("debug-value3").innerHTML  = values_dict.value3.toFixed(3);
	document.getElementById("debug-value4").innerHTML  = values_dict.value4.toFixed(3);
	document.getElementById("debug-value5").innerHTML  = values_dict.value5.toFixed(3);
	document.getElementById("debug-value6").innerHTML  = values_dict.value6.toFixed(3);
	document.getElementById("debug-value7").innerHTML  = values_dict.value7.toFixed(3);
	document.getElementById("debug-value8").innerHTML  = values_dict.value8.toFixed(3);
	document.getElementById("debug-value9").innerHTML  = values_dict.value9.toFixed(3);
	document.getElementById("debug-value10").innerHTML = values_dict.value10.toFixed(3);

} // END OF: "function updateStudentDebugValuesTable( values_dict )"

function clearStudentDebugValuesTable()
{
	document.getElementById("debug-value1").innerHTML  = "xx.xx";
	document.getElementById("debug-value2").innerHTML  = "xx.xx";
	document.getElementById("debug-value3").innerHTML  = "xx.xx";
	document.getElementById("debug-value4").innerHTML  = "xx.xx";
	document.getElementById("debug-value5").innerHTML  = "xx.xx";
	document.getElementById("debug-value6").innerHTML  = "xx.xx";
	document.getElementById("debug-value7").innerHTML  = "xx.xx";
	document.getElementById("debug-value8").innerHTML  = "xx.xx";
	document.getElementById("debug-value9").innerHTML  = "xx.xx";
	document.getElementById("debug-value10").innerHTML = "xx.xx";
} // END OF: "function clearStudentDebugValuesTable()"