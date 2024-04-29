function initializeFileInputs()
{
	// Get the input fields into variables
	var cppfileinput  = document.getElementById("cppfileinput");
	var yamlfileinput = document.getElementById("yamlfileinput");

	// Add an event listener to each
	cppfileinput.addEventListener( "change", cppFileInputChangedEvent,  false);
	yamlfileinput.addEventListener("change", yamlFileInputChangedEvent, false);
}



function cppFileInputChangedEvent(event)
{
	// Get the file list object
	// > There should only be one file
	var files_for_upload = event.target.files;

	// Parse the file
	parseFileDetails( files_for_upload[0] , "cppfiledetails" );

	// Upload the file
	uploadFile( files_for_upload[0] , "StudentControllerServiceCpp" , "any" , "cppfileuploadstatus" );
}



function yamlFileInputChangedEvent(event)
{
	// Get the file list object
	// > There should only be one file
	var files_for_upload = event.target.files;

	// Parse the file
	parseFileDetails( files_for_upload[0] , "yamlfiledetails" );

	// Upload the file
	uploadFile( files_for_upload[0] , "StudentControllerYaml" , "any" , "yamlfileuploadstatus" );
}



function parseFileDetails(file_for_upload,detailsID)
{
	document.getElementById(detailsID).innerHTML = 
		"<p>File information: <br>" +
		"name: <strong>" + file_for_upload.name + "</strong><br>" +
		"type: <strong>" + file_for_upload.type + "</strong><br>" +
		"size: <strong>" + file_for_upload.size/1000.0 + "</strong> kilobytes" +
		"</p>";
}


function uploadFile(file_for_upload,file_id_within_dfall_system,expectedFileType,statusID)
{
	// Create a variable for sending an AJAX request
	var xmlhttp = new XMLHttpRequest();

	// Check that uploads are possible
	var flag_uploads_possible = false;
	if (xmlhttp.upload)
	{
		flag_uploads_possible = true;
	}

	// Perform checks on the file type:
	// > Note: a cpp file type is:    "text/x-c++src"
	//         a yaml file type is:   "application/x-yaml"
	var flag_file_type_isOk = false;
	if (expectedFileType == "any")
	{
		flag_file_type_isOk = true;
	}
	else if (file_for_upload.type == expectedFileType)
	{
		flag_file_type_isOk = true;
	}

	// Perform checks on the file size:
	var flag_file_size_isOk = false;
	if (file_for_upload.size <= document.getElementById("MAX_FILE_SIZE").value)
	{
		flag_file_size_isOk = true;
	}

	// Perform the upload if all checks are passed
	if ( flag_uploads_possible && flag_file_type_isOk && flag_file_size_isOk)
	{
		// Initialise the upload progress field
		document.getElementById(statusID).innerHTML = "Upload progress 0%";

		// Respond to progress update events
		xmlhttp.upload.addEventListener("progress", function(event)
		{
			var percent_complete = parseInt( event.loaded / event.total * 100 );
			document.getElementById(statusID).innerHTML = "Upload progress " + percent_complete + "%";
		}, false);

		// Respond to "ready state change" events
		xmlhttp.onreadystatechange = function(event)
		{
			if (this.readyState == 4 && this.status == 200)
			{
				var base_string = "";
				document.getElementById(statusID).innerHTML = base_string + this.responseText;
			}
		};

		// Put the file_for_upload into a "FormData" object
		var form_data_for_upload = new FormData();
		form_data_for_upload.append( 'file' , file_for_upload );

		// Start the upload
		// > The syntax is the third argument is an optional
		//   boolean for whether the request is asynchronous
		//   or not.
		xmlhttp.open("POST", "upload_dfall_system_file.php?dfallfileid=" + file_id_within_dfall_system, true);
		xmlhttp.send(form_data_for_upload);

	}
	else
	{
		// Display an "ERROR" status
		document.getElementById(statusID).innerHTML = "<strong>ERROR:</strong>";
		// Check if upload was the cause of the error:
		if (!(flag_uploads_possible))
		{
			document.getElementById(statusID).innerHTML += "<br>" + "XMLHttpRequest() does not support upload.";
		}
		// Check if file type is the cause of the error
		if (!(flag_file_type_isOk))
		{
			document.getElementById(statusID).innerHTML += "<br>" + "file type = " + file_for_upload.type + ", but it is required to be = " + expectedFileType;
		}
		// Check if file size is the cause of the error
		if (!(flag_file_size_isOk))
		{
			document.getElementById(statusID).innerHTML += "<br>" + "file size = " + file_for_upload.size + ", is greater than the max allowed of " + document.getElementById("MAX_FILE_SIZE").value + " bytes.";
		}
	}
}