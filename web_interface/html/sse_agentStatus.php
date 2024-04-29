 <?php
	//date_default_timezone_set("America/New_York");
	header("Cache-Control: no-cache");
	header("Content-Type: text/event-stream");
	//header('Connection: keep-alive');

	$event_id_counter = 0;

	while (true) {
		// Every second, send a "ping" event.

		echo "event: agentstatus\n";

		$output = shell_exec("./bashscripts/rosGetStatusJson_forAgent.sh " . $event_id_counter);

		//$curDate = date(DATE_ISO8601);
		//echo 'data: {"time": "' . $curDate . '"}';

		echo "data: $output";
		echo "\n\n";

		// Send a simple message at random intervals.

		$event_id_counter++;

		ob_end_flush();
		flush();
		usleep(250000);
	}
?>