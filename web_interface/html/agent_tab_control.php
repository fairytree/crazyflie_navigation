<table class="git-table">
	<tr>
		<td class="git-button-cell left">
			<button
				class="button-push navy fullwidth"
				id="buttonControlRadioDisconnect"
				onclick="sendRosMessage_outputLabelID('disconnect', 'labelControlRadioDisconnect')"
				>
				disconnect
				<div class="div-for-button-highlight-on-touchscreen navy"></div>
			</button>
		</td>
		<td class="git-button-cell left">
			<button
				class="button-push navy fullwidth"
				id="buttonControlRadioConnect"
				onclick="sendRosMessage_outputLabelID('connect', 'labelControlRadioConnect')"
				>
				connect
				<div class="div-for-button-highlight-on-touchscreen navy"></div>
			</button>
		</td>
	</tr>
	<tr>
		<td
			style="text-align: center;"
			id="labelControlRadioDisconnect"
			>
			&nbsp
		</td>
		<td
			style="text-align: center;"
			id="labelControlRadioConnect"
			>
			&nbsp
		</td>
	</tr>
	<tr>
		<td class="git-button-cell left">
			<button
				class="button-push navy fullwidth"
				id="buttonControlTakeoff"
				onclick="sendRosMessage_outputLabelID('takeoff', 'labelControlTakeOff')"
				>
				take-off
				<div class="div-for-button-highlight-on-touchscreen navy"></div>
			</button> 
		</td>
		<td class="git-button-cell left">
			<button
				class="button-push navy fullwidth"
				id="buttonControlLand"
				onclick="sendRosMessage_outputLabelID('land', 'labelControlLand')"
				>
				land
				<div class="div-for-button-highlight-on-touchscreen navy"></div>
			</button> 
		</td>
	</tr>
	<tr>
		<td
			style="text-align: center;"
			id="labelControlTakeOff"
			>
			&nbsp
		</td>
		<td
			style="text-align: center;"
			id="labelControlLand"
			>
			&nbsp
		</td>
	</tr>
</table>

<br>

<div class="control-tabs">
	<input name="control-tabs" type="radio" id="control-tab-1" checked="checked" class="control-tab-input"/>
	<label for="control-tab-1" class="control-tab-label">Default</label>
	<div class="control-tab-panel" id="control-tab-panel-default">
		<?php
			include("agent_control_tab_default.html");
		?>
	</div>

	<input name="control-tabs" type="radio" id="control-tab-2" class="control-tab-input"/>
	<label for="control-tab-2" class="control-tab-label">Student</label>
	<div class="control-tab-panel" id="control-tab-panel-student">
		<?php
			include("agent_control_tab_student.html");
		?>
	</div>

</div>

<br>