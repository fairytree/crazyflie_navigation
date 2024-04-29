<?php
	include("page_header.html");
?>


<script src="js/sse_agentStatus.js?ver=0.1"></script>


<div class="full-window-fixed">
</div>

<div class="max-width-full-heigth-fixed">

	<div class="top-bar-container">
		<table class="top-bar-buttons-table">
			<tr>
				<td class="top-bar-motorsoff-button-cell">
					<button
						class="button-push red motorsoff"
						id="buttonMotorsOffForAgent"
						onclick="sendRosMessage_outputLabelID('motorsoff', '')"
						>
						MOTORS-OFF
						<div class="div-for-button-highlight-on-touchscreen red"></div>
					</button>
				</td>
				<td class="top-bar-info-button-cell">
					<button class="button-push blue info" onclick="checkForRosAgent()">
						i
						<div class="div-for-button-highlight-on-touchscreen blue"></div>
					</button>
				</td>
			</tr>
		</table>

		<!--
			<div class="top-bar-title">
				Agent (IP <?php echo $_SERVER['REMOTE_ADDR']; ?>)
			</div>
		-->

		<div class="top-bar-status-icons padbelow">
			<div class="on-off-switch type2forstatusbar">
				<input type="checkbox" id="checkboxTopBarStatus" onchange=checkboxTopBarStatus_changed()><label for="checkboxTopBarStatus"></label>
			</div>
			<img id="radio-icon" class="status-icon-radio" src="img/rf_disconnected.png">
			<img id="battery-icon" class="status-icon-battery" src="img/battery_unavailable.png">
			<img id="flying-state-icon" class="status-icon-flying-state" src="img/flying_state_unavailable.png">
		</div>
	</div>


	<div class="main-tabs">
		<input name="tabs" type="radio" id="main-tab-1" checked="checked" class="main-tab-input"/>
		<label for="main-tab-1" class="main-tab-label">Code</label>
		<div class="main-tab-panel">
			<?php
				include("agent_tab_code.html");
			?>
		</div>

		<input name="tabs" type="radio" id="main-tab-2" class="main-tab-input"/>
		<label for="main-tab-2" class="main-tab-label">Compile</label>
		<div class="main-tab-panel">
			<?php
				include("agent_tab_compile.html");
			?>
		</div>

		<input name="tabs" type="radio" id="main-tab-3" class="main-tab-input"/>
		<label for="main-tab-3" class="main-tab-label">Launch</label>
		<div class="main-tab-panel">
			<?php
				include("agent_tab_launch.html");
			?>
		</div>

		<input name="tabs" type="radio" id="main-tab-4" class="main-tab-input"/>
		<label for="main-tab-4" class="main-tab-label">Control</label>
		<div class="main-tab-panel thin-padding">
			<?php
				include("agent_tab_control.php");
			?>
		</div>
	</div>
</div>


<?php
	include("page_footer.html");
?>