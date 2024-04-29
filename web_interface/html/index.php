<?php
	include("page_header.html");
?>

<div class="full-window-fixed">
</div>

<div class="max-width-full-heigth-fixed">

	<div class="top-bar-container">
		<table class="top-bar-buttons-table">
			<tr>
				<td class="top-bar-motorsoff-button-cell">
					<button class="button-push red motorsoff" disabled onclick="checkForRosAgent()">
						MOTORS-OFF
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

		<div class="top-bar-title">
			Browser Interface
		</div>
		<div class="top-bar-subtitle">
			for the D-FaLL System
		</div>
		<div class="top-bar-subtitle padbelow">
			Connected to IP <?php echo $_SERVER['REMOTE_ADDR']; ?>
		</div>
	</div>

	<div class="home-tabs">
		<input name="tabs" type="radio" id="home-tab-1" checked="checked" class="home-tab-input"/>
		<label for="home-tab-1" class="home-tab-label">Enter</label>
		<div class="home-tab-panel">
			<?php
				include("home_tab_enter.html");
			?>
		</div>

		<input name="tabs" type="radio" id="home-tab-2" class="home-tab-input"/>
		<label for="home-tab-2" class="home-tab-label">About</label>
		<div class="home-tab-panel">
			<?php
				include("home_tab_about.html");
			?>
		</div>
	</div>
</div>


<?php
	include("page_footer.html");
?>