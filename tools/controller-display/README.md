# Drive team controller display

Run `Start-ControllerDisplay.ps1` in PowerShell. It opens `http://localhost:8765`; keep the window open and press Ctrl+C to stop the local server. Connect the gamepads before opening the page. If a controller is not detected, press one of its buttons and reload. Select the right browser device from the menu on each controller card; browser indexes can differ from robot USB ports.

The page shows the Offseason `FrcTeleOp` mappings. Holding LB on either controller toggles that controller's alternate action labels. Live highlights and moving stick caps use browser Gamepad input for display only; the page does not connect to or control the robot.
