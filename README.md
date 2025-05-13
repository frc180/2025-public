# Team 180 S.P.A.M. - 2025 Robot Code
This is our public robot code for "Anemone". our robot for the 2025 FIRST Robotics Competion "REEFSCAPE" season.


## Simulation Setup

To properly visualize the robot and game pieces in AdvantageScope:

* Use `Robot/RobotContainer/Drivetrain/getSimPose` for the robot position
* Attach `SmartDashboard/Mechanism 2D` to the robot position for elevator, arm and coral intake visualization
* Use `Algae Poses` for algae positions
* Use `Coral Poses` for coral positions

![Screenshot of AdvantageScope showing the visualization configuration](/assets/sim.png)

## Simulation Controls

Anemone is controlled with a single [Razer Wolverine V3 Tournament Edition](https://www.amazon.com/dp/B0DB6QT8K6), though most of the controls can be utilized through any basic Xbox controller.

### Scoring L4/L3/L2 Coral

Scoring coral works through a two-button combination: holding **X** or **B** to choose the left or right branch of the reef face closest to the robot, and then holding the button corresponding to the height you want to score at:
* L4 - **Right Bumper**
* L3 - **Right Trigger**
* L2 - **Left Bumper**

Holding both buttons will result in the robot auto-aligning and scoring the coral auotmatically.

### Scoring L1 Coral

To score coral at L1, hold **DPad Left**. This will put the arm into scoring position - releasing **DPad Left** will score the coral.

*Note: We had DPad Left mapped to one of the four back paddles of our Razer controller to make this more ergonomic*

### Descoring Algae

Hold **A** and the robot will descore the algae from the reef face closest to the robot. If a coral is in the robot, it will also score the coral.


### Scoring Algae

Hold **DPad Down** and **X** to put the elevator up to score in the barge - releasing **X** will release and score the algae.


*Note: We had DPad Down mapped to one of the four back paddles of our Razer controller to make this more ergonomic*