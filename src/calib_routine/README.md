## how it works

first, user starts robot and sets led brightness. in intrinsic calibration routine, should set to flash mode. in camera-laser routine, should set to strobe mode

then, from GUI, go to calibration page and select the type of calibration, which triggers an **API call that starts the ROS calibration node with a specific calibration routine** on base station. All routine configs are stored in one big calibration config file in pipe_blaser_ros

the calibration node automatically enters phase one after started

when the calibration node receives sensor data from the robot, it does **corner extraction (compressed image)** and **spatial coverage calculation (array of ints)**, and publishes these topics to GUI
the GUI has a button that sends a **custom CalibrationControl service request** to the calibration node to proceed with phase two

in phase two, the calibration node publishes **calibration progress (custom msg comprising n/N data processed, and a bool field with "done" and calib error)** to GUI and when done, save the calibrated params locally on base station, and shuts itself. and by looking at the calibration progress messages, the GUI knows how to animate phase two UI and when to exit the calibration page. the GUI also displays the calib error for QA

In whatever phase, if the CalibrationControl service request says cancel, the calibration node cancels and shuts itself