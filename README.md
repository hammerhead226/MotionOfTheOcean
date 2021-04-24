# MotionOfTheOcean
Motion of the Ocean, a universal autonomous recording and execution program

path_creator.py: python path_creator.py robot_radius module_position_offsets template_path edited_path
so for example, if we had 4 swerve modules a distance 1.328 feet from the center of the robot, positioned at offsets .830 rad, 2.312 rad, 3.971 rad, and 5.453 rad and a template_path called recording.csv and an edited_path file name of edit_recording.csv, the command to be called would be
python path_creator.py 1.328 "[.830, 2.312, 3.971, 5.453]" recording.csv edit_recording.csv
the 'n' key creates a new point, the 'z' key creates a new slow point, left-clicking dragging moves around the parameters, and right-clicking on a point deletes it

smart_speed.py: python galaxy_brain_speed.py robot_radius module_position_offsets robot_parameters input_path sped_up_path
so for example, if we had 4 swerve modules a distance 1.328 feet from the center of the robot, positioned at offsets .830 rad, 2.312 rad, 3.971 rad, and 5.453 rad, robot parameters of 15 ft/s max speed, 2 ft/s min speed for slowpoints, 13 ft/s^2 for maximum acceleration, 19 ft/s^2 for maximum deceleration and 45 ft/s^2 for maximum centripetal acceleration, with an input_path of edit_recording.csv and sped_up_path of sped_recording.csv, the command to be called would be
python galaxy_brain_speed.py 1.328 "[.830, 2.312, 3.971, 5.453]" "[15, 2, 13, 19, 45]" edit_recording.csv sped_recording.csv
