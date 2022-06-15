# Mulitiple-robot-navigation-with-HRC
It's a project which uses ros to compute multiple-robot-formation with PID control algorithm and  keep formation while SLAM and navigation. 
You can control them with launch files including hand-gesture and keyboard.

The tasks will be completed continuously.


Packages are as follows:
1.tianbo_mini and abc_swarm:  the two are basic robot description and driver setting file.
Some basic algorithm files also in it, such as "path_tracking"(it will be used when navigation is done), "PID control"(it'll be used when follower robot keep leader robot's track and folllow it), etc.
Then I add others flies with different function , such as navigation and slam, into them.(Still a little bit fuzzy, needs to be arranged)

2.Gesture: it's the pattern recongnition method to control the follower robot to move.
Hand-Gesture recongnition has been added. Next time will be voice or others way.

Welcome to issue and share your excellent ideas！

