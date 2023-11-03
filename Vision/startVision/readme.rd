WARNING: DO NOT INTERACT WITH EITHER THE KEYBOARD OR THE MOUSE WHILE RUNNING THIS PROGRAM. SEE BELOW FOR EXPLANATION.
WARNING: DO NOT RUN THIS PROGRAM IN NON-TERMINATOR TERMINAL AS IT IS DESIGNED STRICTLY FOR TERMINATOR AND WILL CAUSE UNDEFINED BEHAVIOR OTHERWISE.

To start the Vision Code for all Blimps
	1. Enter into ~/Vision/startVision
	2. Run ./visionService -a

To start the Vision Code for Blimp N
	1. Enter into ~/Vision/startVision
	2. Run ./visionService -n N

To start the Vision Code for multiple Blimps 1 to N
	1. Enter into ~/Vision/startVision
	2. Run ./visionService -n 1 2 3 ... N. Make sure to space out each of the numbers and not to add any commas. 

To stop the Vision Code for Blimp N
	1. Click on the user-managed panel (i.e. the panel at the bottom right of the screen). Will not work otherwise.
	2. Run ./../Viewers/stopVision N

EXPLANATION: 
The code operates by "typing" out commands as each panel is selected. Therefore, anything written by the user shall also be added as the
the program, resulting in it breaking. Furthermore, the commands are "typed" unto the screen it is currently on. Therefore, if the user
moves to another screen, the program shall begin writing out and attempting to execute its commands unto the screen the user has selected.
