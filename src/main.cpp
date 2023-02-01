#include <iostream>
#include <string>
#include "vrep_bridge.h"

#include "controller.h"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;
 
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}


int main()
{
	VRepBridge vb(VRepBridge::CTRL_VELOCITY); // Position controlled 
	const double hz = 1000 ;
	WholeBodyController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData_h(vb.getPosition_h(), vb.getVelocity_h());
		ac.readData_p(vb.getPosition_p(), vb.getVelocity_p());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			ac.readData_h(vb.getPosition_h(), vb.getVelocity_h());
			ac.readData_p(vb.getPosition_p(), vb.getVelocity_p());
			cout << "Initial q of body: " << vb.getPosition_h().transpose() << endl;
			cout << "Initial q of arm: " << vb.getPosition_p().transpose() << endl;
			is_first = false;
			ac.initVelocity_h();

			ac.initPosition_p();

		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
			case '1':
				ac.setMode("go_straight & joint_home");
				break;
			case '2':
				ac.setMode("go_backward & joint_move_right");
				break;
			case '3':
				ac.setMode("go_straight & joint_move_left");
				break;
			case '4':
				ac.setMode("move_circle & joint_move_left");
				break;
			case '5':
				ac.setMode("move_circle & joint_move_right");
				break;
				
			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			vb.setDesiredVelocity_h(ac.getDesiredVelocity());
			vb.setDesiredPosition_p(ac.getDesiredPosition());
			vb.setDesiredTorque_p(ac.getDesiredTorque());	
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}
