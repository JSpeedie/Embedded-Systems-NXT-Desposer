
/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 5 m/s at touchdown
	  * Maximum landing angle should be less than 10 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
           There are older ones! (for bonus credit, find the oldest
           one and send me a description/picture plus info about the
           platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
             the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

             Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                              to angle w.r.t. vertical direction measured clockwise, so that
                              SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                              SONAR_DIST[1] is distance at 10 degrees from vertical
                              SONAR_DIST[2] is distance at 20 degrees from vertical
                              .
                              .
                              .
                              SONAR_DIST[35] is distance at 350 degrees from vertical

                              if distance is '-1' there is no valid reading. Note that updating
                              the sonar readings takes time! Readings remain constant between
                              sonar updates.

             RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                              in the direction of the lander's main thruster.
                              The laser range finder never fails (probably was designed and
                              built by PacoNetics Inc.)

             Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
             PLAT_X	- X position of the landing platform
             PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
             (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                              orientation (i.e. rotation is not w.r.t.
                                              a fixed reference direction).

    					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
             MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
             * The Safety_Override(); function, which determines whether the lander is
               in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

           - Program usage: The program is designed to simulate different failure
                            scenarios. Mode '1' allows for failures in the
                            controls. Mode '2' allows for failures of both
                            controls and sensors. There is also a 'custom' mode
                            that allows you to test your code against specific
                            component failures.

			 Initial lander position, orientation, and velocity are
                            randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
             with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
             maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                                  failing component(s):
                                  1 - Main thruster
                                  2 - Left Thruster
                                  3 - Right Thruster
                                  4 - Horizontal velocity sensor
                                  5 - Vertical velocity sensor
                                  6 - Horizontal position sensor
                                  7 - Vertical position sensor
                                  8 - Angle sensor
                                  9 - Sonar

           e.g.

                Lander_Control easy.ppm 3 1 5 8

                Launches the program on the 'easy.ppm' map, and disables the main thruster,
                vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

           * Be sure to complete the attached REPORT.TXT and submit the report as well as
             your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

     	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
     Standard C libraries
*/
#include <math.h>
#include <stdio.h>

#include "Lander_Control.h"

double better_Position(int direction) {
	double betterPosition = 0;
        for(int i = 0; i < 50; i++){
                if (direction == 0){ // for x
                        betterPosition += Position_X();
                } else if (direction == 1){ // for y
                        betterPosition += Position_Y();
                }
        }
	return betterPosition / 50;
}

double better_Velocity(int direction){
        double betterVelocity = 0;
        for(int i = 0; i < 50; i++){
                if (direction == 0){ // for x
                        betterVelocity += Velocity_X();
                } else if (direction == 1){ // for y
                        betterVelocity += Velocity_Y();
                }
        }
        return betterVelocity / 50;
}

double better_Angle(void){
        double betterAngle = 0;
	for(int i = 0; i < 50; i++){
                betterAngle += Angle();
        }
        return betterAngle / 50;
}



void Lander_Control(void)
{
    /*
      This is the main control function for the lander. It attempts
      to bring the ship to the location of the landing platform
      keeping landing parameters within the acceptable limits.

      How it works:

      - First, if the lander is rotated away from zero-degree angle,
        rotate lander back onto zero degrees.
      - Determine the horizontal distance between the lander and
        the platform, fire horizontal thrusters appropriately
        to change the horizontal velocity so as to decrease this
        distance
      - Determine the vertical distance to landing platform, and
        allow the lander to descend while keeping the vertical
        speed within acceptable bounds. Make sure that the lander
        will not hit the ground before it is over the platform!

      As noted above, this function assumes everything is working
      fine.
*/

/*************************************************
    TO DO: Modify this function so that the ship safely
           reaches the platform even if components and
           sensors fail!

           Note that sensors are noisy, even when
           working properly.

           Finally, YOU SHOULD provide your own
           functions to provide sensor readings,
           these functions should work even when the
           sensors are faulty.

           For example: Write a function Velocity_X_robust()
           which returns the module's horizontal velocity.
           It should determine whether the velocity
           sensor readings are accurate, and if not,
           use some alternate method to determine the
           horizontal velocity of the lander.

           NOTE: Your robust sensor functions can only
           use the available sensor functions and control
           functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
           ACCESS THE SIMULATION STATE. That's cheating,
           I'll give you zero.
**************************************************/

    double VXlim;
    double VYlim;


    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.
    if (fabs(better_Position(0)-PLAT_X)>200) VXlim=25;
    else if (fabs(better_Position(0)-PLAT_X)>100) VXlim=15;
    else VXlim=5;

	/* If the lander is well above the platform */
    if (PLAT_Y-better_Position(1)>200) VYlim=-20;
	/* If the lander is nearing the platform */
    else if (PLAT_Y-better_Position(1)>100) VYlim=-10;
    else VYlim=-4;

    // IMPORTANT NOTE: The code below assumes all components working
    // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
    // fail. More likely, you will need a set of case-based code
    // chunks, each of which works under particular failure conditions.

    // Check for rotation away from zero degrees - Rotate first,
    // use thrusters only when not rotating to avoid adding
    // velocity components along the rotation directions
    // Note that only the latest Rotate() command has any
    // effect, i.e. the rotation angle does not accumulate
    // for successive calls.

	/* x position PID controller variables */
	double error_x = 0;
	double error_x_integral = 0;
	double error_x_prev = 0;
	double k1 = 0.2;
	double k2 = 0;
	double k3 = 0.06;
	double error_vel = 0;
	double error_vel_integral = 0;
	double error_vel_prev = 0;
	double vel_k1 = 4.5;
	double vel_k2 = 0;
	double vel_k3 = 0;
	int angle_range = 30;
	double target_vel = 1;

	// TODO: change this so it works if only a side thruster is available
	if (better_Angle() > 0 + angle_range && better_Angle() < 360 - angle_range) {
        if (better_Angle()>=180) Rotate(360-better_Angle());
        else Rotate(-better_Angle());
        return;
	}

	error_x_prev = error_x;
	error_vel_prev = error_vel;
	/* The error is the difference in the x position of the lander and the platform */
	error_x = better_Position(0) - PLAT_X;
	/* If the lander is to the right of the platform */
	if (error_x > 0) error_vel = (-target_vel) - better_Velocity(0);
	/* If the lander is to the left of the platform */
	else error_vel =  target_vel - better_Velocity(0);
	error_x_integral += error_x;
	error_vel_integral += error_vel;
	/* PID controller */
	/* Get the magnitude of change that's need to be made in the x for
	 * the lander to land safely */
	double change = (k1 * error_x) + (k2 * error_x_integral) + (k3 * (error_x - error_x_prev));
	double change_vel = (vel_k1 * error_vel) + (vel_k2 * error_vel_integral) + (vel_k3 * (error_vel - error_vel_prev));
	change = -1 * change;
	//change_vel = -1 * change_vel;
	double degrees_to_change = change + change_vel;
	printf("degtc = %lf change = %lf change_vel = %lf VYlim = %lf\n", degrees_to_change, change, change_vel, VYlim);
	int vel = 1;
	/* If the lander is over the platform */
	if (PLAT_X - 25 < better_Position(0) && PLAT_X + 25 > better_Position(0)) {
		// If the lander is moving too fast and will crash
		if (better_Velocity(1) < VYlim) {
			printf("dropping too fast\n");
			vel += 0.15;
		} else {
			printf("drop the lander\n");
			vel = 0;
		}
	// If the lander is not over the platform
	} else {
		// Keep it at a slow descent
		if (better_Velocity(1) > -1) {
			vel -= 0.2;
		} else {
			vel += 0.2;
		}
		if (vel > 1.0) { vel == 1.0; }
	}


	if (degrees_to_change > angle_range) {
		degrees_to_change = angle_range;
	} else if (degrees_to_change < -angle_range) {
		degrees_to_change = -angle_range;
	}

    if (Angle() >= 180) {
		Rotate(360-better_Angle() + degrees_to_change);
	} else {
		Rotate(-better_Angle() + degrees_to_change);
	}

	// If the main thruster is working
	if (MT_OK) {
		Main_Thruster(vel);
	// If the right thruster is working
	} else if (RT_OK) {
		Right_Thruster(vel);
	// If the left thruster is working
	} else if (LT_OK) {
		Left_Thruster(vel);
	}

}

void Safety_Override(void)
{
    /*
      This function is intended to keep the lander from
      crashing. It checks the sonar distance array,
      if the distance to nearby solid surfaces and
      uses thrusters to maintain a safe distance from
      the ground unless the ground happens to be the
      landing platform.

      Additionally, it enforces a maximum speed limit
      which when breached triggers an emergency brake
      operation.
    */

/**************************************************
    TO DO: Modify this function so that it can do its
           work even if components or sensors
           fail
**************************************************/

/**************************************************
     How this works:
     Check the sonar readings, for each sonar
     reading that is below a minimum safety threshold
     AND in the general direction of motion AND
     not corresponding to the landing platform,
     carry out speed corrections using the thrusters
**************************************************/

    double DistLimit;
    double Vmag;
    double dmin;

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag=better_Velocity(0)*better_Velocity(0);
    Vmag+=better_Velocity(1)*better_Velocity(1);

    DistLimit=fmax(75,Vmag);

    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
    if (fabs(PLAT_X-better_Position(0))<150&&fabs(PLAT_Y-better_Position(1))<150) return;

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.
    dmin=1000000;
    if (better_Velocity(0)>0)
    {
     for (int i=5;i<14;i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
    }
    else
    {
     for (int i=22;i<32;i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
    }

    // Vertical direction
    dmin=1000000;
    if (better_Velocity(1)>5)      // Mind this! there is a reason for it...
    {
	 printf("mind this!\n");
     for (int i=0; i<5; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
     for (int i=32; i<36; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
    }
    else
    {
     for (int i=14; i<22; i++)
      if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
    }
	int safe_angle_range = 15;
	// Too close to a surface in the horizontal direction
    if (dmin<DistLimit) {
		if (better_Angle() > 0 + safe_angle_range && better_Angle() < 360 - safe_angle_range) {
			if (better_Angle()>=180) Rotate(360-better_Angle());
			else Rotate(-better_Angle());
			return;
		}
		if (better_Velocity(1)>2.0) {
			Main_Thruster(0.0);
		} else {
			// If the lander is about to fly off the map, cut power to the thrusters
			if (better_Position(1) - 30 < 0) {
				Main_Thruster(0.0);
			} else {
				Main_Thruster(1.0);
			}
		}
	}
}
