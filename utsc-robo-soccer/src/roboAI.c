/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  Version: 0.2 - Updated Oct 2, 2014 - F. Estrada
***************************************************************************/

#include "imagecapture/imageCapture.h"
#include "API/robotControl.h"
#include "roboAI.h"     // <--- Look at this header file!
#include <nxtlibc/nxtlibc.h>
#include <stdio.h>
#include <stdlib.h>

#define EPS 0.21
#define MAX_SPEED 60
#define MIN_SPEED 15

#define RED_GOAL_X 462
#define RED_GOAL_Y 1000

#define GREEN_GOAL_X 462
#define GREEN_GOAL_Y 50

#define MAX_X 1024
#define MAX_Y 768

void clear_motion_flags(struct RoboAI *ai)
{
 // Reset all motion flags. See roboAI.h for what each flag represents
 // *You may or may not want to use these*
 ai->st.mv_fwd=0; 
 ai->st.mv_back=0;
 ai->st.mv_bl=0;
 ai->st.mv_br=0;
 ai->st.mv_fl=0;
 ai->st.mv_fr=0;
}

struct blob *id_coloured_blob(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function looks for and identifies a blob with the specified colour.
 // It uses colour contrast betwen the R, G, and B channels to pick the 
 // blob that is closest in colour to the specified target. If multiple
 // blobs with similar colour exist, then it picks the most saturated one.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> R
 //                   1 -> G
 //                   2 -> B
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 //        blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double BCRT=1.05;      // Ball colour ratio threshold
 double c1,c2,c3,m,mi,ma;
 double oc1,oc2,oc3;
 int i;

 oc1=1000;
 oc2=1;
 oc3=1;

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  if (col==0) {c1=p->R; c2=p->G; c3=p->B;}  // detect red
  else if (col==1) {c1=p->G; c2=p->R; c3=p->B;} // detect green
  else if (col==2){c1=p->B; c2=p->G; c3=p->R;}  // detect blue

  // Normalization and range extension
  mi=p->R;
  if (p->G<mi) mi=p->G;
  if (p->B<mi) mi=p->B;
  ma=p->R;
  if (p->G>ma) ma=p->G;
  if (p->B>ma) ma=p->B;

  c1=(c1-mi)/(ma-mi);
  c2=(c2-mi)/(ma-mi);
  c3=(c3-mi)/(ma-mi);
  c1+=.001;
  c2+=.001;
  c3+=.001;
  
  if (c1/c2>BCRT&&c1/c3>BCRT)     // Blob has sufficient colour contrast
  {
   m=(c1/c2)+(c1/c3);       // Total color contrast ch1 vs ch2 and ch3
   if (fnd==NULL||m>(oc1/oc2)+(oc1/oc3))  // Found the first blob with this color, or a more colorful one
   {
    fnd=p;
    oc1=c1;
    oc2=c2;
    oc3=c3;
   }
  }
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
if(ai->st.state == 204) {
        // if the ball is inside the threshold of the goal
        // kick the ball
        printf("We are in state 204\n");
        kick();
        retract();
        ai->st.state -= 2; // go back to chasing the ball
   }
 //   while turning.
 // - Heading (a unit vector in the direction of motion). Not valid
 //   while rotating - possibly valid while turning
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // Note that the blob data
 // structure itself contains another useful vector with the blob
 // orientation (obtained directly from the blob shape, valid at all
 // times even under rotation, but can be pointing backward!)
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 double NOISE_VAR=5;

 // Reset ID flags
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;      // Be sure you check these are not NULL before
 ai->st.self=NULL;      // trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;

 // Find the ball
 p=id_coloured_blob(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;      // New pointer to ball
  ai->st.ballID=1;      // Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;  // Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;

  ai->st.old_bcx=p->cx;     // Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;      // Compute heading direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)     // Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,1);
 else p=id_coloured_blob(ai,blobs,0);
 if (p)
 {
  ai->st.self=p;      // Update pointer to self-blob

  // Adjust Y position if we have calibration data
  if (fabs(p->adj_Y[0][0])>.1)
  {
   dmax=384.0-p->adj_Y[0][0];
   dmin=767.0-p->adj_Y[1][0];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;

  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }

  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob(ai,blobs,0);
 else p=id_coloured_blob(ai,blobs,1);
 if (p)
 {
  ai->st.opp=p; 

  if (fabs(p->adj_Y[0][1])>.1)
  {
   dmax=384.0-p->adj_Y[0][1];
   dmin=767.0-p->adj_Y[1][1];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/5.0;

 //drive_speed(30);     // Start forward motion to establish heading
          // Will move for a few frames.

 track_agents(ai,blobs);    // Call the tracking function to find each agent

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1) // Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  all_stop();
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
  fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;   // <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
  fprintf(stderr,"Penalty mode! let's kick it!\n");
  ai->st.state=100; // <-- Set AI initial state to 100
        break;
 case AI_CHASE:
  fprintf(stderr,"Chasing the ball...\n");
  ai->st.state=200; // <-- Set AI initial state to 200
        break;  
 default:
  fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
  ai->st.state=0;
  }

 all_stop();      // Stop bot,
 ai->runAI = AI_main;   // and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 clear_motion_flags(ai);
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 track_agents(ai,blobs);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is the main AI loop.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
                  state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
                  knows where the robot is, as well as where the opponent and
                  ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
  data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your NXT to get damaged!

  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)   // Initial set up - find own, ball, and opponent blobs
 {
  // Carry            
  //  40                                                                    out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)  // The id_bot() routine will change the AI state to initial state + 1
  {       // if robot identification is successful.
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;
   all_stop();
   clear_motion_flags(ai);
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.self->mx,ai->st.self->my,ai->st.state);
  }
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/

//----------------------------------------------------- SOCCER GAME

  if(ai->st.state == 1) {
  printf("We are in state 1\n");
  ai->st.state++;
  }

  if(ai->st.state == 2){
  printf("We are in state 2\n");
        // if the ball is on the field pivot until direction matches ball
        if(find_ball(ai)) {
                double cos_theta = get_cos_theta_direction_distance(ai);
                double error = orient_pid(cos_theta);

                double *distance = ball_distance_vector(ai);

                // go to next state
                if((error <= 3.00 && error >= 2.51) || !(find_ball(ai))) {
                        ai->st.state ++;
                }
        }
  }

  if(ai->st.state == 3){
    printf("We are in state 3\n");

    // redundancy check to make sure the orientation is correct
    double theta = acos(get_cos_theta_direction_distance(ai));
    double *distance_vector = ball_distance_vector(ai);
    double distance_vector_mag = vector_magnitude(distance_vector, 2);

    // if(theta <= EPS && theta >= 0) {
    // use distance PID to move the robot forward
    double error_d = distance_pid(distance_vector_mag, ai);
    printf("DISTANCE TO BALL %lf\n", error_d);
    // check if the robot returned a negative error
    if(error_d < 0) {
            // if error is negative then our distance from the ball
            // is increasing which means we need to readjust
            // go back to previous state
            ai->st.state --;
    }

    // check if we are close enough to the ball
    if(error_d <= 100 && error_d > 0) {
            // go to the next state if we are
            ai->st.state ++;
    }

    /*
    // go back to the previous state and re-adjust yourself
    } else {
            ai->st.state --;
    }
    */
   }

   if(ai->st.state == 4) {
        // if the ball is inside the threshold of the goal
        // kick the ball
        printf("We are in state 4\n");
        kick();
        retract();
        ai->st.state -= 2; // go back to chasing the ball
   }


//----------------------------------------------------- PENALTY SHOT

  if(ai->st.state == 101) {
      printf("We are in state 101\n");
    ai->st.state++;
  }

  if(ai->st.state == 102) {
    printf("We are in state 102\n");
      // if the ball is on the field pivot until direction matches ball
      if(find_ball(ai)) {
          double cos_theta = get_cos_theta_direction_distance(ai);
          double error = orient_pid(cos_theta);

          double *distance = ball_distance_vector(ai);

          // go to next state
          if((error <= 3.04 && error >= 2.7) || !(find_ball(ai))) {
            ai->st.state ++;
          }
      }
  }

  if(ai->st.state == 103){
    printf("We are in state 103\n");

    // redundancy check to make sure the orientation is correct
    double theta = acos(get_cos_theta_direction_distance(ai));
    double *distance_vector = ball_distance_vector(ai);
    double distance_vector_mag = vector_magnitude(distance_vector, 2);
    // if(theta <= 3.04 && theta >= 2.08) {
    // use distance PID to move the robot forward
    double error_d = distance_pid(distance_vector_mag, ai);
    // check if the robot returned a negative error
    if(error_d < 0) {
      // if error is negative then our distance from the ball
      // is increasing which means we need to readjust
      // go back to previous state
      ai->st.state --;
    }

  // check if we are close enough to the ball
    if(error_d <= 100 && error_d > 0) {
      // go to the next state if we are
      ai->st.state ++;
    }

    /*
    // go back to the previous state and re-adjust yourself
   } else {
          ai->st.state --;
   } */
  }

  if(ai->st.state == 104) {
    // position bot behind ball and facing the goal
  printf("We are in state 104\n");
  ai->st.state ++;
  }

  if(ai->st.state == 105) {
      // if the ball is inside the threshold of the goal
        // kick the ball
      printf("We are in state 105\n");
      kick();
      retract();
      all_stop();
    ai->st.state++;
  }

  if(ai->st.state == 106){
  printf("We are in state 106\n");
      // we just kicked the ball; stop
  all_stop();
  }

//---------------------------------------------------CHASE BALL
  if(ai->st.state == 201){
        printf("We are in state 201\n");
        ai->st.state++;
  }

  if(ai->st.state == 202){
    printf("We are in state 202\n");
    // if the ball is on the field pivot until direction matches ball
    if(find_ball(ai)) {
      double cos_theta = get_cos_theta_direction_distance(ai);
      double error = orient_pid(cos_theta);
      printf("This is my dx %lf\n", ai->st.self->dx);
      printf("This is my dy %lf\n", ai->st.self->dy);

      double *distance = ball_distance_vector(ai);

      printf("This is my x position %lf\n", ai->st.self->cx);
      printf("This is my y position %lf\n", ai->st.self->cy);

      printf("This is ball x position %lf\n", ai->st.ball->cx);
      printf("This is ball y position %lf\n", ai->st.ball->cy);

      printf("This is distance dx %lf\n", distance[0]);
      printf("This is distance dy %lf\n", distance[1]);


      printf("This is my cos(theta) %lf\n", cos_theta);
      printf("This is the angle %lf\n", error);

      // go to next state
      if((error <= EPS && error >= 0) || !(find_ball(ai))) {
              ai->st.state += 2;
      }
    }
  }

  if(ai->st.state == 203){
    printf("We are in state 203\n");

    // redundancy check to make sure the orientation is correct
    double theta = acos(get_cos_theta_direction_distance(ai));
    double *distance_vector = ball_distance_vector(ai);
    double distance_vector_mag = vector_magnitude(distance_vector, 2);
    if(theta <= EPS && theta >= 0) {
        // use distance PID to move the robot forward
        double error_d = distance_pid(distance_vector_mag, ai);
        // check if the robot returned a negative error
        if(error_d < 0) {
                // if error is negative then our distance from the ball
                // is increasing which means we need to readjust
                // go back to previous state
                ai->st.state --;
        }

      // check if we are close enough to the ball
        if(error_d <= 100 && error_d > 0) {
                // go to the next state if we are
                ai->st.state ++;
        }
    // go back to the previous state and re-adjust yourself
        } else {
                ai->st.state --;
        }
   }

   if(ai->st.state == 204) {
        // if the ball is inside the threshold of the goal
        // kick the ball
        printf("We are in state 204\n");
        kick();
        retract();
        ai->st.state -= 2; // go back to chasing the ball
   }



  //fprintf(stderr,"Just trackin'!\n"); // bot, opponent, and ball.
  track_agents(ai,blobs);   // Currently, does nothing but endlessly track
 }

}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

// figure out if the ball is even on the field
int find_ball(struct RoboAI *ai) {
  return (ai->st.ball != NULL);
}

// figure out if we are in the boundaries
int check_boundaries(struct RoboAI *ai) {
  double sx = ai->st.self->cx;
  double sy = ai->st.self->cy;

  if (!((sx <= MAX_X && sx >= 0) && (sy <= MAX_Y && sy >= 0))) {
    return 0;
  } else {
    return 1;
  }
}

// dot product calculator
double dot_product(double v[], double u[], int n) {
    double result = 0.0;
    for (int i = 0; i < n; i++) {
      result += v[i] * u[i];
    }
    return result;
}

// vector magnitude calculator
double vector_magnitude(double *v, int n) {
  double magnitude = 0.0;
  for(int i = 0; i < n; i++) {
    magnitude += pow(v[i], 2);
  }
  return sqrt(magnitude);
}

// calculate cos theta for any two vectors
double get_cos_theta(double *v, double *u) {
  
  // get dot product of the two vectors
  double dot_product_result = dot_product(v, u, 2);

  // calculate the magnitude of the two vectors
  double v_mag = vector_magnitude(v, 2);
  double u_mag = vector_magnitude(u, 2);
  double vector_mag = v_mag * u_mag;

  // calculate cos theta
  double cos_theta = dot_product_result / vector_mag;

  return cos_theta;

}

// get the orientation of our robot with respect to the ball using 
// the result from the dot product/ mag of vectors, aka cos(theta)
double get_cos_theta_direction_distance(struct RoboAI *ai) {

  // if you devide the dot_product by the product of the magnitudes of the vectors
  // you get this; trying to get theta to 0 aka bringing cos(theta) to 1
  double cos_theta;

  // put the current direction in a vector
  double current_direction_vector[2];
  current_direction_vector[0] = ai->st.self->dx;
  current_direction_vector[1] = ai->st.self->dy;

  // get the vector for the distance to the ball
  double *distance_vector = ball_distance_vector(ai);

  cos_theta = get_cos_theta(distance_vector, current_direction_vector);

  free(distance_vector);

  return cos_theta;

}

double heading_direction_cos_theta(struct RoboAI *ai) {

  // if you devide the dot_product by the product of the magnitudes of the vectors
  // you get this; trying to get theta to 0 aka bringing cos(theta) to 1
  double cos_theta;

  // put the current direction in a vector
  double current_direction_vector[2];
  current_direction_vector[0] = ai->st.self->dx;
  current_direction_vector[1] = ai->st.self->dy;

  double current_heading_vector[2];
  current_heading_vector[0] = ai->st.self->mx;
  current_heading_vector[1] = ai->st.self->my;
  cos_theta = get_cos_theta(current_heading_vector, current_direction_vector);

  double theta = acos(cos_theta);

  return theta;
}

// calculate x and y distnace to the ball
double *ball_distance_vector(struct RoboAI *ai) {

  double sx = ai->st.self->cx;
  double sy = ai->st.self->cy;

  double bx = ai->st.ball->cx;
  double by = ai->st.ball->cy;

  double *distance =  malloc(sizeof(double) * 2);
  distance[0] = sx - bx;
  distance[1] = sy - by;
  
  return distance;
}

double *near_opponent(struct RoboAI *ai){
  double sx = ai->st.self->cx;
  double sy = ai->st.self->cy;

  double bx = ai->st.opp->cx;
  double by = ai->st.opp->cy;

  double *distance =  malloc(sizeof(double) * 2);
  distance[0] = sx - bx;
  distance[1] = sy - by;
  
  return distance;
}


// calculate x and y distnace to the ball last frame
double *old_ball_distance(struct RoboAI *ai) {

  double sx = ai->st.old_scx;
  double sy = ai->st.old_scy;

  double bx = ai->st.old_bcx;
  double by = ai->st.old_bcy;

  double *distance =  malloc(sizeof(double) * 2);
  distance[0] = fabs(sx - bx);
  distance[1] = fabs(sy - by);
  
  return distance;
}

// logic for dealing with opponent
void opponent_fsm(double *distance) {

  // keep track of our last distance to opponent
  static double old_distance = 0;
  // get the magnitude of the vector to opponent from our bot
  double distance_to_opp = vector_magnitude(distance, 2);
  // check if we are too close to opponent
  if(distance_to_opp <= 50) {
    // go back
    reverse();
    if(old_distance > distance_to_opp) {
      all_stop();
      reverse();
    }
  } 
  old_distance = distance_to_opp;
}

// PID controller to reduce cos theta to zero
double orient_pid(double cos_theta) {

  double epsilon = 0.1;
  double kp = 5;
  double kd = 1;
  double ki = 1;

  static double pre_error = 0;
  static double integral = 0;
  static double time = 0.01;
  static double last_output = 0;
  static int last_pivot = -1;
  double error;
  double derivative;
  double raw_output;

  // Calculate P, I, D
  error = acos(cos_theta); // P

  // In case of error too small then stop integration
  if(fabs(error) > epsilon) { // I
    integral = integral + error * time; // figure out how to get time 
  }

  derivative = fabs(error - pre_error) / time; // D

  raw_output = kp * error + ki * integral + kd * derivative;
  printf("pre-error, error: %lf, %lf\n", pre_error, error);

  time += 0.01;

  int output = fabs((int)raw_output);

  // make sure our output is not entirely insane
  if(output > MAX_SPEED) {
    output = MAX_SPEED;
  } else if(output < MIN_SPEED) {
    output = MIN_SPEED;
  }
  // check if our angle is getting bigger
  if(pre_error < error) {
    // if our angle is getting bigger than it means we are pivoting the wrong way
    // this function makes sure we pivot the other way if that is the case
    int inverse_of_last_pvt = 0 - last_pivot;
    my_pivot(inverse_of_last_pvt, output);
    last_pivot = inverse_of_last_pvt;
  } else {
    // if our angle is indeed getting smaller then keep pivoting the same way
    my_pivot(last_pivot, output);
    last_pivot = last_pivot;
  }
  printf("Left is -1; Right is 1\n");
  printf("Turned %d, with this output %d\n", last_pivot, output);
  pre_error = error;
  return error;
}

// my pivot function to decide where to pivot
void my_pivot(int n, int input) {

  // if our last pivot was right turn left
  if(n == 1){
    pivot_right_speed(input);
    all_stop();
  } else if(n == -1) {
    pivot_left_speed(input);
    all_stop();
  }
}


// PID controller for distance between ball and robot
double distance_pid(double vector_mag, struct RoboAI *ai) {

  double epsilon = 0.1;
  double kp = 5;
  double kd = 1;
  double ki = 1;

  static double pre_error = 0;
  static double integral = 0;
  static double time = 0.01;
  static int distance_check;
  double error;
  double derivative;
  double raw_output;

  // Calculate P, I, D
  error = vector_mag; // P

  // In case of error too small then stop integration
  if(fabs(error) > epsilon) { // I
    integral = integral + error * time; // figure out how to get time 
  }

  derivative = fabs(error - pre_error) / time; // D

  raw_output = kp * error + ki * integral + kd * derivative;
  printf("pre-error, error: %lf, %lf\n", pre_error, error);
  time += 0.01;

  // ideally we would know which way to pivot for maximum efficiency but for now
  // we can just pivot right or left and each time this function is called cos_theta would be updated
  // based on new measurements
  int output = fabs((int)raw_output);

  // make sure our output is not entirely insane
  if(output > MAX_SPEED) {
    output = MAX_SPEED;
  } else if(output < MIN_SPEED) {
    output = MIN_SPEED;
  }

  printf("Drive forward output %d\n", output);

  // check if our distance is getting bigger
  if(pre_error < error) {
    // if it is getting bigger let the FSM know that so it can go back a state and
    // re-orient the bot
    distance_check = -1;
  } else {
    // let the fsm know everything is okay
    distance_check = 1;
  }

  //double *distance_to_opp = near_opponent(ai);
  if(check_boundaries(ai)){
    //opponent_fsm(distance_to_opp);
    drive_speed(output);
    all_stop();
  } else {
    turn_around();
  }
  pre_error = error;
  return error * distance_check;
 }



 // do a 180 turn
 void turn_around() {
  for(int i = 0; i < 6; i++) {
    pivot_left();
  }
}
