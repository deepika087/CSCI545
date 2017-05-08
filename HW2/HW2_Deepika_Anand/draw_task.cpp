/*!=============================================================================
  ==============================================================================

                                draw_task.cpp

  ==============================================================================
  \remarks

  a simple task for point to point movements in cartesian space

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_user.h"

/*! defines */

/* local variables */
static double      time_step;
static double     *cart;
static SL_Cstate  *ctarget;
static SL_Cstate  *cnext;
static int        *cstatus;
static SL_DJstate *target;
static int         firsttime = TRUE;
static double      movement_time = 1.0;
static double      tau;

/* global functions */
extern "C" void
add_draw_task(void);

/* local functions */
static int  init_draw_task(void);
static int  run_draw_task(void);
static int  change_draw_task(void);
static void init_vars(void);
static int  calculate_min_jerk_next_step (SL_Cstate *curr_state,
					  SL_Cstate *des_state,
					  double tau,
					  double delta_t,
					  SL_Cstate *next_states);
 
/*!*****************************************************************************
 *******************************************************************************
\note  add_draw_task
\date  Feb 1999
\remarks 

adds the task to the task menu

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
add_draw_task( void )

{
  int i, j;
  static int firsttime = TRUE;
  
  if (firsttime) {
    firsttime = FALSE;

    cart    = my_vector(1,n_endeffs*6);
    ctarget = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cnext   = (SL_Cstate *) my_calloc(n_endeffs+1,sizeof(SL_Cstate),MY_STOP);
    cstatus = my_ivector(1,n_endeffs*6);
    target  = (SL_DJstate *)my_calloc(n_dofs+1,sizeof(SL_DJstate),MY_STOP);


    addTask("Draw Task", init_draw_task, 
	    run_draw_task, change_draw_task);
  }

}    

/*!*****************************************************************************
 *******************************************************************************
  \note  init_draw_task
  \date  Dec. 1997

  \remarks 

  initialization for task

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

       none

 ******************************************************************************/
static int 
init_draw_task(void)
{
  int    j, i;
  char   string[100];
  double max_range=0;
  int    ans;
  double aux;
  int    flag = FALSE;
  int    iaux;
  
  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Goto task can only be run if no other task is running!\n");
    return FALSE;
  }

  /* initialize some variables */
  init_vars();
  time_step = 1./(double)task_servo_rate;

  
  /* go to the default state */
  for (i=1; i<=n_dofs; ++i)
    target[i] = joint_default_state[i];

  target[R_SFE].th = -1.5;
  target[R_HR].th  = 1.0;
  target[R_EB].th  = 1.5;

  if (!go_target_wait_ID(target))
    return FALSE;

  movement_time = 1.0;
  tau = movement_time;

  /* we move with the right hand */
  cstatus[(RIGHT_HAND-1)*6+_X_] = TRUE;
  cstatus[(RIGHT_HAND-1)*6+_Y_] = TRUE;
  cstatus[(RIGHT_HAND-1)*6+_Z_] = TRUE;

  /* choose as target 15 cm distance in x direction */
  ctarget[RIGHT_HAND].x[_X_] = cart_des_state[RIGHT_HAND].x[_X_] ;
  ctarget[RIGHT_HAND].x[_Y_] = cart_des_state[RIGHT_HAND].x[_Y_];
  ctarget[RIGHT_HAND].x[_Z_] = cart_des_state[RIGHT_HAND].x[_Z_] - 0.04;


  /* the cnext state is the desired state as seen form this program */
  for (i=1; i<=n_endeffs;++i) 
    cnext[i] = cart_des_state[i];

  /* ready to go */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
    
  if (ans != 1) 
    return FALSE;
  
  scd();

  return TRUE;

}

static void
init_vars(void) 
{
  if (firsttime) {
    firsttime = FALSE;
    ivec_zero(cstatus);
    vec_zero(cart);
    bzero((char *)&(ctarget[1]),n_endeffs*sizeof(ctarget[1]));
  }
}

/*!*****************************************************************************
 *******************************************************************************
  \note  run_goto_task
  \date  Dec. 1997

  \remarks 

  run the task from the task servo: REAL TIME requirements!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int myTarget = 0;
static int 
run_draw_task(void)
{
  int j, i;
  double sum=0;
  double aux;

  /* has the movement time expired? I intentially run 0.5 sec longer */
  if (tau <= -0.5*0 ) {
	  if (myTarget == 0) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.05/2;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.017;
		  tau = 1;
		  myTarget = 1;
		  run_draw_task();
	  } else if (myTarget == 1) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.05/2;
		  tau = 1;
		  myTarget = 3;
		  run_draw_task();
	  } else if (myTarget == 3) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.03;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.017;
		  tau = 1;
		  myTarget = 4;
		  run_draw_task();
	  } else if (myTarget == 4) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.01;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.017;
		  tau = 1;
		  myTarget = 5;
		  run_draw_task();
	  } else if (myTarget == 5) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.009;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.017;
		  tau = 1;
		  myTarget = 6;
		  run_draw_task();
	  } else if (myTarget == 6) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.01/2;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.017;
		  tau = 1;
		  myTarget = 7;
		  run_draw_task();
	  } else if (myTarget == 7) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.025;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.01;
		  tau = 1;
		  myTarget = 8;
		  run_draw_task();
	  } else if (myTarget == 8) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.025;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.007;
		  tau = 1;
		  myTarget = 9;
		  run_draw_task();
	  } else if (myTarget == 9) {
		  ctarget[RIGHT_HAND].x[_X_] += 0.005;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.03;
		  tau = 1;
		  myTarget = 10;
		  run_draw_task();
	  } else if (myTarget == 10) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.03;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.02;
		  tau = 1;
		  myTarget = 11;
		  run_draw_task();
	  } else if (myTarget == 11) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.02;
		  tau = 1;
		  myTarget = 12;
		  run_draw_task();
	  } else if (myTarget == 12) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.02;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.02;
		  tau = 1;
		  myTarget = 13;
		  run_draw_task();
	  } else if (myTarget == 13) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.02;
		  ctarget[RIGHT_HAND].x[_Z_] += 0.015;
		  tau = 1;
		  myTarget = 14;
		  run_draw_task();
	  }  else if (myTarget == 14) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.02;
		  tau = 1;
		  myTarget = 15;
		  run_draw_task();
	  } else if (myTarget == 15) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.018;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.005;
		  tau = 1;
		  myTarget = 16;
		  run_draw_task();
	  } else if (myTarget == 16) {
		  ctarget[RIGHT_HAND].x[_X_] -= 0.02;
		  ctarget[RIGHT_HAND].x[_Z_] -= 0.03;
		  tau = 1;
		  myTarget = 17;
		  run_draw_task();
	  }
	  else {
		  freeze();
	  }
    return TRUE; 
  }

  /* progress by min jerk in cartesian space */
  calculate_min_jerk_next_step(cnext,ctarget,tau,time_step,cnext);
  tau -= time_step;
 
  /* shuffle the target for the inverse kinematics */
  for (i=1; i<=n_endeffs; ++i) {
    for (j=1; j<=N_CART; ++j) {
      aux  = cnext[i].x[j] - cart_des_state[i].x[j];
      cart[(i-1)*6+j] = cnext[i].xd[j] + 20.*aux;
    }
  }

  /* inverse kinematics */
  for (i=1; i<=n_dofs; ++i) {
    target[i].th = joint_des_state[i].th;
  }
  if (!inverseKinematics(target,endeff,joint_opt_state,
			 cart,cstatus,time_step)) {
    freeze();
    return FALSE;
  }


  // assign desired state
  for (i=1; i<=n_dofs; ++i) {

    joint_des_state[i].thd  = target[i].thd;
    joint_des_state[i].th   = target[i].th;

    // check range of motion violation
    if (joint_des_state[i].th > joint_range[i][MAX_THETA]) {
      joint_des_state[i].th = joint_range[i][MAX_THETA];
      joint_des_state[i].thd = 0.0;
    }
    if (joint_des_state[i].th < joint_range[i][MIN_THETA]) {
      joint_des_state[i].th = joint_range[i][MIN_THETA];
      joint_des_state[i].thd = 0.0;
    }
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
  \note  change_draw_task
  \date  Dec. 1997

  \remarks 

  changes the task parameters

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static int 
change_draw_task(void)
{
  int j, i;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  calculate_min_jerk_next_step
\date  August 1994
   
\remarks 

        given a current cart state and a target cart
	state as well as movement duration, the next increment
	for the cart states is calculated. Note that this 
	is done in only in cartesian dimensions with active status.
	NOTE that this function requires velocity and accelerations
	as input as well!!!

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     curr_states: the current state
 \param[in]     des_states : the desired state
 \param[in]     tau        : the desired movement duration until the goal is
	                 reached.
 \param[in]     dt         : at which delta time is the next_states from now
 \param[out]    next_states: the next state after dt

 ******************************************************************************/
static int 
calculate_min_jerk_next_step (SL_Cstate *curr_state,
			      SL_Cstate *des_state,
			      double tau,
			      double delta_t,
			      SL_Cstate *next_state)

{
  double t1,t2,t3,t4,t5;
  double tau1,tau2,tau3,tau4,tau5;
  int    i,j;

  if (delta_t > tau || tau < 1./(double)task_servo_rate || delta_t <= 0) {
    return FALSE;
  }

  t1 = delta_t;
  t2 = t1 * delta_t;
  t3 = t2 * delta_t;
  t4 = t3 * delta_t;
  t5 = t4 * delta_t;

  tau1 = tau;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  for (j=1; j<=n_endeffs; ++j) {
    for (i=1; i<=N_CART; ++i) {

      if (cstatus[(j-1)*6+i]) {
	
	/* calculate the constants */
	
	const double dist   = des_state[j].x[i] - curr_state[j].x[i];
	const double p1     = des_state[j].x[i];
	const double p0     = curr_state[j].x[i];
	const double a1t2   = des_state[j].xdd[i];
	const double a0t2   = curr_state[j].xdd[i];
	const double v1t1   = des_state[j].xd[i];
	const double v0t1   = curr_state[j].xd[i];
	
	const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 
	  3.*(v0t1 + v1t1)/tau4;
	const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
	  (8.*v0t1 + 7.*v1t1)/tau3; 
	const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
	  (6.*v0t1 + 4.*v1t1)/tau2; 
	const double c4 = curr_state[j].xdd[i]/2.;
	const double c5 = curr_state[j].xd[i];
	const double c6 = curr_state[j].x[i];
	
	next_state[j].x[i]   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
	next_state[j].xd[i]  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
	next_state[j].xdd[i] = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
	
      }
    }
  }
  
  return TRUE;
}
