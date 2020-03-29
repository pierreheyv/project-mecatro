#ifndef _CTRL_IO_H_
#define _CTRL_IO_H_

#define NB_U_SWITCH 2

// ID of the right and left sides
enum{R_ID, L_ID};

/// Controller inputs
typedef struct CtrlIn
{
	double t; ///< time [s]
	double r_wheel_speed; ///< right wheel speed [Htz]
	double l_wheel_speed; ///< left wheel speed [Htz]

	//variables for beam detection
	double angle;
	double distance;
	double direction;

	int u_switch[NB_U_SWITCH];
} CtrlIn;

/// Controller outputs
typedef struct CtrlOut
{
	/*! \brief wheel commands
	 *
	 * Each wheel is commanded by a voltage in the range [-24 V ; +24 V]. To avoid duty cycles problems, it is better
	 * to work in the range [-0.9*24 V ; +0.9*24 V].
	 *
	 * 'wheel_commands' is a command signal bounded in [-100 ; 100], proportional to the voltage sent to the corresponding
	 * wheel motor. Here are three examples of voltages corresponding to 'wheel_commands' values:
	 *   -100 corresponds to -0.9*24 V
	 *      0 corresponds to       0 V
	 *    100 corresponds to +0.9*24 V
	 */
	double wheel_commands[2]; ///< wheel motors (R_ID or L_ID) commands [-], bounded in [-100 ; 100]

	/*! \brief tower command
	 *
	 * The tower command is similar to the one of the wheels ('wheel_commands'), with the same duty cycle limit corresponding
	 * to a range of [-0.9*24 V ; +0.9*24 V]. However, this motor is too powerful for this tower. Consequently, it is
	 * better to limit 'tower_command' in a range of [-15 ; 15].
	 */
	double tower_command; ///< tower motor command [-], bounded in [-100 ; 100]

} CtrlOut;

#endif
