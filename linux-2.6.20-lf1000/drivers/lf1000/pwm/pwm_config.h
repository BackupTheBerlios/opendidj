/* LF1000 Pulse Width Modulator (PWM) Driver
 *
 * pwm_config.h -- PWM settings.
 *
 * Andrey Yurovsky <andrey@cozybit.com> */

#ifndef PWM_CONFIG_H
#define PWM_CONFIG_H

/* number of PWM output channels */
#ifdef CPU_MF2530F
#define PWM_NUM_CHANNELS	4
#elif defined CPU_LF1000
#define PWM_NUM_CHANNELS	3
#endif

/* source clock that PWM state machine uses */
#define PWM_CLK_SRC			PLL1		/* PLL number */

/* PWM derived clock settings */
#define PWM_CLOCK_HZ			10240000
#define PWM_POLARITY			POL_BYP

#endif

