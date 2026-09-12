#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * Vehicle and safety calibration.
 *
 * The wheel geometry values below are safe integration defaults only.  Measure
 * the installed wheel diameter and left/right tyre-centre spacing, then update
 * them before closed-loop road testing.
 */
#define CAR_WHEEL_RADIUS_M                 0.0325f
#define CAR_WHEEL_TRACK_M                  0.1800f

#define CAR_ENCODER_MOTOR_PPR              13U
#define CAR_ENCODER_QUADRATURE_FACTOR      4U
#define CAR_MOTOR_GEAR_RATIO               30U
#define CAR_ENCODER_COUNTS_PER_WHEEL_REV   \
  (CAR_ENCODER_MOTOR_PPR * CAR_ENCODER_QUADRATURE_FACTOR * \
   CAR_MOTOR_GEAR_RATIO)

#define CAR_PWM_MAX_COMMAND                7200
#define CAR_MAX_WHEEL_RPM                  180.0f
#define CAR_WHEEL_TEST_MAX_RPM             40

#define CAR_LINEAR_MIN_MPS                 (-1.0f)
#define CAR_LINEAR_MAX_MPS                 1.0f
#define CAR_ANGULAR_MIN_RPS                (-3.0f)
#define CAR_ANGULAR_MAX_RPS                3.0f
#define CAR_LATERAL_MIN_MPS                (-1.0f)
#define CAR_LATERAL_MAX_MPS                1.0f

/* This vehicle uses four mecanum wheels in the M1/M2/M3/M4 layout. */
#define AX_MOTOR_ENABLE_STRAFE             1

/*
 * Logical right-strafe sequence is FL+, FR-, RL-, RR+.
 * AX_MOTOR_SetWheelSpeeds maps those wheel positions to physical channels
 * C+, A-, B-, D+ before applying the per-channel forward polarity below.
 */
#define CAR_MECANUM_STRAFE_RIGHT_SIGN      (1)

#define CAR_CONTROL_PERIOD_MS              10U
#define CAR_SAFETY_PERIOD_MS               10U
#define CAR_STATUS_PERIOD_MS               100U
#define CAR_COMM_TIMEOUT_MS                300U

/* Motors must be disabled and all four filtered wheel speeds below this value
   before the MPU6500 zero-bias tracker is allowed to learn. */
#define CAR_IMU_STATIONARY_RPM_X10         10

/* Front HC-SR04 interlock. Distances are from the sensor, not the bumper. */
#define CAR_ULTRASONIC_STOP_MM             200U
#define CAR_ULTRASONIC_RELEASE_MM          300U
#define CAR_ULTRASONIC_STALE_MS            250U
#define CAR_ULTRASONIC_RELEASE_HOLD_MS     500U
#define CAR_OBSTACLE_RETREAT_MAX_MPS       0.15f

/*
 * Optional physical BLE-pairing request button. PB0 uses the internal pull-up;
 * wire a normally-open momentary button between PB0 and GND. Pairing is only
 * requested after the vehicle has remained stopped throughout the long press.
 */
#define CAR_BLE_PAIR_BUTTON_ENABLED        1U
#define CAR_BLE_PAIR_HOLD_MS               3000U
#define CAR_BLE_PAIR_SIGNAL_MS             1000U
#if (CAR_BLE_PAIR_BUTTON_ENABLED == 1U)
#define CAR_BLE_PAIR_BUTTON_PRESSED()      \
  (HAL_GPIO_ReadPin(BLE_PAIR_BUTTON_GPIO_Port, BLE_PAIR_BUTTON_Pin) == \
   GPIO_PIN_RESET)
#else
#define CAR_BLE_PAIR_BUTTON_PRESSED()      (0U)
#endif

/*
 * Optional Orange Pi shutdown button. PC5 uses the internal pull-up. Wire a
 * normally-closed momentary button between PC5 and GND: released holds PC5
 * low, while pressing opens the circuit and makes PC5 high. A three-second
 * press is accepted only after all STM32 motor outputs are already zero.
 * RESET cannot serve this purpose because the MCU does not execute while held
 * in reset.
 */
#define CAR_OPI_SHUTDOWN_BUTTON_ENABLED    0U
#define CAR_OPI_SHUTDOWN_HOLD_MS           3000U
#if (CAR_OPI_SHUTDOWN_BUTTON_ENABLED == 1U)
#define CAR_OPI_SHUTDOWN_BUTTON_PRESSED()  \
  (HAL_GPIO_ReadPin(OPI_SHUTDOWN_BUTTON_GPIO_Port, OPI_SHUTDOWN_BUTTON_Pin) == \
   GPIO_PIN_SET)
#else
#define CAR_OPI_SHUTDOWN_BUTTON_PRESSED()  (0U)
#endif

/* Keep NRST reset independent from Orange Pi shutdown. */
#define CAR_OPI_SHUTDOWN_ON_NRST_ENABLED   0U
#define CAR_OPI_SHUTDOWN_ON_NRST_DELAY_MS  800U

/* Conservative initial gains. Tune each installed wheel with the chassis up. */
#define CAR_PID_KP                         8.0f
#define CAR_PID_KI                         24.0f
#define CAR_PID_KD                         0.02f
#define CAR_PID_KFF                        12.0f
#define CAR_PID_INTEGRAL_LIMIT             220.0f

/*
 * Static-friction compensation used only while a commanded wheel has not yet
 * reached 5 RPM. Each wheel can be calibrated independently on the vehicle.
 * 1200 / 7200 is about 16.7% duty and is a conservative initial value.
 */
#define CAR_MOTOR_START_MIN_TARGET_RPM     15U
#define CAR_MOTOR_MOVING_RPM_X10           50U
#define CAR_MOTOR_M1_START_COMMAND         1200U
#define CAR_MOTOR_M2_START_COMMAND         1200U
#define CAR_MOTOR_M3_START_COMMAND         1200U
#define CAR_MOTOR_M4_START_COMMAND         1200U

/*
 * Logical wheel order: front-left, front-right, rear-left, rear-right.
 * Physical driver order is A=front-left, B=rear-left, C=front-right,
 * D=rear-right. Final wheels-off-ground calibration gives the electrical
 * forward signs A-, B+, C+, D-.
 * Encoder polarity remains auto-calibrated after sustained motion.
 */
#define CAR_MOTOR_M1_FORWARD_SIGN          (-1)
#define CAR_MOTOR_M2_FORWARD_SIGN          (-1)
#define CAR_MOTOR_M3_FORWARD_SIGN          (1)
#define CAR_MOTOR_M4_FORWARD_SIGN          (1)
#define CAR_ENCODER_M1_FORWARD_SIGN        (1)
#define CAR_ENCODER_M2_FORWARD_SIGN        (1)
#define CAR_ENCODER_M3_FORWARD_SIGN        (1)
#define CAR_ENCODER_M4_FORWARD_SIGN        (1)

/*
 * No physical emergency-stop input is assigned in AVG_V1.ioc.  Keep this 0
 * until a normally-closed input has been wired and configured.  When enabled,
 * CAR_PHYSICAL_ESTOP_RELEASED() must evaluate to non-zero only while the button
 * circuit is healthy and released.
 */
#define CAR_PHYSICAL_ESTOP_ENABLED         0U
#if (CAR_PHYSICAL_ESTOP_ENABLED == 1U)
#define CAR_PHYSICAL_ESTOP_RELEASED()      \
  (HAL_GPIO_ReadPin(CAR_ESTOP_GPIO_Port, CAR_ESTOP_Pin) == GPIO_PIN_SET)
#else
#define CAR_PHYSICAL_ESTOP_RELEASED()      (1U)
#endif

#endif /* CAR_CONFIG_H */







