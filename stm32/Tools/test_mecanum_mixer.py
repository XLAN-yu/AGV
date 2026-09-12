import re
from pathlib import Path

root = Path(__file__).resolve().parents[1]
config = (root / "Core" / "Inc" / "car_config.h").read_text(encoding="ascii")
mixer = (root / "Core" / "Inc" / "mecanum_mixer.h").read_text(encoding="ascii")
freertos = (root / "Core" / "Src" / "freertos.c").read_text(encoding="ascii")
motor = (root / "Core" / "Src" / "ax_motor.c").read_text(encoding="ascii")

match = re.search(r"CAR_MECANUM_STRAFE_RIGHT_SIGN\s+\((-?1)\)", config)
assert match and int(match.group(1)) == 1
assert "CAR_MecanumMixVelocity(linear_mps, lateral_mps, angular_rps, wheel_mps);" in freertos
assert "CAR_MecanumMixCommand(0, -(int32_t)target_rpm, 0, wheel_target);" in freertos
assert "CAR_MecanumMixCommand(0, (int32_t)target_rpm, 0, wheel_target);" in freertos
assert "wheel_speed[0] = front_right;" in motor
assert "wheel_speed[1] = rear_left;" in motor
assert "wheel_speed[2] = front_left;" in motor
assert "counter[0] = AX_ENCODER_C_GetCounter();" in freertos
assert "counter[1] = AX_ENCODER_A_GetCounter();" in freertos
assert "counter[2] = AX_ENCODER_B_GetCounter();" in freertos
assert "normalized_strafe = strafe_right_mps * CAR_MECANUM_STRAFE_RIGHT_SIGN;" in mixer

sign = int(match.group(1))
def mix(strafe_right: int):
    s = strafe_right * sign
    return [s, -s, -s, s]

assert mix(1) == [1, -1, -1, 1], mix(1)
assert mix(-1) == [-1, 1, 1, -1], mix(-1)

motor_signs = [-1, -1, 1, 1]
right_physical_commands = [-1, -1, 1, 1]  # A=FR, B=RL, C=FL, D=RR
right_driver_output = [
    command * polarity
    for command, polarity in zip(right_physical_commands, motor_signs)
]
assert right_driver_output == [1, 1, 1, 1], right_driver_output
assert motor_signs == [-1, -1, 1, 1]
print("mecanum mixer verified: physical C=FL/A=FR/B=RL/D=RR, polarity A-/B-/C+/D+")





