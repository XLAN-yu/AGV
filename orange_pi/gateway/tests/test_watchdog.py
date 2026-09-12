import pytest

from safety import (
    ClearEstopMessage,
    DriveMessage,
    EstopMessage,
    MessageValidationError,
    ObstacleGuardMessage,
    ShutdownOrangePiMessage,
    SafetyController,
    VisionFollowMessage,
    WheelTestMessage,
    parse_client_message,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def ready_controller() -> tuple[SafetyController, FakeClock]:
    clock = FakeClock()
    controller = SafetyController(clock=clock)
    controller.set_serial_ready(True)
    controller.begin_client()
    return controller, clock


def test_watchdog_forces_zero_at_200ms_and_repeats_zero() -> None:
    controller, clock = ready_controller()
    decision = controller.accept_drive(DriveMessage(0.4, -0.2, 1))
    assert decision.accepted

    clock.advance(0.199)
    assert controller.poll_watchdog() is None

    clock.advance(0.001)
    action = controller.poll_watchdog()
    assert action is not None
    assert action.newly_tripped
    assert (action.output.linear, action.output.angular) == (0.0, 0.0)

    clock.advance(0.05)
    assert controller.poll_watchdog() is None
    clock.advance(0.05)
    repeated = controller.poll_watchdog()
    assert repeated is not None
    assert not repeated.newly_tripped


def test_wheel_test_uses_watchdog_and_rejects_faults() -> None:
    controller, clock = ready_controller()
    decision = controller.request_wheel_test(WheelTestMessage(0, 30, 1))
    assert decision.accepted
    clock.advance(0.2)
    assert controller.poll_watchdog() is not None

    controller.observe_hardware_fault(0x31)
    rejected = controller.request_wheel_test(WheelTestMessage(1, 30, 2))
    assert not rejected.accepted
    assert rejected.reason == "hardware_fault_latched"


def test_serial_disconnect_forces_stop_and_rejects_drive() -> None:
    controller, _ = ready_controller()
    assert controller.accept_drive(DriveMessage(0.3, 0.0, 5)).accepted

    stop = controller.set_serial_ready(False)

    assert stop.cause == "serial_disconnected"
    assert controller.current.linear == 0
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 6)).accepted


def test_obstacle_guard_changes_only_before_control_and_requires_boolean() -> None:
    controller, _ = ready_controller()
    controller.obstacle_guard_supported = True
    message = parse_client_message('{"type":"obstacle_guard","enabled":false,"seq":5}')
    assert message == ObstacleGuardMessage(False, 5)
    accepted = controller.request_obstacle_guard(message)
    assert accepted.accepted
    controller.confirm_obstacle_guard(False)
    assert not controller.obstacle_guard_enabled
    assert controller.accept_drive(DriveMessage(0.2, 0.0, 6)).accepted
    active = controller.request_obstacle_guard(ObstacleGuardMessage(True, 7))
    assert not active.accepted
    assert active.reason == "control_active"
    with pytest.raises(MessageValidationError):
        parse_client_message('{"type":"obstacle_guard","enabled":1,"seq":8}')


def test_shutdown_requires_confirmation_and_latches_estop() -> None:
    controller, _ = ready_controller()
    message = parse_client_message(
        '{"type":"shutdown_orange_pi","seq":9,"confirm":true}'
    )
    assert message == ShutdownOrangePiMessage(9)
    decision = controller.request_orange_pi_shutdown(message)
    assert decision.accepted
    assert decision.reason == "shutdown_scheduled"
    assert controller.estop_latched
    with pytest.raises(MessageValidationError):
        parse_client_message('{"type":"shutdown_orange_pi","seq":10,"confirm":false}')


def test_estop_is_latched_until_confirmed_clear() -> None:
    controller, _ = ready_controller()
    controller.latch_estop(EstopMessage(10))

    blocked = controller.accept_drive(DriveMessage(0.2, 0.0, 11))
    assert not blocked.accepted
    assert blocked.reason == "estop_latched"
    assert controller.estop_latched

    requested = controller.request_clear_estop(ClearEstopMessage(12))
    assert requested.accepted
    assert requested.reason == "awaiting_stm32_ack"
    assert controller.clear_estop_pending
    assert controller.estop_latched
    still_blocked = controller.accept_drive(DriveMessage(0.2, 0.0, 13))
    assert not still_blocked.accepted

    cleared = controller.confirm_clear_estop()
    assert cleared.accepted
    assert not controller.estop_latched
    assert controller.current.linear == 0


def test_clear_rejection_and_timeout_keep_estop_latched() -> None:
    controller, clock = ready_controller()
    controller.latch_estop(EstopMessage(20))

    assert controller.request_clear_estop(ClearEstopMessage(21)).accepted
    rejected = controller.reject_clear_estop("stm32_rejected_hardware_fault")
    assert not rejected.accepted
    assert controller.estop_latched
    assert not controller.clear_estop_pending
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 22)).accepted

    assert controller.request_clear_estop(ClearEstopMessage(23)).accepted
    clock.advance(0.999)
    assert controller.poll_clear_estop_timeout() is None
    clock.advance(0.001)
    timed_out = controller.poll_clear_estop_timeout()
    assert timed_out is not None
    assert timed_out.reason == "stm32_ack_timeout"
    assert controller.estop_latched
    assert not controller.clear_estop_pending


def test_serial_disconnect_cancels_pending_clear_without_unlocking() -> None:
    controller, _ = ready_controller()
    controller.latch_estop(EstopMessage(30))
    assert controller.request_clear_estop(ClearEstopMessage(31)).accepted

    controller.set_serial_ready(False)

    assert controller.estop_latched
    assert not controller.clear_estop_pending
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 32)).accepted


def test_hardware_estop_status_latches_but_false_status_never_unlocks() -> None:
    controller, _ = ready_controller()
    assert not controller.estop_latched

    asserted = controller.observe_hardware_estop(True)
    assert asserted.reason == "hardware_estop_latched"
    assert controller.estop_latched
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 40)).accepted

    observed_clear = controller.observe_hardware_estop(False)
    assert observed_clear.reason == "hardware_estop_false_observed"
    assert controller.estop_latched
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 41)).accepted


def test_clear_without_gateway_latch_is_terminal_failure() -> None:
    controller, _ = ready_controller()

    decision = controller.request_clear_estop(ClearEstopMessage(50))

    assert not decision.accepted
    assert decision.reason == "estop_not_latched"
    assert not controller.clear_estop_pending


def test_hardware_fault_latches_until_matching_safety_reset() -> None:
    controller, _ = ready_controller()
    assert controller.accept_drive(DriveMessage(0.3, 0.0, 60)).accepted

    fault = controller.observe_hardware_fault(7)
    assert fault.reason == "hardware_fault_latched"
    assert controller.hardware_fault_latched
    assert controller.hardware_fault_code == 7
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 61)).accepted

    controller.observe_hardware_fault(0)
    assert controller.hardware_fault_latched
    assert controller.hardware_fault_code == 7
    assert not controller.accept_drive(DriveMessage(0.3, 0.0, 62)).accepted

    assert controller.request_clear_estop(ClearEstopMessage(63)).accepted
    assert controller.confirm_clear_estop().accepted
    assert not controller.hardware_fault_latched
    assert controller.hardware_fault_code == 0
    assert controller.accept_drive(DriveMessage(0.0, 0.0, 64)).accepted


def test_front_obstacle_allows_only_slow_straight_retreat() -> None:
    controller, _ = ready_controller()
    transition = controller.observe_hardware_fault(0x30)
    assert transition.reason == "obstacle_blocked"
    assert controller.obstacle_blocked
    assert not controller.hardware_fault_latched

    forward = controller.accept_drive(DriveMessage(0.2, 0.0, 1))
    assert not forward.accepted
    assert forward.reason == "obstacle_direction_blocked"
    assert forward.output is not None and forward.output.linear == 0.0

    turning_retreat = controller.accept_drive(DriveMessage(-0.1, 0.2, 2))
    assert not turning_retreat.accepted
    sideways = controller.accept_drive(DriveMessage(-0.1, 0.0, 3, lateral=0.1))
    assert not sideways.accepted

    retreat = controller.accept_drive(DriveMessage(-0.5, 0.0, 4))
    assert retreat.accepted
    assert retreat.reason == "obstacle_retreat"
    assert retreat.output is not None and retreat.output.linear == -0.15
    assert retreat.output.angular == 0.0 and retreat.output.lateral == 0.0

    zero = controller.accept_drive(DriveMessage(0.0, 0.0, 5))
    assert zero.accepted
    controller.observe_hardware_fault(0)
    assert not controller.obstacle_blocked
    assert controller.accept_drive(DriveMessage(0.2, 0.1, 6)).accepted


def test_camera_follow_uses_existing_fault_and_watchdog_chain() -> None:
    controller, clock = ready_controller()
    start = controller.request_vision_follow(VisionFollowMessage(True, "red", 1))
    assert start.accepted
    command = controller.accept_autonomy(0.12, -0.4)
    assert command.accepted
    assert command.output is not None and command.output.cause == "vision_follow"

    controller.observe_hardware_fault(0x30)
    blocked = controller.accept_autonomy(0.12, 0.0)
    assert not blocked.accepted
    assert blocked.reason == "obstacle_direction_blocked"
    assert blocked.output is not None and blocked.output.linear == 0.0

    controller.observe_hardware_fault(0)
    assert controller.accept_autonomy(0.08, 0.0).accepted
    clock.advance(0.2)
    timed_out = controller.poll_watchdog()
    assert timed_out is not None and timed_out.output.linear == 0.0

    stopped = controller.request_vision_follow(VisionFollowMessage(False, "red", 2))
    assert stopped.accepted and stopped.output is not None
    assert stopped.output.linear == 0.0


def test_duplicate_drive_sequence_is_rejected_but_uint32_wrap_is_allowed() -> None:
    controller, _ = ready_controller()
    assert controller.accept_drive(DriveMessage(0.0, 0.0, 0xFFFFFFFF)).accepted
    assert not controller.accept_drive(DriveMessage(0.0, 0.0, 0xFFFFFFFF)).accepted
    assert controller.accept_drive(DriveMessage(0.0, 0.0, 0)).accepted
