import pytest

from safety import (
    ClearEstopMessage,
    DriveMessage,
    EstopMessage,
    MessageValidationError,
    VisionFollowMessage,
    WheelTestMessage,
    parse_client_message,
)


def test_valid_messages_are_parsed_without_string_coercion() -> None:
    assert parse_client_message(
        '{"type":"drive","linear":0.2,"angular":-0.45,"seq":126}'
    ) == DriveMessage(0.2, -0.45, 126)
    assert parse_client_message(
        '{"type":"drive","linear":0.1,"lateral":-0.3,"angular":0,"seq":127}'
    ) == DriveMessage(0.1, 0.0, 127, -0.3)
    assert parse_client_message({"type": "estop", "seq": 127}) == EstopMessage(127)
    assert parse_client_message(
        {"type": "clear_estop", "seq": 128, "confirm": True}
    ) == ClearEstopMessage(128)
    assert parse_client_message(
        {"type": "vision_follow", "enabled": True, "target": "red", "seq": 129}
    ) == VisionFollowMessage(True, "red", 129)
    assert parse_client_message(
        {"type": "wheel_test", "wheel": 2, "rpm": -30, "seq": 130}
    ) == WheelTestMessage(2, -30, 130)


@pytest.mark.parametrize(
    "message,code",
    [
        ({"type": "drive", "linear": "0.2", "angular": 0, "seq": 1}, "invalid_drive"),
        ({"type": "drive", "linear": True, "angular": 0, "seq": 1}, "invalid_drive"),
        ({"type": "drive", "linear": 1.01, "angular": 0, "seq": 1}, "drive_out_of_range"),
        ({"type": "drive", "linear": 0, "angular": 3.01, "seq": 1}, "drive_out_of_range"),
        ({"type": "drive", "linear": 0, "lateral": 1.01, "angular": 0, "seq": 1}, "drive_out_of_range"),
        ({"type": "drive", "linear": 0, "angular": 0, "seq": True}, "invalid_seq"),
        ({"type": "drive", "linear": 0, "angular": 0, "seq": -1}, "invalid_seq"),
        (
            {"type": "drive", "linear": 0, "angular": 0, "seq": 1, "pwm": 99},
            "invalid_fields",
        ),
        ({"type": "clear_estop", "seq": 2, "confirm": False}, "confirmation_required"),
        (
            {"type": "vision_follow", "enabled": True, "target": "yellow", "seq": 3},
            "invalid_vision_target",
        ),
        ({"type": "wheel_test", "wheel": 4, "rpm": 30, "seq": 4}, "invalid_wheel"),
        ({"type": "wheel_test", "wheel": 0, "rpm": 0, "seq": 4}, "invalid_wheel_rpm"),
    ],
)
def test_invalid_messages_are_rejected(message: dict[str, object], code: str) -> None:
    with pytest.raises(MessageValidationError) as caught:
        parse_client_message(message)
    assert caught.value.code == code


def test_non_finite_json_numbers_are_rejected() -> None:
    with pytest.raises(MessageValidationError) as caught:
        parse_client_message('{"type":"drive","linear":NaN,"angular":0,"seq":1}')
    assert caught.value.code == "invalid_json_number"
