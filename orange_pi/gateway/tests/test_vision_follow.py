import pytest

from app import Settings, _is_loopback_client
from vision_follow import FollowPolicy, TargetObservation, camera_capture_sources


def test_numeric_v4l2_device_is_preferred_for_opencv() -> None:
    sources = camera_capture_sources("/dev/video3")
    assert 3 in sources
    assert sources.index(3) < sources.index("/dev/video3")


def test_camera_preview_rejects_non_local_clients() -> None:
    class Client:
        def __init__(self, host: str) -> None:
            self.host = host

    class Request:
        def __init__(self, host: str | None) -> None:
            self.client = Client(host) if host else None

    assert _is_loopback_client(Request("127.0.0.1"))
    assert _is_loopback_client(Request("::1"))
    assert not _is_loopback_client(Request("10.42.0.23"))
    assert not _is_loopback_client(Request(None))


def test_gpio_shutdown_is_disabled_until_helper_token_is_configured(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROVER_GPIO_SHUTDOWN_TOKEN", raising=False)
    assert Settings.from_environment().gpio_shutdown_token is None
    monkeypatch.setenv("ROVER_GPIO_SHUTDOWN_TOKEN", "a" * 32)
    settings = Settings.from_environment()
    assert settings.gpio_shutdown_token == "a" * 32


def test_target_loss_and_distance_never_generate_reverse() -> None:
    policy = FollowPolicy()
    assert policy.command(TargetObservation(False, target="red")).linear == 0.0
    close = policy.command(TargetObservation(True, 0.0, 0.2, 1.0, "red"))
    assert close.linear == 0.0 and close.angular == 0.0


def test_follow_policy_centres_before_advancing_and_stays_bounded() -> None:
    policy = FollowPolicy()
    right = policy.command(TargetObservation(True, 0.5, 0.01, 1.0, "blue"))
    assert right.linear == 0.0
    assert -0.8 <= right.angular < 0.0
    centred = policy.command(TargetObservation(True, 0.0, 0.01, 1.0, "green"))
    assert 0.0 < centred.linear <= 0.12
    assert centred.angular == 0.0


def test_person_uses_height_ratio_and_invalid_limits_are_rejected() -> None:
    policy = FollowPolicy()
    far = policy.command(TargetObservation(True, 0.0, 0.2, 0.8, "person"))
    assert 0.0 < far.linear <= 0.12
    reached = policy.command(TargetObservation(True, 0.0, 0.6, 0.8, "person"))
    assert reached.linear == 0.0
    with pytest.raises(ValueError):
        FollowPolicy(max_linear=0.2)
