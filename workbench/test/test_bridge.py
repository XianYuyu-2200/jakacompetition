"""ROS 桥逻辑回归；需 ROS Python 环境，但不创建节点或启动仿真。"""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from workbench.ros_bridge import round_snapshot, close_reference_node


class RoundSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.current = {"run_state": "finished", "scene": {"seed": 42},
                        "score": {"collected_count": 6}}

    def test_finished_round_keeps_result(self):
        self.assertEqual(round_snapshot(self.current), self.current)

    def test_reset_hides_latched_scene_and_score(self):
        self.current["run_state"] = "idle"
        self.assertEqual(round_snapshot(self.current),
                         {"run_state": "idle", "scene": {}, "score": None})
        # 重复发出状态也不能恢复上一轮内容；不依赖回调到达顺序。
        self.assertEqual(round_snapshot(self.current)["scene"], {})

    def test_new_round_never_displays_old_score(self):
        for state in ("armed", "running"):
            self.current["run_state"] = state
            public = round_snapshot(self.current)
            self.assertEqual(public["scene"], {"seed": 42})
            self.assertIsNone(public["score"])

    def test_early_score_is_shown_after_finish(self):
        self.current["run_state"] = "running"
        self.assertIsNone(round_snapshot(self.current)["score"])
        self.current["run_state"] = "finished"
        self.assertEqual(round_snapshot(self.current)["score"], {"collected_count": 6})

    def test_default_is_empty_idle(self):
        self.assertEqual(round_snapshot({}),
                         {"run_state": "idle", "scene": {}, "score": None})


class ReferenceCleanupTests(unittest.TestCase):
    def test_background_scene_client_closes_before_main_node(self):
        calls = Mock()
        node = SimpleNamespace(scene_cli=SimpleNamespace(close=calls.close),
                               destroy_node=calls.destroy)
        close_reference_node(node)
        self.assertEqual([call[0] for call in calls.mock_calls], ["close", "destroy"])

    def test_main_node_is_destroyed_even_if_scene_close_fails(self):
        node = SimpleNamespace(scene_cli=SimpleNamespace(close=Mock(side_effect=RuntimeError("close failed"))),
                               destroy_node=Mock())
        with self.assertRaises(RuntimeError):
            close_reference_node(node)
        node.destroy_node.assert_called_once_with()



class ReferenceOutcomeTests(unittest.TestCase):
    def test_done_does_not_mean_success(self):
        # No ROS graph or motion: fake nodes exercise the same bridge exit path.
        from workbench.ros_bridge import reference
        for done, succeeded, expected in ((True, False, 2), (True, True, 0), (False, False, 2)):
            with self.subTest(done=done, succeeded=succeeded):
                node = SimpleNamespace(wait_for_scene=Mock(return_value=True),
                                       run=Mock(), done=done, succeeded=succeeded,
                                       scene_cli=SimpleNamespace(close=Mock()), destroy_node=Mock())
                with patch('rclpy.init'), patch('rclpy.ok', return_value=False), \
                     patch('jaka_competition_kit.ref_track1.Track1Reference', return_value=node), \
                     patch('workbench.ros_bridge.emit'):
                    self.assertEqual(reference(.3), expected)
                node.scene_cli.close.assert_called_once()
                node.destroy_node.assert_called_once()


if __name__ == "__main__":
    unittest.main()


def test_pose_move_requires_complete_model_limits():
    from workbench.ros_bridge import move_joints
    node = SimpleNamespace(destroy_node=Mock())
    backend = SimpleNamespace(joint_names=[f'joint_{i}' for i in range(1, 7)],
                              _limits={}, goto_joints=Mock())
    with patch('rclpy.init'), patch('rclpy.shutdown'), \
         patch('rclpy.create_node', return_value=node), \
         patch('jaka_competition_kit.backend.MoveGroupBackend', return_value=backend), \
         patch('workbench.ros_bridge._spin_until', return_value=False), \
         patch('workbench.ros_bridge.emit') as emit:
        assert move_joints([0.0] * 6) == 2
        backend.goto_joints.assert_not_called()
        assert '限位' in emit.call_args.args[0]['detail']


def test_pose_move_rejects_target_outside_actual_model_limits():
    from workbench.ros_bridge import move_joints
    node = SimpleNamespace(destroy_node=Mock())
    backend = SimpleNamespace(joint_names=[f'joint_{i}' for i in range(1, 7)],
                              _limits={f'joint_{i}': (-1, 1) for i in range(1, 7)},
                              goto_joints=Mock())
    with patch('rclpy.init'), patch('rclpy.shutdown'), \
         patch('rclpy.create_node', return_value=node), \
         patch('jaka_competition_kit.backend.MoveGroupBackend', return_value=backend), \
         patch('workbench.ros_bridge._spin_until', return_value=True), \
         patch('workbench.ros_bridge.emit'):
        assert move_joints([0, 2, 0, 0, 0, 0]) == 2
        backend.goto_joints.assert_not_called()


def test_guarded_goal_release_before_acceptance_cancels_owned_goal():
    from workbench.ros_bridge import guarded_move_goal
    result = Mock()
    result.done.side_effect = [False, True]
    result.result.return_value = SimpleNamespace(status=5)
    cancel = Mock()
    cancel.done.return_value = True
    cancel.result.return_value = SimpleNamespace(goals_canceling=[1])
    handle = SimpleNamespace(accepted=True, get_result_async=Mock(return_value=result),
                             cancel_goal_async=Mock(return_value=cancel))
    pending = Mock()
    pending.done.side_effect = [False, True]
    pending.result.return_value = handle
    action = SimpleNamespace(send_goal_async=Mock(return_value=pending))
    deadman = Mock(side_effect=[True, False])
    with patch('rclpy.ok', return_value=True), patch('rclpy.spin_once'):
        answer = guarded_move_goal(object(), action, object(), deadman)
    assert answer['outcome'] == 'stopped'
    handle.cancel_goal_async.assert_called_once()
    action.send_goal_async.assert_called_once()


def test_guarded_goal_cancel_ack_is_not_terminal_confirmation():
    from workbench.ros_bridge import guarded_move_goal
    result = Mock()
    result.done.return_value = False
    cancel = Mock()
    cancel.done.return_value = True
    cancel.result.return_value = SimpleNamespace(goals_canceling=[1])
    handle = SimpleNamespace(accepted=True, get_result_async=Mock(return_value=result),
                             cancel_goal_async=Mock(return_value=cancel))
    pending = Mock()
    pending.done.return_value = True
    pending.result.return_value = handle
    action = SimpleNamespace(send_goal_async=Mock(return_value=pending))
    with patch('rclpy.ok', return_value=True), patch('rclpy.spin_once'):
        answer = guarded_move_goal(object(), action, object(),
                                   Mock(side_effect=[True, False]), cancel_timeout=0)
    assert answer['outcome'] == 'uncertain'
    handle.cancel_goal_async.assert_called_once()


def test_guarded_goal_never_sends_if_deadman_absent():
    from workbench.ros_bridge import guarded_move_goal
    action = SimpleNamespace(send_goal_async=Mock())
    assert guarded_move_goal(object(), action, object(), lambda: False)['outcome'] == 'stopped'
    action.send_goal_async.assert_not_called()


def test_guarded_goal_late_acceptance_timeout_is_uncertain():
    from workbench.ros_bridge import guarded_move_goal
    pending = Mock()
    pending.done.return_value = False
    action = SimpleNamespace(send_goal_async=Mock(return_value=pending))
    with patch('rclpy.ok', return_value=True):
        answer = guarded_move_goal(object(), action, object(), lambda: True, accept_timeout=0)
    assert answer['outcome'] == 'uncertain'


def test_pipe_deadman_expires_stale_buffer_and_eof():
    import os
    import time
    from workbench.ros_bridge import PipeDeadman
    read, write = os.pipe()
    try:
        deadman = PipeDeadman(read, timeout=0.1)
        os.write(write, f'H{time.monotonic() - 1:.9f}\n'.encode())
        assert not deadman()
    finally:
        os.close(read)
        os.close(write)
    read, write = os.pipe()
    try:
        deadman = PipeDeadman(read)
        os.write(write, f'H{time.monotonic():.9f}\n'.encode())
        assert deadman()
        os.close(write)
        write = -1
        assert not deadman()
    finally:
        os.close(read)
        if write != -1:
            os.close(write)


def test_guarded_goal_success_status_requires_moveit_success():
    from workbench.ros_bridge import guarded_move_goal
    result = Mock()
    result.done.return_value = True
    result.result.return_value = SimpleNamespace(status=4, result=SimpleNamespace(error_code=SimpleNamespace(val=-1)))
    handle = SimpleNamespace(accepted=True, get_result_async=Mock(return_value=result))
    pending = Mock()
    pending.done.return_value = True
    pending.result.return_value = handle
    action = SimpleNamespace(send_goal_async=Mock(return_value=pending))
    assert guarded_move_goal(object(), action, object(), lambda: True)['outcome'] == 'failed'


def test_continuous_jog_targets_are_small_and_respect_limits():
    from workbench.ros_bridge import _bounded_jog_target
    assert _bounded_jog_target(0.0, (-2.0, 2.0), 1) == 0.02
    assert _bounded_jog_target(0.0, (-2.0, 2.0), -1) == -0.02
    assert _bounded_jog_target(1.975, (-2.0, 2.0), 1) == 1.98
    assert _bounded_jog_target(1.98, (-2.0, 2.0), 1) is None
    assert _bounded_jog_target(-1.98, (-2.0, 2.0), -1) is None


def test_continuous_jog_rejects_invalid_segment_configuration():
    import pytest
    from workbench.ros_bridge import _bounded_jog_target
    with pytest.raises(ValueError):
        _bounded_jog_target(0.0, (-1.0, 1.0), 1, segment=float('inf'))


def test_guarded_goal_cancel_rejection_waits_for_racing_terminal_result():
    from workbench.ros_bridge import guarded_move_goal
    result = Mock()
    result.done.side_effect = [False, False, True]
    result.result.return_value = SimpleNamespace(
        status=4, result=SimpleNamespace(error_code=SimpleNamespace(val=1)))
    cancel = Mock()
    cancel.done.return_value = True
    cancel.result.return_value = SimpleNamespace(goals_canceling=[])
    handle = SimpleNamespace(accepted=True, get_result_async=Mock(return_value=result),
                             cancel_goal_async=Mock(return_value=cancel))
    pending = Mock()
    pending.done.return_value = True
    pending.result.return_value = handle
    action = SimpleNamespace(send_goal_async=Mock(return_value=pending))
    deadman = Mock(side_effect=[True, False, False, False])
    with patch('rclpy.ok', return_value=True), patch('rclpy.spin_once'):
        answer = guarded_move_goal(object(), action, object(), deadman)
    assert answer['outcome'] == 'completed'
