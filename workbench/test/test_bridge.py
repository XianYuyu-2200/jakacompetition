"""ROS 桥的纯逻辑测试；不导入 ROS，不启动仿真。"""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

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


if __name__ == "__main__":
    unittest.main()
