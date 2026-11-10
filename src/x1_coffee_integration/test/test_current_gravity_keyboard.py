#!/usr/bin/env python3
"""键盘补偿会话离线回归，无 ROS 和实机依赖。"""
import ast
import types
import runpy
from pathlib import Path
import unittest

NS = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/keyboard_tcp_teleop.py'))
Session = NS['CurrentGravitySession']

class CurrentSessionTest(unittest.TestCase):
    def test_enter_needs_actual_confirmation(self):
        session = Session()
        self.assertFalse(session.enter(lambda: True, lambda: False))
        self.assertTrue(session.locked)
        self.assertFalse(session.enter(lambda: self.fail('重复进入'), lambda: True))

    def test_request_timeout_keeps_lock(self):
        session = Session()
        self.assertFalse(session.enter(lambda: False, lambda: self.fail('未接受请求')))
        self.assertTrue(session.locked)
        self.assertFalse(session.exit(lambda: True, lambda: False))
        self.assertTrue(session.locked)

    def test_already_exited_requires_measured_hold(self):
        session = Session()
        self.assertTrue(session.enter(lambda: True, lambda: True))
        self.assertEqual(session.state, 'active')
        self.assertTrue(session.exit(lambda: False, lambda: True))
        self.assertFalse(session.locked)

    def test_exception_never_unlocks(self):
        session = Session()
        def error():
            raise RuntimeError('连接失效')
        with self.assertRaises(RuntimeError):
            session.enter(error, lambda: True)
        self.assertTrue(session.locked)
        with self.assertRaises(RuntimeError):
            session.exit(error, lambda: True)
        self.assertTrue(session.locked)

class LateEnterTest(unittest.TestCase):
    def test_unresolved_enter_cannot_unlock_from_old_hold(self):
        source = Path(NS['__file__']).read_text()
        method = next(n for n in ast.walk(ast.parse(source))
                      if isinstance(n, ast.FunctionDef) and n.name == 'leave_current')
        scope = {'rclpy': types.SimpleNamespace(spin_until_future_complete=lambda *a, **k: None)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<leave_current>', 'exec'), scope)
        calls = []
        session = Session()
        session.state = 'unknown'
        node = types.SimpleNamespace(current_session=session, motion_requested=True,
            current_enter_future=types.SimpleNamespace(done=lambda: False),
            current_query=lambda name: calls.append(name) or True,
            current_confirm=lambda **kwargs: self.fail('迟到进入之前禁止确认退出'))
        self.assertFalse(scope['leave_current'](node))
        self.assertTrue(session.locked)
        self.assertFalse(node.motion_requested)
        self.assertEqual(calls, ['exit'])

class JointFreshnessTest(unittest.TestCase):
    def test_duplicate_and_backward_messages_cannot_extend_quiet_window(self):
        method = next(n for n in ast.walk(ast.parse(Path(NS['__file__']).read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == '_on_joints')
        now=[1.0]
        scope=dict(NS,JointState=object,time=types.SimpleNamespace(monotonic=lambda: now[0]))
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<on_joints>', 'exec'), scope)
        node=types.SimpleNamespace(joint_stamp_ns=0,quiet_since=None,
                                   joint_received_at=None,measured={})
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1,nanosec=0)),
            name=NS['COFFEE_JOINTS'],position=[0.]*15,velocity=[0.]*15)
        receive=lambda:scope['_on_joints'](node,msg)
        receive()
        self.assertEqual(node.quiet_since,1.)
        now[0]=1.4
        receive()
        self.assertIsNone(node.quiet_since)
        self.assertEqual(node.joint_received_at,1.)
        msg.header.stamp.sec=0;receive()
        self.assertEqual(node.joint_received_at,1.)
        msg.header.stamp.sec=2;receive()
        self.assertEqual(node.quiet_since,1.4)
        msg.header.stamp.sec=3;msg.velocity[0]=.1;receive()
        self.assertIsNone(node.quiet_since)

if __name__ == '__main__':
    unittest.main()
