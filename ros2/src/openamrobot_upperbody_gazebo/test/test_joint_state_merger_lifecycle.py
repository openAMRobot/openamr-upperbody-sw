# SPDX-License-Identifier: MIT
"""Real rclpy signal-during-callback regression; no simulation result is inferred."""
from pathlib import Path
import os
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/joint_state_merger.py'


@pytest.mark.parametrize('signal_name', ['SIGINT', 'SIGTERM'])
def test_signal_finishes_active_callback_before_context_shutdown(signal_name):
    code = r'''
import importlib.util, os, signal, sys, time
from rclpy.node import Node
spec=importlib.util.spec_from_file_location('merger',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Probe(Node):
 def __init__(self, description):
  super().__init__('merger_callback_probe');self.timer=self.create_timer(.01,self.callback)
 def callback(self):
  os.kill(os.getpid(),getattr(signal,sys.argv[2]));time.sleep(.05)
  assert self.context.ok(), 'context destroyed during active callback'
  print('callback completed with live context',flush=True)
 def destroy_node(self):
  assert self.context.ok(), 'context destroyed before node'
  print('node destroyed with live context',flush=True);super().destroy_node()
assert m.main(args=[],factory=Probe)==0
'''
    env = dict(os.environ, ROS_DOMAIN_ID='179', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    done = subprocess.run([sys.executable, '-c', code, str(SCRIPT), signal_name], env=env,
                          text=True, capture_output=True, timeout=15)
    assert done.returncode == 0, done.stdout + done.stderr
    assert 'callback completed with live context' in done.stdout
    assert 'node destroyed with live context' in done.stdout
    assert 'Traceback' not in done.stderr


def test_wrapper_uses_installed_upstream_joint_merger_without_reimplementing_it():
    text = SCRIPT.read_text()
    assert 'from joint_state_publisher.joint_state_publisher import JointStatePublisher' in text
    assert 'factory=JointStatePublisher' in text
    assert 'except RuntimeError' not in text
