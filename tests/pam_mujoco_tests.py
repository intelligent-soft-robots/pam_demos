import os,subprocess,signal,time
from unittest import TestCase

class PamMujocoTest(TestCase):

    def setUp(self,mujoco_id="pam_mujoco_test"):
        time.sleep(1)
        self._mujoco_id=mujoco_id
        command = ["pam_mujoco_no_xterms",self._mujoco_id]
        self._proc = subprocess.Popen(
            command, start_new_session=True, stderr=subprocess.STDOUT)
        time.sleep(1)
        
    def tearDown(self):
        pgid = os.getpgid(self._proc.pid)
        os.killpg(pgid, signal.SIGINT)
        # a subprocess.TimeoutExpired
        # will be thrown if the process
        # does not die in the next 3 seconds
        self._proc.wait(3)
