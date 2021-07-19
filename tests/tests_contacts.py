import o80,pam_mujoco,time
from .pam_mujoco_tests import PamMujocoTest

class ContactsTestCase(PamMujocoTest):

    def setUp(self):
        super().setUp()
        self._ball_segment_id="pam_mujoco_test_ball"
        
    def tearDown(self):
        super().tearDown()

    def _get_one_ball_handle(self,accelerated):
        ball_control = pam_mujoco.MujocoItem.CONSTANT_CONTROL
        ball_contact = pam_mujoco.ContactTypes.table
        ball = pam_mujoco.MujocoItem(self._ball_segment_id,
                                     control=ball_control,
                                     contact_type=ball_contact)
        graphics=True
        handle = pam_mujoco.MujocoHandle(self._mujoco_id, # see PamMujocoTest
                                         table=True,
                                         balls=(ball,),
                                         graphics=graphics,
                                         accelerated_time=accelerated)

        return handle

    @staticmethod
    def _velocity(p1,p2,duration):
        """
        constant velocity when going from p1 to p2
        over duration (sec)
        """
        return [(a-b)/duration for a,b in zip(p2,p1)]

    @staticmethod
    def _dropping(start,end,duration,
                  handle,ball,ball_segment_id):
        # deativating contacts
        handle.deactivate_contact(ball_segment_id)
        # going to start position
        ball.add_command(start,[0,0,0],o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
        ball.pulse_and_wait()
        # starting with clean contact
        handle.reset_contact(ball_segment_id)
        # reactivating contacts
        handle.activate_contact(ball_segment_id)
        ## going to end point
        velocity = ContactsTestCase._velocity(start,end,duration)
        ball.add_command(end,velocity,o80.Duration_us.seconds(duration),o80.Mode.QUEUE)
        ball.pulse_and_wait()
        ## breathing a bit
        time.sleep(1)

    def _test_through_table(self,accelerated):
        handle = self._get_one_ball_handle(accelerated)
        frontend = handle.frontends[self._ball_segment_id]
        # trajectory that goes through the table
        start = (1.0,1.5,0.5)
        end = (1.1,0.5,-1.0)
        duration = 1 # second
        start_iteration = frontend.latest().get_iteration()
        self._dropping(start,end,duration,
                       handle,frontend,self._ball_segment_id)
        # getting all ball observations
        obs = frontend.get_observations_since(start_iteration)
        # getting all z values
        ball_zs = [o.get_position()[2] for o in obs]
        # z value of the table
        table = pam_mujoco.models.Table
        table_z = table.default_position[2]+table.default_size[2]/2.0
        # checking if ball when through table
        for ball_z in ball_zs:
            self.assertTrue(ball_z>=table_z)
    
        
    def test_throught_table(self):
        """
        test that balls do not go through the
        table (realtime)
        """
        self._test_through_table(False)

    #def test_throught_table_accelerated(self):
    #    """
    #    test that balls do not go through the
    #    table (accelerated_time)
    #    """
    #    self._test_through_table(True)



    
