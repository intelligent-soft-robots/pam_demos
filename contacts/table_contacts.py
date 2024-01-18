import math
import time
import o80
from handle_contacts import MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID
from handle_contacts import get_handle

robot1_contact = False
robot2_contact = False
table_contact = True

handle = get_handle(robot1_contact, robot2_contact, table_contact)
ball = handle.frontends[BALL_SEGMENT_ID]


def _distance(p1, p2):
    return math.sqrt([(a - b) ** 2 for a, b in zip(p1, p2)])


def _velocity(p1, p2, duration):
    return [(a - b) / duration for a, b in zip(p2, p1)]


def _dropping(start, end, duration):
    global handle, ball

    # deativating contacts
    handle.deactivate_contact(BALL_SEGMENT_ID, "table")

    # going to start position
    ball.add_command(
        start, [0, 0, 0], o80.Duration_us.milliseconds(400), o80.Mode.QUEUE
    )
    ball.pulse_and_wait()
    time.sleep(2)

    # starting with clean contact
    handle.reset_contact(BALL_SEGMENT_ID, "table")

    # reactivating contacts
    handle.activate_contact(BALL_SEGMENT_ID, "table")

    # going to end point
    velocity = _velocity(start, end, duration)
    ball.add_command(end, velocity, o80.Duration_us.seconds(duration), o80.Mode.QUEUE)
    ball.pulse_and_wait()

    # breathing a bit
    time.sleep(1)


starts = [
    (0.5, 1.0, 2.5),
]


for start in starts:
    duration = 1
    end = (0.7, 1.2, 0.0)
    _dropping(start, end, duration)
