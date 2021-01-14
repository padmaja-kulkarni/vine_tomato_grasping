import rospy

class PickPlaceStateMachine(object):

    def __init__(self, pick_place, update_rate, node_name):
        """Generates a state machine

        In order to start the state machine see `run`.

        :param float update_rate: update rate in Hz
        """

        self.node_name = node_name
        self._update_rate = update_rate
        self._pick_place = pick_place

        self._is_idle = True
        self._command = None
        self._shutdown_requested = False

    def run(self):
        rate = rospy.Rate(self._update_rate)
        while not self._shutdown_requested:
            # if self._is_idle:
            #     self._process_idle_state()
            # else:
            #     self._process_transform_state()
            self._pick_place.take_action()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

    # def _process_idle_state(self):
    #     command = self._input.command
    #
    #     if command is None:
    #         return
    #
    #     if command == "transform":
    #         object_pose = self._input.object_pose
    #
    #         if object_pose is None:
    #             rospy.logwarn("[%s] Cannot transform pose, since object_pose is still empty!", self.node_name)
    #             self._input.command_rejected()
    #         else:
    #             rospy.logdebug("[%s] executing transform command", self.node_name)
    #             self._command = command
    #             self._object_pose = object_pose
    #             self._is_idle = False
    #             self._input.command_accepted()
    #
    #     elif command == "e_init":
    #         self._input.command_accepted()
    #         self._input.command_completed()
    #
    #     else:
    #         self._input.command_rejected()
    #
    # def _process_transform_state(self):
    #     result = self._transform_pose.generate_action_poses(self._object_pose)
    #     self._input.command_completed(result)
    #     self._transition_to_idle_state()
    #
    # def _transition_to_idle_state(self):
    #     self._is_idle = True
    #     self._command = None
    #     rospy.logdebug("[{0}] Transitioned to idle".format(self.node_name))

    def request_shutdown(self):
        self._shutdown_requested = True
