import rospy


class ObjectDetectionStateMachine(object):

    def __init__(self, object_detection, state_input, update_rate, node_name):
        self.node_name = node_name
        self._update_rate = update_rate
        self._input = state_input
        self._object_detection = object_detection

        self._is_idle = True
        self._command = None
        self._shutdown_requested = False

    def run(self):

        rate = rospy.Rate(self._update_rate)
        while not self._shutdown_requested:
            if self._is_idle:
                self._process_idle_state()
            else:
                self._process_detect_state()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

    def _process_idle_state(self):
        command = self._input.command

        if command is None:
            return

        elif command == "detect_truss":
            rospy.logdebug("[OBEJCT DETECTION] Detect truss")
            self._is_idle = False

            # self.take_picture = True
            # result = self.detect_object()

        elif command == "save_image":
            rospy.logdebug("[OBEJCT DETECTION] Take picture")
            self.take_picture = True
            result = self.log_image()

        elif command == "e_init":
            self._input.command_accepted()
            self._input.command_completed()

        else self.command is not None:
            self._input.command_rejected()


    def _process_detect_state(self):
        self._object_detection.take_action()

        result = self.detect_object()

    def _transition_to_idle_state(self):
        pass

    def request_shutdown(self):
        pass