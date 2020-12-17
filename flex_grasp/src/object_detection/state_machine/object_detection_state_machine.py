import rospy
from flex_grasp.msg import FlexGraspErrorCodes

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
            self._is_idle = False
            self.command = command
            self._object_detection.collect_messages()
            self._input.command_accepted()

        elif command == "save_image":
            self._is_idle = False
            self.command = command
            self._object_detection.collect_messages()
            self._input.command_accepted()

        elif command == "e_init":
            self._input.command_accepted()
            self._input.command_completed()

        else:
            self._input.command_rejected()

    def _process_detect_state(self):
        if self.command == "detect_truss":
            if self._object_detection.wait_for_messages():
                self._object_detection.log_input_messages()
                result = self._object_detection.detect_object()
            else:
                result = FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        elif self.command == "save_image":
            if self._object_detection.wait_for_messages():
                self._object_detection.log_input_messages()
                result = self._object_detection.save_data()
            else:
                result = FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        else:
            result = FlexGraspErrorCodes.Failure

        self._object_detection.reset()
        self._input.command_completed(result)
        self._transition_to_idle_state()

    def _transition_to_idle_state(self):
        self._is_idle = True
        self._command = None
        rospy.logdebug("[{0}] Transitioned to idle".format(self.node_name))

    def request_shutdown(self):
        rospy.loginfo("[{0}] Shutdown requested".format(self.node_name))
        self._shutdown_requested = True
