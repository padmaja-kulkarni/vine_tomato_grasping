import rospy
from flex_grasp.msg import FlexGraspErrorCodes


DEFAULT_DREAM = True

class ObjectDetectionStateMachine(object):

    def __init__(self, object_detection, state_input, update_rate, node_name):
        self.node_name = node_name
        self._update_rate = update_rate
        self._input = state_input
        self._object_detection = object_detection

        self._is_idle = True
        self._command = None
        self._shutdown_requested = False

        self.dream = rospy.get_param("dream_camera", DEFAULT_DREAM)

        if self.dream:
            rospy.loginfo("[{0}] Object detection launched in dream mode!".format(self.node_name))

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
            rospy.logdebug("[{0}] Detect truss".format(self.node_name))
            self._is_idle = False
            self.command = command
            self._input.take_picture = True
            if self.dream:
                self._input.load_messages()

            self._input.command_accepted()

        elif command == "save_image":
            rospy.logdebug("[{0}] Take picture".format(self.node_name))
            self._is_idle = False
            self.command = command
            self._input.take_picture = True
            self._input.command_accepted()

        elif command == "e_init":
            self._input.command_accepted()
            self._input.command_completed()

        else:
            self._input.command_rejected()

    def _process_detect_state(self):
        if self.command == "detect_truss":
            if self._input.wait_for_messages():

                # we do not wat to log the dream
                if not self.dream:
                    self._input.log_messages()
                self._set_data()
                result = self._object_detection.detect_object(save_result=not self.dream)
            else:
                result = FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        elif self.command == "save_image":
            if self._input.wait_for_messages():
                self._input.log_messages()
                self._set_data()
                result = self._object_detection.log_image()
            else:
                result = FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        else:
            result = FlexGraspErrorCodes.Failure

        self._input.command_completed(result)
        self._transition_to_idle_state()

    def _set_data(self):
        self._object_detection.color_image = self._input.color_image
        self._object_detection.depth_image = self._input.depth_image
        self._object_detection.pcl = self._input.pcl
        self._object_detection.color_info = self._input.camera_info
        self._object_detection.pwd_data = self._input.data_path
        self._object_detection.id = self._input.id

    def _transition_to_idle_state(self):
        self._is_idle = True
        self._command = None
        rospy.logdebug("[{0}] Transitioned to idle".format(self.node_name))

    def request_shutdown(self):
        pass