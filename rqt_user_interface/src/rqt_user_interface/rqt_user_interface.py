import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String

class RqtFlexGrasp(Plugin):

    def __init__(self, context):
        super(RqtFlexGrasp, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtFlexGrasp')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_user_interface'), 'resource', 'flex_grasp.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtFlexGraspUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.pub_command = rospy.Publisher("pipelineState",
                                      String, queue_size=10, latch=False)

        # basic commands
        self._widget.SleepButton.clicked[bool].connect(self.handle_sleep)
        self._widget.HomeButton.clicked[bool].connect(self.handle_home)
        self._widget.OpenButton.clicked[bool].connect(self.handle_open)
        self._widget.CloseButton.clicked[bool].connect(self.handle_close)
        self._widget.CalibrateButton.clicked[bool].connect(self.handle_calibrate)

        # tasks
        self._widget.DetectTomatoButton.clicked[bool].connect(self.handle_detect_tomato)
        self._widget.DetectTrussButton.clicked[bool].connect(self.handle_detect_truss)
        self._widget.SaveImageButton.clicked[bool].connect(self.handle_save_image)

        self._widget.PointButton.clicked[bool].connect(self.handle_point)
        self._widget.PickPlaceButton.clicked[bool].connect(self.handle_pick_place)
        self._widget.PickButton.clicked[bool].connect(self.handle_pick)
        self._widget.PlaceButton.clicked[bool].connect(self.handle_place)

    def shutdown_plugin(self):
        self.pub_command.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def handle_sleep(self):
        self.pub_command.publish("sleep")

    def handle_home(self):
        self.pub_command.publish("home")

    def handle_open(self):
        self.pub_command.publish("open")

    def handle_close(self):
        self.pub_command.publish("close")
        
    def handle_calibrate(self):
        self.pub_command.publish("calibrate")

    def handle_detect_tomato(self):
        self.pub_command.publish("detect_tomato")

    def handle_detect_truss(self):
        self.pub_command.publish("detect_truss")
        
    def handle_save_image(self):
        self.pub_command.publish("save_image")

    def handle_point(self):
        self.pub_command.publish("point")

    def handle_pick_place(self):
        self.pub_command.publish("pick_place")

    def handle_pick(self):
        self.pub_command.publish("pick")

    def handle_place(self):
        self.pub_command.publish("place")
