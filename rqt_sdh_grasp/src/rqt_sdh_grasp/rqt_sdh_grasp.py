import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String

class RqtSdhGrasp(Plugin):

    def __init__(self, context):
        super(RqtSdhGrasp, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtSdhGrasp')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_sdh_grasp'), 'resource', 'sdh_grasp.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtSdhGraspUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.pub_grasp = rospy.Publisher("/px150/pipelineState",
                                      String, queue_size=10, latch=False)

        self._widget.graspButton.clicked[bool].connect(self.handle_grasp)
        self._widget.stopButton.clicked[bool].connect(self.handle_stop)
        self._widget.openButton.clicked[bool].connect(self.handle_open)
        self._widget.homeButton.clicked[bool].connect(self.handle_home)

    def shutdown_plugin(self):
        self.pub_grasp.unregister()

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

    def handle_grasp(self):
        self.pub_grasp.publish("pick")

    def handle_stop(self):
        self.pub_grasp.publish("detect")

    def handle_open(self):
        self.pub_grasp.publish("open")

    def handle_home(self):
        self.pub_grasp.publish("home")
