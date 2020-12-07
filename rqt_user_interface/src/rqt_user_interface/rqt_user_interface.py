import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMenu
from std_msgs.msg import String, Bool
from flex_shared_resources.msg import SpawnInstruction

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
#        if not args.quiet:
#            print 'arguments: ', args
#            print 'unknowns: ', unknowns

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

        self.pub_command = rospy.Publisher("pipeline_command",
                                      String, queue_size=10, latch=False)

        self.pub_experiment = rospy.Publisher("experiment",
                                      Bool, queue_size=1, latch=True)
                                      

        self.pub_experiment_pwd = rospy.Publisher("experiment_pwd",
                                      String, queue_size=1, latch=True)

        self.pub_spawn_command = rospy.Publisher("model_spawner/e_in",
                                      SpawnInstruction, queue_size=1, latch=True)

        self.experiment = False
        self._widget.ExperimentButton.setCheckable(True)
        
        # basic commands
        self._widget.SleepButton.clicked[bool].connect(lambda: self.pub_command.publish("sleep"))
        self._widget.HomeButton.clicked[bool].connect(lambda: self.pub_command.publish("home"))
        self._widget.ReadyButton.clicked[bool].connect(lambda: self.pub_command.publish("ready"))
        self._widget.OpenButton.clicked[bool].connect(lambda: self.pub_command.publish("open"))
        self._widget.CloseButton.clicked[bool].connect(lambda: self.pub_command.publish("close"))
        self._widget.CalibrateButton.clicked[bool].connect(lambda: self.pub_command.publish("calibrate"))
        self._widget.CalibrateHeightButton.clicked[bool].connect(lambda: self.pub_command.publish("calibrate_height"))

        self._widget.SpawnTrussButton.clicked[bool].connect(self.handle_spawn_truss)
        self._widget.DetectTrussButton.clicked[bool].connect(lambda: self.pub_command.publish("detect_truss"))
        self._widget.SaveImageButton.clicked[bool].connect(lambda: self.pub_command.publish("save_image"))

        self._widget.PickPlaceButton.clicked[bool].connect(lambda: self.pub_command.publish("pick_place"))
        self._widget.PickButton.clicked[bool].connect(lambda: self.pub_command.publish("pick"))
        self._widget.PlaceButton.clicked[bool].connect(lambda: self.pub_command.publish("place"))
        self._widget.ExperimentButton.clicked.connect(self.handle_experiment)

        def handle_detect_tomato(self):
            self.pub_command.publish("detect_tomato")


        # spawn types dropdown
        options = ['3d', '2d']
        initialize_drop_down_button(self._widget.SelectSpawnTypeButton, options, self.handle_spawn_type)
        self.spawn_type = options[0]

        # experiment name dropdown
        options = ['default', 'simple', 'moderate', 'advanced']
        initialize_drop_down_button(self._widget.ExperimentNameButton, options, self.handle_experiment_name)
        self.experiment_name = options[0]
        
        # experiment id dropdown
        self.update_pwd()
        options = self.get_experiment_ids()
        initialize_drop_down_button(self._widget.ExperimentIDButton, options, self.handle_experiment_id)
        self.experiment_id = options[0]

    def get_experiment_ids(self):
        options = []
        if not os.path.isdir(self.pwd_truss_type):
            options.append(str(1).zfill(3) + "  (new)")
        else:
            contents = os.listdir(self.pwd_truss_type)
            if len(contents) == 0:
                options.append(str(1).zfill(3) + " (new)")
            else:
                contents.sort()
                for file_name in contents:
                    file_id = int(file_name[:3])
                    options.append(str(file_id).zfill(3))
                options.append(str(file_id + 1).zfill(3) + " (new)")

        return options

    def update_button_options(self):
        button = self._widget.ExperimentIDButton

        # get ids
        options = self.get_experiment_ids()

        # add to menu  
        button.clear()
        for option in options:
            button.addItem(option)
                    
    def update_pwd(self):
        self.pwd_truss_type = os.path.join(os.getcwd(), 'thesis_data', self.experiment_name)

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

    def handle_experiment(self):
        self.experiment = self._widget.ExperimentButton.isChecked()
        self.pub_experiment.publish(self.experiment)

    def handle_spawn_type(self):
        self.spawn_type = str(self._widget.SelectSpawnTypeButton.currentText())

    def handle_spawn_truss(self):
        spawn_instruction = SpawnInstruction(type=SpawnInstruction.SPAWN, model_type=self.spawn_type)
        self.pub_spawn_command.publish(spawn_instruction)


    def handle_experiment_name(self):
        button = self._widget.ExperimentNameButton
        value = str(button.currentText())
        if self.experiment_name != value:
            self.experiment_name = value
            rospy.logdebug("Updated experiment name to %s", self.experiment_name)
            self.update_pwd()
            self.update_button_options()
            self.handle_experiment_id()
            
            self.pwd_experiment = os.path.join(self.pwd_truss_type, self.experiment_id)
            self.pub_experiment_pwd.publish(self.pwd_experiment)
        
    def handle_experiment_id(self):
        button = self._widget.ExperimentIDButton
        value = str(button.currentText())[0:3]
        
        if self.experiment_id != value:
            self.experiment_id = value
            rospy.logdebug("Updated experiment id to %s", self.experiment_id)
            
            self.update_pwd()
            self.pwd_experiment = os.path.join(self.pwd_truss_type, self.experiment_id)
            self.pub_experiment_pwd.publish(self.pwd_experiment)

def initialize_drop_down_button(button, options, cb):
    button.clear()
    button.setEditable(True)
    for option in options:
        button.addItem(option)

    button.activated.connect(cb)
    return button