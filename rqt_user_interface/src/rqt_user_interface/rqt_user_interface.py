import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMenu
from std_msgs.msg import String, Bool

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

        self.pub_command = rospy.Publisher("pipeline_command",
                                      String, queue_size=10, latch=False)

        self.pub_experiment = rospy.Publisher("experiment",
                                      Bool, queue_size=1, latch=True)
                                      

        self.pub_experiment_pwd = rospy.Publisher("experiment_pwd",
                                      String, queue_size=1, latch=True)
                                      

        self.go = False
        self._widget.ExperimentButton.setCheckable(True)
        # self._widget.ExperimentButton.toggle()
        
        # basic commands
        self._widget.SleepButton.clicked[bool].connect(self.handle_sleep)
        self._widget.HomeButton.clicked[bool].connect(self.handle_home)
        self._widget.OpenButton.clicked[bool].connect(self.handle_open)
        self._widget.CloseButton.clicked[bool].connect(self.handle_close)
        self._widget.CalibrateButton.clicked[bool].connect(self.handle_calibrate)

        # tasks
        # self._widget.DetectTomatoButton.clicked[bool].connect(self.handle_detect_tomato)
        self._widget.DetectTrussButton.clicked[bool].connect(self.handle_detect_truss)
        self._widget.SaveImageButton.clicked[bool].connect(self.handle_save_image)

        self._widget.PointButton.clicked[bool].connect(self.handle_point)
        self._widget.PickPlaceButton.clicked[bool].connect(self.handle_pick_place)
        self._widget.PickButton.clicked[bool].connect(self.handle_pick)
        self._widget.PlaceButton.clicked[bool].connect(self.handle_place)
        self._widget.ExperimentButton.clicked.connect(self.handle_experiment) # [bool].connect(self.handle_experiment)

        #
        self.truss_type_button = self._widget.TrussTypeButton
        self.truss_type_button.clear()
        self.truss_type_button.setEditable(True)
        options = ['default', 'simple', 'moderate', 'advanced']
        for option in options:
            self.truss_type_button.addItem(option)
        self.truss_type_button.activated.connect(self.handle_truss_type)
        self.truss_type = options[0]
        self.experiment_id = '001'
        self.update_pwd()
        
        #
        self.experiment_id_button = self._widget.ExperimentIDButton
        self.experiment_id_button.setEditable(True)
        self.update_id()
        self.experiment_id_button.activated.connect(self.handle_experiment_id)        

    def update_id(self):
        # get ids
        options= []
        if not os.path.isdir(self.pwd_truss_type):
            options.append(str(1).zfill(3) + "  (new)")
        else:
            contents = os.listdir(self.pwd_truss_type)
            print(contents)
            if len(contents) == 0:
                options.append(str(1).zfill(3) + " (new)")
            else:
                contents.sort()
                for file_name in contents:
                    file_id = int(file_name[:3])
                    options.append(str(file_id).zfill(3))
                options.append(str(file_id + 1).zfill(3) + " (new)")

        # add to menu  
        self.experiment_id_button.clear()
        print(options)
        for option in options:
            self.experiment_id_button.addItem(option)
                    
    def update_pwd(self):
        truss_type = self.truss_type
        self.pwd_truss_type = os.path.join(os.sep, 'home', 'taeke', 'Documents', 'thesis_data', truss_type)

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
        
    def handle_experiment(self):
        self.experiment = self._widget.ExperimentButton.isChecked()
        self.pub_experiment.publish(self.experiment)
        
    def handle_truss_type(self):
        value =  str(self.truss_type_button.currentText())
        if self.truss_type != value:
            self.truss_type = value
            rospy.loginfo("Updated truss type to %s", self.truss_type)
            self.update_pwd()
            self.update_id()
            self.handle_experiment_id()
            
            self.pwd_experiment = os.path.join(self.pwd_truss_type, self.experiment_id)
            self.pub_experiment_pwd.publish(self.pwd_experiment)
        
    def handle_experiment_id(self):
        value =  str(self.experiment_id_button.currentText())[0:3]
        
        if self.experiment_id != value:
            self.experiment_id = value
            rospy.loginfo("Updated experiment id to %s", self.experiment_id)
            
            self.update_pwd()
            self.pwd_experiment = os.path.join(self.pwd_truss_type, self.experiment_id)
            self.pub_experiment_pwd.publish(self.pwd_experiment)