from util import initialize_drop_down_button
import rospy
import os
from std_msgs.msg import String


class ExperimentPathInterface(object):
    experiment_name_options = ['default', 'simple', 'moderate', 'advanced']

    def __init__(self, experiment_name_button, experiment_id_button):

        self.experiment_name_path = None
        self.experiment_id_path = None

        self.experiment_name_button = experiment_name_button
        self.experiment_id_button = experiment_id_button

        self.experiment_name = None
        self.experiment_id = None
        self.pub_experiment_pwd = rospy.Publisher("experiment_pwd", String, queue_size=1, latch=True)

        # experiment name dropdown

        initialize_drop_down_button(self.experiment_name_button, self.experiment_name_options, self.handle_experiment_name)
        self.handle_experiment_name()

        # experiment id dropdown
        initialize_drop_down_button(self.experiment_id_button,  self.get_experiment_ids(), self.handle_experiment_id)
        self.handle_experiment_id()

        self.experiment_id_path = os.path.join(self.experiment_name_path, self.experiment_id)
        self.pub_experiment_pwd.publish(self.experiment_id_path)

    def handle_experiment_name(self):
        """
            callback called when experiment name is changed, updates experiment id and publishes path
        """
        value = str(self.experiment_name_button.currentText())
        if self.experiment_name != value:
            self.experiment_name = value
            rospy.logdebug("Updated experiment name to %s", self.experiment_name)

        self.update_experiment_name_path()

        self.update_experiment_id_button()
        self.handle_experiment_id()

    def handle_experiment_id(self):
        print "experiment_id_button"

        value = str(self.experiment_id_button.currentText())[0:3]

        if self.experiment_id != value:
            self.experiment_id = value
            rospy.logdebug("Updated experiment id to %s", self.experiment_id)

            self.update_experiment_name_path()
            self.experiment_id_path = os.path.join(self.experiment_name_path, self.experiment_id)
            self.pub_experiment_pwd.publish(self.experiment_id_path)

    def get_experiment_ids(self):
        options = []
        if not os.path.isdir(self.experiment_name_path):
            options.append(str(1).zfill(3) + "  (new)")
        else:
            contents = os.listdir(self.experiment_name_path)
            if len(contents) == 0:
                options.append(str(1).zfill(3) + " (new)")
            else:
                contents.sort()
                for file_name in contents:
                    file_id = int(file_name[:3])
                    options.append(str(file_id).zfill(3))
                options.append(str(file_id + 1).zfill(3) + " (new)")

        return options

    def update_experiment_id_button(self):
        button = self.experiment_id_button

        # get ids
        options = self.get_experiment_ids()

        # add to menu
        button.clear()
        for option in options:
            button.addItem(option)

    def update_experiment_name_path(self):
        self.experiment_name_path = os.path.join(os.getcwd(), 'thesis_data', self.experiment_name)
