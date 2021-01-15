from util import initialize_drop_down_button
import rospy
import os
from std_msgs.msg import String


class ExperimentPathInterface(object):
    EXPERIMENT_NAME_OPTIONS = ['default', 'simple', 'moderate', 'advanced']
    LOG_FOLDER = 'results'
    NEW_STRING = "  (new)"

    def __init__(self, experiment_name_button, experiment_id_button):

        self.experiment_name_button = experiment_name_button
        self.experiment_id_button = experiment_id_button

        self.experiment_name = None
        self.experiment_id = None
        self.pub_experiment_pwd = rospy.Publisher("experiment_pwd", String, queue_size=1, latch=True)
        self.prev_experiment_ids = []

        # experiment name dropdown
        initialize_drop_down_button(self.experiment_name_button, self.EXPERIMENT_NAME_OPTIONS, self.handle_experiment_name)
        self.handle_experiment_name()

        # experiment id dropdown
        initialize_drop_down_button(self.experiment_id_button,  self.get_experiment_ids(), self.handle_experiment_id)
        self.handle_experiment_id()

        self.publish_experiment_path()


    def handle_experiment_name(self):
        """
            callback called when experiment name is changed, updates experiment name
        """
        value = str(self.experiment_name_button.currentText())
        if self.experiment_name != value:
            self.experiment_name = value
            rospy.logdebug("Updated experiment name to %s", self.experiment_name)

        # Update id button even if experiment name did not change, since folder structure may have been changed!
        self.update_experiment_id_button()
        self.handle_experiment_id()

    def handle_experiment_id(self):
        """
            callback called when experiment name or id is changed, updates experiment id and publishes path
        """
        self.update_experiment_id_button()
        value = str(self.experiment_id_button.currentText())[0:3]

        if self.experiment_id != value:
            self.experiment_id = value
            rospy.logdebug("Updated experiment id to %s", self.experiment_id)

        self.publish_experiment_path()

    def update_experiment_id_button(self):
        """
            updates the options in the experiment_id_button if they changed compared to previous call
        """
        # get ids
        experiment_ids = self.get_experiment_ids()

        # add to menu when changed
        if self.prev_experiment_ids != experiment_ids:

            self.experiment_id_button.clear()
            for index, experiment_id in enumerate(experiment_ids):
                self.experiment_id_button.addItem(experiment_id)

                if experiment_id == self.experiment_id:
                    self.experiment_id_button.setCurrentIndex(index)

            self.prev_experiment_ids = experiment_ids

    def get_experiment_ids(self):
        """
            returns a list of ids available in the experiment_name_path, and an additional id which creates a new folder
        """
        experiment_name_path = self.get_experiment_name_path()

        options = []
        if not os.path.isdir(experiment_name_path):
            options.append(str(1).zfill(3) + self.NEW_STRING)
        else:
            contents = os.listdir(experiment_name_path)
            if len(contents) == 0:
                options.append(str(1).zfill(3) + self.NEW_STRING)
            else:
                contents.sort()
                for file_name in contents:
                    file_id = file_name[:3]
                    if file_id.isdigit():
                        options.append(file_id.zfill(3))
                if len(options) > 0:
                    file_id = int(options[-1])
                else:
                    file_id = 0
                options.append(str(file_id + 1).zfill(3) + self.NEW_STRING)

        return options

    def get_experiment_name_path(self):
        return os.path.join(os.getcwd(), self.LOG_FOLDER, self.experiment_name)

    def publish_experiment_path(self):
        pwd = os.path.join(self.get_experiment_name_path(), self.experiment_id)
        self.pub_experiment_pwd.publish(pwd)
