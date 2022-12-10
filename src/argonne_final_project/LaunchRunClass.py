#!/usr/bin/env python
#This is a class to launch and run nodes. See the example at the bottom
import roslaunch, rospkg, os,rospy
from time import sleep
print('howdy')

class CustomNodeRun(object):
    """
    Create A class that can run nodes from a launch file with given parameters.
    """
    def __init__(self,package,executable,args = '',args2 = ''):
        print('got here')
        self.package = package
        self.executable = executable 
        print(args)
        self.args = args
        self.args2 = args2
        self.node = roslaunch.core.Node(package, executable, output = "screen",args=self.args)

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.process = None

    def start(self):
        """
            Function:
                Start the node.
        """
        self.process = self.launch.launch(self.node)
        return self

    def stop(self):
        """
            Function:
                Kill the node.
        """
        if self.process is not None:
            if self.process.is_alive():
                self.process.stop()

    def is_alive(self):
        """
            Function:
                Check the status of the node, whether it is running or killed.
        """
        if self.process is None:return False
        return self.process.is_alive() 



class CustomLaunchRun(object):
    """
    Run a launch file from this class.
    """
    def __init__(self,package,executable):
        self.package = package
        self.executable = executable
        rospack = rospkg.RosPack()

        path = rospack.get_path(self.package)

        def find(name, path):
            for root, dirs, files in os.walk(path):
                if name in files:
                    return os.path.join(root, name)
            return None

        self.launchFile = find(self.executable,path)
        if self.launchFile == None:  raise Exception("Could not find %s in package %s"%(self.executable,self.package))

        self.uuid = None
        self.launch = None

    def start(self):
        """
            Function:
                Run the launch file give the path.
        """
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.launchFile])

        self.launch.start()
        return self
    def stop(self):
        """
            Function:
                Ends the launch.
        """
        if self.launch is not None: self.launch.shutdown()

