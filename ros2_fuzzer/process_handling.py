"""
ROS Fuzzer process health handling module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy
import psutil


class FuzzedLocalProcessHandler:
    """
    Health checking class for locally running ROS nodes.
    """
    def __init__(self, node_name):
        """
        Health checker initialization.

        :param node_name: Target node that will be queried for aliveness.
        """
        self.node_name = node_name
        self.node_pid = -1
        self.master_conn = ServerProxy('http://127.0.0.1:11311')
        self.get_node_pid()


    def get_node_pid(self):
        """
        Get the target node PID.
        """
        code, msg, val = self.master_conn.lookupNode('', self.node_name)
        if code == 1:
            node_conn = ServerProxy(val)
            node_code, node_msg, node_val = node_conn.getPid('')
            if code == 1:
                self.node_pid = int(node_val)
        else:
            raise Exception(msg)


    def check_if_alive(self):
        """
        Check if the target node's PID is alive. Target function to use in test cases.

        :return: Status of the node. True if the node is alive, False otherwise.
        """
        return psutil.pid_exists(self.node_pid)






