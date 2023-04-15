import sys
import rclpy
import time

from rclpy.node import Node
from rmf_task_msgs.srv import SubmitTask
from rmf_task_msgs.msg import TaskDescription, Loop

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

class TaskSubmitter(Node):

    def __init__(self):
        super().__init__('task_submitter')
        self.cli = self.create_client(SubmitTask, '/submit_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            sys.stdout.write("/submit_task service not available, waiting again...")
            sys.stdout.flush()
        self.req = SubmitTask.Request()
        self.req.requester = "elixir task submitter"
        # self.req.description.start_time = 0
        # self.req.description.priority = 1
        self.req.description.task_type.type = 1
        self.req.description.loop.num_loops = 1

    def send_request(self, start_name, finish_name):
        self.req.description.loop.task_id = "elixir_" + str(time.time())
        self.req.description.loop.start_name = start_name
        self.req.description.loop.finish_name = finish_name

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        

def main(args=None):
    try: 
        rclpy.init(args=args)

        task_submitter = TaskSubmitter()

        for line in sys.stdin:
            line = line.strip()
            if line == "" : break

            values = line.split(",")
            if len(values) != 2:
                sys.stdout.write("invalid: 2 arguments not given")
                sys.stdout.flush()

            response = task_submitter.send_request(values[0], values[1])

            if response.success:
                sys.stdout.write(f"success task_id: {response.task_id}")
                sys.stdout.flush()
            else:
                #sys.stdout.write(f"fail task_id: {response.task_id} message: {response.message}")
                sys.stdout.write("fail")
                sys.stdout.flush()

        task_submitter.destroy_node()
        rclpy.shutdown()
    except:
        task_submitter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
