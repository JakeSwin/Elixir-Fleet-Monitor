import os
import sys
import rclpy
import time
import yaml
import base64
import subprocess
from PIL import Image

from os import path
from rclpy.node import Node
from rmf_building_map_msgs.srv import GetBuildingMap

from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE, SIG_DFL)

class ReadBuildingMap(Node):

    def __init__(self):
        super().__init__('read_building_map')
        self.cli = self.create_client(GetBuildingMap, '/get_building_map')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            sys.stdout.write("/get_building_map service not available, waiting again...")
            sys.stdout.flush()
        self.req = GetBuildingMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        

def main(args=None):
    try: 
        rclpy.init(args=args)

        read_building_map = ReadBuildingMap()

        response = read_building_map.send_request()

        folder_path = path.normpath(path.join(path.abspath(__file__), "..", ".."))
        level = response.building_map.levels[0]

        im = base64.b64encode(level.images[0].data)
        with open(path.join(folder_path, "current_map.txt"), "w") as file:
            file.write(im.decode("utf-8"))

        with open(path.join(folder_path, "current_map.png"), "wb") as file:
            file.write(base64.b64decode(im))

        verticies_list = [{"x": i.x, "y": i.y} for i in level.nav_graphs[0].vertices]
        edges_list = [{"v1_idx": i.v1_idx, "v2_idx": i.v2_idx} for i in level.nav_graphs[0].edges]
        image_topics = subprocess.run(["ros2", "topic", "find", "sensor_msgs/msg/Image"], stdout=subprocess.PIPE).stdout.decode("utf-8").split("\n")

        data = {
            "name": response.building_map.name,
            level.name: {
                "locations": [
                    {index: vertex.name} for index, vertex in zip(
                        range(len(level.nav_graphs[0].vertices)), level.nav_graphs[0].vertices
                    ) if vertex.name != ''
                ],
                "map": {
                    "length": len(level.images[0].data),
                    "name": level.images[0].name,
                    "x_offset": level.images[0].x_offset,
                    "y_offset": level.images[0].y_offset,
                    "yaw": level.images[0].yaw,
                    "scale": level.images[0].scale,
                    "encoding": level.images[0].encoding,
                    "data": path.join(folder_path, "current_map.txt")
                },
                "nav_graph": {
                    "verticies": {
                        key: value for key, value in zip(
                            range(len(verticies_list)), verticies_list
                        )
                    },
                    "edges": {
                        key: value for key, value in zip(
                            range(len(edges_list)), edges_list
                        )
                    }
                }
            }
        }

        if len(image_topics) >= 1 and not image_topics[0] == "":
            data["image_topics"] = list(filter(None, image_topics))

        with Image.open(path.join(folder_path, "current_map.png")) as im:
            width, height = im.size
            data[level.name]["map"]["width"] = width
            data[level.name]["map"]["height"] = height

        with open(path.join(folder_path, "current_level.yaml"), "w") as file:
            yaml.dump(data, file)
        
        sys.stdout.write("Config generated successfully")
        sys.stdout.flush()

        read_building_map.destroy_node()
        rclpy.shutdown()
    except:
        read_building_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
