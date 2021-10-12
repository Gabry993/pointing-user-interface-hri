from typing import Any
import numpy as np

import rclpy
import rclpy.node
from std_srvs.srv import Empty
from conveyor_msgs.srv import AddPack, SetBeltSpeed


class PackGenerator(rclpy.node.Node):  # type: ignore
    def __init__(self) -> None:
        super(PackGenerator, self).__init__("pack_generator")
        self.belt_speed = self.declare_parameter("belt_speed", 0.0).value
        self.package_length = self.declare_parameter("package_length", 0.5).value
        self.n_packs = self.declare_parameter("n_packs", 6).value
        self.red_packs = self.declare_parameter("n_reds", 0).value

        self.add_pack_cli = self.create_client(AddPack, 'add_pack')
        self.set_belt_speed_cli = self.create_client(SetBeltSpeed, 'set_belt_speed')
        self.reset_cli = self.create_client(Empty, 'reset_pack_sim')

        self.speed_req = SetBeltSpeed.Request()
        self.speed_req.speed = self.belt_speed
        self.set_belt_speed_cli.wait_for_service()
        self.set_belt_speed_cli.call_async(self.speed_req)

        self.uid: int = 0
        self.generate_srv = self.create_service(Empty, 'generate_packs', self.generate_packs_cb)

        self.get_logger().info("Ready")

    def generate_packs_cb(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.reset_cli.call_async(Empty.Request())
        self.set_belt_speed_cli.call_async(self.speed_req)
        linsp = np.linspace(0, 10, self.n_packs+1)
        positions = [
            np.random.uniform(linsp[i]+0.60, linsp[i+1]-0.60) for i in range(len(linsp)-1)]
        classes = np.array([2]*self.n_packs)
        reds = np.random.choice(range(len(classes)), self.red_packs, replace=False)
        classes[reds] = 0
        for i, position in enumerate(positions):
            class_ = classes[i]
            pack_req = AddPack.Request()
            pack_req.uid = self.uid
            pack_req.kind = int(class_)
            pack_req.position = (position % 5.) / 5.
            pack_req.belt = int(np.floor(position/5))
            self.add_pack_cli.call_async(pack_req)
            self.uid += 1
        return response


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PackGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
