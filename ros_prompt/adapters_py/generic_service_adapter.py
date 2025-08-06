from rclpy.task import Future
from ros_prompt.utilities.ros_type_utils import set_nested_attr
import os
from pathlib import Path

class GenericServiceAdapter:
    """
    Create a thin, reusable wrapper around any ROS 2 service.

    Args:
        node          rclpy.node.Node - calling node
        service_name  str            - fully-qualified service name
        srv_type      type           - generated *srv* class (e.g. nav2_msgs.srv.SaveMap)
        timeout_sec   float          - wait-for-service timeout
    """
    def __init__(self, node, service_name, srv_type, timeout_sec: float = 5.0):
        self.node = node
        self.node.get_logger().info(
            f"Creating GenericServiceAdapter for service: {service_name} "
            f"with type: {srv_type.__name__}"
        )
        self.cli = node.create_client(srv_type, service_name)
        self.service_name = service_name

    # NOTE: returns the Future so callers can watch completion if they like
    def execute(self, **kwargs) -> Future:
        self.node.get_logger().info(
            f"Calling service {self.cli.srv_name} with data: {kwargs}"
        )
        self.ensure_available()
        self._prepare_map_path(kwargs)
        req = self.cli.srv_type.Request()
        for field, value in kwargs.items():
            set_nested_attr(req, field, value)
        return self.cli.call_async(req)
    
    def ensure_available(self, timeout_sec=5.0):
        if not self.cli.service_is_ready():
            self.node.get_logger().info(f"Waiting for {self.service_name} …")
            if not self.cli.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"Service {self.service_name} unavailable after {timeout_sec}s")
    
    @staticmethod
    def _prepare_map_path(kwargs: dict, stem_key: str = "map_url", dir_key: str = "map_dir"):
        """
        Normalise the directory and stem parameters in-place.
        Accepts either:
        • map_url = "/abs/path/stem"           (classic Nav2 option)
        • map_dir + map_stem = dir + stem      (ROS Prompt convenience)
        """
        if dir_key in kwargs:
            base_dir = Path(os.path.expanduser(kwargs[dir_key])).resolve()
            base_dir.mkdir(parents=True, exist_ok=True)
            stem = kwargs.get("map_stem", "map")
            kwargs["map_url"] = str(base_dir / stem)
        elif stem_key in kwargs:
            p = Path(os.path.expanduser(kwargs[stem_key])).resolve()
            p.parent.mkdir(parents=True, exist_ok=True)
            kwargs[stem_key] = str(p)

