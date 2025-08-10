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
        self.service_name = service_name
        self.timeout_sec = timeout_sec
        self.cli = node.create_client(srv_type, service_name)
        self.node.get_logger().info(
            f"Creating GenericServiceAdapter for service: {service_name} "
            f"with type: {srv_type.__name__}"
        )

    def execute(self, **kwargs) -> Future:
        self.node.get_logger().info(
            f"Calling service {self.cli.srv_name} with data: {kwargs}"
        )
        self.ensure_available(timeout_sec=self.timeout_sec)

        # Normalize path-related params *before* building the request
        self._prepare_map_path(kwargs)

        # Build request and set only provided fields
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
        # Compose from map_dir + map_stem
        if "map_dir" in kwargs:
            base_dir = Path(os.path.expanduser(kwargs["map_dir"])).resolve()
            base_dir.mkdir(parents=True, exist_ok=True)
            stem = kwargs.get("map_stem", "map")
            kwargs["name"] = str(base_dir / stem)
            # Drop keys that do not exist in slam_toolbox/srv/SaveMap
            kwargs.pop("map_dir", None)
            kwargs.pop("map_stem", None)
            kwargs.pop("map_topic", None)
            kwargs.pop("image_format", None)
            kwargs.pop("map_mode", None)
            kwargs.pop("free_thresh", None)
            kwargs.pop("occupied_thresh", None)
            return
