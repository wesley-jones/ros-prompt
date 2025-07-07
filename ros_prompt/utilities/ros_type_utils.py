import importlib

def import_msg_class(self, msg_type_str):
    # Accepts "geometry_msgs/msg/Vector3" or "geometry_msgs/Vector3"
    if '/msg/' not in msg_type_str and '/' in msg_type_str:
        pkg, msg = msg_type_str.split('/')
        msg_type_str = f"{pkg}/msg/{msg}"
    # self.get_logger().info(f"Importing message class for: {msg_type_str}")
    mod_path, class_name = msg_type_str.replace('/', '.').rsplit('.', 1)
    mod = importlib.import_module(mod_path)
    return getattr(mod, class_name)