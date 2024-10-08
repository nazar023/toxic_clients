import yaml

def get_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def get_param(node: Node, name, default_value = None):
    node.declare_parameter(name, default_value, ParameterDescriptor(dynamic_typing=True))
    return node.get_parameter(name).value
