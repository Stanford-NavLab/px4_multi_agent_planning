""" 
File containing utility functions for working with ROS 2 parameters
"""

import rclpy

def get_param_value(param_val):
    """Retreive value from ParameterValue.msg.

    Uses type field to determine what type the value is.

    """

    param_type = param_val.type

    if param_type == 1:
        value = param_val.bool_value
    elif param_type == 2:
        value = param_val.integer_value
    elif param_type == 3:
        value = param_val.double_value
    elif param_type == 4:
        value = param_val.string_value
    elif param_type == 5:
        value = param_val.byte_array_value
    elif param_type == 6:
        value = param_val.bool_array_value
    elif param_type == 7:
        value = param_val.integer_array_value
    elif param_type == 8:
        value = param_val.double_array_value
    elif param_type == 9:
        value = param_val.string_array_value
    elif param_type == 0:
        print("Parameter not set")
    else:
        print("Invalid ParameterValue msg")
    
    return value