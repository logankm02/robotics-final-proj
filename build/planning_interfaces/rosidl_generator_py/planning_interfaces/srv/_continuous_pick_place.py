# generated from rosidl_generator_py/resource/_idl.py.em
# with input from planning_interfaces:srv/ContinuousPickPlace.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ContinuousPickPlace_Request(type):
    """Metaclass of message 'ContinuousPickPlace_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('planning_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'planning_interfaces.srv.ContinuousPickPlace_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__continuous_pick_place__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__continuous_pick_place__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__continuous_pick_place__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__continuous_pick_place__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__continuous_pick_place__request

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ContinuousPickPlace_Request(metaclass=Metaclass_ContinuousPickPlace_Request):
    """Message class 'ContinuousPickPlace_Request'."""

    __slots__ = [
        '_pick_scan_pose',
        '_place_scan_pose',
        '_pick_distance',
        '_retreat_distance',
        '_place_distance',
        '_place_rotation_y_deg',
    ]

    _fields_and_field_types = {
        'pick_scan_pose': 'geometry_msgs/PoseStamped',
        'place_scan_pose': 'geometry_msgs/PoseStamped',
        'pick_distance': 'float',
        'retreat_distance': 'float',
        'place_distance': 'float',
        'place_rotation_y_deg': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.pick_scan_pose = kwargs.get('pick_scan_pose', PoseStamped())
        from geometry_msgs.msg import PoseStamped
        self.place_scan_pose = kwargs.get('place_scan_pose', PoseStamped())
        self.pick_distance = kwargs.get('pick_distance', float())
        self.retreat_distance = kwargs.get('retreat_distance', float())
        self.place_distance = kwargs.get('place_distance', float())
        self.place_rotation_y_deg = kwargs.get('place_rotation_y_deg', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.pick_scan_pose != other.pick_scan_pose:
            return False
        if self.place_scan_pose != other.place_scan_pose:
            return False
        if self.pick_distance != other.pick_distance:
            return False
        if self.retreat_distance != other.retreat_distance:
            return False
        if self.place_distance != other.place_distance:
            return False
        if self.place_rotation_y_deg != other.place_rotation_y_deg:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pick_scan_pose(self):
        """Message field 'pick_scan_pose'."""
        return self._pick_scan_pose

    @pick_scan_pose.setter
    def pick_scan_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'pick_scan_pose' field must be a sub message of type 'PoseStamped'"
        self._pick_scan_pose = value

    @builtins.property
    def place_scan_pose(self):
        """Message field 'place_scan_pose'."""
        return self._place_scan_pose

    @place_scan_pose.setter
    def place_scan_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'place_scan_pose' field must be a sub message of type 'PoseStamped'"
        self._place_scan_pose = value

    @builtins.property
    def pick_distance(self):
        """Message field 'pick_distance'."""
        return self._pick_distance

    @pick_distance.setter
    def pick_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pick_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pick_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pick_distance = value

    @builtins.property
    def retreat_distance(self):
        """Message field 'retreat_distance'."""
        return self._retreat_distance

    @retreat_distance.setter
    def retreat_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'retreat_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'retreat_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._retreat_distance = value

    @builtins.property
    def place_distance(self):
        """Message field 'place_distance'."""
        return self._place_distance

    @place_distance.setter
    def place_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'place_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'place_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._place_distance = value

    @builtins.property
    def place_rotation_y_deg(self):
        """Message field 'place_rotation_y_deg'."""
        return self._place_rotation_y_deg

    @place_rotation_y_deg.setter
    def place_rotation_y_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'place_rotation_y_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'place_rotation_y_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._place_rotation_y_deg = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ContinuousPickPlace_Response(type):
    """Metaclass of message 'ContinuousPickPlace_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('planning_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'planning_interfaces.srv.ContinuousPickPlace_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__continuous_pick_place__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__continuous_pick_place__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__continuous_pick_place__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__continuous_pick_place__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__continuous_pick_place__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ContinuousPickPlace_Response(metaclass=Metaclass_ContinuousPickPlace_Response):
    """Message class 'ContinuousPickPlace_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_slides_picked',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'slides_picked': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.slides_picked = kwargs.get('slides_picked', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        if self.slides_picked != other.slides_picked:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def slides_picked(self):
        """Message field 'slides_picked'."""
        return self._slides_picked

    @slides_picked.setter
    def slides_picked(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'slides_picked' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'slides_picked' field must be an integer in [-2147483648, 2147483647]"
        self._slides_picked = value


class Metaclass_ContinuousPickPlace(type):
    """Metaclass of service 'ContinuousPickPlace'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('planning_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'planning_interfaces.srv.ContinuousPickPlace')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__continuous_pick_place

            from planning_interfaces.srv import _continuous_pick_place
            if _continuous_pick_place.Metaclass_ContinuousPickPlace_Request._TYPE_SUPPORT is None:
                _continuous_pick_place.Metaclass_ContinuousPickPlace_Request.__import_type_support__()
            if _continuous_pick_place.Metaclass_ContinuousPickPlace_Response._TYPE_SUPPORT is None:
                _continuous_pick_place.Metaclass_ContinuousPickPlace_Response.__import_type_support__()


class ContinuousPickPlace(metaclass=Metaclass_ContinuousPickPlace):
    from planning_interfaces.srv._continuous_pick_place import ContinuousPickPlace_Request as Request
    from planning_interfaces.srv._continuous_pick_place import ContinuousPickPlace_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
