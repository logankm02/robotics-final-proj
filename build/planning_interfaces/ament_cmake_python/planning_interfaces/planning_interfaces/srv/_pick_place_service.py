# generated from rosidl_generator_py/resource/_idl.py.em
# with input from planning_interfaces:srv/PickPlaceService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PickPlaceService_Request(type):
    """Metaclass of message 'PickPlaceService_Request'."""

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
                'planning_interfaces.srv.PickPlaceService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pick_place_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pick_place_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pick_place_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pick_place_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pick_place_service__request

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


class PickPlaceService_Request(metaclass=Metaclass_PickPlaceService_Request):
    """Message class 'PickPlaceService_Request'."""

    __slots__ = [
        '_pick_pose',
        '_place_pose',
    ]

    _fields_and_field_types = {
        'pick_pose': 'geometry_msgs/PoseStamped',
        'place_pose': 'geometry_msgs/PoseStamped',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.pick_pose = kwargs.get('pick_pose', PoseStamped())
        from geometry_msgs.msg import PoseStamped
        self.place_pose = kwargs.get('place_pose', PoseStamped())

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
        if self.pick_pose != other.pick_pose:
            return False
        if self.place_pose != other.place_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pick_pose(self):
        """Message field 'pick_pose'."""
        return self._pick_pose

    @pick_pose.setter
    def pick_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'pick_pose' field must be a sub message of type 'PoseStamped'"
        self._pick_pose = value

    @builtins.property
    def place_pose(self):
        """Message field 'place_pose'."""
        return self._place_pose

    @place_pose.setter
    def place_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'place_pose' field must be a sub message of type 'PoseStamped'"
        self._place_pose = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PickPlaceService_Response(type):
    """Metaclass of message 'PickPlaceService_Response'."""

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
                'planning_interfaces.srv.PickPlaceService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pick_place_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pick_place_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pick_place_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pick_place_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pick_place_service__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PickPlaceService_Response(metaclass=Metaclass_PickPlaceService_Response):
    """Message class 'PickPlaceService_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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


class Metaclass_PickPlaceService(type):
    """Metaclass of service 'PickPlaceService'."""

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
                'planning_interfaces.srv.PickPlaceService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__pick_place_service

            from planning_interfaces.srv import _pick_place_service
            if _pick_place_service.Metaclass_PickPlaceService_Request._TYPE_SUPPORT is None:
                _pick_place_service.Metaclass_PickPlaceService_Request.__import_type_support__()
            if _pick_place_service.Metaclass_PickPlaceService_Response._TYPE_SUPPORT is None:
                _pick_place_service.Metaclass_PickPlaceService_Response.__import_type_support__()


class PickPlaceService(metaclass=Metaclass_PickPlaceService):
    from planning_interfaces.srv._pick_place_service import PickPlaceService_Request as Request
    from planning_interfaces.srv._pick_place_service import PickPlaceService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
