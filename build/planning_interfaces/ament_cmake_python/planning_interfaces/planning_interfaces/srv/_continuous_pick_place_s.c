// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "planning_interfaces/srv/detail/continuous_pick_place__struct.h"
#include "planning_interfaces/srv/detail/continuous_pick_place__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool planning_interfaces__srv__continuous_pick_place__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("planning_interfaces.srv._continuous_pick_place.ContinuousPickPlace_Request", full_classname_dest, 74) == 0);
  }
  planning_interfaces__srv__ContinuousPickPlace_Request * ros_message = _ros_message;
  {  // pick_scan_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "pick_scan_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->pick_scan_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // place_scan_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "place_scan_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->place_scan_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pick_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "pick_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pick_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // retreat_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "retreat_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->retreat_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // place_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "place_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->place_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // place_rotation_y_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "place_rotation_y_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->place_rotation_y_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * planning_interfaces__srv__continuous_pick_place__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ContinuousPickPlace_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("planning_interfaces.srv._continuous_pick_place");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ContinuousPickPlace_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  planning_interfaces__srv__ContinuousPickPlace_Request * ros_message = (planning_interfaces__srv__ContinuousPickPlace_Request *)raw_ros_message;
  {  // pick_scan_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->pick_scan_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pick_scan_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // place_scan_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->place_scan_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "place_scan_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pick_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pick_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pick_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // retreat_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->retreat_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "retreat_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // place_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->place_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "place_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // place_rotation_y_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->place_rotation_y_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "place_rotation_y_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "planning_interfaces/srv/detail/continuous_pick_place__struct.h"
// already included above
// #include "planning_interfaces/srv/detail/continuous_pick_place__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool planning_interfaces__srv__continuous_pick_place__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[76];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("planning_interfaces.srv._continuous_pick_place.ContinuousPickPlace_Response", full_classname_dest, 75) == 0);
  }
  planning_interfaces__srv__ContinuousPickPlace_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // slides_picked
    PyObject * field = PyObject_GetAttrString(_pymsg, "slides_picked");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->slides_picked = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * planning_interfaces__srv__continuous_pick_place__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ContinuousPickPlace_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("planning_interfaces.srv._continuous_pick_place");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ContinuousPickPlace_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  planning_interfaces__srv__ContinuousPickPlace_Response * ros_message = (planning_interfaces__srv__ContinuousPickPlace_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slides_picked
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->slides_picked);
    {
      int rc = PyObject_SetAttrString(_pymessage, "slides_picked", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
