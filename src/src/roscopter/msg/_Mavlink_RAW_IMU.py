# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from roscopter/Mavlink_RAW_IMU.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Mavlink_RAW_IMU(genpy.Message):
  _md5sum = "f00561d290c2da804223c20d12427677"
  _type = "roscopter/Mavlink_RAW_IMU"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
#time since boot
uint64 time_usec
#linear acc
int32 xacc
int32 yacc
int32 zacc
#gyro
int32 xgyro
int32 ygyro
int32 zgyro
#magnetic field
int32 xmag  
int32 ymag
int32 zmag

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','time_usec','xacc','yacc','zacc','xgyro','ygyro','zgyro','xmag','ymag','zmag']
  _slot_types = ['std_msgs/Header','uint64','int32','int32','int32','int32','int32','int32','int32','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,time_usec,xacc,yacc,zacc,xgyro,ygyro,zgyro,xmag,ymag,zmag

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Mavlink_RAW_IMU, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.time_usec is None:
        self.time_usec = 0
      if self.xacc is None:
        self.xacc = 0
      if self.yacc is None:
        self.yacc = 0
      if self.zacc is None:
        self.zacc = 0
      if self.xgyro is None:
        self.xgyro = 0
      if self.ygyro is None:
        self.ygyro = 0
      if self.zgyro is None:
        self.zgyro = 0
      if self.xmag is None:
        self.xmag = 0
      if self.ymag is None:
        self.ymag = 0
      if self.zmag is None:
        self.zmag = 0
    else:
      self.header = std_msgs.msg.Header()
      self.time_usec = 0
      self.xacc = 0
      self.yacc = 0
      self.zacc = 0
      self.xgyro = 0
      self.ygyro = 0
      self.zgyro = 0
      self.xmag = 0
      self.ymag = 0
      self.zmag = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Q9i().pack(_x.time_usec, _x.xacc, _x.yacc, _x.zacc, _x.xgyro, _x.ygyro, _x.zgyro, _x.xmag, _x.ymag, _x.zmag))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 44
      (_x.time_usec, _x.xacc, _x.yacc, _x.zacc, _x.xgyro, _x.ygyro, _x.zgyro, _x.xmag, _x.ymag, _x.zmag,) = _get_struct_Q9i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Q9i().pack(_x.time_usec, _x.xacc, _x.yacc, _x.zacc, _x.xgyro, _x.ygyro, _x.zgyro, _x.xmag, _x.ymag, _x.zmag))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 44
      (_x.time_usec, _x.xacc, _x.yacc, _x.zacc, _x.xgyro, _x.ygyro, _x.zgyro, _x.xmag, _x.ymag, _x.zmag,) = _get_struct_Q9i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_Q9i = None
def _get_struct_Q9i():
    global _struct_Q9i
    if _struct_Q9i is None:
        _struct_Q9i = struct.Struct("<Q9i")
    return _struct_Q9i