#pragma once

#define ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(msg, new_name, alloc) \
  template <class Allocator>                                     \
  struct msg##_;                                                 \
  typedef msg##_<alloc<void> > new_name;                         \
  typedef std::shared_ptr<new_name> new_name##Ptr;               \
  typedef std::shared_ptr<new_name const> new_name##ConstPtr;

#define ROS_DECLARE_MESSAGE(msg) ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(msg, msg, std::allocator)
