// Copyright 2008, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef PLUGINLIB__CLASS_LOADER_IMP_HPP_
#define PLUGINLIB__CLASS_LOADER_IMP_HPP_

// Only the ClassLoader<T> members that genuinely depend on the plugin base class T are
// defined here; they must stay in a header because T is chosen by the consumer.
// Everything else is compiled once into libpluginlib via ClassLoaderImpl.

#include <memory>
#include <string>
#include <utility>

#include "rcutils/logging_macros.h"

#include "./class_loader.hpp"

namespace pluginlib
{

template<class T>
template<typename ... Args>
requires InterfaceConstructible<T, Args...>
std::shared_ptr<T> ClassLoader<T>::createSharedInstance(
  const std::string & lookup_name,
  Args &&... args)
{
  return createUniqueInstance(lookup_name, std::forward<Args>(args)...);
}

template<class T>
template<typename ... Args>
requires InterfaceConstructible<T, Args...>
UniquePtr<T> ClassLoader<T>::createUniqueInstance(const std::string & lookup_name, Args &&... args)
{
  RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
    "Attempting to create managed (unique) instance for class %s.",
    lookup_name.c_str());

  if (!isClassLoaded(lookup_name)) {
    loadLibraryForClass(lookup_name);
  }

  try {
    std::string class_type = getClassType(lookup_name);
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader", "%s maps to real class type %s",
      lookup_name.c_str(), class_type.c_str());

    UniquePtr<T> obj = lowlevel_class_loader_.createUniqueInstance<T>(class_type,
        std::forward<Args>(args)...);

    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
      "std::unique_ptr to object of real type %s created.",
      class_type.c_str());

    return obj;
  } catch (const class_loader::CreateClassException & ex) {
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
      "Exception raised by low-level multi-library class loader when attempting "
      "to create instance of class %s.",
      lookup_name.c_str());
    throw pluginlib::CreateClassException(ex.what());
  }
}

template<class T>
template<typename ... Args>
requires InterfaceConstructible<T, Args...>
T * ClassLoader<T>::createUnmanagedInstance(const std::string & lookup_name, Args &&... args)
{
  RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
    "Attempting to create UNMANAGED instance for class %s.",
    lookup_name.c_str());

  if (!isClassLoaded(lookup_name)) {
    loadLibraryForClass(lookup_name);
  }

  T * instance = nullptr;
  try {
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
      "Attempting to create instance through low level multi-library class loader.");
    std::string class_type = getClassType(lookup_name);
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader", "%s maps to real class type %s",
      lookup_name.c_str(), class_type.c_str());
    instance = lowlevel_class_loader_.createUnmanagedInstance<T>(class_type,
        std::forward<Args>(args)...);
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
      "Instance of type %s created.",
      class_type.c_str());
  } catch (const class_loader::CreateClassException & ex) {
    RCUTILS_LOG_DEBUG_NAMED("pluginlib.ClassLoader",
      "Exception raised by low-level multi-library class loader when attempting "
      "to create UNMANAGED instance of class %s.",
      lookup_name.c_str());
    throw pluginlib::CreateClassException(ex.what());
  }
  return instance;
}

template<class T>
bool ClassLoader<T>::isClassLoaded(const std::string & lookup_name)
{
  return lowlevel_class_loader_.isClassAvailable<T>(getClassType(lookup_name));
}

}  // namespace pluginlib

#endif  // PLUGINLIB__CLASS_LOADER_IMP_HPP_
