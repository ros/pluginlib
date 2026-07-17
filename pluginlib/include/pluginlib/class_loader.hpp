// Copyright 2009, Willow Garage, Inc. All rights reserved.
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

#ifndef PLUGINLIB__CLASS_LOADER_HPP_
#define PLUGINLIB__CLASS_LOADER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "class_loader/interface_traits.hpp"
#include "class_loader/multi_library_class_loader.hpp"
#include "pluginlib/class_loader_impl.hpp"
#include "pluginlib/exceptions.hpp"

namespace pluginlib
{

template<typename T>
using UniquePtr = class_loader::ClassLoader::UniquePtr<T>;

/// Satisfied when interface T can be constructed from Args according to its InterfaceTraits.
/**
 * Replaces the std::enable_if_t<class_loader::is_interface_constructible_v<...>> SFINAE guard
 * on the create*Instance() methods with a named constraint, yielding clearer diagnostics.
 */
template<typename T, typename ... Args>
concept InterfaceConstructible = class_loader::is_interface_constructible_v<T, Args...>;

/// A class to help manage and load classes.
/**
 * Everything that does not depend on the plugin base class T lives in ClassLoaderImpl,
 * which is compiled once into libpluginlib. Only the operations below genuinely need T,
 * so only they are instantiated per plugin base class.
 */
template<class T>
class ClassLoader : public ClassLoaderImpl
{
public:
  /**
   * \param package The package containing the base class
   * \param base_class The type of the base class for classes to be loaded
   * \param attrib_name The attribute to search for in manifest.xml files, defaults to "plugin"
   * \param plugin_xml_paths The list of paths of plugin.xml files, defaults to be crawled via
   *   ros::package::getPlugins()
   * \throws pluginlib::ClassLoaderException if package manifest cannot be found
   */
  ClassLoader(
    std::string package,
    std::string base_class,
    std::string attrib_name = std::string("plugin"),
    std::vector<std::string> plugin_xml_paths = std::vector<std::string>())
  : ClassLoaderImpl(
      std::move(package), std::move(base_class), std::move(attrib_name),
      std::move(plugin_xml_paths))
  {
  }

  /// Create an instance of a desired class.
  /**
   * Implicitly calls loadLibraryForClass() to increment the library counter.
   *
   * Deleting the instance and calling unloadLibraryForClass() is automatically
   * handled by the shared pointer.
   * \param lookup_name The name of the class to load
   * \param args Arguments passed to the constructor of the plugin.
   *   Plugin parameters must be declared using class_loader::InterfaceTraits.
   * \throws pluginlib::LibraryLoadException when the library associated with
   *   the class cannot be loaded
   * \throws pluginlib::CreateClassException when the class cannot be instantiated
   * \return An instance of the class
   */
  template<typename ... Args>
  requires InterfaceConstructible<T, Args...>
  std::shared_ptr<T> createSharedInstance(const std::string & lookup_name, Args && ... args);

  /// Create an instance of a desired class.
  /**
   * Implicitly calls loadLibraryForClass() to increment the library counter.
   *
   * Deleting the instance and calling unloadLibraryForClass() is automatically
   * handled by the unique pointer.
   *
   * If you release the wrapped pointer you must manually call the original
   * deleter when you want to destroy the released pointer.
   *
   * \param lookup_name The name of the class to load.
   * \param args Arguments passed to the constructor of the plugin.
   *   Plugin parameters must be declared using class_loader::InterfaceTraits.
   * \throws pluginlib::LibraryLoadException when the library associated with
   *   the class cannot be loaded.
   * \throws pluginlib::CreateClassException when the class cannot be instantiated
   * \return An instance of the class
   */
  template<typename ... Args>
  requires InterfaceConstructible<T, Args...>
  UniquePtr<T> createUniqueInstance(const std::string & lookup_name, Args && ... args);

  /// Create an instance of a desired class.
  /**
   * Implicitly calls loadLibraryForClass() to increment the library counter.
   *
   * \attention The ownership is transferred to the caller, which is responsible
   *   for deleting the instance and calling unloadLibraryForClass()
   *   (in order to decrement the associated library counter and unloading it
   *   if it is no more used).
   * \param lookup_name The name of the class to load
   * \param args Arguments passed to the constructor of the plugin.
   *   Plugin parameters must be declared using class_loader::InterfaceTraits.
   * \throws pluginlib::LibraryLoadException when the library associated with
   *   the class cannot be loaded
   * \throws pluginlib::CreateClassException when the class cannot be instantiated
   * \return An instance of the class
   */
  template<typename ... Args>
  requires InterfaceConstructible<T, Args...>
  T * createUnmanagedInstance(const std::string & lookup_name, Args && ... args);

  /// Check if the library for a given class is currently loaded.
  /**
   * \param lookup_name The lookup name of the class to query
   * \return True if the class is loaded, false otherwise
   */
  bool isClassLoaded(const std::string & lookup_name) override;
};

}  // namespace pluginlib

// Note: The implementation of the methods is in a separate file for clarity.
#include "./class_loader_imp.hpp"

#endif  // PLUGINLIB__CLASS_LOADER_HPP_
