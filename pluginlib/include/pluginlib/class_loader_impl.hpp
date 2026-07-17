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

#ifndef PLUGINLIB__CLASS_LOADER_IMPL_HPP_
#define PLUGINLIB__CLASS_LOADER_IMPL_HPP_

#include <map>
#include <string>
#include <vector>

#include "class_loader/multi_library_class_loader.hpp"
#include "pluginlib/class_desc.hpp"
#include "pluginlib/class_loader_base.hpp"
#include "pluginlib/visibility_control.hpp"

namespace pluginlib
{

/// Non-templated implementation shared by every pluginlib::ClassLoader<T>.
/**
 * None of this class's state or behaviour depends on the plugin base class, so it is
 * compiled once into libpluginlib rather than re-instantiated for every T. Only the
 * handful of genuinely T-dependent operations (creating instances, and querying whether
 * a class of type T is loaded) remain in the ClassLoader<T> template.
 *
 * This class stays abstract: isClassLoaded() is T-dependent and is left for
 * ClassLoader<T> to implement.
 */
class PLUGINLIB_PUBLIC ClassLoaderImpl : public ClassLoaderBase
{
public:
  using ClassMapIterator = std::map<std::string, ClassDesc>::iterator;

  /**
   * \param package The package containing the base class
   * \param base_class The type of the base class for classes to be loaded
   * \param attrib_name The attribute to search for in manifest.xml files, defaults to "plugin"
   * \param plugin_xml_paths The list of paths of plugin.xml files, defaults to be crawled
   * \throws pluginlib::ClassLoaderException if package manifest cannot be found
   */
  ClassLoaderImpl(
    std::string package,
    std::string base_class,
    std::string attrib_name = std::string("plugin"),
    std::vector<std::string> plugin_xml_paths = std::vector<std::string>());

  ~ClassLoaderImpl() override;

  std::vector<std::string> getPluginXmlPaths() override;

  std::vector<std::string> getDeclaredClasses() override;

  std::string getName(const std::string & lookup_name) override;

  std::string getBaseClassType() const override;

  std::string getClassType(const std::string & lookup_name) override;

  std::string getClassDescription(const std::string & lookup_name) override;

  std::string getClassLibraryPath(const std::string & lookup_name) override;

  std::string getClassPackage(const std::string & lookup_name) override;

  std::string getPluginManifestPath(const std::string & lookup_name) override;

  std::vector<std::string> getRegisteredLibraries() override;

  bool isClassAvailable(const std::string & lookup_name) override;

  void loadLibraryForClass(const std::string & lookup_name) override;

  void refreshDeclaredClasses() override;

  int unloadLibraryForClass(const std::string & lookup_name) override;

protected:
  /// Return an error message for an unknown class.
  std::string getErrorStringForUnknownClass(const std::string & lookup_name);

  std::vector<std::string> plugin_xml_paths_;
  // Map from library to class's descriptions described in XML.
  std::map<std::string, ClassDesc> classes_available_;
  std::string package_;
  std::string base_class_;
  std::string attrib_name_;
  class_loader::MultiLibraryClassLoader lowlevel_class_loader_;  // The underlying classloader

private:
  /// Return the paths to plugin.xml files.
  std::vector<std::string> getPluginXmlPaths(
    const std::string & package,
    const std::string & attrib_name);

  /// Return the available classes.
  std::map<std::string, ClassDesc> determineAvailableClasses(
    const std::vector<std::string> & plugin_xml_paths);

  /// Open a package.xml file and extract the package name.
  std::string extractPackageNameFromPackageXML(const std::string & package_xml_path);

  /// Get a list of paths to try to find a library.
  std::vector<std::string> getAllLibraryPathsToTry(
    const std::string & library_name,
    const std::string & exporting_package_name);

  /// Get the standard path separator for the native OS.
  std::string getPathSeparator();

  /// Get the package name from a path to a plugin XML file.
  std::string getPackageFromPluginXMLFilePath(const std::string & path);

  /// Join two filesystem paths together utilizing appropriate path separator.
  std::string joinPaths(const std::string & path1, const std::string & path2);

  /// Parse a plugin XML file.
  void processSingleXMLPluginFile(
    const std::string & xml_file,
    std::map<std::string, ClassDesc> & class_available);

  /// Strip all but the filename from an explicit file path.
  std::string stripAllButFileFromPath(const std::string & path);

  /// Helper function for unloading a shared library.
  int unloadClassLibraryInternal(const std::string & library_path);
};

}  // namespace pluginlib

#endif  // PLUGINLIB__CLASS_LOADER_IMPL_HPP_
