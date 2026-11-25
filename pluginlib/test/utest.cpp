// Copyright (c) 2012, Willow Garage, Inc. All rights reserved.
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

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>  // NOLINT

#include "rcpputils/shared_library.hpp"  // NOLINT
#include "rcutils/env.h"  // NOLINT
#include "rcutils/logging_macros.h"  // NOLINT

#include <test_base.hpp>

TEST(PluginlibTest, unknownPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("test_pluginlib", "test_base::Fubar");
  ASSERT_THROW(
    test_loader.createSharedInstance("test_pluginlib/foobar"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, misspelledPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> bad_test_loader("test_pluginlib", "test_base::Fuba");
  ASSERT_THROW(
    bad_test_loader.createSharedInstance("test_pluginlib/foo"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, invalidPackage) {
  ASSERT_THROW(pluginlib::ClassLoader<test_base::Fubar>("test_pluginlib_bad",
    "test_base::Fubar"),
    pluginlib::ClassLoaderException);
}

TEST(PluginlibTest, brokenPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("test_pluginlib", "test_base::Fubar");
  ASSERT_THROW(
    test_loader.createSharedInstance("test_pluginlib/none"), pluginlib::PluginlibException);
}

TEST(PluginlibTest, workingPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("test_pluginlib", "test_base::Fubar");

  try {
    std::shared_ptr<test_base::Fubar> foo = test_loader.createSharedInstance("test_pluginlib/foo");
    foo->initialize(10.0);
    EXPECT_EQ(100.0, foo->result());
  } catch (pluginlib::PluginlibException & ex) {
    FAIL() << "Throwing exception: " << ex.what();
    return;
  } catch (...) {
    FAIL() << "Uncaught exception";
  }
}

TEST(PluginlibTest, workingPluginCtor) {
  pluginlib::ClassLoader<test_base::FubarWithCtor> test_loader("test_pluginlib",
    "test_base::FubarWithCtor");

  try {
    std::shared_ptr<test_base::FubarWithCtor> foo =
      test_loader.createSharedInstance("test_pluginlib/foo_with_ctor",
      std::make_unique<double>(10.0));
    EXPECT_DOUBLE_EQ(100.0, foo->result());
  } catch (pluginlib::PluginlibException & ex) {
    FAIL() << "Throwing exception: " << ex.what();
    return;
  } catch (...) {
    FAIL() << "Uncaught exception";
  }
}

TEST(PluginlibTest, createUnmanagedInstanceAndUnloadLibrary) {
  RCUTILS_LOG_INFO("Making the ClassLoader...");
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");

  RCUTILS_LOG_INFO("Instantiating plugin...");
  test_base::Fubar * inst = pl.createUnmanagedInstance("test_pluginlib/foo");

  RCUTILS_LOG_INFO("Deleting plugin...");
  delete inst;

  RCUTILS_LOG_INFO("Checking if plugin is loaded with isClassLoaded...");
  if (pl.isClassLoaded("test_pluginlib/foo")) {
    RCUTILS_LOG_INFO("Class is loaded");
  } else {
    FAIL() << "Library containing class should be loaded but isn't.";
  }
  RCUTILS_LOG_INFO("Trying to unload class with unloadLibraryForClass...");
  try {
    pl.unloadLibraryForClass("test_pluginlib/foo");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Could not unload library when I should be able to. " << e.what();
  }
  RCUTILS_LOG_INFO("Done.");
}

TEST(PluginlibTest, createUnmanagedInstanceCtorAndUnloadLibrary) {
  RCUTILS_LOG_INFO("Making the ClassLoader...");
  pluginlib::ClassLoader<test_base::FubarWithCtor> pl("test_pluginlib", "test_base::FubarWithCtor");

  RCUTILS_LOG_INFO("Instantiating plugin...");
  test_base::FubarWithCtor * inst = pl.createUnmanagedInstance("test_pluginlib/foo_with_ctor",
    std::make_unique<double>(10.0));
  EXPECT_DOUBLE_EQ(100.0, inst->result());

  RCUTILS_LOG_INFO("Deleting plugin...");
  delete inst;

  RCUTILS_LOG_INFO("Checking if plugin is loaded with isClassLoaded...");
  if (pl.isClassLoaded("test_pluginlib/foo_with_ctor")) {
    RCUTILS_LOG_INFO("Class is loaded");
  } else {
    FAIL() << "Library containing class should be loaded but isn't.";
  }
  RCUTILS_LOG_INFO("Trying to unload class with unloadLibraryForClass...");
  try {
    pl.unloadLibraryForClass("test_pluginlib/foo_with_ctor");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Could not unload library when I should be able to. " << e.what();
  }
  RCUTILS_LOG_INFO("Done.");
}

TEST(PluginlibTest, createManagedInstanceAndUnloadLibrary) {
  RCUTILS_LOG_INFO("Making the ClassLoader...");
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");

  RCUTILS_LOG_INFO("Instantiating plugin...");
  {
    std::shared_ptr<test_base::Fubar> inst = pl.createSharedInstance("test_pluginlib/foo");
  }

  RCUTILS_LOG_INFO("Checking if plugin is loaded with isClassLoaded...");
  if (pl.isClassLoaded("test_pluginlib/foo")) {
    RCUTILS_LOG_INFO("Class is loaded");
  } else {
    FAIL() << "Library containing class should be loaded but isn't.";
  }

  RCUTILS_LOG_INFO("Trying to unload class with unloadLibraryForClass...");
  try {
    pl.unloadLibraryForClass("test_pluginlib/foo");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Could not unload library when I should be able to." << e.what();
  }
  RCUTILS_LOG_INFO("Done.");
}

TEST(PluginlibTest, brokenXML) {
  try {
    pluginlib::ClassLoader<test_base::Fubar> test_loader("test_pluginlib", "test_base::Fubar",
      "plugin_test");
    test_loader.createSharedInstance("test_pluginlib/foo");
  } catch (pluginlib::PluginlibException & /*ex*/) {
    SUCCEED();
    return;
  }

  ADD_FAILURE() << "Didn't throw exception as expected";
}

TEST(PluginlibTest, Splitting) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  EXPECT_EQ(pl.getName("foo::bar"), "bar");
  EXPECT_EQ(pl.getName("foo/bar"), "bar");
  EXPECT_EQ(pl.getName("foo/bar:baz"), "baz");
  EXPECT_EQ(pl.getName("foo:bar/baz"), "baz");
  EXPECT_EQ(pl.getName("foo"), "foo");
  EXPECT_EQ(pl.getName(""), "");
}

TEST(PluginlibTest, classInformation) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");

  EXPECT_EQ("test_base::Fubar", pl.getBaseClassType());

  // Only the classes deriving from test_base::Fubar are declared.
  std::vector<std::string> declared = pl.getDeclaredClasses();
  std::sort(declared.begin(), declared.end());
  EXPECT_EQ(
    (std::vector<std::string>{
    "test_pluginlib/bar", "test_pluginlib/foo", "test_pluginlib/none"}),
    declared);

  EXPECT_TRUE(pl.isClassAvailable("test_pluginlib/foo"));
  EXPECT_FALSE(pl.isClassAvailable("test_pluginlib/foo_with_ctor"));
  EXPECT_EQ("test_plugins::Foo", pl.getClassType("test_pluginlib/foo"));
  EXPECT_EQ("This is a foo plugin.", pl.getClassDescription("test_pluginlib/foo"));
  EXPECT_EQ("test_pluginlib", pl.getClassPackage("test_pluginlib/foo"));

  const std::vector<std::string> xml_paths = pl.getPluginXmlPaths();
  ASSERT_EQ(1u, xml_paths.size());
  EXPECT_EQ("test_plugins.xml", std::filesystem::path(xml_paths[0]).filename());
  EXPECT_EQ(xml_paths[0], pl.getPluginManifestPath("test_pluginlib/foo"));

  const std::string library_path = pl.getClassLibraryPath("test_pluginlib/foo");
  EXPECT_TRUE(std::filesystem::exists(library_path)) << library_path;
}

TEST(PluginlibTest, unknownClassInformation) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  const std::string unknown = "test_pluginlib/unknown";

  EXPECT_FALSE(pl.isClassAvailable(unknown));
  EXPECT_FALSE(pl.isClassLoaded(unknown));
  EXPECT_EQ("", pl.getClassType(unknown));
  EXPECT_EQ("", pl.getClassDescription(unknown));
  EXPECT_EQ("", pl.getClassPackage(unknown));
  EXPECT_EQ("", pl.getPluginManifestPath(unknown));
  EXPECT_THROW(pl.getClassLibraryPath(unknown), pluginlib::LibraryLoadException);
  EXPECT_THROW(pl.loadLibraryForClass(unknown), pluginlib::LibraryLoadException);
  EXPECT_THROW(pl.unloadLibraryForClass(unknown), pluginlib::LibraryUnloadException);
}

TEST(PluginlibTest, unloadClassNotLoaded) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  EXPECT_THROW(
    pl.unloadLibraryForClass("test_pluginlib/foo"), pluginlib::LibraryUnloadException);
}

TEST(PluginlibTest, registeredLibraries) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  EXPECT_TRUE(pl.getRegisteredLibraries().empty());

  pl.loadLibraryForClass("test_pluginlib/foo");
  EXPECT_TRUE(pl.isClassLoaded("test_pluginlib/foo"));
  EXPECT_EQ(
    std::vector<std::string>{pl.getClassLibraryPath("test_pluginlib/foo")},
    pl.getRegisteredLibraries());

  pl.unloadLibraryForClass("test_pluginlib/foo");
  EXPECT_TRUE(pl.getRegisteredLibraries().empty());
}

TEST(PluginlibTest, classMissingFromLibrary) {
  // The library of 'test_pluginlib/none' loads, but it does not export the class.
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  EXPECT_THROW(
    pl.createUnmanagedInstance("test_pluginlib/none"), pluginlib::CreateClassException);
}

TEST(PluginlibTest, refreshDeclaredClasses) {
  pluginlib::ClassLoader<test_base::Fubar> pl("test_pluginlib", "test_base::Fubar");
  std::vector<std::string> declared = pl.getDeclaredClasses();

  // Refreshing without any loaded library keeps the same classes.
  pl.refreshDeclaredClasses();
  EXPECT_EQ(declared, pl.getDeclaredClasses());

  // Refreshing with a loaded library keeps the classes usable.
  std::shared_ptr<test_base::Fubar> foo = pl.createSharedInstance("test_pluginlib/foo");
  pl.refreshDeclaredClasses();
  EXPECT_EQ(declared, pl.getDeclaredClasses());
  ASSERT_TRUE(pl.isClassLoaded("test_pluginlib/foo"));
  foo = pl.createSharedInstance("test_pluginlib/foo");
  foo->initialize(10.0);
  EXPECT_EQ(100.0, foo->result());
}

// Writes 'content' to 'path', creating the parent directories.
static void writeFile(const std::filesystem::path & path, const std::string & content)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream(path) << content;
}

// Writes a plugin description XML, next to a package.xml for 'package_name',
// in a fresh temporary directory. Returns the path to the plugin XML.
static std::string writePluginXml(
  const std::string & test_name, const std::string & package_name, const std::string & content)
{
  const std::filesystem::path dir =
    std::filesystem::temp_directory_path() / ("pluginlib_utest_" + test_name);
  std::filesystem::remove_all(dir);
  writeFile(dir / "package.xml",
    "<package format=\"2\"><name>" + package_name + "</name></package>");
  writeFile(dir / "plugins.xml", content);
  return (dir / "plugins.xml").string();
}

TEST(PluginlibTest, libraryNameWithLibPrefix) {
  // 'libtest_plugins' is found as 'test_plugins' on every platform.
  const std::string xml_path = writePluginXml(
    "libraryNameWithLibPrefix", "test_pluginlib",
    "<library path=\"libtest_plugins\">"
    "<class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>"
    "</library>");
  pluginlib::ClassLoader<test_base::Fubar> pl(
    "test_pluginlib", "test_base::Fubar", "plugin", {xml_path});

  std::shared_ptr<test_base::Fubar> foo = pl.createSharedInstance("test_pluginlib/foo");
  foo->initialize(10.0);
  EXPECT_EQ(100.0, foo->result());
}

TEST(PluginlibTest, libraryNotFound) {
  const std::string xml_path = writePluginXml(
    "libraryNotFound", "test_pluginlib",
    "<library path=\"does_not_exist\">"
    "<class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>"
    "</library>");
  pluginlib::ClassLoader<test_base::Fubar> pl(
    "test_pluginlib", "test_base::Fubar", "plugin", {xml_path});

  EXPECT_THROW(pl.getClassLibraryPath("test_pluginlib/foo"), pluginlib::LibraryLoadException);
  EXPECT_THROW(pl.createSharedInstance("test_pluginlib/foo"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, libraryFailsToLoad) {
  // Install a package 'broken_pkg' whose plugin library is not a valid shared library.
  const std::filesystem::path prefix =
    std::filesystem::temp_directory_path() / "pluginlib_utest_libraryFailsToLoad_prefix";
  std::filesystem::remove_all(prefix);
  writeFile(prefix / "share" / "ament_index" / "resource_index" / "packages" / "broken_pkg", "");
  const std::string library_name = rcpputils::get_platform_library_name("broken_plugins");
  writeFile(prefix / "lib" / library_name, "not a shared library");
  writeFile(prefix / "bin" / library_name, "not a shared library");

  const char * ament_prefix_path = nullptr;
  ASSERT_EQ(nullptr, rcutils_get_env("AMENT_PREFIX_PATH", &ament_prefix_path));
  const std::string original_ament_prefix_path = ament_prefix_path;
  const char separator = std::filesystem::path::preferred_separator == '\\' ? ';' : ':';
  const std::string new_ament_prefix_path =
    prefix.string() + separator + original_ament_prefix_path;
  ASSERT_TRUE(rcutils_set_env("AMENT_PREFIX_PATH", new_ament_prefix_path.c_str()));

  const std::string xml_path = writePluginXml(
    "libraryFailsToLoad", "broken_pkg",
    "<library path=\"broken_plugins\">"
    "<class name=\"broken_pkg/foo\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>"
    "</library>");
  {
    pluginlib::ClassLoader<test_base::Fubar> pl(
      "test_pluginlib", "test_base::Fubar", "plugin", {xml_path});
    try {
      pl.createSharedInstance("broken_pkg/foo");
      ADD_FAILURE() << "Expected pluginlib::LibraryLoadException";
    } catch (const pluginlib::LibraryLoadException & ex) {
      EXPECT_NE(std::string::npos, std::string(ex.what()).find("Failed to load library"))
        << ex.what();
    }
  }

  EXPECT_TRUE(rcutils_set_env("AMENT_PREFIX_PATH", original_ament_prefix_path.c_str()));
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
