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

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>  // NOLINT

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

// Writes a plugin description XML next to a package.xml for 'test_pluginlib'
// in a fresh temporary directory, so that libraries are looked up in the
// mock install prefix of 'test_pluginlib'. Returns the path to the plugin XML.
static std::string writePluginXml(const std::string & test_name, const std::string & content)
{
  const std::filesystem::path dir =
    std::filesystem::temp_directory_path() / ("pluginlib_utest_" + test_name);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "package.xml") <<
    "<package format=\"2\"><name>test_pluginlib</name></package>\n";
  const std::filesystem::path xml_path = dir / "plugins.xml";
  std::ofstream(xml_path) << content;
  return xml_path.string();
}

TEST(PluginlibTest, libraryWithoutPathIsSkipped) {
  // A <library> without a usable 'path' attribute must be skipped, and the
  // remaining libraries must still be parsed (this used to loop forever).
  const std::string xml_path = writePluginXml(
    "libraryWithoutPathIsSkipped",
    "<class_libraries>\n"
    "  <library>\n"
    "    <class name=\"test_pluginlib/no_path\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>\n"
    "  </library>\n"
    "  <library path=\"\">\n"
    "    <class name=\"test_pluginlib/empty_path\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>\n"
    "  </library>\n"
    "  <library path=\"test_plugins\">\n"
    "    <class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>\n"
    "  </library>\n"
    "</class_libraries>\n");

  pluginlib::ClassLoader<test_base::Fubar> loader(
    "test_pluginlib", "test_base::Fubar", "plugin", {xml_path});

  EXPECT_FALSE(loader.isClassAvailable("test_pluginlib/no_path"));
  EXPECT_FALSE(loader.isClassAvailable("test_pluginlib/empty_path"));
  ASSERT_TRUE(loader.isClassAvailable("test_pluginlib/foo"));

  std::shared_ptr<test_base::Fubar> foo = loader.createSharedInstance("test_pluginlib/foo");
  foo->initialize(10.0);
  EXPECT_EQ(100.0, foo->result());
}

TEST(PluginlibTest, libraryPathWithDirectories) {
  // The library is found by its file name when the 'path' attribute also
  // contains directories that do not exist in the search paths.
  const std::string xml_path = writePluginXml(
    "libraryPathWithDirectories",
    "<library path=\"some/dir/test_plugins\">\n"
    "  <class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\""
    " base_class_type=\"test_base::Fubar\"/>\n"
    "</library>\n");

  pluginlib::ClassLoader<test_base::Fubar> loader(
    "test_pluginlib", "test_base::Fubar", "plugin", {xml_path});

  std::shared_ptr<test_base::Fubar> foo = loader.createSharedInstance("test_pluginlib/foo");
  foo->initialize(10.0);
  EXPECT_EQ(100.0, foo->result());
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
