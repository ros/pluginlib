/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include "./test_base.h"

TEST(PluginlibTest, unknownPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");
  ASSERT_THROW(test_loader.createInstance("pluginlib/foobar"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, misspelledPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> bad_test_loader("pluginlib", "test_base::Fuba");
  ASSERT_THROW(bad_test_loader.createInstance("pluginlib/foo"), pluginlib::LibraryLoadException);
}

TEST(PluginlibTest, invalidPackage) {
  ASSERT_THROW(pluginlib::ClassLoader<test_base::Fubar>("pluginlib_bad",
    "test_base::Fubar"),
    pluginlib::ClassLoaderException);
}

TEST(PluginlibTest, brokenPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");
  ASSERT_THROW(test_loader.createInstance("pluginlib/none"), pluginlib::PluginlibException);
}

TEST(PluginlibTest, workingPlugin) {
  pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar");

  try {
    boost::shared_ptr<test_base::Fubar> foo = test_loader.createInstance("pluginlib/foo");
    foo->initialize(10.0);
    EXPECT_EQ(100.0, foo->result());
  } catch (pluginlib::PluginlibException & ex) {
    FAIL() << "Throwing exception: " << ex.what();
    return;
  } catch (...) {
    FAIL() << "Uncaught exception";
  }
}

TEST(PluginlibTest, createUnmanagedInstanceAndUnloadLibrary) {
  ROS_INFO("Making the ClassLoader...");
  pluginlib::ClassLoader<test_base::Fubar> pl("pluginlib", "test_base::Fubar");

  ROS_INFO("Instantiating plugin...");
  test_base::Fubar * inst = pl.createUnmanagedInstance("pluginlib/foo");

  ROS_INFO("Deleting plugin...");
  delete inst;

  ROS_INFO("Checking if plugin is loaded with isClassLoaded...");
  if (pl.isClassLoaded("pluginlib/foo")) {
    ROS_INFO("Class is loaded");
  } else {
    FAIL() << "Library containing class should be loaded but isn't.";
  }
  ROS_INFO("Trying to unload class with unloadLibraryForClass...");
  try {
    pl.unloadLibraryForClass("pluginlib/foo");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Could not unload library when I should be able to.";
  }
  ROS_INFO("Done.");
}

TEST(PluginlibTest, createManagedInstanceAndUnloadLibrary) {
  ROS_INFO("Making the ClassLoader...");
  pluginlib::ClassLoader<test_base::Fubar> pl("pluginlib", "test_base::Fubar");

  ROS_INFO("Instantiating plugin...");
  {
    boost::shared_ptr<test_base::Fubar> inst = pl.createInstance("pluginlib/foo");
  }

  ROS_INFO("Checking if plugin is loaded with isClassLoaded...");
  if (pl.isClassLoaded("pluginlib/foo")) {
    ROS_INFO("Class is loaded");
  } else {
    FAIL() << "Library containing class should be loaded but isn't.";
  }

  ROS_INFO("Trying to unload class with unloadLibraryForClass...");
  try {
    pl.unloadLibraryForClass("pluginlib/foo");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Could not unload library when I should be able to.";
  }
  ROS_INFO("Done.");
}

TEST(PluginlibTest, brokenXML) {
  try {
    pluginlib::ClassLoader<test_base::Fubar> test_loader("pluginlib", "test_base::Fubar",
      "plugin_test");
    test_loader.createInstance("pluginlib/foo");
  } catch (pluginlib::PluginlibException & ex) {
    SUCCEED();
    return;
  }

  ADD_FAILURE() << "Didn't throw exception as expected";
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
