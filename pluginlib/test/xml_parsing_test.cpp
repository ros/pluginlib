// Copyright 2026 Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the copyright holder nor the names of its
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
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>  // NOLINT

#include <test_base.hpp>

namespace
{

using Loader = pluginlib::ClassLoader<test_base::Fubar>;

const char kFooClass[] =
  "<class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\""
  " base_class_type=\"test_base::Fubar\">"
  "<description>Foo</description></class>";

class XmlParsingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = std::filesystem::temp_directory_path() /
      (std::string("pluginlib_xml_parsing_test_") + info->name());
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(dir_);
  }

  // Writes 'content' to 'name' inside the test directory and returns its path.
  std::string write(const std::string & name, const std::string & content)
  {
    const std::filesystem::path path = dir_ / name;
    std::ofstream(path) << content;
    return path.string();
  }

  // Writes a package.xml for 'test_pluginlib' and a plugin XML with 'content'.
  std::string writePluginXml(const std::string & content)
  {
    write("package.xml", "<package format=\"2\"><name>test_pluginlib</name></package>");
    return write("plugins.xml", content);
  }

  Loader makeLoader(const std::string & xml_path)
  {
    return Loader("test_pluginlib", "test_base::Fubar", "plugin", {xml_path});
  }

  std::filesystem::path dir_;
};

TEST_F(XmlParsingTest, missingFileIsSkipped) {
  Loader loader = makeLoader((dir_ / "does_not_exist.xml").string());
  EXPECT_TRUE(loader.getDeclaredClasses().empty());
}

TEST_F(XmlParsingTest, malformedFileIsSkipped) {
  Loader loader = makeLoader(writePluginXml("this is not xml <"));
  EXPECT_TRUE(loader.getDeclaredClasses().empty());
}

TEST_F(XmlParsingTest, unexpectedRootTagIsSkipped) {
  Loader loader = makeLoader(
    writePluginXml(std::string("<plugins><library path=\"test_plugins\">") +
    kFooClass + "</library></plugins>"));
  EXPECT_TRUE(loader.getDeclaredClasses().empty());
}

TEST_F(XmlParsingTest, invalidFileDoesNotHideOtherFiles) {
  const std::string valid = writePluginXml(
    std::string("<library path=\"test_plugins\">") + kFooClass + "</library>");
  const std::string invalid = write("invalid.xml", "<plugins/>");

  Loader loader("test_pluginlib", "test_base::Fubar", "plugin", {invalid, valid});
  EXPECT_EQ(std::vector<std::string>{"test_pluginlib/foo"}, loader.getDeclaredClasses());
}

TEST_F(XmlParsingTest, classLibrariesRootWithSeveralLibraries) {
  Loader loader = makeLoader(
    writePluginXml(
      std::string("<class_libraries>") +
      "<library path=\"test_plugins\">" + kFooClass + "</library>"
      "<library path=\"other_plugins\">"
      "<class name=\"test_pluginlib/bar\" type=\"test_plugins::Bar\""
      " base_class_type=\"test_base::Fubar\"/>"
      "</library>"
      "</class_libraries>"));

  EXPECT_TRUE(loader.isClassAvailable("test_pluginlib/foo"));
  EXPECT_TRUE(loader.isClassAvailable("test_pluginlib/bar"));
  EXPECT_EQ("test_plugins::Bar", loader.getClassType("test_pluginlib/bar"));
}

TEST_F(XmlParsingTest, classWithoutNameUsesType) {
  Loader loader = makeLoader(
    writePluginXml(
      "<library path=\"test_plugins\">"
      "<class type=\"test_plugins::Foo\" base_class_type=\"test_base::Fubar\"/>"
      "</library>"));

  EXPECT_TRUE(loader.isClassAvailable("test_plugins::Foo"));
  EXPECT_EQ("test_plugins::Foo", loader.getClassType("test_plugins::Foo"));
}

TEST_F(XmlParsingTest, descriptionIsOptional) {
  Loader loader = makeLoader(
    writePluginXml(
      "<library path=\"test_plugins\">"
      "<class name=\"test_pluginlib/no_description\" type=\"test_plugins::Foo\""
      " base_class_type=\"test_base::Fubar\"/>"
      "<class name=\"test_pluginlib/empty_description\" type=\"test_plugins::Foo\""
      " base_class_type=\"test_base::Fubar\"><description/></class>"
      "</library>"));

  EXPECT_EQ(
    "No 'description' tag for this plugin in plugin description file.",
    loader.getClassDescription("test_pluginlib/no_description"));
  EXPECT_EQ("", loader.getClassDescription("test_pluginlib/empty_description"));
}

TEST_F(XmlParsingTest, classesOfOtherBaseTypesAreIgnored) {
  Loader loader = makeLoader(
    writePluginXml(
      std::string("<library path=\"test_plugins\">") + kFooClass +
      "<class name=\"test_pluginlib/other\" type=\"test_plugins::Other\""
      " base_class_type=\"test_base::Other\"/>"
      "</library>"));

  EXPECT_TRUE(loader.isClassAvailable("test_pluginlib/foo"));
  EXPECT_FALSE(loader.isClassAvailable("test_pluginlib/other"));
}

TEST_F(XmlParsingTest, classWithoutTypeThrows) {
  const std::string xml = writePluginXml(
    "<library path=\"test_plugins\">"
    "<class name=\"test_pluginlib/foo\" base_class_type=\"test_base::Fubar\"/>"
    "</library>");
  EXPECT_THROW(makeLoader(xml), pluginlib::ClassLoaderException);
}

TEST_F(XmlParsingTest, classWithoutBaseClassTypeThrows) {
  const std::string xml = writePluginXml(
    "<library path=\"test_plugins\">"
    "<class name=\"test_pluginlib/foo\" type=\"test_plugins::Foo\"/>"
    "</library>");
  EXPECT_THROW(makeLoader(xml), pluginlib::ClassLoaderException);
}

TEST_F(XmlParsingTest, packageNameIsReadFromNearestPackageXml) {
  const std::string xml = writePluginXml(
    std::string("<library path=\"test_plugins\">") + kFooClass + "</library>");
  Loader loader = makeLoader(xml);

  EXPECT_EQ("test_pluginlib", loader.getClassPackage("test_pluginlib/foo"));
  EXPECT_EQ(xml, loader.getPluginManifestPath("test_pluginlib/foo"));
}

TEST_F(XmlParsingTest, packageNameIsEmptyWithoutPackageXml) {
  // No package.xml next to the plugin XML nor in any parent directory.
  Loader loader = makeLoader(
    write("plugins.xml", std::string("<library path=\"test_plugins\">") + kFooClass +
    "</library>"));

  ASSERT_TRUE(loader.isClassAvailable("test_pluginlib/foo"));
  EXPECT_EQ("", loader.getClassPackage("test_pluginlib/foo"));
}

TEST_F(XmlParsingTest, packageNameIsEmptyForInvalidPackageXml) {
  const std::string plugin_xml =
    std::string("<library path=\"test_plugins\">") + kFooClass + "</library>";
  const std::vector<std::string> invalid_package_xmls = {
    "<not_a_package/>",
    "<package format=\"2\"><version>1.0.0</version></package>",
    "<package format=\"2\"><name/></package>",
  };

  for (const auto & package_xml : invalid_package_xmls) {
    SCOPED_TRACE(package_xml);
    write("package.xml", package_xml);
    Loader loader = makeLoader(write("plugins.xml", plugin_xml));

    ASSERT_TRUE(loader.isClassAvailable("test_pluginlib/foo"));
    EXPECT_EQ("", loader.getClassPackage("test_pluginlib/foo"));
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
