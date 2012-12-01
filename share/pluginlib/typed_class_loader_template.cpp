#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <vector>
#include <string>
#include <iostream>

@

extern "C"
{
  std::vector<std::string> availablePlugins(const std::string& package_name)
  {
    pluginlib::ClassLoader<$> class_loader(package_name, "$");
    return(class_loader.getDeclaredClasses());    
  }

  bool loadPlugin(const std::string& package_name, const std::string& class_name)
  {
    pluginlib::ClassLoader<$> class_loader(package_name, "$");
    try
    {
      class_loader.createInstance(class_name);      
      return true;
    }
    catch(...)
    {
      return false;
    }
  }

  std::string whereIsPluginLocated(const std::string& package_name, const std::string& class_name)
  {
    pluginlib::ClassLoader<$> class_loader(package_name, "$");
    try
    {
      return class_loader.getClassLibraryPath(class_name);
    }
    catch(...)
    {
      return ("Could not find location of plugin " + class_name);
    }
  }
}
