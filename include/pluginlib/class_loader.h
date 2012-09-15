/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
#ifndef PLUGINLIB_CLASS_LOADER_H
#define PLUGINLIB_CLASS_LOADER_H

#include "ros/console.h"

#include "pluginlib/boost_fs_wrapper.h"
#include "pluginlib/class_desc.h"
#include "boost/algorithm/string.hpp"
#include "ros/package.h"
#include "tinyxml.h"
#include <vector>
#include <map>
#include <class_loader.h> //mas - From plugins

namespace pluginlib
{
  /**
   * @class PluginlibException
   * @brief A base class for all pluginlib exceptions that inherits from std::runtime_exception
   */
  class PluginlibException: public std::runtime_error
  {
    public:
      PluginlibException(const std::string error_desc) : std::runtime_error(error_desc) {}
  };

  /**
   * @class LibraryLoadException
   * @brief An exception class thrown when pluginlib is unable to load the library associated with a given plugin
   */
  class LibraryLoadException: public PluginlibException
  {
    public:
      LibraryLoadException(const std::string error_desc) : PluginlibException(error_desc) {}
  };

  /**
   * @class LibraryUnloadException
   * @brief An exception class thrown when pluginlib is unable to unload the library associated with a given plugin
   */
  class LibraryUnloadException: public PluginlibException
  {
    public:
      LibraryUnloadException(const std::string error_desc) : PluginlibException(error_desc) {}
  };

  /**
   * @class CreateClassException
   * @brief An exception class thrown when pluginlib is unable to create the class associated with a given plugin
   */
  class CreateClassException: public PluginlibException
  {
    public:
      CreateClassException(const std::string error_desc) : PluginlibException(error_desc) {}
  };

  /**
   * Pure virtual base class of pluginlib::ClassLoader which is not
   * templated.  This allows the writing of non-templated manager code
   * which can call all the administrative functions of ClassLoaders -
   * everything except createClassInstance(), createInstance()
   * and createUnmanagedInstance().
   */
  class ClassLoaderBase
  {
    public:
      /**
       * @brief Empty virtual destructor
       */
      virtual ~ClassLoaderBase() {}

      /**
       * @brief  Returns a list of all available classes for this ClassLoader's base class type
       * @return A vector of strings corresponding to the names of all available classes
       */
      virtual std::vector<std::string> getDeclaredClasses() = 0;

      /**
       * @brief  Refreshs the list of all available classes for this ClassLoader's base class type
       * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
       */
      virtual void refreshDeclaredClasses() = 0;

      /**
       * @brief  Strips the package name off of a lookup name
       * @param lookup_name The name of the plugin
       * @return The name of the plugin stripped of the package name
       */
      virtual std::string getName(const std::string& lookup_name) = 0;

      /**
       * @brief  Checks if the class associated with a plugin name is available to be loaded
       * @param lookup_name The name of the plugin 
       * @return True if the plugin is available, false otherwise
       */
      virtual bool isClassAvailable(const std::string& lookup_name) = 0;

      /**
       * @brief  Given the lookup name of a class, returns the type of the derived class associated with it
       * @param lookup_name The name of the class 
       * @return The name of the associated derived class
       */
      virtual std::string getClassType(const std::string& lookup_name) = 0;

      /**
       * @brief  Given the lookup name of a class, returns its description
       * @param lookup_name The lookup name of the class 
       * @return The description of the class
       */
      virtual std::string getClassDescription(const std::string& lookup_name) = 0;

      /**
       * @brief  Given the lookup name of a class, returns the type of the associated base class
       * @return The type of the associated base class
       */
      virtual std::string getBaseClassType() const = 0;

      /**
       * @brief  Given the name of a class, returns name of the containing package
       * @param lookup_name The name of the class 
       * @return The name of the containing package
       */
      virtual std::string getClassPackage(const std::string& lookup_name) = 0;

      /**
       * @brief  Given the name of a class, returns the path of the associated plugin manifest
       * @param lookup_name The name of the class
       * @return The path of the associated plugin manifest
       */
      virtual std::string getPluginManifestPath(const std::string& lookup_name) = 0;

      /**
       * @brief Checks if a given class is currently loaded
       * @param  lookup_name The lookup name of the class to query
       * @return True if the class is loaded, false otherwise
       */
      virtual bool isClassLoaded(const std::string& lookup_name) = 0;

      /**
       * @brief  Attempts to load a class with a given name
       * @param lookup_name The lookup name of the class to load
       * @exception pluginlib::LibraryLoadException Thrown if the library for the class cannot be loaded
       */
      virtual void loadLibraryForClass(const std::string & lookup_name) = 0;

      /**
       * @brief  Attempts to unload a class with a given name
       * @param lookup_name The lookup name of the class to unload
       * @exception pluginlib::LibraryUnloadException Thrown if the library for the class cannot be unloaded
       * @return The number of pending unloads until the library is removed from memory
       */
      virtual int unloadLibraryForClass(const std::string& lookup_name) = 0;

      /**
       * @brief  Returns the libraries that are registered and can be loaded
       * @return A vector of strings corresponding to the names of registered libraries
       */
      virtual std::vector<std::string> getRegisteredLibraries() = 0;

      /**
       * @brief  Given the name of a class, returns the path to its associated library
       * @param lookup_name The name of the class 
       * @return The path to the associated library
       */
      virtual std::string getClassLibraryPath(const std::string& lookup_name) = 0;
  };

  /**
   * @class ClassLoader
   * @brief A class to help manage and load classes
   */
  template <class T>
    class ClassLoader: public ClassLoaderBase
    {
      private:
        typedef std::map<std::string, unsigned int> LibraryCountMap;

      public:
        typedef typename std::map<std::string, ClassDesc>::iterator ClassMapIterator;

      public:
        /**
         * @brief  Constructor for a ClassLoader
         * @param package The package containing the base class
         * @param base_class The type of the base class for classes to be loaded
         * @param attrib_name The attribute to search for in manifext.xml files, defaults to "plugin"
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         */
        ClassLoader(std::string package, std::string base_class, std::string attrib_name = std::string("plugin"));

        /**
         * @brief  Destructor for ClassLoader 
         */
        ~ClassLoader();

        /**
         * @brief  Returns a list of all available classes for this ClassLoader's base class type
         * @return A vector of strings corresponding to the names of all available classes
         */
        std::vector<std::string> getDeclaredClasses();

        /**
         * @brief  Refreshs the list of all available classes for this ClassLoader's base class type
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         */
        void refreshDeclaredClasses();

        /**
         * @brief  Strips the package name off of a lookup name
         * @param lookup_name The name of the plugin
         * @return The name of the plugin stripped of the package name
         */
        std::string getName(const std::string& lookup_name);

        /**
         * @brief  Checks if the class associated with a plugin name is available to be loaded
         * @param lookup_name The name of the plugin 
         * @return True if the plugin is available, false otherwise
         */
        bool isClassAvailable(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns the type of the derived class associated with it
         * @param lookup_name The name of the class 
         * @return The name of the associated derived class
         */
        std::string getClassType(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns its description
         * @param lookup_name The lookup name of the class 
         * @return The description of the class
         */
        std::string getClassDescription(const std::string& lookup_name);

        /**
         * @brief  Given the lookup name of a class, returns the type of the associated base class
         * @return The type of the associated base class
         */
        std::string getBaseClassType() const;

        /**
         * @brief  Given the name of a class, returns name of the containing package
         * @param lookup_name The name of the class 
         * @return The name of the containing package
         */
        std::string getClassPackage(const std::string& lookup_name);

        /**
         * @brief  Given the name of a class, returns the path of the associated plugin manifest
         * @param lookup_name The name of the class
         * @return The path of the associated plugin manifest
         */
        std::string getPluginManifestPath(const std::string& lookup_name);

        /**
         * @brief  Creates an instance of a desired class, optionally loading the associated library automatically if necessary
         * @param  lookup_name The name of the class to load
         * @param  auto_load Specifies whether or not to automatically load the library containing the class, set to true by default
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         * @deprecated use either createInstance() or createUnmanagedInstance().
         */
        __attribute__((deprecated)) T* createClassInstance(const std::string& lookup_name, bool auto_load = true);
        
        /**
         * @brief  Creates an instance of a desired class (which implicitly calls loadLibraryForClass() to increment the library counter). Deleting the instance and calling unloadLibraryForClass() is automatically handled by the shared pointer.
         * @param  lookup_name The name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         */
        boost::shared_ptr<T> createInstance(const std::string& lookup_name);
        
        /**
         * @brief  Creates an instance of a desired class (which implicitly calls loadLibraryForClass() to increment the library counter).
         * @attention The ownership is transfered to the caller, which is responsible for deleting the instance and calling unloadLibraryForClass() (in order to decrement the associated library counter and unloading it if it is no more used).
         * @param  lookup_name The name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown when the library associated with the class cannot be loaded
         * @exception pluginlib::CreateClassException Thrown when the class cannot be instantiated
         * @return An instance of the class
         */
        T* createUnmanagedInstance(const std::string& lookup_name);

        /**
         * @brief Checks if a given class is currently loaded
         * @param  lookup_name The lookup name of the class to query
         * @return True if the class is loaded, false otherwise
         */
        bool isClassLoaded(const std::string& lookup_name);

        /**
         * @brief  Attempts to load the library containing a class with a given name and increments a counter for the library
         * @param lookup_name The lookup name of the class to load
         * @exception pluginlib::LibraryLoadException Thrown if the library for the class cannot be loaded
         */
        void loadLibraryForClass(const std::string & lookup_name);

        /**
         * @brief  Decrements the counter for the library containing a class with a given name and attempts to unload it if the counter reaches zero
         * @param lookup_name The lookup name of the class to unload
         * @exception pluginlib::LibraryUnloadException Thrown if the library for the class cannot be unloaded
         * @return The number of pending unloads until the library is removed from memory
         */
        int unloadLibraryForClass(const std::string& lookup_name);

        /**
         * @brief  Returns the libraries that are registered and can be loaded
         * @return A vector of strings corresponding to the names of registered libraries
         */
        std::vector<std::string> getRegisteredLibraries();

        /**
         * @brief  Given the name of a class, returns the path to its associated library
         * @param lookup_name The name of the class 
         * @return The path to the associated library
         */
        std::string getClassLibraryPath(const std::string& lookup_name);

      private:
        /**
          * Deleter for boost shared pointer.
          * @param p The instance
          * @param lookup_name The name of the class
          */
        void garbageInstance(T* p, const std::string& lookup_name);

        /**
         * @brief  Unloads a previously dynamically loaded lobrary
         * @param library_path The library to unload
         * @return True if the library was successfully unloaded, false otherwise
         */
        bool unloadClassLibrary(const std::string& library_path);

        /**
         * @brief  Dynamicaly loads a library
         * @param library_path The library to unload
         * @return True if the library was successfully loaded, false otherwise
         */
        bool loadClassLibrary(const std::string& library_path);

        /**
         * @brief  Returns the names of the classes that are available in a given library
         * @param  library_path The path to the library
         * @return A vector of strings corresponding to the names of the classes in the library
         */
        std::vector<std::string> getClassesInLibrary(const std::string & library_path);

        /**
         * @brief  Returns the libraries that are currently loaded
         * @return A vector of strings corresponding to the names of loaded libraries
         */
        std::vector<std::string> getLoadedLibraries();

        /**
         * @brief  Helper function for loading a shared library
         * @param  library_path The path to the library to load
         * @param  list_name The name of the class list to load
         */
        void loadClassLibraryInternal(const std::string& library_path, const std::string& list_name_arg = std::string(""));

        /**
         * @brief  Helper function for unloading a shared library
         * @param  library_path The path to the library to unload
         * @return The number of pending unloads until the library is removed from memory
         */
        int unloadClassLibraryInternal(const std::string& library_path);

        /**
         * @brief  Returns the available classes
         * @exception pluginlib::LibraryLoadException Thrown if package manifest cannot be found
         * @return A map of class names and the corresponding descriptions
         */
        std::map<std::string, ClassDesc> determineAvailableClasses();

        /**
         * @brief  Returns an error message for an unknown class
         * @param lookup_name The name of the class 
         * @return The error message
         */
        std::string getErrorStringForUnknownClass(const std::string& lookup_name);

        //used for proper unloading of automatically loaded libraries
        LibraryCountMap loaded_libraries_;

        // map from library to class's descriptions  
        // This is all available classes found in xml
        std::map<std::string, ClassDesc> classes_available_;
        std::string package_;
        std::string base_class_;
        std::string attrib_name_;

        plugins::ClassLoader plugins_class_loader_;  
    };

};

#include "class_loader_imp.h"

#endif //PLUGINLIB_CLASS_LOADER_H
