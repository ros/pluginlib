^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pluginlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.2 (2016-03-14)
-------------------
* update maintainer
* Merge pull request `#35 <https://github.com/ros/pluginlib/issues/35>`_ from jspricke/license_fix
  Remove Boost Software License from license tag
* Remove Boost Software License from license tag
  The Boost Software License was only in there for Poco, as can be seen in
  6e0659f. As Poco was removed in 44ab6fb and all remaining Files have a
  BSD header, let's remove the Boost tag as well, to be consistent.
* Merge pull request `#34 <https://github.com/ros/pluginlib/issues/34>`_ from ros/throw_exception_invalid_library
  Fix wrong package name test
* Throw an exception if ClassLoader can't be instantiated due to an invalid package name
* Merge pull request `#33 <https://github.com/ros/pluginlib/issues/33>`_ from clearpathrobotics/getName-split-fix
  Add ":" to split function within getName.
* Add ":" to split function within getName.
  getName split wasn't supporting `::` as the delimiter for the package name and
  the plugin name.
* Contributors: Esteve Fernandez, Jochen Sprickerhof, Mikael Arguedas, Mike O'Driscoll

1.10.1 (2014-12-23)
-------------------
* Remove GTEST_FOUND from CMakeLists.txt
* Check that GTest is installed before running tests.
  Fixes `#29 <https://github.com/ros/pluginlib/issues/29>`_
* Merge pull request `#28 <https://github.com/ros/pluginlib/issues/28>`_ from ros/local-script
  Make plugin_macro_update rosrunnable and remove it from global PATH
* Moved plugin_macro_update script to scripts directory. Made plugin_macro_update rosrunnable and removed it from global PATH
* Contributors: Esteve Fernandez

1.10.0 (2014-05-08 14:56)
-------------------------

1.9.25 (2014-05-08 20:37)
-------------------------
* Merge pull request `#26 <https://github.com/ros/pluginlib/issues/26>`_ from ros/cmake-modules-tinyxml
  Use cmake_modules to find TinyXML
* Use cmake_modules to find TinyXML
* Merge pull request `#25 <https://github.com/ros/pluginlib/issues/25>`_ from ros/check-release-libraries
  Check for release libraries in debug builds
* Check for release libraries in debug builds
* Merge pull request `#24 <https://github.com/ros/pluginlib/issues/24>`_ from ros/refresh_with_force_recrawl
  update refreshDeclaredClasses to force recrawl (fix `#23 <https://github.com/ros/pluginlib/issues/23>`_)
* update refreshDeclaredClasses to force recrawl (fix `#23 <https://github.com/ros/pluginlib/issues/23>`_)
* Contributors: Dirk Thomas, Esteve Fernandez

1.9.24 (2014-03-11)
-------------------
* Merge pull request `#22 <https://github.com/ros/pluginlib/issues/22>`_ from ros/remove_wrong_exception
  Remove invalid exception when no plugins are found
* remove invalid exception when no plugins are found
* Update maintainer field
* Contributors: Dirk Thomas, Esteve Fernandez

1.9.23 (2013-10-04)
-------------------
* Merge pull request `#21 <https://github.com/ros/pluginlib/issues/21>`_ from ros/expose_plugin_paths
  expose plugin paths in ClassLoader
* expose plugin paths in ClassLoader
* Contributors: Dirk Thomas, Mirza Shah

1.9.22 (2013-08-21)
-------------------
* Fixed use of __FILE_\_ macro in deprecation warning
* Merge branch 'groovy-devel' of https://github.com/ros/pluginlib into groovy-devel
* Merge branch 'groovy-devel' of https://github.com/ros/pluginlib into groovy-devel
* Merge branch 'groovy-devel' of https://github.com/ros/pluginlib into groovy-devel
* Added libdl to plugin_tool link args...temporary fix
* Contributors: Mirza Shah

1.9.21 (2013-07-14)
-------------------
* Merge pull request `#16 <https://github.com/ros/pluginlib/issues/16>`_ from danepowell/feature/deprecated-hints
  Looks good, thanks @danepowell
* Added file hint for deprecated warnings.
* check for CATKIN_ENABLE_TESTING
* remove mainpage.dox
* Contributors: Dane Powell, Dirk Thomas, Mirza Shah

1.9.20 (2013-04-18)
-------------------
* Added another unit test for managed instance case.
* Fixed a regression that broke unload call. Added a unit test for this case.
* Contributors: Mirza Shah

1.9.19 (2013-03-23)
-------------------
* Merge pull request `#13 <https://github.com/ros/pluginlib/issues/13>`_ from davetcoleman/groovy-devel
  Converted ROS_DEBUG and ROS_WARN calls
* Converted ROS_DEBUG and ROS_WARN calls to ROS_DEBUG_NAMED and ROS_WARN_NAMED calls
* Contributors: Dave Coleman, Mirza Shah

1.9.18 (2013-01-28)
-------------------
* Merge pull request `#11 <https://github.com/ros/pluginlib/issues/11>`_ from marioprats/groovy-devel
  Support for boost filesystem v2
* Support for boost filesystem v2
* Added more debug information
* Contributors: Mario Prats, Mirza Shah

1.9.17 (2012-12-27)
-------------------
* More useful debug messages
* More debug messages
* Fixed incorrect debug message in plugin description XML parsing
* Contributors: Mirza Shah

1.9.16 (2012-12-21)
-------------------
* Removed old file
* Annotated deprecation warning with more info
* Made python script global installable
* Added a script to recursively update deprecated pluginlib macro
* added missing license header
* modified dep type of catkin
* Contributors: Aaron Blasdel, Dirk Thomas, Mirza Shah

1.9.15 (2012-12-13 17:22)
-------------------------
* Updated registration macros to be easier and deprecated older ones. Also cleaned up code violating standard
* Added wg copyright notice
* Contributors: Mirza Shah

1.9.14 (2012-12-13 15:20)
-------------------------
* lookup name (i.e. magic name) is now optional. Further cleanup...alphabetized methods, broke up some.
* Contributors: Mirza Shah

1.9.13 (2012-12-11)
-------------------
* Merge branch 'master' of https://github.com/ros/pluginlib
* Made robust to plugin package having different name from the folder it came from.
* Contributors: Mirza Shah

1.9.12 (2012-12-06)
-------------------
* Cleaned up debug output a little more
* Contributors: Mirza Shah

1.9.11 (2012-11-26)
-------------------
* Fixed a regression that somehow got back in there that was causing a race condition in multithreaded code, this will fix gazebo issues
* Bug fixes
* Contributors: Mirza Shah, mirzashah

1.9.10 (2012-11-21)
-------------------
* Working on plugintool still, to the train
* Updated plugin_tool
* Create plugin_tool, still problems at runtime with it
* Contributors: Mirza Shah

1.9.9 (2012-11-16)
------------------
* Minor fix where library was being unloaded for old load/unload reference counting, not needed anymore as class_loader handles that
* Contributors: Mirza Shah

1.9.8 (2012-11-14)
------------------
* refactored to return reasonable library path before loading the library
* Merge branch 'master' of https://github.com/ros/pluginlib
* Updated registration macros to correct legacy PLUGINLIB_REGISTER_CLASS macro as well as cleaned up comments
* Contributors: Dirk Thomas, Mirza Shah

1.9.7 (2012-11-08)
------------------
* updated catkin_package(DEPENDS)
* add missing Boost_INCLUDE_DIRS
* Contributors: Dirk Thomas

1.9.6 (2012-11-07)
------------------
* Added more debug messages and fixed a bug where managed instances do not auto open library
* Contributors: Mirza Shah

1.9.5 (2012-11-06)
------------------
* Changed ROS_ERROR to ROS_DEBUG
* Contributors: Mirza Shah

1.9.4 (2012-11-05)
------------------
* Updated to 1.9.4
* Removed more cruft and made pluginlib header only
* Removed unnecessary boost_fs_wrapper target, pluginlib now purely header only
* Merge branch 'master' of https://github.com/ros/pluginlib
* Made error message more meaningful
* Contributors: Mirza Shah

1.9.3 (2012-10-31)
------------------
* Fix to check for package.xml and not just manifest.xml when trying to verify a package.
* Contributors: Mirza Shah

1.9.2 (2012-10-25)
------------------
* fixed deps for downstream packages
* Contributors: Dirk Thomas

1.9.1 (2012-10-24 22:02)
------------------------
* fix missing dep for downstream projects
* remove redundant deps
* Contributors: Dirk Thomas

1.9.0 (2012-10-24 18:31)
------------------------
* renamed test target
* remove obsolete files
* Updates before merging
* Fixed dependency in package.xml and minor touchups
* Broke up code into further files
* Catkinized pluginlib and completed integration more or less with class_loader. Heavy mods to pluginlib::ClassLoader to handle constraints of Catkin as well as delegate housekeeping to class_loader::ClassLoader
* Fixed some renamed identifiers from class_loader used in a unit test
* Updated to utilize newly renamed class_loader (formerly plugins) library with new file names, functions, identifiers, etc
* Removed explicit dependency that should have been automatically imported from dependent package in CMakeLists.txt
* Fixed unhandled exception to make all unit tests pass
* Removed mention of console bridge in CMakeLists.txt, plugins now probably exports
* Finished mods to utilize lower level plugins library. One test still failing, will get to that soon, but basics seem to be ok
* Modding pluginlib to use new plugins library. Not done, but just doing it tosync with my laptop
* Removed Poco and updated CMake and manifest files to depend on lower level plugins library
* Contributors: Dirk Thomas, Mirza Shah, mirzashah

1.8.6 (2012-10-09)
------------------
* boost is definitely a runtime dependency
* added missing boost include dirs
* updated cmake min version to 2.8.3
* Contributors: Dirk Thomas, Vincent Rabaud

1.8.5 (2012-10-01)
------------------
* add missing roslib dependency that happens in class_loader_imp.h
* Contributors: Vincent Rabaud

1.8.4 (2012-09-30)
------------------
* updated to latest catkin
* Added tag 1.8.3 for changeset 05b0ebc238e5
* Contributors: Dirk Thomas

1.8.3 (2012-09-07)
------------------
* added tinyxml to project depends
* Added tag 1.8.2 for changeset c837303582d9
* Contributors: Dirk Thomas

1.8.2 (2012-09-06)
------------------
* updated pkg-config in manifest.xml
* updated catkin variables
* Added tag 1.8.1 for changeset 63d020c13ad6
* Contributors: Dirk Thomas

1.8.1 (2012-09-04)
------------------
* Missing LIBRARIES and DEPENDS specifiers from CMakeLists.txt, now added.
* catkin-ized
* Added tag fuerte for changeset fe4c8afbef30
* Added tag pluginlib-1.8.0 for changeset 77d8131adf5f
* 1.8.0
* updated api doc for load/create/unload methods
* renamed new methods using shorter name for encouraged method
* added cmake macro for hiding plugin symbols and respective rosbuild export
* updated class loader according to updated REP 121
* add auto-unload for libraries using boost shared pointer
* Added tag unstable for changeset 48e1c97daa3c
* Added tag pluginlib-1.7.2 for changeset 639e6ef8b5ac
* pluginlib 1.7.2
* pluginlib: added a pure-virtual base class for ClassLoader called ClassLoaderBase, which is not templated.  Only one function of ClassLoader is actually templated.  This allows client code to not be templated where it doesn't need to be.
* Added tag unstable for changeset 454e3e9bf01c
* Added tag pluginlib-1.7.1 for changeset 97b41d64500a
* reving for release 1.7.1
* patch 4 for `#4887 <https://github.com/ros/pluginlib/issues/4887>`_
* Added tag unstable for changeset da6fa1d8a8e1
* Added tag pluginlib-1.7.0 for changeset bdaaeae92e5b
* ignore bin
* accepting patch from ticket `#4887 <https://github.com/ros/pluginlib/issues/4887>`_ REP 116 implementation
* switching to default branch for unstable development.  reving release number and adding hgignore rules
* Added tag electric for changeset 0b70a8a58d24
* Added tag pluginlib-1.6.0 for changeset 7b22046fb28c
* 1.6.0 marker
* Added tag electric for changeset 87e3b8f41cc2
* Added tag unstable for changeset 9e1101cc0af9
* Added tag pluginlib-1.5.1 for changeset 5e920e4f10de
* reving for release
* merge
* add explicit link against tinyxml, because users of our libraries will need to link against it
* Added tag unstable for changeset b92d43c9a7d1
* Added tag pluginlib-1.5.0 for changeset 225e7a4092eb
* link poco_lite with tinyxml
* remove namespace to be compatible with tinyxml sysdep
* removing back depend on common
* removing rosdep.yaml, rule is in ros/rosdep.yaml
* fixed tinyxml
* converting to unary stack (separated from common)
* applied patch from 4923, to support boost 1.46
* patch from Nick Butko osx compatability
* adding unittest melonee forgot to commit
* adding pluginlib tests
* patch for osx linking `#4094 <https://github.com/ros/pluginlib/issues/4094>`_
* Fixed exception comments
* Added Ubuntu platform tags to manifest
* Fixing bug where the incorrect library path was passed to dlopen from pluginlib... oops.
* fix in latest for `#4013 <https://github.com/ros/pluginlib/issues/4013>`_ to isolate boost filesystem calls into a library
* patch from Wim `#3346 <https://github.com/ros/pluginlib/issues/3346>`_ reviewed by Eitan and I
* Adding getName and isClassAvailable function calls to the class loader
* inlining to avoid multiple definitions
* macro deprecation
* adding warning about deprecated macro PLUGINLIB_REGISTER_CLASS
* pluginlib now takes pkg/type arguments, new macro PLUGINLIB_DECLARE_CLASS
* pluginlib now robust to malformed manifests
* Adding more descriptive error messages when libaries fail to load
* Remove use of deprecated rosbuild macros
* doc review completed http://www.ros.org/wiki/pluginlib/Reviews/2009-10-06_Doc_Review
* fixing documentation link
* fixing `#2894 <https://github.com/ros/pluginlib/issues/2894>`_
* Removing ROS_ERRORS in favor of adding information to the exceptions thrown
* migration part 1
* Contributors: Dave Hershberger, Dirk Thomas, Ken Conley, Mirza Shah, Tully Foote, eitan, gerkey, kwc, mwise, rusu, tfoote, vpradeep, wheeler
