#ifndef PLUGINLIB_EXCEPTIONS_H_DEFINED
#define PLUGINLIB_EXCEPTIONS_H_DEFINED

#include <stdexcept>

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

}

#endif
