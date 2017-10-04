#include <pluginlib/class_list_macros.h>
#include "test_base.h"
#include "test_plugins.h"

PLUGINLIB_EXPORT_CLASS(test_plugins::Foo, test_base::Fubar)
PLUGINLIB_EXPORT_CLASS(test_plugins::Bar, test_base::Fubar)
