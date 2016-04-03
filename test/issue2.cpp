#include <pluginlib/class_loader.h>
#include <console_bridge/console.h>
#include "test_base.h"

boost::shared_ptr<test_base::Fubar> test() {
  // declaring the Loader static runs into issue during program shutdown
  // the order of releasing static variables is undefined
  // during unloading of the lib, class_loader access already freed mutexes
  static pluginlib::ClassLoader<test_base::Fubar> loader("pluginlib", "test_base::Fubar");

  boost::shared_ptr<test_base::Fubar> foo = loader.createInstance("pluginlib/foo");
  foo->initialize(10.0);
  return foo;
}

int main(int argc, char **argv)
{
  // another bug in rosconsole_bridge interferes: disable rosconsole_bridge
  console_bridge::restorePreviousOutputHandler();

  boost::shared_ptr<test_base::Fubar> foo = test();
  foo.reset();
  return 0;
}
