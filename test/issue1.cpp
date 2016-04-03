#include <pluginlib/class_loader.h>
#include "test_base.h"

boost::shared_ptr<test_base::Fubar> test() {
  // destruction of loader automatically tries to unload the lib, which causes
  // a SEVERE WARNING in class_loader, because pointers from the lib are still in use

  pluginlib::ClassLoader<test_base::Fubar> loader("pluginlib", "test_base::Fubar");
  boost::shared_ptr<test_base::Fubar> foo = loader.createInstance("pluginlib/foo");
  foo->initialize(10.0);
  return foo;
}

int main(int argc, char **argv)
{
  boost::shared_ptr<test_base::Fubar> foo = test();
  foo.reset();
  return 0;
}
