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

#ifndef TEST_BASE_HPP_
#define TEST_BASE_HPP_

#include <memory>

#include <visibility_control.hpp>
#include <class_loader/interface_traits.hpp>

namespace test_base
{
class TEST_PLUGINLIB_FIXTURE_PUBLIC Fubar
{
public:
  virtual void initialize(double foo) = 0;
  virtual double result() = 0;
  virtual ~Fubar() {}

protected:
  Fubar() {}
};

class TEST_PLUGINLIB_FIXTURE_PUBLIC FubarWithCtor
{
public:
  virtual double result() = 0;
  virtual ~FubarWithCtor() = default;

protected:
  FubarWithCtor() = default;
};
}  // namespace test_base

template<>
struct class_loader::InterfaceTraits<test_base::FubarWithCtor>
{
  // Using `std::unique_ptr<double>` to test forwarding of move-only types.
  using constructor_parameters = class_loader::ConstructorParameters<std::unique_ptr<double>>;
};

static_assert(
  class_loader::is_interface_constructible_v<test_base::FubarWithCtor, std::unique_ptr<double>>,
  "BaseWithInterfaceCtor should be interface constructible with the specifed types."
);

static_assert(
  class_loader::is_interface_constructible_v<test_base::FubarWithCtor, std::unique_ptr<double>&&>,
  "BaseWithInterfaceCtor should be interface constructible with the specifed types."
);

#endif  // TEST_BASE_HPP_
