// Copyright (c) 2024 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef ARUKU__CONFIG__GRPC__CALL_DATA_GET_CONFIG_HPP__
#define ARUKU__CONFIG__GRPC__CALL_DATA_GET_CONFIG_HPP__

#include "aruku/config/grpc/call_data.hpp"

namespace aruku
{
class CallDataGetConfig
  : CallData<aruku_interfaces::proto::Empty, aruku_interfaces::proto::ConfigWalking>
  {
  public:
    CallDataGetConfig(
      aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string & path);

  protected:
    void AddNextToCompletionQueue() override;
    void WaitForRequest() ;
    void HandleRequest() ;
  };
} // namespace aruku

#endif // ARUKU__CONFIG__GRPC__CALL_DATA_GET_CONFIG_HPP__
