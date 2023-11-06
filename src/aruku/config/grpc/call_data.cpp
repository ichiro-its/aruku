// Copyright (c) 2023 Ichiro ITS
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

#include "aruku/config/grpc/call_data.hpp"

namespace aruku
{
template <class ConfigRequest, class ConfigReply>
CallData<ConfigRequest, ConfigReply>::CallData(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: status_(CREATE), service_(service), cq_(cq), responder_(&ctx_), path_(path)
{
}

template <class ConfigRequest, class ConfigReply>
void CallData<ConfigRequest, ConfigReply>::Proceed()
{
  if (status_ == CREATE) {
    status_ = PROCESS;
    WaitForRequest();
  } else if (status_ == PROCESS) {
    AddNextToCompletionQueue();
    HandleRequest();
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
}

template class CallData<aruku_interfaces::proto::SetWalking, aruku_interfaces::proto::Empty>;
template class CallData<aruku_interfaces::proto::ConfigWalking, aruku_interfaces::proto::Empty>;
template class CallData<aruku_interfaces::proto::Empty, aruku_interfaces::proto::ConfigWalking>;

}  // namespace aruku