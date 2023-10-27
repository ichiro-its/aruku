#include "aruku/config/grpc/call_data.hpp"

namespace aruku {
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

} // namespace aruku