#ifndef __ARUKU__CONFIG__GRPC__CALL_DATA_HPP__
#define __ARUKU__CONFIG__GRPC__CALL_DATA_HPP__

#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "aruku/config/grpc/call_data_base.hpp"
#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"

namespace aruku
{
template <class ConfigRequest, class ConfigReply>
class CallData : public CallDataBase
{
public:
  CallData(
    aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string path);

  void Proceed();

protected:
  virtual void AddNextToCompletionQueue() = 0;

  enum CallStatus { CREATE, PROCESS, FINISH };

  CallStatus status_;  // The current serving state.

  aruku_interfaces::proto::Config::AsyncService * service_;

  const std::string path_;

  grpc::ServerCompletionQueue * cq_;
  grpc::ServerContext ctx_;
  ConfigRequest request_;
  ConfigReply reply_;
  grpc::ServerAsyncResponseWriter<ConfigReply> responder_;
};
}  // namespace aruku

#endif  // __ARUKU__CONFIG__GRPC__CALL_DATA_HPP__