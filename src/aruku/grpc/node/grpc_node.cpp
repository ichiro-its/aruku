#include "aruku/grpc/node/grpc_node.hpp"
#include "aruku/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include <string>

using grpc::ServerBuilder;
using namespace std::chrono_literals;
namespace aruku {
GrpcNode::~GrpcNode() {
  server_->Shutdown();
  cq_->Shutdown();
  // grpcthread->detach();
}
void GrpcNode::signintHandler(int signum) {
  static GrpcNode &w = GrpcNode::getInstance();
  w.get()->continues = false;
  std::cout << "exit" << std::endl;
  exit(1);
}
GrpcNode &GrpcNode::getInstance() {
  static GrpcNode *w = new GrpcNode();
  return *w;
}
void GrpcNode::set(GrpcNode *grpcnode) { grpcnode_ = grpcnode; }
GrpcNode *GrpcNode::get() { return grpcnode_; }
void GrpcNode::run(uint16_t port, const std::string path) {
  std::string server_address = absl::StrFormat("0.0.0.0:%d", port);
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);
  cq_ = builder.AddCompletionQueue();
  signal(SIGINT, GrpcNode::signintHandler);
  continues = true;
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on :" << port << std::endl;
  static GrpcNode &w = GrpcNode::getInstance();
  w.set(this);
  signal(SIGINT, GrpcNode::signintHandler);
  grpcthread = std::make_shared<std::thread>([&path, this]() {
    new CallData(&service_, cq_.get(), path);
    void *tag; // uniquely identifies a request.
    bool ok = true;
    while (ok && continues) {
      std::cout << "runned" << std::endl;
      if (this->cq_->Next(&tag, &ok) && ok)
        static_cast<CallData *>(tag)->proceed();
    }
  });
  grpcthread->join();

  // rclcpp::CallbackGroup::SharedPtr cg =
  // node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // timer_ = node->create_wall_timer(1ms, [this, &tag, &ok]() {
  // });
}
GrpcNode::CallData::CallData(
    ros2_ws::aruku::proto::Config::AsyncService *service,
    grpc::ServerCompletionQueue *cq, const std::string path)
    : service_(service), cq_(cq), responder_(&ctx_), status_(CREATE),
      path_(path) {
  proceed();
}
void GrpcNode::CallData::proceed() {
  if (status_ == CREATE) {
    status_ = PROCESS;
    service_->RequestGetConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
  } else if (status_ == PROCESS) {
    new CallData(service_, cq_, path_);

    // The actual processing.
    aruku::Config config(path_);
    std::string prefix("Hello ");
    reply_.set_json_kinematic(config.get_config("kinematic"));
    reply_.set_json_walking("{'id': 2   }");
    std::cout << "ok << s" << std::endl;

    // And we are done! Let the gRPC runtime know we've finished, using the
    // memory address of this instance as the uniquely identifying tag for
    // the event.
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
}
} // namespace aruku