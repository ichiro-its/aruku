// Copyright (c) 2021 Ichiro ITS
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

#ifndef ARUKU__GRPC__NODE__GRPC_NODE_HPP_
#define ARUKU__GRPC__NODE__GRPC_NODE_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "aruku.grpc.pb.h"
#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"

using ros2_ws::aruku::proto::Config;

namespace aruku {
class GrpcNode {
public:
  ~GrpcNode();
  void run(uint16_t port, const std::string path);
  static GrpcNode &getInstance();

private:
  GrpcNode *grpcnode_;

  ros2_ws::aruku::proto::Config::AsyncService service_;
  bool continues;

  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  std::unique_ptr<grpc::Server> server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<std::thread> grpcthread;
  static void signintHandler(int signum);
  void set(GrpcNode *grpcnod);
  GrpcNode *get();
};
class CallDataBase {
public:
  virtual void proceed() = 0;
  CallDataBase();

protected:
  virtual void waitForRequest() = 0;
  virtual void handleRequest() = 0;
};

template <class RequestType, class ReplyType> class CallDataT : CallDataBase {
protected:
  enum CallStatus { CREATE, PROCESS, FINISH };
  CallStatus status_;
  const std::string path_;
  grpc::ServerCompletionQueue *cq_;
  grpc::ServerContext ctx_;
  ros2_ws::aruku::proto::Config::AsyncService *service_;
  RequestType request_;
  grpc::ServerAsyncResponseWriter<ReplyType> responder_;
  ReplyType reply_;
  virtual void addNextToCompletionQueue() = 0;

public:
  CallDataT(ros2_ws::aruku::proto::Config::AsyncService *service,
            grpc::ServerCompletionQueue *cq, const std::string path);
  virtual void proceed() override;
};

class CallDataGetConfig : CallDataT<ros2_ws::aruku::proto::Empty,
                                    ros2_ws::aruku::proto::ConfigWalking> {
public:
  CallDataGetConfig(ros2_ws::aruku::proto::Config::AsyncService *service,
                    grpc::ServerCompletionQueue *cq, const std::string path);

protected:
  virtual void addNextToCompletionQueue() override;
  virtual void waitForRequest() override;
  virtual void handleRequest() override;
};

class CallDataSetConfig : CallDataT<ros2_ws::aruku::proto::ConfigWalking,
                                    ros2_ws::aruku::proto::Empty> {
public:
  CallDataSetConfig(ros2_ws::aruku::proto::Config::AsyncService *service,
                    grpc::ServerCompletionQueue *cq, const std::string path);

protected:
  virtual void addNextToCompletionQueue() override;
  virtual void waitForRequest() override;
  virtual void handleRequest() override;
};
} // namespace aruku

#endif