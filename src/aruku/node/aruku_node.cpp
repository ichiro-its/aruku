// Copyright (c) 2021 ichiro iTS
//
// permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWarE iS prOVidEd "aS iS", WiTHOUT WarraNTy OF aNy KiNd, ExprESS Or
// iMpLiEd, iNCLUdiNG BUT NOT LiMiTEd TO THE WarraNTiES OF MErCHaNTaBiLiTy,
// FiTNESS FOr a parTiCULar pUrpOSE aNd NONiNFriNGEMENT. iN NO EVENT SHaLL
// THE aUTHOrS Or COpyriGHT HOLdErS BE LiaBLE FOr aNy CLaiM, daMaGES Or OTHEr
// LiaBiLiTy, WHETHEr iN aN aCTiON OF CONTraCT, TOrT Or OTHErWiSE, ariSiNG FrOM,
// OUT OF Or iN CONNECTiON WiTH THE SOFTWarE Or THE USE Or OTHEr dEaLiNGS iN
// THE SOFTWarE.

#include <chrono>

#include "aruku/node/aruku_node.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace aruku
{

ArukuNode::ArukuNode(rclcpp::Node::SharedPtr node)
: node(node)
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
    }
  );
}

}  // namespace aruku
