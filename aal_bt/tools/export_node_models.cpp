// Copyright 2026 KAS-lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <iostream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "aal_bt/ensure_configuration.hpp"

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<aal_bt::EnsureConfiguration>("EnsureConfiguration");
  std::cout << BT::writeTreeNodesModelXML(factory) << std::endl;
  return 0;
}
