// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
// using namespace std;
using namespace std::chrono_literals;

class dead_time_manager
{
public:
  dead_time_manager(const std::chrono::milliseconds dead_time);
  bool is_in_dead_time(bool &changed);
  void start();

private:
  bool in_dead_time;
  std::chrono::time_point<std::chrono::system_clock> start_time;
  const std::chrono::milliseconds dead_time;
};
