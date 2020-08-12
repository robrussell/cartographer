/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_

#include "cartographer/mapping/pose_graph_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace testing {

class MockPoseGraph : public mapping::PoseGraphInterface {
 public:
  MockPoseGraph() = default;
  ~MockPoseGraph() override = default;

  MOCK_METHOD(void, RunFinalOptimization, (), (override));
  MOCK_METHOD((mapping::MapById<mapping::SubmapId, SubmapData>),
              GetAllSubmapData, (), (const, override));
  MOCK_METHOD((mapping::MapById<mapping::SubmapId, SubmapPose>),
              GetAllSubmapPoses, (), (const, override));
  MOCK_METHOD(transform::Rigid3d, GetLocalToGlobalTransform, (int),
              (const, override));
  MOCK_METHOD((mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>),
              GetTrajectoryNodes, (), (const, override));
  MOCK_METHOD((mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>),
              GetTrajectoryNodePoses, (), (const, override));
  MOCK_METHOD((std::map<int, mapping::PoseGraphInterface::TrajectoryState>),
              GetTrajectoryStates, (), (const, override));
  MOCK_METHOD((std::map<string, transform::Rigid3d>), GetLandmarkPoses, (),
              (const, override));
  MOCK_METHOD(void, SetLandmarkPose,
              (const string&, const transform::Rigid3d&, const bool),
              (override));
  MOCK_METHOD(void, DeleteTrajectory, (int), (override));
  MOCK_METHOD(bool, IsTrajectoryFinished, (int), (const, override));
  MOCK_METHOD(bool, IsTrajectoryFrozen, (int), (const, override));
  MOCK_METHOD((std::map<int, mapping::PoseGraphInterface::TrajectoryData>),
              GetTrajectoryData, (), (const, override));
  MOCK_METHOD(std::vector<Constraint>, constraints, (), (const, override));
  MOCK_METHOD(mapping::proto::PoseGraph, ToProto, (bool), (const, override));
  MOCK_METHOD(void, SetGlobalSlamOptimizationCallback,
              (GlobalSlamOptimizationCallback callback), (override));
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_POSE_GRAPH_H_
