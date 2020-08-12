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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using testing::_;

namespace cartographer {
namespace mapping {
namespace testing {

class MockMapBuilder : public mapping::MapBuilderInterface {
 public:
  MOCK_METHOD(
      int, AddTrajectoryBuilder,
      (const std::set<SensorId>& expected_sensor_ids,
       const mapping::proto::TrajectoryBuilderOptions& trajectory_options,
       mapping::MapBuilderInterface::LocalSlamResultCallback
           local_slam_result_callback),
      (override));
  MOCK_METHOD(int, AddTrajectoryForDeserialization,
              (const mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
                   options_with_sensor_ids_proto),
              (override));
  MOCK_METHOD(mapping::TrajectoryBuilderInterface*, GetTrajectoryBuilder,
              (int trajectory_id), (const, override));
  MOCK_METHOD(void, FinishTrajectory, (int trajectory_id), (override));
  MOCK_METHOD(string, SubmapToProto,
              (const mapping::SubmapId&,
               mapping::proto::SubmapQuery::Response*),
              (override));
  MOCK_METHOD(void, SerializeState, (bool, io::ProtoStreamWriterInterface*),
              (override));
  MOCK_METHOD(bool, SerializeStateToFile, (bool, const string&), (override));
  MOCK_METHOD((std::map<int, int>), LoadState,
              (io::ProtoStreamReaderInterface*, bool), (override));
  MOCK_METHOD((std::map<int, int>), LoadStateFromFile, (const string&, bool),
              (override));
  MOCK_METHOD(int, num_trajectory_builders, (), (const, override));
  MOCK_METHOD(mapping::PoseGraphInterface*, pose_graph, (), (override));
  MOCK_METHOD(const std::vector<
              mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&());
};

}  // namespace testing
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_TESTING_MOCK_MAP_BUILDER_H_
