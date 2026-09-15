#include <gtest/gtest.h>
#include "bonxai_ros/bonxai_server.hpp"

namespace Bonxai {
struct RemoteEvidenceTestAccess {
  static RemoteSourceLayer & apply(BonxaiServer & server, const std::string & id,
    uint64_t stamp, uint8_t state) {
    auto & layer=server.remote_sources_[id];
    if(!layer.occupancy) server.reset_remote_source(layer,1);
    surf_multirobot_msgs::msg::VoxelDelta d;
    d.map_epoch=1;d.source_id=id;d.resolution=.05;d.header.frame_id="map";
    d.x={20};d.y={0};d.z={20};d.state={state};d.observation_time_ns={stamp};
    server.apply_voxel_delta(layer,d);return layer;
  }
  static std::pair<std::set<CoordT>,std::set<CoordT>> fused(BonxaiServer & server) {
    std::pair<std::set<CoordT>,std::set<CoordT>> states;
    server.get_fused_voxel_states(states.first,states.second,true,true);return states;
  }
};

TEST(RemoteEvidence, ExplicitFreeCreatesSpaceAndUnknownDoesNotClearOtherSources)
{
  using D=surf_multirobot_msgs::msg::VoxelDelta;
  rclcpp::init(0,nullptr);
  {
    BonxaiServer server(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("map_storage.save_on_shutdown",false)}));
    const CoordT c{20,0,20};
    auto & drone=RemoteEvidenceTestAccess::apply(server,"drone",10000000000ULL,D::STATE_FREE);
    EXPECT_TRUE(drone.occupancy->isFree(c));
    EXPECT_TRUE(drone.occupancy->isUnknown({21,0,20}));
    EXPECT_TRUE(drone.occupancy->isUnknown({20,1,20}));
    EXPECT_TRUE(drone.occupancy->isUnknown({20,0,21}));
    EXPECT_TRUE(RemoteEvidenceTestAccess::fused(server).second.count(c));
    RemoteEvidenceTestAccess::apply(server,"drone",20000000000ULL,D::STATE_UNKNOWN);
    EXPECT_TRUE(drone.occupancy->isUnknown(c));
    EXPECT_FALSE(RemoteEvidenceTestAccess::fused(server).second.count(c));
    // Delayed packets cannot undo the withdrawal.
    RemoteEvidenceTestAccess::apply(server,"drone",15000000000ULL,D::STATE_FREE);
    EXPECT_TRUE(drone.occupancy->isUnknown(c));
    RemoteEvidenceTestAccess::apply(server,"peer",10000000000ULL,D::STATE_OCCUPIED_STATIC);
    EXPECT_TRUE(RemoteEvidenceTestAccess::fused(server).first.count(c));
    EXPECT_FALSE(RemoteEvidenceTestAccess::fused(server).second.count(c));
    RemoteEvidenceTestAccess::apply(server,"drone",30000000000ULL,D::STATE_FREE);
    EXPECT_TRUE(RemoteEvidenceTestAccess::fused(server).second.count(c));
    // UNKNOWN must also remove legacy DELETE/free evidence from this source.
    RemoteEvidenceTestAccess::apply(server,"drone",40000000000ULL,D::STATE_DELETE);
    RemoteEvidenceTestAccess::apply(server,"drone",50000000000ULL,D::STATE_UNKNOWN);
    EXPECT_TRUE(RemoteEvidenceTestAccess::fused(server).first.count(c));
    EXPECT_FALSE(RemoteEvidenceTestAccess::fused(server).second.count(c));
  }
  rclcpp::shutdown();
}
}  // namespace Bonxai
