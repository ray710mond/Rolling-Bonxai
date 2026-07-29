#include <gtest/gtest.h>

#include "bonxai_ros/fusion_policy.hpp"

namespace
{

constexpr uint64_t kSecond = 1000000000ULL;
constexpr uint64_t kTolerance = 5U * kSecond;

TEST(FusionPolicy, OccupiedWinsInsideToleranceInEitherOrder)
{
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(
      true, 10U * kSecond, true, 15U * kSecond, kTolerance),
    Bonxai::FusedVoxelState::Occupied);
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(
      true, 15U * kSecond, true, 10U * kSecond, kTolerance),
    Bonxai::FusedVoxelState::Occupied);
}

TEST(FusionPolicy, NewerFreeWinsOutsideTolerance)
{
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(
      true, 10U * kSecond, true, 15U * kSecond + 1U, kTolerance),
    Bonxai::FusedVoxelState::Free);
}

TEST(FusionPolicy, NewerOccupiedWinsOutsideTolerance)
{
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(
      true, 15U * kSecond + 1U, true, 10U * kSecond, kTolerance),
    Bonxai::FusedVoxelState::Occupied);
}

TEST(FusionPolicy, ASingleObservationRetainsItsLocalState)
{
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(true, 1U, false, 0U, kTolerance),
    Bonxai::FusedVoxelState::Occupied);
  EXPECT_EQ(
    Bonxai::resolve_fused_voxel_state(false, 0U, true, 1U, kTolerance),
    Bonxai::FusedVoxelState::Free);
}

}  // namespace
