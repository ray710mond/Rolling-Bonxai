#pragma once

#include <cstdint>

namespace Bonxai
{

enum class FusedVoxelState
{
  Unknown,
  Free,
  Occupied
};

inline FusedVoxelState resolve_fused_voxel_state(
  bool has_occupied, uint64_t newest_occupied_ns,
  bool has_free, uint64_t newest_free_ns, uint64_t tolerance_ns)
{
  if (!has_occupied && !has_free) {
    return FusedVoxelState::Unknown;
  }
  if (!has_occupied) {
    return FusedVoxelState::Free;
  }
  if (!has_free) {
    return FusedVoxelState::Occupied;
  }
  const uint64_t difference = newest_occupied_ns >= newest_free_ns ?
    newest_occupied_ns - newest_free_ns :
    newest_free_ns - newest_occupied_ns;
  if (difference <= tolerance_ns || newest_occupied_ns > newest_free_ns) {
    return FusedVoxelState::Occupied;
  }
  return FusedVoxelState::Free;
}

}  // namespace Bonxai
