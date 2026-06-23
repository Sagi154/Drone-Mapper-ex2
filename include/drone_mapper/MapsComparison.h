#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class MapsComparison {
public:
    /// Compares origin and target maps; returns one score (0–100) per target.
    /// @param origin Hidden / reference map.
    /// @param targets Output maps to score against origin.
    /// @return Parallel scores aligned with targets.
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets);
};

} // namespace drone_mapper
