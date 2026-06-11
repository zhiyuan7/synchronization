#pragma once

#include "sync/coreAlgoBase.hpp"

// ---------------------------------------------------------------------------
// TimeDomainCalc：基于互相关峰值搜索的时域核心算法
// ---------------------------------------------------------------------------
class TimeDomainCalc : public CoreAlgoBase {
public:
    TimeDomainCalc() = default;

    CoreResult compute(
        std::vector<double>& xn,
        std::vector<double>& yn,
        int    N,
        double resample_hz,
        double max_offset_ms,
        bool   detrend
    ) override;
};
