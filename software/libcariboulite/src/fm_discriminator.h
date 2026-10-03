#pragma once
// Complex current * conjugate(previous), with legacy float operation order.
// Caller owns previous-IQ history, scale and angle policy (approximate NBFM
// versus full atan2f WBFM). No normalization, state, allocation or reset here.
static inline void fm_conjugate_product(float i, float q, float pi, float pq,
                                        float* re, float* im)
{
    *re = i*pi + q*pq;
    *im = q*pi - i*pq;
}
