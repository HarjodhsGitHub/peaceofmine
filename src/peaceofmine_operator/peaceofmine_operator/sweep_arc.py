"""Locate a broad peak relative to the shoulders and measured noise of one sweep."""
import math
from statistics import median


def sweep_arc(samples, polarity=1):
    if type(polarity) is not int or polarity not in (-1, 1):
        raise ValueError('Detector response direction must be positive or negative')
    if len(samples) < 9:
        raise ValueError('Not enough readings to recognise a sweep arc')
    bins = {}
    raw = []
    for sample in samples:
        angle, voltage = (sample[k] for k in ('angle_deg', 'volts'))
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (angle, voltage)):
            raise ValueError('Invalid reading in sweep arc')
        bins.setdefault(round(angle), []).append((angle, polarity*voltage))
        raw.append(polarity*voltage)
    curve = [(median(v[0] for v in values), median(v[1] for v in values))
             for _, values in sorted(bins.items())]
    if len(curve) < 7:
        raise ValueError('Sweep did not cover enough angles to recognise an arc')
    x, values = zip(*curve)
    max_gap = 2.5*median(b-a for a,b in zip(x,x[1:]))
    # Symmetric angle smoothing avoids the time filter's direction-dependent lag.
    y = [median(values[max(0, i-1):i+2]) for i in range(len(values))]
    differences = [b-a for a, b in zip(raw, raw[1:])]
    center = median(differences)
    noise = median(abs(d-center) for d in differences)/(.6744897501960817*math.sqrt(2))
    candidates = []
    i = 1
    while i < len(y)-1:
        end = i
        while end+1 < len(y) and y[end+1] == y[i]:
            end += 1
        if y[i] > y[i-1] and end < len(y)-1 and y[end] > y[end+1]:
            peak = (i+end)//2
            left = right = peak
            for j in range(peak-1, -1, -1):
                if y[j] > y[peak]: break
                if y[j] < y[left]: left = j
            for j in range(peak+1, len(y)):
                if y[j] > y[peak]: break
                if y[j] < y[right]: right = j
            baseline = max(y[left], y[right])
            prominence = y[peak]-baseline
            half = y[peak]-prominence/2
            lo = hi = peak
            while lo > left and y[lo] > half: lo -= 1
            while hi < right and y[hi] > half: hi += 1
            if lo < peak < hi and prominence > 0 and prominence >= 3*noise:
                left_cross = x[lo]+(half-y[lo])*(x[lo+1]-x[lo])/(y[lo+1]-y[lo])
                right_cross = x[hi-1]+(half-y[hi-1])*(x[hi]-x[hi-1])/(y[hi]-y[hi-1])
                width = right_cross-left_cross
                if width >= 3 and all(b-a <= max_gap for a,b in zip(x[lo:hi],x[lo+1:hi+1])):
                    weights = [(x[j], y[j]-half) for j in range(lo+1,hi) if y[j] > half]
                    angle = sum(a*w for a,w in weights)/sum(w for _,w in weights)
                    candidates.append(dict(angle_deg=round(angle, 4), prominence_v=prominence,
                                           noise_v=noise, width_deg=width,
                                           left_deg=left_cross, right_deg=right_cross))
        i = end+1
    if not candidates:
        raise ValueError('No clear arc above local noise; sweep across both sides of the target and confirm again')
    return max(candidates, key=lambda item:item['prominence_v'])
