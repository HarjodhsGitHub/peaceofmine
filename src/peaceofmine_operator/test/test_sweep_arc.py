import math
import random
import unittest
from peaceofmine_operator.sweep_arc import sweep_arc


def curve(center=17., baseline=2.45, height=.015, polarity=1, reverse=False, noise=.0008):
    rng=random.Random(7)
    values=[]
    for i in range(1801):
        angle=-90+i*.1
        signal=baseline+polarity*height*math.exp(-.5*((angle-center)/6)**2)
        values.append(dict(angle_deg=angle,volts=signal+rng.gauss(0,noise),
                           filtered_volts=signal+rng.gauss(0,noise*.15)))
    return list(reversed(values)) if reverse else values


class SweepArcTest(unittest.TestCase):
    def test_center_survives_baseline_amplitude_and_direction_changes(self):
        # Raw-voltage curves use one-degree bins, without the causal display filter.
        for baseline,height,reverse in ((2.45,.015,False),(2.6,.006,True),(1.8,.03,False)):
            peak=sweep_arc(curve(baseline=baseline,height=height,reverse=reverse))
            self.assertAlmostEqual(peak['angle_deg'],17.,delta=.5)
            self.assertGreater(peak['prominence_v'],height*.9)
            self.assertGreater(peak['width_deg'],10.)
            self.assertAlmostEqual(peak['right_deg']-peak['left_deg'],peak['width_deg'])
            self.assertLess(peak['left_deg'],peak['angle_deg'])
            self.assertGreater(peak['right_deg'],peak['angle_deg'])

    def test_fast_opposed_passes_do_not_inherit_time_filter_lag(self):
        centers=[]
        for direction in (1,-1):
            rng=random.Random(4)
            points=[]
            for i in range(101):
                angle=direction*(-90+i*1.8)  # 36 degrees/s sampled at 20 Hz.
                raw=2.45+.015*math.exp(-.5*((angle-17.)/6)**2)+rng.gauss(0,.0006)
                lagged=2.45+.015*math.exp(-.5*((angle-17.-direction*3.6)/6)**2)
                points.append(dict(angle_deg=angle,volts=raw,filtered_volts=lagged))
            centers.append(sweep_arc(points)['angle_deg'])
        for center in centers:self.assertAlmostEqual(center,17.,delta=1.)
        self.assertLess(abs(centers[0]-centers[1]),2.)

    def test_negative_voltage_response_uses_the_existing_calibration_polarity(self):
        self.assertAlmostEqual(sweep_arc(curve(polarity=-1),-1)['angle_deg'],17.,delta=.5)

    def test_quiet_noise_and_flat_signal_do_not_create_an_arc(self):
        for noise in (0.,.0008,.003):
            with self.assertRaisesRegex(ValueError,'No clear arc'):
                sweep_arc(curve(height=0.,noise=noise))

    def test_relative_noise_rejects_a_bump_buried_in_noise(self):
        with self.assertRaisesRegex(ValueError,'No clear arc'):
            sweep_arc(curve(height=.001,noise=.003))

    def test_spike_step_and_monotonic_drift_are_not_broad_arcs(self):
        for form in ('spike','step','slope'):
            values=[]
            for a in range(-60,61):
                y=2.+(.1 if form=='spike' and a==0 else .01*(a>0) if form=='step' else a*.0001 if form=='slope' else 0.)
                values.append(dict(angle_deg=a,volts=y,filtered_volts=y))
            with self.assertRaisesRegex(ValueError,'No clear arc'):
                sweep_arc(values)

    def test_incomplete_edge_peak_or_angle_gap_is_not_confirmed(self):
        edge=[p for p in curve(center=89.) if p['angle_deg']<=90]
        gap=[p for p in curve() if not 14<p['angle_deg']<21]
        for values in (edge,gap):
            with self.assertRaises(ValueError): sweep_arc(values)

    def test_invalid_or_insufficient_samples_fail_explicitly(self):
        with self.assertRaises(ValueError): sweep_arc([])
        with self.assertRaises(ValueError): sweep_arc(curve(),0)
        values=curve();values[20]['volts']=math.nan
        with self.assertRaisesRegex(ValueError,'Invalid reading'): sweep_arc(values)


if __name__=='__main__': unittest.main()
