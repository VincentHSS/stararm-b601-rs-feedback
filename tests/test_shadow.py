import unittest
from stararm_rs_feedback.shadow import ShadowPolicy

class ShadowTests(unittest.TestCase):
    def test_empty_residual_has_no_ordinary_damping(self):
        p=ShadowPolicy()
        for j in range(60):
            self.assertEqual(p.update([-.2,.5,.6,-.25,.3,.2],100+j*.05,True),[0]*7)

    def test_sustained_load_is_bounded_and_putdown_releases(self):
        p=ShadowPolicy()
        for j in range(100):power=p.update([1.,-2.8,4.8,2.7,2.,3.],100+j*.05,True)
        self.assertTrue(all(0<x<=150 for x in power[1:4]))
        self.assertEqual([power[i] for i in (0,4,5,6)],[0]*4)
        for j in range(100):power=p.update([0.]*6,105+j*.05,True)
        self.assertEqual(power,[0]*7)

    def test_invalid_window_clears_output_immediately(self):
        p=ShadowPolicy()
        for j in range(100):p.update([0.,-3.,5.,3.,0.,0.],100+j*.05,True)
        self.assertEqual(p.update([0.]*6,105.,False),[0]*7)

    def test_short_spike_and_gap_do_not_accumulate_confirmation(self):
        for gap in (False,True):
            p=ShadowPolicy()
            for j in range(5):self.assertEqual(p.update([0.,-3.,5.,3.,0.,0.],100+j*.05,True),[0]*7)
            if gap:self.assertEqual(p.update([0.,-3.,5.,3.,0.,0.],101.,True),[0]*7)
            else:self.assertEqual(p.update([0.]*6,100.25,True),[0]*7)

if __name__=='__main__':unittest.main()
