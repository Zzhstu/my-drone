import json,sys,unittest
from pathlib import Path
from dataclasses import fields
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_view_policy import JointViewPolicy,View

class ActiveGhostScoreTest(unittest.TestCase):
    def policy(self,scale=1.):
        data=json.loads((Path(__file__).parent/'fixtures/ghost_score_155110_114021.json').read_text());p=data['parameters'];p['joint_active_ghost_gain_multiplier']=scale
        return JointViewPolicy(lambda k,d:p.get(k,d)),data
    def view(self,row):
        names={f.name for f in fields(View)}-{'cell','xy','score'}
        return View((0,0),tuple(row['xy']),**{k:v for k,v in row.items() if k in names})
    def test_recorded_candidate_scores_reproduced_then_active_gain_flips_local_rank(self):
        p,d=self.policy();front,ghost=[self.view(row) for row in d['candidates']]
        for row,v in zip(d['candidates'],[front,ghost]):self.assertAlmostEqual(p._score(v),row['score'],places=12)
        self.assertLess(p._score(ghost),p._score(front))
        boosted,_=self.policy(1.3)
        self.assertGreater(boosted._score(ghost),boosted._score(front))
        self.assertAlmostEqual(boosted._score(ghost)/p._score(ghost),1.3)
        self.assertEqual(boosted._score(front),p._score(front))
    def test_cleared_history_surface_and_ordinary_views_unchanged(self):
        old,d=self.policy();new,_=self.policy(1.3);v=self.view(d['candidates'][1])
        for kind,group in [('history_review',(1,2)),('surface_inspection',None),('frontier',None),('verification',None)]:
            v.kind=kind;v.group=group;self.assertEqual(old._score(v),new._score(v))
    def test_home_zero_remains_zero_after_boost(self):
        p,d=self.policy(1.3);v=self.view(d['candidates'][1]);v.ghost_reward_scale=0
        self.assertEqual(p._score(v),0)
    def test_invalid_nonpositive_or_nonfinite_multiplier_rejected(self):
        for scale in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):self.policy(scale)

if __name__=='__main__':unittest.main()
