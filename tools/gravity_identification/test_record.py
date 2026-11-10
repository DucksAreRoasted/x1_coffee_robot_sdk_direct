import json,unittest
from pathlib import Path
from record import validated_joints

class RecordScopeTest(unittest.TestCase):
    def test_right_evidence_does_not_authorize_left(self):
        cfg=json.loads(Path(__file__).with_name('right_arm_session.json').read_text())
        self.assertEqual(len(validated_joints(cfg,'r')),7)
        with self.assertRaises(ValueError): validated_joints(cfg,'l')
        cfg['joints']['Joint_r3']['sign']=None
        with self.assertRaises(ValueError): validated_joints(cfg,'r')

if __name__=='__main__':unittest.main()
