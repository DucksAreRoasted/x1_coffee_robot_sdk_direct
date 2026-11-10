import unittest
from prepare import conversion_from_log

class PrepareTest(unittest.TestCase):
    def log(self):
        return '\n'.join(f'● 轴 {i} 额定力矩 {i}.00 N·m' for i in range(1,16))

    def test_complete_repeated_log_preserves_unverified_direction(self):
        result=conversion_from_log(self.log()+'\n'+self.log())
        self.assertEqual(result['joints']['Joint_r7']['rated_nm'],15)
        self.assertFalse(result['torque_signs_verified'])
        self.assertTrue(all(j['sign'] is None for j in result['joints'].values()))

    def test_missing_or_conflicting_values_rejected(self):
        for text in ['', self.log()+'\n轴 6 额定力矩 99.00 N·m',
                     self.log()+'\n轴 6 额定力矩 0.00 N·m']:
            with self.assertRaises(ValueError): conversion_from_log(text)

if __name__=='__main__': unittest.main()
