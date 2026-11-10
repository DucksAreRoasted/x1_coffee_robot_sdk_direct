import unittest
from motor_diagnostics import parse_diagnostics


class MotorDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.text = '\n'.join(f'[电机诊断] axis={i} index=0x6075 rated_current_ma=5400' for i in range(1, 16))

    def test_repeated_values_and_hardware_version_do_not_identify_structure(self):
        result = parse_diagnostics(self.text+'\n'+self.text+'\n[电机诊断] axis=14 index=0x1009 value=1005')
        self.assertEqual(result['Joint_r6']['rated_current_ma'], 5400)
        self.assertEqual(result['Joint_r6']['versions']['0x1009'], '1005')
        self.assertIsNone(result['Joint_r6']['structure_version'])

    def test_missing_invalid_and_conflicting_values(self):
        for text in ['', self.text.replace('axis=15 ', 'axis=16 '),
                     self.text.replace('5400', '0'),
                     self.text+'\n[电机诊断] axis=14 index=0x6075 rated_current_ma=6000']:
            with self.assertRaises(ValueError): parse_diagnostics(text)

    def test_conflicting_versions(self):
        with self.assertRaises(ValueError):
            parse_diagnostics(self.text+'\n[电机诊断] axis=14 index=0x1009 value=1005\n[电机诊断] axis=14 index=0x1009 value=1006')
