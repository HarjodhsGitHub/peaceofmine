import copy
import json
from pathlib import Path
import tempfile
import unittest
from peaceofmine_operator.ads1115 import DEFAULTS
from peaceofmine_operator.lean_migration import migrated, migrate_file
from peaceofmine_operator.camera_settings import DEFAULTS as CAMERAS


class MigrationTest(unittest.TestCase):
    def document(self):
        cameras=copy.deepcopy(CAMERAS); cameras['forward']['source']='virtual'
        adc=dict(config=copy.deepcopy(DEFAULTS),routes=['both']*8,
                 probe_trigger=dict(enabled=True,mux=5,level=3.,direction='above'))
        adc['config']['channels'][4]['multiplier']=5
        return dict(schema_version=1,adc=adc,cameras=cameras)

    def test_backup_wiring_calibration_reset_and_idempotence(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'operator_config.json'; original=json.dumps(self.document());path.write_text(original)
            migrate_file(str(path))
            value=json.loads(path.read_text())
            self.assertEqual(path.with_name(path.name+'.pre-lean-v1.bak').read_text(),original)
            self.assertEqual([c['mux'] for c in value['adc']['config']['channels'] if c['enabled']],[4,7])
            self.assertEqual(value['adc']['routes'][4],'probe');self.assertEqual(value['adc']['routes'][7],'detector')
            self.assertEqual(value['adc']['probe_trigger']['mux'],4);self.assertFalse(value['adc']['probe_trigger']['enabled'])
            self.assertEqual(value['adc']['config']['channels'][4]['multiplier'],1)
            self.assertEqual(value['cameras']['forward']['source'],'auto')
            value['cameras']['forward']['source']='virtual';path.write_text(json.dumps(value))
            migrate_file(str(path));self.assertEqual(json.loads(path.read_text()),value)

    def test_simulation_preserves_hardware_adc(self):
        document=self.document(); result=migrated(document,True)
        self.assertEqual(result['adc'],document['adc'])
