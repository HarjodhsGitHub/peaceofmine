"""Catch undefined XML launch variables without starting hardware nodes."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


class LaunchXmlTest(unittest.TestCase):
    def test_mavros_uses_vehicle_namespace(self):
        path = Path(__file__).parents[2] / 'svea_core/launch/lli.xml'
        root = ET.parse(path).getroot()
        node = next(node for node in root.iter('node') if node.attrib.get('pkg') == 'mavros')
        # MAVROS adds its own child namespace; another layer hides state/RC
        # and wheel telemetry from the operator's safety subscriptions.
        self.assertNotIn('namespace', node.attrib)
        for remap in node.findall('remap'):
            self.assertTrue(remap.attrib['from'].startswith('mavros/'))
            self.assertTrue(remap.attrib['to'].startswith('mavros/_unused/'))

    def test_operator_variables_are_declared(self):
        path = Path(__file__).parents[1] / 'launch/operator.launch.xml'
        root = ET.parse(path).getroot()
        declared = {arg.attrib['name'] for arg in root.findall('arg')}
        referenced = {name for element in root.iter() for value in element.attrib.values()
                      for name in re.findall(r'\$\(var\s+([^\s)]+)\)', value)}
        self.assertEqual(referenced - declared, set())


if __name__ == '__main__':
    unittest.main()
