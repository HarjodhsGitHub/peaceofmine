"""Catch undefined XML launch variables without starting hardware nodes."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


class LaunchXmlTest(unittest.TestCase):
    def test_mavros_uses_only_the_vehicle_launch_namespace(self):
        path = Path(__file__).parents[2] / 'svea_core/launch/lli.xml'
        root = ET.parse(path).getroot()
        group = root.find('group')
        self.assertEqual(group.find('push_ros_namespace').attrib['namespace'], '$(var name)')
        mavros = group.find("node[@pkg='mavros']")
        self.assertNotIn('namespace', mavros.attrib)
        self.assertEqual(mavros.attrib['output'], 'screen')
        self.assertEqual(mavros.attrib['ros_args'], '--log-level info')

    def test_operator_variables_are_declared(self):
        path = Path(__file__).parents[1] / 'launch/operator.launch.xml'
        root = ET.parse(path).getroot()
        declared = {arg.attrib['name'] for arg in root.findall('arg')}
        referenced = {name for element in root.iter() for value in element.attrib.values()
                      for name in re.findall(r'\$\(var\s+([^\s)]+)\)', value)}
        self.assertEqual(referenced - declared, set())


if __name__ == '__main__':
    unittest.main()
