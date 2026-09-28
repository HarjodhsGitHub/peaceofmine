"""Catch undefined XML launch variables without starting hardware nodes."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

from peaceofmine_operator import bringup

LAUNCH = Path(__file__).parents[1] / 'launch'


def declared_args(path):
    return {arg.attrib['name'] for arg in ET.parse(path).getroot().findall('arg')}


class LaunchXmlTest(unittest.TestCase):
    def test_operator_variables_are_declared(self):
        for path in sorted(LAUNCH.glob('*.xml')):
            with self.subTest(path=path.name):
                root = ET.parse(path).getroot()
                referenced = {name for element in root.iter() for value in element.attrib.values()
                              for name in re.findall(r'\$\(var\s+([^\s)]+)\)', value)}
                self.assertEqual(referenced - declared_args(path), set())

    def test_setup_forwards_every_bringup_setting(self):
        root = ET.parse(LAUNCH / 'operator_setup.launch.xml').getroot()
        forwarded = {param.attrib['name'].removeprefix('bringup.') for param in root.iter('param')
                     if param.attrib['name'].startswith('bringup.')}
        self.assertEqual(forwarded, set(bringup.DEFAULTS))

    def test_subsystem_arguments_exist_in_child_launches(self):
        for name, subsystem in bringup.SUBSYSTEMS.items():
            with self.subTest(subsystem=name):
                self.assertLessEqual(set(subsystem['args']), declared_args(LAUNCH / subsystem['launch']))


if __name__ == '__main__':
    unittest.main()
