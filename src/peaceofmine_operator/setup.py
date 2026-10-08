from glob import glob
from setuptools import find_packages, setup


package_name = 'peaceofmine_operator'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'operator_config.json']),
        (f'share/{package_name}/launch', glob('launch/*.py') + glob('launch/*.xml')),
        (f'share/{package_name}/dashboard', [p for p in glob('dashboard/*') if __import__('os').path.isfile(p)]),
        (f'lib/{package_name}', glob('scripts/*.py')),
        *[(f'share/{package_name}/' + str(path.parent), [str(path)])
          for path in __import__('pathlib').Path('dashboard/gnss').rglob('*') if path.is_file()],
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Nils Kiefer',
    maintainer_email='21311514+nilskiefer@users.noreply.github.com',
    description='Safe browser operator interface and simulation payloads for PeaceOfMine.',
    license='Apache-2.0',
)
