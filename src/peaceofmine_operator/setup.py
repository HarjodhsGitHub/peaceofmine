from glob import glob
from setuptools import find_packages, setup


package_name = 'peaceofmine_operator'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py') + glob('launch/*.xml')),
        (f'share/{package_name}/dashboard', glob('dashboard/*')),
        (f'lib/{package_name}', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nils Kiefer',
    maintainer_email='21311514+nilskiefer@users.noreply.github.com',
    description='Safe browser operator interface and simulation payloads for PeaceOfMine.',
    license='Apache-2.0',
)
