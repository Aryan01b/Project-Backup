from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reassign_pkg'

executables = []

for filename in os.listdir(os.path.join(os.getcwd(), package_name)):
    if filename.endswith('.py') and not filename.startswith('_'):
        file = filename[:-3]
        executables.append(f"{file} = {package_name}.{file}:main")


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robox',
    maintainer_email='arlikararyan@gmail.com',
    description='Assignment FlytBase',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': executables,
    },
)
