from setuptools import find_packages, setup
import os

package_name = 'colorsort_arm'

data_files_list = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'urdf'),   ['urdf/planar_arm.urdf']),
    (os.path.join('share', package_name, 'launch'), ['launch/colorsort.launch.py']),
]

# Optionally install a sample image if present
if os.path.exists('sample.jpg'):
    data_files_list.append((os.path.join('share', package_name), ['sample.jpg']))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sourav',
    maintainer_email='sourav@todo.todo',
    description='Color-based pick-and-place demo with a simple planar arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_detector = colorsort_arm.color_detector:main',
            'arm_controller  = colorsort_arm.arm_controller:main',
        ],
    },
)
