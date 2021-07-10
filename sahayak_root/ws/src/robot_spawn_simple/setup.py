from setuptools import setup
import os
from glob import glob

package_name = 'robot_spawn_simple'

package_data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]


def make_package_data(base, dir):
    package_data_current = []
    files_current = []
    for f in glob(os.path.join(dir, "*"), recursive=False):
        if os.path.isfile(f):
            files_current.append(f)
        else:
            package_data_current += make_package_data(base, f)
    package_data_current += [(os.path.join(base, dir), files_current)]
    return package_data_current


package_data_files += make_package_data('share/robot_spawn_simple/', 'worlds')
package_data_files += make_package_data('share/robot_spawn_simple/', 'resource')
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aayush',
    maintainer_email='aayush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_driver = robot_spawn_simple.simple_driver:main'
        ],
    },
)
