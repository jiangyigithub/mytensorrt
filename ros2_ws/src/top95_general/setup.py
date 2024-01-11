# import os
# from glob import glob
# from setuptools import setup

# package_name = 'top95_general'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         #('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name, glob('launch/*.launch.py')),
#         ('share/' + package_name, glob('urdf/*')),
#         #(os.path.join('share', package_name), glob('urdf/*')),
#         ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='TODO: Package maintainer',
#     maintainer_email='TODO: Package maintainer email',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={},
#     )
import os
from glob import glob
from setuptools import setup

package_name = 'top95_general'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config/radar_gen5'), glob('config/radar_gen5/*')),
        (os.path.join('share', package_name, 'config/radar_pointcloud'), glob('config/radar_pointcloud/*')),
        (os.path.join('share', package_name, 'config/t2t'), glob('config/t2t/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'radar_gen5'), glob('radar_gen5/*')),
        (os.path.join('share', package_name, 'config/btcs'), glob('btcs/*')),
        (os.path.join('share', package_name, 'config/btcs_offline'), glob('btcs_offline/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boj1yh',
    maintainer_email='joachim.boerger@de.bosch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

