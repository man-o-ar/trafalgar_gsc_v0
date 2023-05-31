import os
from glob import glob
from setuptools import setup

package_name = 'gsc'

components = "gsc/components"
nodes = "gsc/nodes"
utils = "gsc/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[ package_name, nodes, components, utils ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    
    ]
    ,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ManOAR',
    maintainer_email='contact@manoar.com',
    description='base nodes for the ground station controller',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = gsc.nodes.__gui:main'
        ],
    }
)
