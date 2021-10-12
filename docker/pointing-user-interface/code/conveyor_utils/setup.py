from setuptools import setup
import glob

package_name = 'conveyor_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
    ],
    # include_package_data=True,
    # exclude_package_data={package_name: 'playground.py'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerome',
    maintainer_email='jerome@idsia.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_strips = conveyor_utils.test_strips:main',
            'draw_packages = conveyor_utils.draw_packages:main',
            'dummy_tracker = conveyor_utils.dummy_tracker:main',
            'belt_control = conveyor_utils.belt_control:main',
            'feeder = conveyor_utils.feeder:main',
            'multi_feeder = conveyor_utils.multi_feeder:main'
        ],
    },
)
