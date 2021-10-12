from setuptools import setup
from glob import glob

package_name = 'iros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.launch")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabri',
    maintainer_email='gabri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_logger = iros.task_logger:main',
            'multi_task_logger = iros.multi_task_logger:main',
            'pack_generator = iros.pack_generator:main',
            'experiment = iros.experiment:main',
            'gui_experiment = iros.gui_experiment:main',
        ],
    },

)
