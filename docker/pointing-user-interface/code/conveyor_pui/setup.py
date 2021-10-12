from setuptools import setup

package_name = 'conveyor_pui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'pui_node = conveyor_pui.pui_node:main',
            'single_LED_pui_node = conveyor_pui.single_LED_pui_node:main'
        ],
    },
)
