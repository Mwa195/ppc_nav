from setuptools import setup

package_name = 'ppc_pkg'

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
    maintainer='mwa',
    maintainer_email='mwa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = ppc_pkg.mission_node:main',
            'behaviour_node = ppc_pkg.behaviour_node:main',
            'local_planner_node = ppc_pkg.local_planner_node:main',
            'global_planner_node = ppc_pkg.global_planner_node:main'
        ],
    },
)
