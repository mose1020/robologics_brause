from setuptools import setup

package_name = 'r2e_demos'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'test_ros_env = r2e_demos.test_ros_env:main',
            'r2e_cubes = r2e_demos.r2e_cubes:main',
            'geometrie_pub = r2e_demos.geometrie_pub:main',
            'geometrie_pub_2 = r2e_demos.geometrie_pub_2:main',
            'geometrie_sub = r2e_demos.geometrie_sub:main'
        ],
    },
)
