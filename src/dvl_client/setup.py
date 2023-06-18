from setuptools import setup

package_name = 'dvl_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='mason',
    maintainer_email='mason@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_client = dvl_client.dvl_client:main',
        ],
    },
)
