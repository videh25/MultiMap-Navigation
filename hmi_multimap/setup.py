from setuptools import find_packages, setup

package_name = 'hmi_multimap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='videh22',
    maintainer_email='videh22@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping = hmi_multimap.mapping:main',
            'multimat_nav_test = hmi_multimap.multimap_nav_client:main',
        ],
    },
)
