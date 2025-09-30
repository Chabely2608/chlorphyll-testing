from setuptools import find_packages, setup

package_name = 'chl_pkg'

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
    maintainer='chabelypecina',
    maintainer_email='chabelypecina@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chl_data = chl_pkg.chl_conversion:main",
            "chl_getcsv = chl_pkg.chl_getcsv:main"
        ],
    },
)
