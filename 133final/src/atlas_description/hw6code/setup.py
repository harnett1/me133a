from setuptools import setup

package_name = 'hw6code'

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
    description='The 133a HW5 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw5p1sol = hw6code.hw5p1sol:main',
            'hw5p2sol = hw6code.hw5p2sol:main',
            'hw5p4sol = hw6code.hw5p4sol:main',
            'KinematicChain = hw6code.KinematicChain:main',
            'hw6p1    = hw6code.hw6p1:main',
            'hw6p2    = hw6code.hw6p2:main',
            'hw6p3    = hw6code.hw6p3:main',
            'hw6p4    = hw6code.hw6p4:main',
            'hw6p5    = hw6code.hw6p5:main',
        ],
    },
)
