from setuptools import setup
from glob import glob

package_name = 'hw5code'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
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
            'hw4p3sol = hw5code.hw4p3sol:main',
            'hw4p4sol = hw5code.hw4p4sol:main',
            'hw5p1    = hw5code.hw5p1:main',
            'hw5p2    = hw5code.hw5p2:main',
            'hw5p2a   = hw5code.hw5p2a:main',
            'hw5p2b   = hw5code.hw5p2b:main',
            'hw5p3    = hw5code.hw5p3:main',
            'hw5p4    = hw5code.hw5p4:main',
            'KinematicChain = hw5code.KinematicChain:main',
        ],
    },
)


