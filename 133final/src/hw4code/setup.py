from setuptools import setup

package_name = 'hw4code'

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
    description='The 133a HW4 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw3p3sol = hw4code.hw3p3sol:main',
            'hw3p4sol = hw4code.hw3p4sol:main',
            'hw3p5sol = hw4code.hw3p5sol:main',
            'hw4p3    = hw4code.hw4p3:main',
            'hw4p4    = hw4code.hw4p4:main',
        ],
    },
)
