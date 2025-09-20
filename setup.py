from setuptools import find_packages, setup

package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # <-- This must match your inner folder!
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_pkg.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmh',
    maintainer_email='mahmoudmohamedhelmy8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = test_pkg.publisher:main',
            'sub = test_pkg.sub:main',
            'turt = test_pkg.turt:main',
            'turtle_move = test_pkg.turtle_move:main',
            'gampad_node = test_pkg.gampad_node:main',
        ],
    },
)
