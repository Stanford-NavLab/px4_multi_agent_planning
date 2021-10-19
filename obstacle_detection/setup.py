from setuptools import setup

package_name = 'obstacle_detection'

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
    maintainer='navlab-exxact-18',
    maintainer_email='adamdai97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth = obstacle_detection.ground_truth:main',
            'visualize_map = obstacle_detection.visualize_map:main',
            'ground_truth_cylinders = obstacle_detection.ground_truth_cylinders:main',
        ],
    },
)
