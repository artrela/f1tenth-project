from setuptools import setup

package_name = 'PyVO'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alec Trela, Sridevi Kaza',
    maintainer_email='atrela@andrew.cmu.edu, sridevik@andrew.cmu.edu',
    description='A prototype visual odometry package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = visual_odometry.visual_odometry:main',
        ],
    },
)
