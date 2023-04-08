from setuptools import setup
import os
from glob import glob

package_name = 'a_star_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), ('share/' + package_name + '/worlds', glob('worlds/*')), (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandeep',
    maintainer_email='ssandy086@gmail.com',
    # scripts=['a_star_tb3/a_star_tb3_script.py'],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['a_star_tb3_script.py = a_star_tb3.a_star_tb3_script:main'],
    },
)