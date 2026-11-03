from setuptools import setup
from glob import glob
package_name='gazeboteach_lifecycle'
setup(name=package_name,version='0.1.0',packages=[package_name],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='GazeboTeach',maintainer_email='teaching@example.invalid',description='Lifecycle exercise',license='MIT',
 entry_points={'console_scripts':['sensor = gazeboteach_lifecycle.sensor:main']})
