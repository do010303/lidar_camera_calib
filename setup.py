from setuptools import setup
from glob import glob
package_name = 'lidar_camera_calib'
setup(name=package_name, version='0.0.1', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Calibration Prototype', maintainer_email='calib@example.com',
      description='2D LiDAR camera checkerboard point-plane extrinsic calibration',
      license='MIT',
      entry_points={'console_scripts': [
          'capture = lidar_camera_calib.capture:main',
          'solve = lidar_camera_calib.solve:main',
          'validate = lidar_camera_calib.validate:main',
          'evaluate = lidar_camera_calib.evaluate:main',
          'check_range = lidar_camera_calib.check_range:main',
      ]})
