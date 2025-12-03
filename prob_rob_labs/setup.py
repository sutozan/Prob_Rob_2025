from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'prob_rob_labs'

launch_files = glob('launch/*_launch.py') + \
    glob('launch/*_launch.xml') + \
    glob('launch/*.yaml') + \
    glob('launch/*.yml')

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), launch_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilija Hadzic',
    maintainer_email='ih2435@columbia.edu',
    description='Prob Rob Labs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_localization = ekf_localization.ekf_localization:main',
            'variance_estimation = variance_estimation.variance_estimation:main',
            'covariance_calculations = covariance_calculations.covariance_calculations:main',
            'measurment_calculations = measurment_calculations.measurment_calculations:main',
            'vision_geometry = vision_geometry.vision_geometry:main',
            'euclidean_distance_determiner = euclidean_distance_determiner.euclidean_distance_determiner:main',
            'Odometry_Tracking = Odometry_Tracking.Odometry_Tracking:main',
            'StateExtraction = StateExtraction.StateExtraction:main',
            'Lab3_DataCollection = Lab3_DataCollection.Lab3_DataCollection:main',
            'door_opener = door_opener.door_opener:main',
            'image_mean_feature_x = image_mean_feature_x.image_mean_feature_x:main',
            'flaky_door_opener = flaky_door_opener.flaky_door_opener:main',
        ],
    }
)
