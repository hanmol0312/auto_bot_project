from setuptools import setup
from glob import glob
import os

package_name = 'auto_bot'

setup(

    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'meshes'),glob('meshes/*')),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
        (os.path.join('share',package_name,'worlds'),glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anmol',
    maintainer_email='ec21b1086@iiitdm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving_auto_bot = auto_bot.driver_node:main',
            'goal_auto_bot= auto_bot.reach_goal:main',
            'vid_saving=auto_bot.video_saver_node:main',
            'maze_solving=auto_bot.maze_solving:main',
            'bot_localization=auto_bot.auto_bot_localization:main',
            'utilities=auto_bot.utilities:main',
            'configuration=auto_bot.config:main',
            'bot_map=auto_bot.bot_mapping:main'
            'bot_path_planning=auto_bot.bot_pathplanning:main'
            'bot_motion_plan=auto_bot.bot_motion_planning:main'
        ],
    },
    
)
