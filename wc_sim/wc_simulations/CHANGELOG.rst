^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wc_simulations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.6 (2202-05-26)
------------------
* ROS2 Humble Hawksbill supported

2.2.5 (2021-08-25)
------------------
* Release for ROS2 Rolling
* Contributors: Will Son

2.2.4 (2021-06-14)
------------------
* Release for ROS2 Galactic
* Separate world and robot models(#162)
* Clean up unncessary files
* Use wc_common mesh modeling
* Independent wc_simulations package
* Contributors: Joep Tool, Will Son

2.2.3 (2021-04-12)
------------------
* Update required keyword arguments
* Clear up exec_depend
* Fix Waffle Pi wheel inertia
* Contributors: ruffsl, Will Son

2.2.2 (2021-02-24)
------------------
* Remove shared objects built in older version
* Contributors: Will Son

2.2.1 (2021-01-13)
------------------
* Eloquent Elusor EOL
* Add missing imu joint in sdf
* Append Gazebo model path
* Portable fix, launch description revise
* Ament lint applied
* Contributors: minwoominwoominwoo7, Rayman, seanyen, ashe kim, Will Son

2.2.0 (2020-06-29)
------------------
* Wc Drive node implementation
* Additional Gazebo maps added
* argument tags in the sdf file replaced with remapping tags
* Low polygon 3D modeling applied for simulation
* Contributors: Ryan Shim, Mikael Arguedas, Will Son

2.1.0 (2019-09-10)
------------------
* ROS 2 migration of wc_fake_node package
* Modified rviz config (background colour changed to white, tf display disabled)
* Added wc_house and related world, model files
* Contributors: Ryan Shim, Darby Lim, Pyo

2.0.1 (2019-09-05)
------------------
* Modified dependency packages
* Modified launch directory
* Added a launch file for robot state publisher
* Contributors: Darby Lim, Pyo

2.0.0 (2019-08-20)
------------------
* Supported ROS 2 Dashing Diademata
* Updated the CHANGELOG and version to release binary packages
* Contributors: Darby Lim, Pyo

1.3.0 (2020-06-29)
------------------
* Wc Autorace 2020 implemented
* Remove the plugin_path from gazebo_ros export
* Remove *nix path separator
* Contributors: Ashe Kim, Ben Wolsieffer, Sean Yen

1.2.0 (2019-01-22)
------------------
* move out the init() from ROS_ASSERT `#68 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/68>`_
* moved <scene> into <world> `#65 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/65>`_
* modified ML stage
* delete unused param
* update algorithm and modified variable more clearly
* Contributors: Darby Lim, Gilbert, Louise Poubel, Pyo

1.1.0 (2018-07-20)
------------------
* added wc Waffle Pi
* modified uri path
* modified autorace
* delete remap
* Contributors: Darby Lim, Gilbert, Pyo

1.0.2 (2018-06-01)
------------------
* added mission.launch modified model.sdf
* deleted wc's gazebo plugins
* modified autorace gazebo
* merged pull request `#53 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/53>`_ `#52 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/52>`_ `#51 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/51>`_ `#50 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/50>`_ `#49 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/49>`_
* Contributors: Gilbert, Darby Lim, Pyo

1.0.1 (2018-05-30)
------------------
* resolving dependency issues:
  http://build.ros.org/job/Kbin_dj_dJ64__wc_gazebo__debian_jessie_amd64__binary/2/
* Contributors: Pyo

1.0.0 (2018-05-29)
------------------
* added world for wc_autorace
* added world for wc_machine_learning
* merged pull request `#46 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/46>`_ from AuTURBO/develop
  add wc_autorace world'
* merged pull request `#48 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/48>`_ `#47 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/47>`_ `#44 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/44>`_ `#42 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/42>`_ `#41 <https://github.com/ROBOTIS-GIT/wc_simulations/issues/41>`_
* Contributors: Darby Lim, Gilbert, hyunoklee, Pyo

0.2.4 (2018-03-14)
------------------
* solved DuplicateVersionsException error
* Contributors: Pyo

0.2.3 (2018-03-14)
------------------
* solved DuplicateVersionsException error
* Contributors: Pyo

0.2.2 (2018-03-14)
------------------
* added line feed into metapackage
* Contributors: Pyo

0.2.1 (2018-03-14)
------------------
* added worlds for gazebo and wc
* deleted wc_gazebo_plugin and merged into wc_gazebo_ros package
* Contributors: Darby Lim

0.2.0 (2018-03-13)
------------------
* added Wc Waffle Pi
* added slam with multiple tb3
* added multi example
* added wc_house
* added wc_house
* added msg function
* modified gazebo plugin
* modified tb3 control
* modified sensor param
* modified camera position
* modified image_listener
* modified cmake file
* modified spwn model name
* modified multi slam param
* modified camera position
* modified folder name
* Contributors: Darby Lim

0.1.7 (2017-08-16)
------------------
* renamed missed the install rule (worlds -> models)
* Contributors: Darby Lim, Tully Foote

0.1.6 (2017-08-14)
------------------
* modified folder name and model path
* updated rviz and add static tf publisher for depth camera
* Contributors: Darby Lim

0.1.5 (2017-06-09)
------------------
* modified make files for dependencies
* updated wc sim
* updated world config
* Contributors: Darby Lim

0.1.4 (2017-05-23)
------------------
* added as new meta-packages and version update (0.1.4)
* Contributors: Darby Lim, Pyo
