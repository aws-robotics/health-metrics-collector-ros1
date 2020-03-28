^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package health_metric_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-03-20)
------------------
* Update to use non-legacy ParameterReader API (`#7 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/7>`_)
  * Update to use non-legacy ParameterReader API
  * increment package version
* Contributors: M. M

2.0.2 (2020-03-28)
------------------
* Merge pull request `#25 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/25>`_ from aws-robotics/version_2.0.2
  Bumpinng package version to match bloom release
* Bumpinng package version to match bloom release
* fix collecting of memory metrics data (`#24 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/24>`_)
* update changelog to be compatible with catkin_generate_changelog (`#21 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/21>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* increment patch version (`#20 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/20>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Use standard CMake macros for adding gtest/gmock tests (`#16 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/16>`_)
  * modify health_metric_collector to use add_rostest_gmock()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* [Master branch] CE pipeline migration (`#12 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/12>`_)
  * cherry picking Get dependencies from rosdep instead of building from release-v1 to master
  * cherry picking ce pipeline migration commit from release-v1 to master
  * untag dependency version
  * remove cloudwatch_metrics_collector as exec depend
* Release 2.0.0 (`#9 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/9>`_)
  * Release 2.0.0
  * 2.0.0
* Update to use non-legacy ParameterReader API (`#7 <https://github.com/aws-robotics/health-metrics-collector-ros1/issues/7>`_)
  * Update to use non-legacy ParameterReader API
  * increment package version
* Contributors: AAlon, Abby Xu, M. M, Miaofei Mei, Ragha Prasad

1.0.0 (2019-03-20)
------------------
