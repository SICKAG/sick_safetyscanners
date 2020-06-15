^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safetyscanners
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2020-06-15)
------------------
* fixing correct offset in field geometries
* Fixed error in reading chars for device name and project name
* feat(diagnostics): Sensor state diagnostics
  Exposes sensor hardware information and sensor state.
* Filter out max range values to INF according to REP 117
* Correct first initialization of m_time_offset
* boost::asio API changes in 1.70+
* Catch exceptions by const ref.
* Fix error_code comparison to int.
* Contributors: Chad Rockey, Jad Haj Mustafa, Lennart Puck, Mike Purvis, Rein Appeldoorn

1.0.4 (2019-12-16)
------------------
* Fixed Bug with enum for interface type
* Eoved enums in class
  Enums are not in the class scope where they are used.
  Prevents redefinitions and pollution of namespace.
* Correctet variable index for username command
* Typecode read and parsed from variable
* Used static casts instead of implicit conversion
* Contributors: Lennart Puck

1.0.3 (2019-07-15)
------------------
* erasing completed frames from map. 
* Fixed error on startup that no scan was visualised
  The fix should prevent the node from starting without
  publishing any data. The error appears to be related to
  minor rounding errors, thus setting the resolution smaller then
  the lowest resolution. But not equal start and end angles.
  This should fix issue #11 and #12
* added initialisation of use_pers_config
* Merge Pull Request #9
  Removing the possibilities to use the angles from the sensor it self.
  Since dynamic reconfigure can only be set up for one frame.
* removed tcp port from configuration since it can not be configured in the sensor
* added parameter to use persistent config
* Added methods to request persistent data from sensor
* added all parameters to launch file
* updated persistent and current config command and parser to use config data instead of field data
* removed unused end angle from field data
* added datastructure for configs
* Fix issue with m_angle_offset.  Remove use_sick_angles
* Use C++ STL to reduce risk of memory corruption
* Change ReadWriteHelper to namespace functions instead of a stateless class
* Contributors: Chad Rockey, Jonathan Meyer, Lennart Puck, NicolasLoeffler

1.0.2 (2019-01-15)
------------------
* Read the start angle of the field data from the persistent config instead of the current config
* Changed to 0 angle being at the front of the scan
* Allow system to choose the host udp port from the ephemeral range.  Resolve typo -> IPAdress to IPAddress
* Changed default frame_id name to scan
* Change publish_frequency parameter to be skip parameter. 
* Add time_offset parameter to adjust scan system timestamps
* Added median reflector bit in message and code
* Added active case number to the service call
* Field data is returned as a vector for all fields
* Added publisher und service server for field data and output paths
* Added Start angle and beam resolution to field data

1.0.1 (2018-10-31)
------------------

* Initial Release
