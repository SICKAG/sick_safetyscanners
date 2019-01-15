^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safetyscanners
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
