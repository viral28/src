Change log
==========

1.4.0 (2016-08-31)
==================

* API changes:
  * Added commands to set FireWire protocol (used in sawIntuitiveResearchKit console configuration JSON files)
* Deprecated features:
  * None
* New features:
  * CMake: separated components from applications/examples (catkin build 0.4 compatible)
* Bug fixes:
  * None

1.3.0 (2016-01-08)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Added serial number display in Qt widget
  * Added command/event to change actuator coupling on the fly
  * Added command to bias encoders based on pots using multiple samples (averaging pot noise)
* Bug fixes:
  * Minor fixes to support older C++ compilers

1.2.0 (2015-10-18)
==================

* API changes:
  * Updated XML config file version to 2 for fix in digital inputs
* Deprecated features:
  * None
* New features:
  * Improved velocity estimation
  * Added commands to support PWM (used for SUJ lift motor and MUX)
* Bug fixes:
  * Encoder to pots safety check now looks for multiple consecutive failures to avoid intempestive errors
  * XML value "pressed" was used to negate values twice for events but once for state table
  * Fixed issue when power was lost on error that prevented powering back on

1.1.1 (2015-05-15)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * In current calibration example, preload encoders to avoid encoder bit overflow

1.1.0 (2015-04-28)
==================

* API changes:
  * Change to SI units, i.e. translations are now assumed to be in meters, not millimeters.
  * Encoder/pots discrepencies are now treated as errors.
  * Requires mechatronics library Amp1394 version 1.1 or higher.
  * Removed encoder bits to position offset, now assumes the offset if preloaded on FPGA
* Deprecated features:
  * None
* New features:
  * Added `Error`, `Warning` and `Status` events with `std::string` payload.
  * Preload encoders when component starts.
  * All encoders are now relative to mid-range to avoid overflow.
  * Added detection of encoder bit overflow.
  * C++ exceptions are now caught and trigger cisstMultiTask events `Error`s.
* Bug fixes:
  * None


1.0.1 (2014-12-30)
==================

* No change log file, initial release.
