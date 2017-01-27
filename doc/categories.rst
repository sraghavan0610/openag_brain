Categories
==========

The OpenAg system defines a list of "categories" which can be used to describe
the functionality contained in a firmware/software module/input/output. For
example, the firmware module for the Atlas Scientific DO sensor itself belongs
to the "sensors" and "calibration" categories because it outputs sensor data
and has inputs for calibration. The dissolved oxygen output from this firmware
module belong to the "sensors" category because it represents a sensor reading,
and the inputs to this module used for calibration belong to the "calibration"
category.

On any given run of the system, it is possible to specify a list of categories
that should be enabled. By default, all categories are enabled except for
"calibration". Entities that do not specify a category will always be enabled.

The categories are defined in the :py:module::`openag.categories` module.

.. automodule:: openag.categories
   :members:
