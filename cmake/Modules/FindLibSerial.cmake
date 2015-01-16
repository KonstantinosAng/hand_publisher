# - Try to find LibSerial headers and libraries.
#
# Usage of this module as follows:
#
#     find_package(LibSerial)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# Variables defined by this module:
#
#  LibSerial_FOUND              System has LibSerial libs/headers
#  LibSerial_LIBRARIES          The LibSerial libraries
#  LibSerial_INCLUDE_DIR        The location of LibSerial headers

find_path(LibSerial_INCLUDE_DIR NAMES include/SerialStream.h)
find_library(LibSerial_LIBRARIES NAMES serial)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibSerial DEFAULT_MSG
    LibSerial_LIBRARIES
    LibSerial_INCLUDE_DIR
)

mark_as_advanced(
    LibSerial_LIBRARIES
    LibSerial_INCLUDE_DIR
)
