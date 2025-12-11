"""
Rules for sensor libraries.
"""

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_gazebo//gazebo:headers.bzl", "gz_export_header")

def gz_sensor_library(name, hdrs, **kwargs):
    """
    Adds a cc_library target for the sensor with an auto-generated Export.hh header.

    Args:
        name: Name of the library
        hdrs: The headers to be added to the cc_library target
        **kwargs: Forwarded to the cc_library target
    """
    name_upper_case = name.upper()
    name_title_case = name.title()

    export_hh_path = "include/gz/sensors/" + name + "/Export.hh"
    gz_export_header(
        name = name_title_case + "SensorExport",
        out = export_hh_path,
        export_base = "GZ_SENSORS_" + name_upper_case,
        lib_name = "gz-sensors-" + name,
    )

    cc_library(
        name = name,
        hdrs = hdrs + [export_hh_path],
        **kwargs
    )
