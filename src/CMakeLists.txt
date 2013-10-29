
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libmass_matrix_test-<target>.so
# and installs in the directory lib/orocos/mass_matrix_test/
#
orocos_component(mass_matrix_test mass_matrix_test-component.hpp mass_matrix_test-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/mass_matrix_test/ directory
#
orocos_install_headers(mass_matrix_test-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

