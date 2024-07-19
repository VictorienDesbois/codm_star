# project properties
project('cmapf', 'cpp', default_options: [
  'default_library=static', 
  'debug=true', 
  'warning_level=0', 
  'optimization=g'])

# include directories
include_dir = include_directories(
  'include',
  'include/cca_star',
  'include/codm_star'
)

# src
src_files = [
  meson.source_root() + '/src/graph.cpp',
  meson.source_root() + '/src/instance_loader.cpp',
  meson.source_root() + '/src/codm_star/codm_star.cpp',
  meson.source_root() + '/src/codm_star/subplanners_manager.cpp',
  meson.source_root() + '/src/cca_star/cca_star.cpp',
]
src_dir = [
  meson.source_root() + '/src/main.cpp' # necessary
] + src_files

# dependencies
boost_dep = dependency('boost', modules : ['graph', 'program_options']) # boost library that contains the module graph
rapidxml_dep = dependency('rapidxml') # xml parser, subproject library
gtest_dep = dependency('gtest', main : true, required : true) # test library

dep = [
  boost_dep, 
  rapidxml_dep,
  gtest_dep
]

# produce the executable
executable('rcmapf_solver', src_dir, dependencies: dep, include_directories: include_dir)

subdir('tests')
