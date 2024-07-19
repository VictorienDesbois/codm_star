# test files
test_files = [
	'test_graph.cpp',
	'test_meta_agent.cpp',
	'tests.cpp'
] + src_files

# produce the tests
run_tests = executable(
	'tests', test_files,
	dependencies : dep,
	include_directories: include_dir
)

test('gtest test', run_tests)