
class ExperimentInputs:

	def __init__(self):

		# all the algorithms that will be tested 
		self.algo_to_test = self.__add_algo_with_swapping([
			"CODM",
			"CODM-BIDIR", 
			"CODM-OPT",
			"CCASTAR",
			"ODRM", 
		])

		# represent the parallel thread
		# We made this choice because the first thread consume less memory than the second
		# ODrM* and CODM* are more memory intensive than the rest.
		self.algo_threads = [
			self.__add_algo_with_swapping(["CODM-BIDIR", "CODM-OPT"]),
			self.__add_algo_with_swapping(["CODM"]),
			self.__add_algo_with_swapping(["ODRM"]),
			self.__add_algo_with_swapping(["CCASTAR"]),
		]

		# the map of the benchmark
		self.map_list = [
			"offices_range1_agents",
			"parallel_range4_agents",
			"pyramid_range3_3d_agents",
			"small_obstacles_range3_3d_agents"
		]

		# limit of each solver per instance
		self.time_limit = 120
		self.memory_limit = 4000000

		# folders where to find experiments
		self.graph_folder = "../../generation/graphs/"
		self.exp_folder = "../../generation/data/"

		# The "ids" represent the number of agents
		# of each experiments
		self.exp_ids = [2, 5, 10, 15]
		self.exp_ids = self.exp_ids + list(range(20, 90, 10))
		self.exp_ids = self.exp_ids + list(range(100, 325, 25))


		self.algo_to_plot = ["CCASTAR", "ODRM", "CODM", "CODM-OPT"]
		self.algo_to_plot_swapping = self.__add_algo_with_swapping(self.algo_to_plot)

		self.output_folder = "../plot_results/"
		self.swapping_output_folder = "../swapping_plot_results/"


	def __add_algo_with_swapping(self, algo_list):

		# add suffix "_SWAPPING" after all the algo names

		algo_to_test_swapping = []

		for algo in algo_list:
			algo_to_test_swapping.append(algo + "_SWAPPING")

		return algo_list + algo_to_test_swapping