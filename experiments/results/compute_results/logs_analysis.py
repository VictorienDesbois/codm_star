import subprocess
import sys
import os 
import glob
import re
import resource
import pandas as pd
import matplotlib.pyplot as plt
import io

from exp_inputs import ExperimentInputs


class LogAnalysis:

	def __init__(self, algo_list, logs_folder):

		print("Analysis of log files.")

		self.success_plot_dict = {}
		self.time_plot_dict = {}
		self.cost_plot_dict = {}
		self.algo_list = algo_list

		for algo_name in algo_list:

			for log_file in self.__gather_log_file(f"{logs_folder}/{algo_name}_logs"):
				csv_content = str()

				if "CCASTAR" in algo_name:
					csv_content = self.__parse_ca_star_logs(log_file)
				else:
					csv_content = self.__parse_rcmapf_solver_logs(log_file)

				filename = log_file.replace(".txt", ".csv")

				print(f"result written in {filename}")

				with open(filename, 'w') as file:
					file.write(csv_content)

				# csv_df = pd.read_csv(io.StringIO(csv_content), delimiter=';')
				csv_df = pd.read_csv(filename, delimiter=';')
				df_len = len(csv_df)
				nb_success = csv_df["success"].sum()
				time_list = csv_df["time"].to_list()
				cost_list = csv_df["longest_path"].to_list()
				success_rate = nb_success / df_len

				bench_name = csv_df["exp"][0]
				pattern = r"^(.*?)_.*_agents(\d+)_"

				# Use re.search to match the pattern
				match = re.search(pattern, bench_name)

				if match:
				    map_name = match.group(1)
				    number_of_agents = int(match.group(2))
				    self.__add_to_success_plot(algo_name, map_name, number_of_agents, success_rate)
				    self.__add_to_time_plot(algo_name, map_name, time_list)
				    self.__add_to_cost_plot(algo_name, map_name, cost_list)


	def make_success_plot_picture(self, algo_to_show, algo_renaming, output_folder):

		plt.rcParams.update({'font.size': 14})
		
		markers_id = ["x", "o", "D", "v", "1", "2"]
		colors_id = ["r", "y", "b", "c", "m", "g"]
		alg_id_in_graph = 0

		map_dict_result = self.__create_plot_dict(self.success_plot_dict, algo_to_show)

		label = "Success"

		for map_id, (map_name, plot_list) in enumerate(map_dict_result.items()):
			alg_id_in_graph = 0
			plt.clf()
			for (algo_name, plot_data) in plot_list:

				if algo_name in algo_renaming:
					algo_name = algo_renaming[algo_name]

				plt.plot(
					list(plot_data.keys()), 
					list(plot_data.values()), 
					color=colors_id[alg_id_in_graph], 
					marker=markers_id[alg_id_in_graph], 
					linestyle="solid", 
					label=algo_name
				)
				
				alg_id_in_graph = alg_id_in_graph + 1

			if "office" in map_name:
				plt.legend(loc="upper right")

			picture_name = f"{map_name}_{label}_result"
			plt.xlabel("Nb agents")
			plt.ylabel(f"{label} rate")
			plt.xlim((0, 5))
			plt.ylim((0, 1.1))
			plt.savefig(f"{output_folder}{picture_name}.pdf")
			print(f"{label} plot of the map {map_name} at: {output_folder}{picture_name}.pdf")


	def make_time_plot_picture(self, algo_to_show, algo_renaming, output_folder):
		self.__make_snake_plot_picture(self.time_plot_dict, algo_to_show, "Time", algo_renaming, output_folder)


	def make_cost_plot_picture(self, algo_to_show, algo_renaming, output_folder):
		self.__make_snake_plot_picture(self.cost_plot_dict, algo_to_show, "Cost", algo_renaming, output_folder)


	def __make_snake_plot_picture(self, dict_plot, algo_to_show, label, algo_renaming, output_folder):
		
		plt.rcParams.update({'font.size': 14})

		colors_id = ["r", "y", "b", "c", "m", "g"]
		alg_id_in_graph = 0

		map_dict_result = self.__create_plot_dict(dict_plot, algo_to_show)

		for map_id, (map_name, plot_list) in enumerate(map_dict_result.items()):
			
			alg_id_in_graph = 0
			plt.clf()
			
			for (algo_name, plot_data) in plot_list:
				
				if algo_name in algo_renaming:
					algo_name = algo_renaming[algo_name]

				plt.plot(
					list(range(len(plot_data))), 
					list(plot_data), 
					color=colors_id[alg_id_in_graph], 
					linestyle="solid", 
					label=algo_name
				)

				alg_id_in_graph = alg_id_in_graph + 1

			if "office" in map_name:
				plt.legend(loc="upper left")

			picture_name = f"{map_name}_{label}_result"
			plt.xlabel("Instance")
			plt.ylabel(f"{label}")
			plt.savefig(f"{output_folder}{picture_name}.pdf")
			print(f"{label} plot of the map {map_name} at: {output_folder}{picture_name}.pdf")


	def __create_plot_dict(self, dict_plot, algo_to_show):

		map_dict_result = {}

		for algo_name, map_dict in dict_plot.items():
			
			if algo_name in algo_to_show:
				for map_name, plot_data in map_dict.items():

					if not (map_name in map_dict_result):
						map_dict_result[map_name] = []

					map_dict_result[map_name].append((algo_name, plot_data))

		return map_dict_result


	def __add_to_success_plot(self, algo_name, map_name, nb_agents, success_rate):
		
		if not (algo_name in self.success_plot_dict):
			self.success_plot_dict[algo_name] = {}

		if not (map_name in self.success_plot_dict[algo_name]):
			self.success_plot_dict[algo_name][map_name] = {}

		self.success_plot_dict[algo_name][map_name][nb_agents] = float(success_rate)


	def __add_to_time_plot(self, algo_name, map_name, time_list):
		self.__make_snake_plot(self.time_plot_dict, algo_name, map_name, time_list)


	def __add_to_cost_plot(self, algo_name, map_name, cost_list):
		self.__make_snake_plot(self.cost_plot_dict, algo_name, map_name, cost_list)


	def __make_snake_plot(self, dict_plot, algo_name, map_name, curr_list):
		
		if not (algo_name in dict_plot):
			dict_plot[algo_name] = {}

		if not (map_name in dict_plot[algo_name]):
			dict_plot[algo_name][map_name] = []

		dict_plot[algo_name][map_name] = dict_plot[algo_name][map_name] + curr_list
		dict_plot[algo_name][map_name].sort()


	def __gather_log_file(self, folder_path):

	    # Define the pattern to match files
	    pattern = os.path.join(folder_path, f"*.txt")
	    
	    # Use glob to find all files matching the pattern
	    matching_files = glob.glob(pattern)

	    return matching_files


	def __parse_ca_star_logs(self, logfile):
		
		# compile the results of CA* algorithm
		result = "exp;success;longest_path;time\n"

		with open(logfile,'r') as f:

			bench = str()
			time = 0
			cost = 0

			for line in f:
				m = re.search('>>>>> (.*)', line)
				if m:
					bench=m.group(1)
			    
				m = re.search('Longest path: ([0-9]+)', line)
				if m:
					cost = int(m.group(1))

				m = re.search('(.*)user', line)
				if m:
					time = float(m.group(1))
					is_success = 0 if cost <= 0 else 1
					result += f"{bench};{is_success};{cost};{time}\n"

		return result


	def __parse_rcmapf_solver_logs(self, logfile):

		# compile the results of CODM* / ODrM* algorithms
		result = "exp;success;longest_path;time\n"

		with open(logfile,'r') as f:
			bench = ""
			time=0
			memory=0
			success=0
			longest_path=0

			for line in f:
				# each new line start by >>>>>
				m = re.search('>>>>> (.*)', line)
				if m:
					bench=m.group(1)

				m = re.search('Execution found!', line)
				if m:
					success = 1

				m = re.search('Longest path: ([0-9]+)', line)
				if m:
					longest_path=int(m.group(1))

				m = re.search('([0-9]+)maxresident', line)
				if m:
					memory = int(int(m.group(1)) / 1000) 

				m = re.search('(.*)user', line)
				if m:
					time = float(m.group(1))
					result += f"{bench};{success};{longest_path};{time}\n"

		return result


def main():

	exp_inputs = ExperimentInputs()

	log_analysis = LogAnalysis(exp_inputs.algo_to_test, "logs")

	algo_renaming = {
	 "CODM": "CODM*",
	 "CODM_SWAPPING": "CODM*",
	 "CODM-BIDIR": "CODM*-bidir*",
	 "CODM-BIDIR_SWAPPING": "CODM*-bidir*",
	 "CODM-OPT": "CODM*-opt*",
	 "CODM-OPT_SWAPPING": "CODM*-opt*",
	 "CCASTAR": "CA*",
	 "CCASTAR_SWAPPING": "CA*",
	}

	# classic plot pictures
	log_analysis.make_success_plot_picture(exp_inputs.algo_to_plot, algo_renaming, exp_inputs.output_folder)
	log_analysis.make_time_plot_picture(exp_inputs.algo_to_plot, algo_renaming, exp_inputs.output_folder)
	log_analysis.make_cost_plot_picture(exp_inputs.algo_to_plot, algo_renaming, exp_inputs.output_folder)

	# swapping plot pictures
	log_analysis.make_success_plot_picture(exp_inputs.algo_to_plot_swapping, algo_renaming, exp_inputs.swapping_output_folder)
	log_analysis.make_time_plot_picture(exp_inputs.algo_to_plot_swapping, algo_renaming, exp_inputs.swapping_output_folder)
	log_analysis.make_cost_plot_picture(exp_inputs.algo_to_plot_swapping, algo_renaming, exp_inputs.swapping_output_folder)


if __name__ == "__main__":
	main()