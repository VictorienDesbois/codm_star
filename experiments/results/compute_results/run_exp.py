import subprocess
import sys
import os 
import glob
import re
import resource

from exp_inputs import ExperimentInputs


def get_codm_command(
    graph_folder: str,
    exp_file: str,
    subsolver_type: str,
    swapping_conflict: bool,
    verbose: bool,
):
    return './rcmapf_solver -g "{0}" -e "{1}" -a "{2}" -s {3} -v {4}'.format(
        graph_folder, exp_file, subsolver_type, swapping_conflict, verbose
    )


def get_ccastar_command(graph_folder: str, exp_file: str):
    return "./castar -a CASTAR -e {0} -G {1} --collisions CHECK_COLLISIONS --nbst 100 --verbose false --subsolver CASTAR".format(
        exp_file, graph_folder
    )


def get_ccastar_swapping_command(graph_folder: str, exp_file: str):
    return "./castar_swapping -a CASTAR -e {0} -G {1} --collisions CHECK_COLLISIONS --nbst 100 --verbose false --subsolver CASTAR".format(
        exp_file, graph_folder
    )


def make_parallel_command(command_list):
    result = str()

    for i, command in enumerate(command_list):
        if i < len(command_list) - 1:
            result += "{0} &\n".format(command)
        else:
            result += "{0} & wait".format(command)

    return result


def gather_files(folder_path, exp_name):

    # Define the pattern to match files
    pattern = os.path.join(folder_path, f"{exp_name}*.exp")
    
    # Use glob to find all files matching the pattern
    matching_files = glob.glob(pattern)
    
    return matching_files


def solver_command_to_logfile(bench, command, logfile):
    return f"echo \"\n>>>>> {bench}\" >> {logfile}\ntime {command} >> {logfile} 2>&1"


def get_command_from_alg_ids(alg_id, graph_folder, exp_file, exp_name, logfile_path):
	
	# return the commands associated with each solvers
	# manage the case when we want to solve the instance with swapping conflict management

	command = str()
	
	match alg_id:
		case "CCASTAR":
			command = get_ccastar_command(graph_folder, exp_file)
		case "CCASTAR_SWAPPING":
			command = get_ccastar_swapping_command(graph_folder, exp_file)
		case codm_id:
			if "_SWAPPING" in codm_id:
				codm_id = codm_id.replace("_SWAPPING", "")
				command = get_codm_command(graph_folder, exp_file, codm_id, True, False)
			else:
				command = get_codm_command(graph_folder, exp_file, codm_id, False, False)

	return solver_command_to_logfile(exp_name, command, logfile_path)


def init_algo_swapping(algo_list):

	# add suffix "_SWAPPING" after all the algo names

	algo_to_test_swapping = []

	for algo in algo_list:
		algo_to_test_swapping.append(algo + "_SWAPPING")

	return algo_list + algo_to_test_swapping


def init_folders_commands(algo_list, folder):

	# init commands to create folders

	init = f"mkdir {folder}\n"
	init += f"cd {folder}\n"

	remove_command = str()
	make_command = str()

	for algo in algo_list:
		remove_command += f"rm -rf {algo}_logs\n"
		make_command += f"mkdir -p {algo}_logs\n"
	
	return init + remove_command + make_command


def init_commands_map(map_list, exp_folder, graph_folder, algo_to_test, exp_ids):

	# init all the commands needed to run associated with each algorithms
	# to test

	commands_map = {}

	for nb_agents in exp_ids:

		for map_name in map_list:

			current_exp_name = "{0}{1}_".format(map_name, nb_agents)
			all_exp_files = gather_files(exp_folder, current_exp_name)

			for exp_file in all_exp_files:

				splitted_expfile = exp_file.split(".")
				splitted_expfile = splitted_expfile[len(splitted_expfile)-2]
				splitted_expfile = splitted_expfile.split("_")
				exp_id = splitted_expfile[len(splitted_expfile)-1]

				for alg_id in algo_to_test:

					exp_name = f"{map_name}{nb_agents}"
					logfile_path = f"logs/{alg_id}_logs/{exp_name}.txt"
					current_command = get_command_from_alg_ids(
						alg_id, 
						graph_folder, 
						exp_file,
						f"{exp_name}_{exp_id}",
						logfile_path
					)

					print_command = f"echo \"solver: {alg_id}; instance: {exp_name}_{exp_id}\""
					if alg_id in commands_map:
						commands_map[alg_id].append((print_command, current_command))
					else:
						commands_map[alg_id] = [ (print_command, current_command) ]

	return commands_map


def get_limit_command(time_limit, memory_limit):
	return f"ulimit -t {time_limit}; ulimit -v {memory_limit}; ulimit -m {memory_limit};"


def init_thread_commands(time_limit, memory_limit, threads_by_alg, commands_map):

	thread_command = []

	limit_command = get_limit_command(time_limit, memory_limit)
	nb_thread = len(threads_by_alg)

	for i in range(nb_thread):
		thread_command.append("{")


	for key, val in commands_map.items():
		for i, alg_list in enumerate(threads_by_alg):
			if key in alg_list:
				for (echo_command, command) in val:
					thread_command[i] += f" {echo_command}; ( {limit_command} {command} ); "


	for i in range(nb_thread):
		thread_command[i] += " }"


	return thread_command


def main():

	exp_inputs = ExperimentInputs()

	init_folder_algo = init_folders_commands(exp_inputs.algo_to_test, "logs") 
	subprocess.run(init_folder_algo, shell=True)

	print("Init commands")

	commands_map = init_commands_map(
		exp_inputs.map_list, 
		exp_inputs.exp_folder, 
		exp_inputs.graph_folder, 
		exp_inputs.algo_to_test, 
		exp_inputs.exp_ids
	)

	print("Init threads")

	thread_command = init_thread_commands(
		exp_inputs.time_limit, 
		exp_inputs.memory_limit, 
		exp_inputs.algo_threads, 
		commands_map
	)

	run_command = make_parallel_command(thread_command)

	filename = "run_all_experiments.sh"

	with open(filename, 'w') as file:
		file.write(run_command)

	subprocess.call(["sh", f"./{filename}"])


if __name__ == "__main__":
	main()
