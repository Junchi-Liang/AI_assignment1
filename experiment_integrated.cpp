#include "map_maker.h"
#include "uniform_cost_search.h"
#include "abstract_heuristic.h"
#include "integrated_a_star.h"
#include "h1.h"
#include "h2.h"
#include "h3.h"
#include "h4.h"
#include "h5.h"

#include <cstring>

int main(int argc, char* argv[])
{
	heuristic_ns::abstract_heuristic *h0 = new heuristic_ns::abstract_heuristic();
	heuristic_ns::h1 *h1_heuristic = new heuristic_ns::h1();
	heuristic_ns::h2 *h2_heuristic = new heuristic_ns::h2();
	heuristic_ns::h3 *h3_heuristic = new heuristic_ns::h3();
	heuristic_ns::h4 *h4_heuristic = new heuristic_ns::h4();
	heuristic_ns::h5 *h5_heuristic = new heuristic_ns::h5();

	map_maker_ns::map_maker map_input;
	map_maker_ns::result_path path_output;
	int n_heuristic, start_col, start_row, goal_col, goal_row, expanded, overall_expanded;
	double w1, w2;


	if (argc < 5)
	{
		printf("usage: experiment path_to_map_text w1 w2 n_of_heuristic h0 h1 ... hn visualize(optional, y or n) output_image_path(optional)\n");
	}
	else
	{

		map_input.read_text((char*)(argv[1]));
		start_col = map_input.map_output.get_col_start();
		start_row = map_input.map_output.get_row_start();
		goal_col = map_input.map_output.get_col_goal();
		goal_row = map_input.map_output.get_row_goal();

		w1 = atof((char*)(argv[2]));
		w2 = atof((char*)(argv[3]));
		n_heuristic = atoi((char*)(argv[4]));
		solver_ns::integrated_a_star solver(n_heuristic, w1, w2);
		solver_ns::uniform_cost_search answer(h0);
		double answer_path_cost;
		int answer_expanded;

		for (int i = 0; i <= n_heuristic; i++)
		{
			if (strcmp((char*)(argv[5 + i]), "1") == 0)
				solver.set_heuristic(i, h1_heuristic);
			else if (strcmp((char*)(argv[5 + i]), "2") == 0)
				solver.set_heuristic(i, h2_heuristic);
			else if (strcmp((char*)(argv[5 + i]), "3") == 0)
				solver.set_heuristic(i, h3_heuristic);
			else if (strcmp((char*)(argv[5 + i]), "4") == 0)
				solver.set_heuristic(i, h4_heuristic);
			else if (strcmp((char*)(argv[5 + i]), "5") == 0)
				solver.set_heuristic(i, h5_heuristic);
			else
				solver.set_heuristic(i, h0);
		}

		answer.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, answer_expanded);
		answer_path_cost = path_output.compute_cost(map_input.map_output);
		auto start_time = std::chrono::steady_clock::now();
		if (!solver.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, expanded, overall_expanded))
			printf("no path\n");
		else
		{
			auto end_time = std::chrono::steady_clock::now();
			auto diff_time = end_time - start_time;
			double diff_sec = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count()) / 1000000;
			diff_sec /= 1000;
			int memUsedByThis = util_ns::get_value_for_memory();
			double path_cost = path_output.compute_cost(map_input.map_output);

			// path_cost path_cost_ratio expanded_nodes overal_expanded_nodes time_in_sec memory_in_kb
			printf("%lf %lf %d %d %lf %d\n", path_cost, path_cost / answer_path_cost, expanded, overall_expanded, diff_sec, memUsedByThis);
		}

		if (argc > 6 + n_heuristic && strcmp((char*)(argv[6 + n_heuristic]), "y") == 0) // visualize
		{
			cv::Mat map_input_img = map_input.show_map_img();
			cv::namedWindow("input");
			cv::imshow("input", map_input_img);
			cv::Mat map_result_img = map_input.show_result(map_input.map_output, path_output);
			cv::namedWindow("output");
			cv::imshow("output", map_result_img);
			cv::waitKey();
		}
	}
	
	delete[] h0;
	h0 = NULL;
	delete[] h1_heuristic;
	h1_heuristic = NULL;
	delete[] h2_heuristic;
	h2_heuristic = NULL;
	delete[] h3_heuristic;
	h3_heuristic = NULL;
	delete[] h4_heuristic;
	h4_heuristic = NULL;
	delete[] h5_heuristic;
	h5_heuristic = NULL;

	return 0;
}
