#include "map_maker.h"
#include "uniform_cost_search.h"
#include "abstract_heuristic.h"

#include <cstring>

int main(int argc, char* argv[])
{
	heuristic_ns::abstract_heuristic *heuristic = new heuristic_ns::abstract_heuristic();
	solver_ns::uniform_cost_search solver(heuristic);
	map_maker_ns::map_maker map_input;
	map_maker_ns::result_path path_output;
	int start_col, start_row, goal_col, goal_row, expanded;

	if (argc < 3)
		printf("usage: experiment path_to_map_text heuristic_number visualize(optional, y or n) output_image_path(optional)\n");
	map_input.read_text((char*)(argv[1]));
	start_col = map_input.map_output.get_col_start();
	start_row = map_input.map_output.get_row_start();
	goal_col = map_input.map_output.get_col_goal();
	goal_row = map_input.map_output.get_row_goal();

	if (!solver.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, path_output, expanded))
		printf("no path\n");
	else
		printf("%f %d\n", path_output.compute_cost(map_input.map_output), expanded);
	
	delete[] heuristic;
	heuristic = NULL;

	if (argc > 3 && strcmp((char*)(argv[3]), "y") == 0) // visualize
	{
		cv::Mat map_input_img = map_input.show_map_img();
		cv::namedWindow("input");
		cv::imshow("input", map_input_img);
		cv::Mat map_result_img = map_input.show_result(map_input.map_output, path_output);
		cv::namedWindow("output");
		cv::imshow("output", map_result_img);
		cv::waitKey();
	}

	if (argc > 4) // store the output
	{
		
	}

	return 0;
}
