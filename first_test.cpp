#include "map_maker.h"
#include "abstract_heuristic.h"
#include "uniform_cost_search.h"
#include "h1.h"

int main()
{
	map_maker_ns::map_maker map_input;
	map_input.read_text("data.txt");
	cv::Mat map_input_img = map_input.show_map_img();
	cv::namedWindow("Map_CS520_input");
	cv::imshow("Map_CS520_input", map_input_img);
	//map_input.write_img_to_disk("map_input.jpg");
/*
	map_maker_ns::map_maker new_map;	
	new_map.map_build();
	new_map.write_text_to_disk("new_data.txt");
	cv::Mat new_map_img = new_map.show_map_img();
*/

	heuristic_ns::abstract_heuristic *heuristic = new heuristic_ns::abstract_heuristic();
	heuristic_ns::abstract_heuristic *h1_h = new heuristic_ns::h1();
	solver_ns::uniform_cost_search solver_0(heuristic);
	solver_ns::uniform_cost_search solver_1(h1_h);
	map_maker_ns::result_path result_0, result_1;
	int expanded, start_col, start_row, goal_col, goal_row;
	start_col = map_input.map_output.get_col_start();
	start_row = map_input.map_output.get_row_start();
	goal_col = map_input.map_output.get_col_goal();
	goal_row = map_input.map_output.get_row_goal();
	solver_0.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, result_0, expanded);
	solver_1.solve(start_col, start_row, goal_col, goal_row, map_input.map_output, result_1, expanded);
	delete heuristic;
	delete h1_h;
/*
	printf("expanded = %d\n", expanded);
	if (temp)
		printf("path found\n");
	else
		printf("no path found\n");
*/
/*
	cv::Mat map_result = map_input.show_result(map_input.map_output, result);
	cv::namedWindow("map_cs520_output");
	cv::imshow("map_cs520_output", map_result);
*/
	//map_input.write_img_to_disk("new_map.jpg");
	cv::Mat map_result_0 = map_input.show_result(map_input.map_output, result_0);
	cv::Mat map_result_1 = map_input.show_result(map_input.map_output, result_1);
	cv::namedWindow("map_cs520_output_0");
	cv::imshow("map_cs520_output_0", map_result_0);
	cv::namedWindow("map_cs520_output_1");
	cv::imshow("map_cs520_output_1", map_result_1);
	



	cv::waitKey();
	return 0;
}

