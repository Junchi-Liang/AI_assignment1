#include "map_maker.h"
#include "abstract_heuristic.h"
#include "uniform_cost_search.h"

int main()
{
	map_maker_ns::map_maker old_map;
	old_map.read_text("data.txt");
	cv::Mat old_map_img = old_map.show_map_img();
	cv::namedWindow("Map_CS520_old");
	cv::imshow("Map_CS520_old", old_map_img);
	old_map.write_img_to_disk("old_map.jpg");

	map_maker_ns::map_maker new_map;	
	new_map.map_build();
	new_map.write_text_to_disk("new_data.txt");
	cv::Mat new_map_img = new_map.show_map_img();

	heuristic_ns::abstract_heuristic *heuristic = new heuristic_ns::abstract_heuristic();
	solver_ns::uniform_cost_search solver(heuristic);
	map_maker_ns::result_path result;
	int expanded;
	bool temp = solver.solve(new_map.map_output.get_start_cell()[0], new_map.map_output.get_start_cell()[1], new_map.map_output.get_goal_cell()[0], \
		new_map.map_output.get_goal_cell()[1], new_map.map_output, result, expanded);
	delete heuristic;
	cv::Mat new_map_result = new_map.show_result(new_map.map_output, result);
	cv::namedWindow("map_cs520_new");
	cv::imshow("map_cs520_new", new_map_result);
	new_map.write_img_to_disk("new_map.jpg");


	cv::waitKey();
	return 0;
}

